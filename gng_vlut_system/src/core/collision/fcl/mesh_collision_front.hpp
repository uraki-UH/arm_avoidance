#pragma once

#ifdef USE_FCL

#include "collision/fcl/fcl_collision_detector.hpp"
#include <fcl/narrowphase/detail/traversal/collision/mesh_collision_traversal_node.h>

namespace collision::detail {

// 元のBVHを覆う探索境界の保持上限と、細分化の蓄積を抑える再構築周期。
constexpr std::size_t max_mesh_front_pairs = 8192;
constexpr std::size_t max_mesh_front_uses = 32;
using mesh_collision_node = fcl::detail::MeshCollisionTraversalNodeOBBRSS<double>;

// 上限到達後も衝突検査を継続し、不完全な探索境界だけを破棄する記録器。
struct mesh_front_builder {
  std::vector<std::pair<int, int>> &pairs;
  bool has_overflow = false;

  void append(int first_idx, int second_idx) {
    if (has_overflow) return;
    if (pairs.size() == max_mesh_front_pairs) {
      pairs.clear();
      has_overflow = true;
      return;
    }
    pairs.emplace_back(first_idx, second_idx);
  }
};

// FCLと同じ包絡・分岐順・三角形述語による探索。現在姿勢での全境界の再検査。
inline bool has_mesh_surface_collision(mesh_collision_node &node,
                                       int first_idx, int second_idx,
                                       mesh_front_builder &front) {
  const auto &first = node.model1->getBV(first_idx);
  const auto &second = node.model2->getBV(second_idx);
  if (!fcl::overlap(node.R, node.T, first.bv, second.bv)) {
    front.append(first_idx, second_idx);
    return false;
  }
  if (first.isLeaf() && second.isLeaf()) {
    front.append(first_idx, second_idx);
    node.mesh_collision_node::leafTesting(first_idx, second_idx);
    return node.result->isCollision();
  }
  if (second.isLeaf() || (!first.isLeaf() && first.bv.size() > second.bv.size())) {
    return has_mesh_surface_collision(node, first.leftChild(), second_idx, front) ||
           has_mesh_surface_collision(node, first.rightChild(), second_idx, front);
  }
  return has_mesh_surface_collision(node, first_idx, second.leftChild(), front) ||
         has_mesh_surface_collision(node, first_idx, second.rightChild(), front);
}

// 幾何不変時のbool判定専用。接触点・距離・費用の問合せは通常FCL経路のまま。
inline void check_mesh_collision_with_front(
    fcl::CollisionObject<double> &first, fcl::CollisionObject<double> &second,
    const fcl::CollisionRequest<double> &request, fcl::CollisionResult<double> &result,
    self_collision_pose_cache_entry &entry, bool is_front_reversed) {
  if (first.getNodeType() != fcl::BV_OBBRSS || second.getNodeType() != fcl::BV_OBBRSS ||
      request.enable_contact || request.enable_cost || request.num_max_contacts != 1) {
    entry.mesh_front.clear();
    entry.next_mesh_front.clear();
    fcl::collide(&first, &second, request, result);
    return;
  }
  const auto &first_model =
      static_cast<const fcl::BVHModel<fcl::OBBRSS<double>> &>(*first.collisionGeometry());
  const auto &second_model =
      static_cast<const fcl::BVHModel<fcl::OBBRSS<double>> &>(*second.collisionGeometry());
  mesh_collision_node node;
  if (!fcl::detail::initialize(node, first_model, first.getTransform(),
                               second_model, second.getTransform(), request, result)) {
    entry.mesh_front.clear();
    entry.next_mesh_front.clear();
    fcl::collide(&first, &second, request, result);
    return;
  }
  if (entry.is_front_reversed != is_front_reversed ||
      ++entry.num_front_uses > max_mesh_front_uses) {
    entry.mesh_front.clear();
    entry.num_front_uses = 1;
  }
  entry.is_front_reversed = is_front_reversed;
  entry.next_mesh_front.clear();
  mesh_front_builder next{entry.next_mesh_front};
  bool is_collision = false;
  if (entry.mesh_front.empty()) {
    is_collision = has_mesh_surface_collision(node, 0, 0, next);
  } else {
    for (const auto &pair : entry.mesh_front) {
      if (has_mesh_surface_collision(node, pair.first, pair.second, next)) {
        is_collision = true;
        break;
      }
    }
  }
  if (is_collision || next.has_overflow) entry.mesh_front.clear();
  else entry.mesh_front.swap(entry.next_mesh_front);
  entry.next_mesh_front.clear();
}

}  // collision::detail名前空間の終端

#endif
