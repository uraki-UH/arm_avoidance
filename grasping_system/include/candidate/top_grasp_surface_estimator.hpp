#pragma once

#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace grasping_system::candidate
{

struct TopGraspSurfaceConfig
{
  Eigen::Vector3d up_axis = Eigen::Vector3d::UnitZ();
  double minimum_protrusion_distance = 0.01;
  std::size_t minimum_region_nodes = 4U;
  double grasp_size_x = 0.061;
  double grasp_size_y = 0.074;
  double footprint_margin = 0.005;
  double footprint_padding = 0.005;
  double tcp_standoff = 0.0;
  std::size_t maximum_candidates = 20U;
  double max_surface_tilt_deg = 25.0;
  bool enable_nonplane_attachment = true;
  // 候補平面起点の接続探索と、接続する開口外平面からの離隔判定
  bool enable_reference_plane_attachment = false;
  // 候補内平均エッジ長に対する入口エッジ長の許容倍率
  double max_attachment_edge_length_ratio = 1.3;
  bool enable_approach_check = true;
  double approach_height = 0.10;
  double approach_margin = 0.01;
};

struct TopGraspSurfaceCandidate
{
  std::uint32_t cluster_id = 0U;
  std::vector<std::uint32_t> node_indices;
  // 平面OBBとは独立した、同一フレーム内の付属ノード添字と局所外形
  std::vector<std::uint32_t> attached_node_indices;
  std::size_t attached_component_num = 0U;
  double target_extent_x = 0.0;
  double target_extent_y = 0.0;
  Eigen::Vector3d tcp_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond tcp_orientation = Eigen::Quaterniond::Identity();
  double extent_x = 0.0;
  double extent_y = 0.0;
  double surface_height = 0.0;
  double footprint_fill_ratio = 0.0;
  std::size_t adjacent_region_count = 0U;
  bool has_neighbor_plane_distance = false;
  double minimum_neighbor_plane_distance = 0.0;
};

struct TopGraspSurfaceResult
{
  std::size_t region_count = 0U;
  std::size_t adjacent_region_pair_count = 0U;
  std::size_t rejected_small_region = 0U;
  std::size_t rejected_invalid_region = 0U;
  std::size_t rejected_oversize_region = 0U;
  std::size_t rejected_low_protrusion_region = 0U;
  std::size_t rejected_surface_tilt = 0U;
  std::size_t rejected_attached_oversize = 0U;
  std::size_t rejected_approach_obstacle = 0U;
  std::vector<TopGraspSurfaceCandidate> candidates;
};

class TopGraspSurfaceEstimator
{
public:
  explicit TopGraspSurfaceEstimator(TopGraspSurfaceConfig config = {})
  : config_(std::move(config))
  {
    validateConfig();
  }

  TopGraspSurfaceResult estimate(
    const ais_gng_msgs::msg::TopologicalMap &map,
    const ais_gng_msgs::msg::PlaneClusterArray &clusters) const
  {
    TopGraspSurfaceResult result;
    result.region_count = clusters.clusters.size();
    if (map.frame_number != clusters.frame_number ||
      map.header.frame_id != clusters.header.frame_id ||
      map.nodes.empty() || clusters.clusters.empty())
    {
      return result;
    }

    const auto [basis_u, basis_v] = horizontalBasis();
    const std::size_t region_count = clusters.clusters.size();
    std::vector<std::uint8_t> candidate_eligible(region_count, 0U);
    std::vector<Footprint> footprints(region_count);
    std::vector<int> owner_by_node(map.nodes.size(), -1);

    for (std::size_t region_index = 0U; region_index < region_count; ++region_index) {
      const auto &cluster = clusters.clusters[region_index];
      const Eigen::Vector3d centroid(
        cluster.centroid.x, cluster.centroid.y, cluster.centroid.z);
      if (!centroid.allFinite() || std::any_of(
          cluster.node_indices.begin(), cluster.node_indices.end(),
          [&map](std::uint32_t node_index) {return node_index >= map.nodes.size();}))
      {
        ++result.rejected_invalid_region;
        continue;
      }
      for (const std::uint32_t node_index : cluster.node_indices) {
        if (owner_by_node[node_index] < 0) {
          owner_by_node[node_index] = static_cast<int>(region_index);
        }
      }
      if (cluster.node_indices.size() < config_.minimum_region_nodes) {
        ++result.rejected_small_region;
        continue;
      }
      const Eigen::Vector3d normal(cluster.normal.x, cluster.normal.y, cluster.normal.z);
      if (!normal.allFinite() || normal.norm() < 1.0e-12) {
        ++result.rejected_invalid_region;
        continue;
      }
      footprints[region_index] = fitFootprint(
        cluster.node_indices, map, basis_u, basis_v);
      // 法線の符号に依存しない、上方向との傾斜角判定
      if (std::abs(normal.normalized().dot(config_.up_axis)) + 1.0e-12 <
        std::cos(config_.max_surface_tilt_deg * std::acos(-1.0) / 180.0))
      {
        ++result.rejected_surface_tilt;
        continue;
      }
      footprints[region_index] = fitFootprint(
        cluster.node_indices, map, basis_u, basis_v);
      if (!footprints[region_index].valid) {
        ++result.rejected_invalid_region;
      } else if (!footprints[region_index].fits) {
        ++result.rejected_oversize_region;
      } else {
        candidate_eligible[region_index] = 1U;
      }
    }

    std::vector<std::unordered_set<std::size_t>> adjacency(region_count);
    std::vector<double> internal_edge_length_sum(region_count, 0.0);
    std::vector<std::size_t> internal_edge_num(region_count, 0U);
    std::vector<std::vector<std::uint32_t>> node_adjacency(
      (config_.enable_nonplane_attachment || config_.enable_reference_plane_attachment) ? map.nodes.size() : 0U);
    std::unordered_map<std::uint32_t, int> component_owner;
    const auto register_owner = [&](std::size_t node_idx, int plane_idx) {
        const auto component_id = map.nodes[node_idx].nonplane_component_id;
        if (plane_idx < 0 || owner_by_node[node_idx] >= 0 ||
          component_id == ais_gng_msgs::msg::TopologicalNode::NONPLANE_COMPONENT_NONE)
        {
          return;
        }
        const auto [it, is_inserted] = component_owner.emplace(component_id, plane_idx);
        if (!is_inserted && it->second != plane_idx) {
          it->second = -2;
        }
      };
    for (std::size_t edge = 0U; edge + 1U < map.edges.size(); edge += 2U) {
      const std::size_t first_node = map.edges[edge];
      const std::size_t second_node = map.edges[edge + 1U];
      if (first_node >= owner_by_node.size() || second_node >= owner_by_node.size()) {
        continue;
      }
      const int first_owner = owner_by_node[first_node];
      const int second_owner = owner_by_node[second_node];
      if (first_owner >= 0 && first_owner == second_owner && first_node != second_node) {
        const auto &first = map.nodes[first_node].pos;
        const auto &second = map.nodes[second_node].pos;
        const double length = (Eigen::Vector3d(first.x, first.y, first.z) -
          Eigen::Vector3d(second.x, second.y, second.z)).norm();
        if (std::isfinite(length) && length > 0.0) {
          internal_edge_length_sum[first_owner] += length;
          ++internal_edge_num[first_owner];
        }
      }
      if (config_.enable_nonplane_attachment || config_.enable_reference_plane_attachment) {
        node_adjacency[first_node].push_back(second_node);
        node_adjacency[second_node].push_back(first_node);
        register_owner(first_node, second_owner);
        register_owner(second_node, first_owner);
      }
      if (first_owner < 0 || second_owner < 0 || first_owner == second_owner) {
        continue;
      }
      adjacency[static_cast<std::size_t>(first_owner)].insert(
        static_cast<std::size_t>(second_owner));
      adjacency[static_cast<std::size_t>(second_owner)].insert(
        static_cast<std::size_t>(first_owner));
    }
    if (config_.enable_reference_plane_attachment) {
      // 非平面連結成分を介した平面間接続。平面内部を経由した推移的な接続の除外
      std::vector<bool> is_visited(map.nodes.size(), false);
      const auto can_bridge = [&](std::size_t node_idx) {
          const auto &node = map.nodes[node_idx];
          return owner_by_node[node_idx] < 0 &&
            std::isfinite(node.pos.x) && std::isfinite(node.pos.y) && std::isfinite(node.pos.z) &&
            !(node.boundary_evidence & ais_gng_msgs::msg::TopologicalNode::BOUNDARY_FREE_SPACE);
        };
      for (std::size_t seed_idx = 0; seed_idx < map.nodes.size(); ++seed_idx) {
        if (is_visited[seed_idx] || !can_bridge(seed_idx)) continue;
        std::vector<std::size_t> pending{seed_idx};
        std::unordered_set<std::size_t> connected_planes;
        is_visited[seed_idx] = true;
        for (std::size_t idx = 0; idx < pending.size(); ++idx) {
          for (const auto next_idx : node_adjacency[pending[idx]]) {
            if (owner_by_node[next_idx] >= 0) {
              connected_planes.insert(static_cast<std::size_t>(owner_by_node[next_idx]));
            } else if (!is_visited[next_idx] && can_bridge(next_idx)) {
              is_visited[next_idx] = true;
              pending.push_back(next_idx);
            }
          }
        }
        for (const auto first_idx : connected_planes) {
          for (const auto second_idx : connected_planes) {
            if (first_idx != second_idx) adjacency[first_idx].insert(second_idx);
          }
        }
      }
    }
    for (const auto &neighbours : adjacency) {
      result.adjacent_region_pair_count += neighbours.size();
    }
    result.adjacent_region_pair_count /= 2U;

    for (std::size_t region_index = 0U; region_index < region_count; ++region_index) {
      if (candidate_eligible[region_index] == 0U) {
        continue;
      }
      const auto &cluster = clusters.clusters[region_index];
      double minimum_plane_distance = std::numeric_limits<double>::infinity();
      std::vector<Eigen::Vector4d> reference_planes;
      for (const std::size_t neighbour_index : adjacency[region_index]) {
        if (config_.enable_reference_plane_attachment &&
          (!footprints[neighbour_index].valid || footprints[neighbour_index].fits)) continue;
        const auto &neighbour = clusters.clusters[neighbour_index];
        Eigen::Vector3d normal(
          neighbour.normal.x, neighbour.normal.y, neighbour.normal.z);
        const Eigen::Vector3d neighbour_centroid(
          neighbour.centroid.x, neighbour.centroid.y, neighbour.centroid.z);
        if (!normal.allFinite() || normal.norm() < 1.0e-12 ||
          !neighbour_centroid.allFinite())
        {
          continue;
        }
        normal.normalize();
        const Eigen::Vector3d candidate_centroid(
          cluster.centroid.x, cluster.centroid.y, cluster.centroid.z);
        // 候補平面の重心側を正とした、参照面法線の統一
        if (normal.dot(candidate_centroid - neighbour_centroid) < 0.0) {
          normal = -normal;
        }
        reference_planes.emplace_back(normal.x(), normal.y(), normal.z(),
          -normal.dot(neighbour_centroid));
        minimum_plane_distance = std::min(
          minimum_plane_distance,
          std::abs(normal.dot(candidate_centroid - neighbour_centroid)));
      }
      const bool has_plane_distance = std::isfinite(minimum_plane_distance);
      if (has_plane_distance &&
        minimum_plane_distance < config_.minimum_protrusion_distance)
      {
        ++result.rejected_low_protrusion_region;
        continue;
      }

      const Footprint &footprint = footprints[region_index];

      const Eigen::Vector3d world_y =
        (basis_u * footprint.local_y_axis.x() +
        basis_v * footprint.local_y_axis.y()).normalized();
      const Eigen::Vector3d world_z = -config_.up_axis;
      const Eigen::Vector3d world_x = world_y.cross(world_z).normalized();
      Eigen::Matrix3d rotation;
      rotation.col(0) = world_x;
      rotation.col(1) = world_y;
      rotation.col(2) = world_z;

      TopGraspSurfaceCandidate candidate;
      candidate.cluster_id = cluster.id;
      candidate.node_indices = cluster.node_indices;
      candidate.tcp_position = basis_u * footprint.center_uv.x() +
        basis_v * footprint.center_uv.y() +
        config_.up_axis * (footprint.maximum_height + config_.tcp_standoff);
      candidate.tcp_orientation = Eigen::Quaterniond(rotation).normalized();
      candidate.extent_x = footprint.extent_x;
      candidate.extent_y = footprint.extent_y;
      candidate.surface_height = footprint.maximum_height;
      candidate.footprint_fill_ratio = footprint.fill_ratio;
      candidate.adjacent_region_count = adjacency[region_index].size();
      candidate.has_neighbor_plane_distance = has_plane_distance;
      if (has_plane_distance) {
        candidate.minimum_neighbor_plane_distance = minimum_plane_distance;
      }
      if (!evaluate_nonplane(
          map, region_index, owner_by_node, node_adjacency, component_owner,
          reference_planes, internal_edge_num[region_index] > 0 ?
            internal_edge_length_sum[region_index] / internal_edge_num[region_index] :
            static_cast<double>(cluster.local_spacing), candidate, result))
      {
        continue;
      }
      result.candidates.push_back(std::move(candidate));
    }

    std::sort(
      result.candidates.begin(), result.candidates.end(),
      [](const TopGraspSurfaceCandidate &first, const TopGraspSurfaceCandidate &second) {
        if (first.surface_height != second.surface_height) {
          return first.surface_height > second.surface_height;
        }
        return first.footprint_fill_ratio > second.footprint_fill_ratio;
      });
    if (result.candidates.size() > config_.maximum_candidates) {
      result.candidates.resize(config_.maximum_candidates);
    }
    return result;
  }

private:
  bool evaluate_nonplane(
    const ais_gng_msgs::msg::TopologicalMap &map, std::size_t plane_idx,
    const std::vector<int> &owner_by_node,
    const std::vector<std::vector<std::uint32_t>> &node_adjacency,
    const std::unordered_map<std::uint32_t, int> &component_owner,
    const std::vector<Eigen::Vector4d> &reference_planes,
    double max_attachment_edge_length,
    TopGraspSurfaceCandidate &candidate, TopGraspSurfaceResult &result) const
  {
    const Eigen::Matrix3d rotation = candidate.tcp_orientation.toRotationMatrix();
    const auto local_point = [&](std::size_t node_idx) -> Eigen::Vector3d {
        const auto &p = map.nodes[node_idx].pos;
        const Eigen::Vector3d delta = Eigen::Vector3d(p.x, p.y, p.z) -
          (candidate.tcp_position - config_.up_axis * config_.tcp_standoff);
        return {delta.dot(rotation.col(0)), delta.dot(rotation.col(1)),
          delta.dot(config_.up_axis)};
      };
    candidate.target_extent_x = candidate.extent_x;
    candidate.target_extent_y = candidate.extent_y;

    // 接続・クラスタ所属によらない観測障害物の確認。未観測空間の安全保証なし
    if (config_.enable_approach_check) {
      for (std::size_t node_idx = 0; node_idx < map.nodes.size(); ++node_idx) {
        const Eigen::Vector3d p = local_point(node_idx);
        if (p.allFinite() && p.z() > 0.0 &&
          p.z() <= config_.approach_height + std::max(0.0, config_.tcp_standoff) &&
          std::abs(p.x()) <= 0.5 * config_.grasp_size_x + config_.approach_margin &&
          std::abs(p.y()) <= 0.5 * config_.grasp_size_y + config_.approach_margin)
        {
          ++result.rejected_approach_obstacle;
          return false;
        }
      }
    }
    if (config_.enable_reference_plane_attachment ? reference_planes.empty() :
      !config_.enable_nonplane_attachment) {
      return true;
    }

    // 訪問済み管理付き接続探索。累積距離による打ち切りなし
    std::queue<std::uint32_t> queue;
    std::vector<bool> is_visited(map.nodes.size(), false);
    for (const auto node_idx : candidate.node_indices) {
      if (is_visited[node_idx]) continue;
      is_visited[node_idx] = true;
      queue.push(node_idx);
    }
    std::unordered_set<std::uint32_t> attached_components;
    Eigen::Vector2d min_target(-0.5 * candidate.extent_x, -0.5 * candidate.extent_y);
    Eigen::Vector2d max_target = -min_target;
    const double usable_x = 0.5 * config_.grasp_size_x - config_.footprint_margin;
    const double usable_y = 0.5 * config_.grasp_size_y - config_.footprint_margin;
    while (!queue.empty()) {
      const auto node_idx = queue.front();
      queue.pop();
      const auto &node = map.nodes[node_idx];
      const Eigen::Vector3d p = local_point(node_idx);
      if (owner_by_node[node_idx] != static_cast<int>(plane_idx)) {
        candidate.attached_node_indices.push_back(node_idx);
        if (node.nonplane_component_id !=
          ais_gng_msgs::msg::TopologicalNode::NONPLANE_COMPONENT_NONE)
          attached_components.insert(node.nonplane_component_id);
        min_target = min_target.cwiseMin(
          (p.head<2>().array() - config_.footprint_padding).matrix());
        max_target = max_target.cwiseMax(
          (p.head<2>().array() + config_.footprint_padding).matrix());
        if (std::abs(p.x()) + config_.footprint_padding > usable_x ||
          std::abs(p.y()) + config_.footprint_padding > usable_y)
        {
          ++result.rejected_attached_oversize;
          return false;
        }
      }
      if (node.boundary_evidence & ais_gng_msgs::msg::TopologicalNode::BOUNDARY_FREE_SPACE) {
        continue;
      }
      for (const auto next_idx : node_adjacency[node_idx]) {
        if (is_visited[next_idx]) continue;
        // 候補平面の外では非平面ノードだけを探索。他平面の取り込み・経由の禁止
        if (owner_by_node[next_idx] >= 0) continue;
        const auto &next = map.nodes[next_idx];
        const auto it = component_owner.find(next.nonplane_component_id);
        if (!config_.enable_reference_plane_attachment &&
          (owner_by_node[next_idx] >= 0 || it == component_owner.end() ||
          it->second != static_cast<int>(plane_idx) ||
          (next.boundary_evidence & ais_gng_msgs::msg::TopologicalNode::BOUNDARY_FREE_SPACE) ||
          (owner_by_node[node_idx] < 0 &&
          node.nonplane_component_id != next.nonplane_component_id)))
        {
          continue;
        }
        const Eigen::Vector3d next_p = local_point(next_idx);
        // 候補平面から非平面への入口エッジを、候補内平均長と許容倍率で選別
        if (config_.enable_reference_plane_attachment &&
          owner_by_node[node_idx] == static_cast<int>(plane_idx) && owner_by_node[next_idx] < 0 &&
          (!std::isfinite(max_attachment_edge_length) || max_attachment_edge_length <= 0.0 ||
          (next_p - p).norm() > max_attachment_edge_length * config_.max_attachment_edge_length_ratio)) continue;
        if (config_.enable_reference_plane_attachment) {
          // 非平面ノードの参照面からの距離判定。近接帯を越えた探索の禁止
          const Eigen::Vector3d world_p(next.pos.x, next.pos.y, next.pos.z);
          if (!world_p.allFinite() ||
            (next.boundary_evidence & ais_gng_msgs::msg::TopologicalNode::BOUNDARY_FREE_SPACE) ||
            std::any_of(reference_planes.begin(), reference_planes.end(),
              [&](const Eigen::Vector4d &plane) {
                return plane.head<3>().dot(world_p) + plane.w() <
                  config_.minimum_protrusion_distance;
              })) continue;
          is_visited[next_idx] = true;
          queue.push(next_idx);
          continue;
        }
        if (!next_p.allFinite())
        {
          continue;
        }
        is_visited[next_idx] = true;
        queue.push(next_idx);
      }
    }
    candidate.attached_component_num = attached_components.size();
    candidate.target_extent_x = max_target.x() - min_target.x();
    candidate.target_extent_y = max_target.y() - min_target.y();
    return true;
  }

  struct Footprint
  {
    bool valid = false;
    bool fits = false;
    Eigen::Vector2d local_y_axis = Eigen::Vector2d::UnitY();
    Eigen::Vector2d center_uv = Eigen::Vector2d::Zero();
    double extent_x = 0.0;
    double extent_y = 0.0;
    double maximum_height = 0.0;
    double fill_ratio = 0.0;
  };

  void validateConfig()
  {
    if (!std::isfinite(config_.max_attachment_edge_length_ratio) ||
      config_.max_attachment_edge_length_ratio <= 0.0) {
      throw std::invalid_argument("入口エッジ長の許容倍率は正の有限値が必要です");
    }
    if (!std::isfinite(config_.max_surface_tilt_deg) ||
      config_.max_surface_tilt_deg < 0.0 || config_.max_surface_tilt_deg > 90.0)
    {
      throw std::invalid_argument("max_surface_tilt_deg must be within [0, 90]");
    }
    for (const double value : {config_.approach_height, config_.approach_margin})
    {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument("接近領域の寸法は非負の有限値が必要です");
      }
    }
    if (config_.enable_approach_check && config_.approach_height <= 0.0) {
      throw std::invalid_argument("接近判定を有効にする場合、接近領域の高さは正の値が必要です");
    }
    if (!std::isfinite(config_.tcp_standoff) || !std::isfinite(config_.footprint_margin) ||
      !std::isfinite(config_.footprint_padding))
    {
      throw std::invalid_argument("footprint offsets must be finite");
    }
    if (!config_.up_axis.allFinite() || config_.up_axis.norm() < 1.0e-12) {
      throw std::invalid_argument("up_axis must be finite and non-zero");
    }
    config_.up_axis.normalize();
    if (!std::isfinite(config_.minimum_protrusion_distance) ||
      config_.minimum_protrusion_distance < 0.0)
    {
      throw std::invalid_argument("minimum_protrusion_distance must be finite and non-negative");
    }
    if (!std::isfinite(config_.grasp_size_x) || !std::isfinite(config_.grasp_size_y) ||
      config_.grasp_size_x <= 0.0 || config_.grasp_size_y <= 0.0 ||
      config_.footprint_margin < 0.0 || config_.footprint_padding < 0.0 ||
      2.0 * config_.footprint_margin >= std::min(config_.grasp_size_x, config_.grasp_size_y))
    {
      throw std::invalid_argument("grasp footprint parameters are invalid");
    }
    if (config_.minimum_region_nodes == 0U || config_.maximum_candidates == 0U) {
      throw std::invalid_argument("region and candidate limits must be positive");
    }
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> horizontalBasis() const
  {
    const Eigen::Vector3d reference =
      std::abs(config_.up_axis.dot(Eigen::Vector3d::UnitX())) < 0.9 ?
      Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d basis_u =
      (reference - config_.up_axis * config_.up_axis.dot(reference)).normalized();
    return {basis_u, config_.up_axis.cross(basis_u).normalized()};
  }

  Footprint fitFootprint(
    const std::vector<std::uint32_t> &node_indices,
    const ais_gng_msgs::msg::TopologicalMap &map,
    const Eigen::Vector3d &basis_u,
    const Eigen::Vector3d &basis_v) const
  {
    Footprint result;
    if (node_indices.size() < config_.minimum_region_nodes) {
      return result;
    }
    std::vector<Eigen::Vector2d> projected;
    projected.reserve(node_indices.size());
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    result.maximum_height = -std::numeric_limits<double>::infinity();
    for (const std::uint32_t node_index : node_indices) {
      if (node_index >= map.nodes.size()) {
        return result;
      }
      const auto &point = map.nodes[node_index].pos;
      const Eigen::Vector3d position(point.x, point.y, point.z);
      if (!position.allFinite()) {
        return result;
      }
      projected.emplace_back(position.dot(basis_u), position.dot(basis_v));
      mean += projected.back();
      result.maximum_height = std::max(
        result.maximum_height, position.dot(config_.up_axis));
    }
    mean /= static_cast<double>(projected.size());

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const Eigen::Vector2d &point : projected) {
      const Eigen::Vector2d delta = point - mean;
      covariance.noalias() += delta * delta.transpose();
    }
    covariance /= static_cast<double>(projected.size());
    const double principal_angle = 0.5 * std::atan2(
      2.0 * covariance(0, 1), covariance(0, 0) - covariance(1, 1));
    const Eigen::Vector2d major_axis(std::cos(principal_angle), std::sin(principal_angle));
    const Eigen::Vector2d minor_axis(-major_axis.y(), major_axis.x());

    const auto bounds = [&projected](const Eigen::Vector2d &axis) {
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = -std::numeric_limits<double>::infinity();
        for (const Eigen::Vector2d &point : projected) {
          const double projection = point.dot(axis);
          minimum = std::min(minimum, projection);
          maximum = std::max(maximum, projection);
        }
        return std::pair<double, double>{minimum, maximum};
      };
    const auto major_bounds = bounds(major_axis);
    const auto minor_bounds = bounds(minor_axis);
    const double major_extent = major_bounds.second - major_bounds.first +
      2.0 * config_.footprint_padding;
    const double minor_extent = minor_bounds.second - minor_bounds.first +
      2.0 * config_.footprint_padding;
    const double usable_x = config_.grasp_size_x - 2.0 * config_.footprint_margin;
    const double usable_y = config_.grasp_size_y - 2.0 * config_.footprint_margin;

    Eigen::Vector2d local_x_axis = Eigen::Vector2d::UnitX();
    double center_x = 0.0;
    double center_y = 0.0;
    if (major_extent <= usable_x && minor_extent <= usable_y) {
      local_x_axis = major_axis;
      result.local_y_axis = minor_axis;
      result.extent_x = major_extent;
      result.extent_y = minor_extent;
      center_x = 0.5 * (major_bounds.first + major_bounds.second);
      center_y = 0.5 * (minor_bounds.first + minor_bounds.second);
      result.fits = true;
    } else if (minor_extent <= usable_x && major_extent <= usable_y) {
      local_x_axis = minor_axis;
      result.local_y_axis = major_axis;
      result.extent_x = minor_extent;
      result.extent_y = major_extent;
      center_x = 0.5 * (minor_bounds.first + minor_bounds.second);
      center_y = 0.5 * (major_bounds.first + major_bounds.second);
      result.fits = true;
    } else {
      result.extent_x = major_extent;
      result.extent_y = minor_extent;
    }
    if (result.fits) {
      result.center_uv = local_x_axis * center_x + result.local_y_axis * center_y;
    }
    result.fill_ratio = (result.extent_x * result.extent_y) /
      std::max(usable_x * usable_y, 1.0e-12);
    result.valid = true;
    return result;
  }

  TopGraspSurfaceConfig config_;
};

}  // namespace grasping_system::candidate
