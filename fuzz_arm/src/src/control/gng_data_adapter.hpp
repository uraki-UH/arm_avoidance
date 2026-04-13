#pragma once

#include "gng/GrowingNeuralGas_online.hpp"
#include "simulation/viz/gng_visualization_interface.hpp"

namespace simulation {

/**
 * @brief GNGデータを可視化用フォーマットに変換するアダプター
 */
class GngDataAdapter {
public:
  template <typename T_GNG>
  static void convert(const T_GNG &gng, std::vector<VisualNode> &out_nodes,
                      std::vector<VisualEdge> &out_edges,
                      const std::unordered_set<int> &influenced_ids = {},
                      const std::unordered_set<int> &path_ids = {},
                      const Eigen::Vector3d &offset = Eigen::Vector3d::Zero(),
                      bool use_coord_edges = false) {

    out_nodes.clear();
    out_edges.clear();

    // ノードの変換
    for (int i = 0; i < (int)gng.getMaxNodeNum(); ++i) {
      const auto &node = gng.nodeAt(i);
      if (node.id == -1)
        continue;
      VisualNode vnode;
      vnode.id = node.id;

      using T_coord = std::decay_t<decltype(node.weight_coord)>;
      if constexpr (T_coord::RowsAtCompileTime == 3) {
        vnode.position = node.weight_coord + offset.cast<float>();
      } else {
        vnode.position.setZero();
        for (int j = 0; j < (int)std::min(3, (int)node.weight_coord.size());
             ++j)
          vnode.position(j) = node.weight_coord(j) + (float)offset(j);
      }

      vnode.level = node.status.level;
      vnode.is_surface = node.status.is_surface;
      vnode.is_active_surface = node.status.is_active_surface;
      vnode.active = node.status.active;
      vnode.is_collision = node.status.is_colliding;
      vnode.is_hazard = node.status.is_danger;
      vnode.is_influence = influenced_ids.count(node.id) > 0;
      vnode.is_path = path_ids.count(node.id) > 0;
      vnode.color = (vnode.level == 0)
                        ? decltype(vnode.color){1.f, 1.f, 0.f, 1.f}
                        : decltype(vnode.color){1.f, 0.5f, 0.f, 1.f};
      out_nodes.push_back(vnode);
    }

    // エッジの変換
    for (const auto &vnode : out_nodes) {
      const auto &neighbors = use_coord_edges ? gng.getNeighborsCoord(vnode.id)
                                              : gng.getNeighborsAngle(vnode.id);
      for (int neighbor_id : neighbors) {
        if (vnode.id < neighbor_id) { // 重複を避ける
          VisualEdge vedge;
          vedge.node1_id = vnode.id;
          vedge.node2_id = neighbor_id;
          vedge.is_path = false;
          vedge.level = vnode.level; // 簡易的に低い方のレベルを想定
          if (use_coord_edges) {
            vedge.color = {0.0f, 1.0f, 0.5f,
                           0.4f}; // Green/Teal for coord edges
          } else {
            vedge.color = {0.6f, 0.6f, 0.6f, 0.4f}; // Grey for angle edges
          }
          out_edges.push_back(vedge);
        }
      }
    }
  }
};

} // namespace simulation
