#pragma once

#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "core_safety/spatial/ispatial_index.hpp"

#include <vector>

namespace robot_sim {
namespace status {

/**
 * @brief Updates GNG node status based on external dynamic collision counters.
 */
template <typename T_angle, typename T_coord> class GNGStatusUpdater {
public:
  GNGStatusUpdater(GNG::GrowingNeuralGas<T_angle, T_coord> *gng) : gng_(gng) {}

  /**
   * @brief Update the spatial index with current GNG node positions.
   */
  void
  update(const std::vector<GNG::NeuronNode<T_angle, T_coord>> &nodes,
         std::shared_ptr<robot_sim::analysis::ISpatialIndex> spatial_index) {
    if (!spatial_index)
      return;

    spatial_index->clear();
    for (const auto &node : nodes) {
      if (node.id != -1) {
        Eigen::Vector3d pos = node.weight_coord.template cast<double>();
        spatial_index->insert(node.id, pos);
      }
    }
    spatial_index->build();
  }

  /**
   * @brief Apply aggregated collision and danger counts to GNG nodes.
   * @param gng Pointer to the GNG object
   * @param collision_counts Aggregated collision counts per node ID
   * @param danger_counts Aggregated danger (proximity) counts per node ID
   */
  void applyCollisionCounts(GNG::GrowingNeuralGas<T_angle, T_coord> *gng,
                            const std::vector<int> &collision_counts,
                            const std::vector<int> &danger_counts) {
    if (!gng)
      return;

    gng->forEachActive([&](int i, auto &node) {
      int c_count = (node.id < (int)collision_counts.size())
                        ? collision_counts[node.id]
                        : 0;
      int d_count =
          (node.id < (int)danger_counts.size()) ? danger_counts[node.id] : 0;

      node.status.collision_count = c_count;
      node.status.is_colliding = (c_count > 0);

      node.status.danger_count = d_count;
      node.status.is_danger = (d_count > 0 && !node.status.is_colliding);
    });
  }

  void applySafetyStatus(GNG::GrowingNeuralGas<T_angle, T_coord> *gng,
                         const std::vector<float> &danger_levels,
                         const std::vector<int> &touched_indices,
                         float collision_threshold = 0.8f,
                         float danger_threshold = 0.1f) {
    if (!gng || touched_indices.empty())
      return;

    for (int nid : touched_indices) {
      if (nid < 0 || (size_t)nid >= (size_t)gng->getMaxNodeNum()) continue;
      
      auto &node = gng->nodeAt(nid);
      if (node.id == -1) continue;

      float danger = (nid < (int)danger_levels.size()) ? danger_levels[nid] : 0.0f;

      // Map continuous danger to status flags
      node.status.is_colliding = (danger >= collision_threshold);
      node.status.is_danger =
          (danger >= danger_threshold && !node.status.is_colliding);

      node.status.collision_count = node.status.is_colliding ? 1 : 0;
      node.status.danger_count = node.status.is_danger ? 1 : 0;
    }
  }

  void applyNeighborSafety(GNG::GrowingNeuralGas<T_angle, T_coord> *gng) {
    if (!gng)
      return;

    size_t num_nodes = gng->getMaxNodeNum();
    std::vector<bool> neighbor_is_threat(num_nodes, false);

    // 1. 周囲に脅威があるノードを特定
    gng->forEachActive([&](int i, const auto &node) {
      const auto &neighbors = gng->getNeighborsAngle(i);
      for (int neighbor_id : neighbors) {
        if (neighbor_id < 0 || (size_t)neighbor_id >= num_nodes)
          continue;
        const auto &nb_node = gng->nodeAt(neighbor_id);
        if (nb_node.status.is_colliding || nb_node.status.is_danger) {
          neighbor_is_threat[i] = true;
          break;
        }
      }
    });

    // 2. フラグの反映
    gng->forEachActive([&](int i, auto &node) {
      if (neighbor_is_threat[i]) {
        if (!node.status.is_colliding) {
          node.status.is_danger = true;
          node.status.danger_count = std::max(node.status.danger_count, 1);
        }
      }
    });
  }

private:
  GNG::GrowingNeuralGas<T_angle, T_coord> *gng_;
};

} // namespace status
} // namespace robot_sim
