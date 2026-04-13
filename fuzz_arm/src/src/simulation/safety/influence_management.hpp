/**
 * @file influence_management.hpp
 * @brief Core logic for tracking and managing obstacle influence on GNG nodes.
 * This module is independent of simulation movement patterns.
 */

#pragma once

#include <Eigen/Dense>
#include <unordered_set>
#include <vector>

#include "gng/GrowingNeuralGas_offline2.hpp"

namespace robot_sim {
namespace simulation {

/**
 * @brief Data structure for a GNG node under obstacle influence.
 */
struct InfluenceNode {
  int node_id;
  Eigen::VectorXf position;
  float danger_level;
  float dist_th;
};

/**
 * @brief Manages the set of GNG nodes affected by dynamic obstacles.
 * This class is designed to be independent of how obstacle positions are
 * generated.
 */
class InfluenceManager {
public:
  InfluenceManager() = default;
  ~InfluenceManager() = default;

  /**
   * @brief Update the set of active nodes based on current robot and obstacle
   * state.
   */
  template <typename GNGType>
  void update(const Eigen::VectorXf &current_q, const GNGType &gng,
              float default_dist_th);

  /**
   * @brief Initialize/Re-scan for nodes influenced by the current state.
   */
  template <typename GNGType>
  void initialize(const Eigen::VectorXf &current_q, const GNGType &gng,
                  float default_dist_th);

  const std::vector<InfluenceNode> &getActiveNodes() const {
    return active_nodes_;
  }

  void clear() {
    active_nodes_.clear();
    active_node_ids_.clear();
    last_nearest_id_ = -1;
  }

private:
  std::vector<InfluenceNode> active_nodes_;
  std::unordered_set<int> active_node_ids_;
  int last_nearest_id_ = -1;
};

// --- InfluenceManager Template Implementations ---
namespace {
template <typename TAngle, typename TCoord>
float nodeDangerLevel(const GNG::NeuronNode<TAngle, TCoord> &node) {
  return node.status.is_colliding ? 1.0f : node.status.is_danger ? 0.5f : 0.0f;
}
} // namespace

#include <queue>

template <typename GNGType>
void InfluenceManager::initialize(const Eigen::VectorXf &current_q,
                                  const GNGType &gng, float default_dist_th) {
  clear();
  const auto &nodes = gng.get_nodes();
  float min_dist = std::numeric_limits<float>::max();
  int nearest_id = -1;

  for (int i = 0; i < (int)nodes.size(); ++i) {
    const auto &node = nodes[i];
    if (node.id != -1) {
      int dim = std::min((int)node.weight_angle.size(), (int)current_q.size());
      float d = (node.weight_angle.head(dim) - current_q.head(dim)).template lpNorm<Eigen::Infinity>();
      if (d < min_dist) {
        min_dist = d;
        nearest_id = i;
      }
      if (d < default_dist_th) {
        if (active_node_ids_.insert(i).second) {
          active_nodes_.push_back(
              {i, node.weight_angle, nodeDangerLevel(node), default_dist_th});
        }
      }
    }
  }
  last_nearest_id_ = nearest_id;
}

template <typename GNGType>
void InfluenceManager::update(const Eigen::VectorXf &current_q,
                              const GNGType &gng, float default_dist_th) {
  // 1. Validate the starting point
  bool entry_valid = false;
  if (last_nearest_id_ != -1) {
    const auto &last_node = gng.nodeAt(last_nearest_id_);
    if (last_node.id != -1) {
      entry_valid = true;
    }
  }

  if (!entry_valid) {
    initialize(current_q, gng, default_dist_th);
    return;
  }

  // 2. Hill Climbing using L-infinity to strictly track the robot
  int curr_id = last_nearest_id_;
  int dim_start = std::min((int)gng.nodeAt(curr_id).weight_angle.size(), (int)current_q.size());
  float curr_dist = (gng.nodeAt(curr_id).weight_angle.head(dim_start) - current_q.head(dim_start)).template lpNorm<Eigen::Infinity>();
  
  bool improved = true;
  while (improved) {
    improved = false;
    for (int nid : gng.getNeighborsAngle(curr_id)) {
      const auto &neighbor = gng.nodeAt(nid);
      if (neighbor.id == -1) continue;
      int dim_n = std::min((int)neighbor.weight_angle.size(), (int)current_q.size());
      float d = (neighbor.weight_angle.head(dim_n) - current_q.head(dim_n)).template lpNorm<Eigen::Infinity>();
      if (d < curr_dist) {
        curr_dist = d;
        curr_id = nid;
        improved = true;
      }
    }
  }
  last_nearest_id_ = curr_id;

  // 3. BFS collection using unified L-infinity distance
  active_nodes_.clear();
  active_node_ids_.clear();

  std::queue<int> q;
  q.push(last_nearest_id_);
  active_node_ids_.insert(last_nearest_id_);

  while (!q.empty()) {
    int id = q.front();
    q.pop();

    const auto &node = gng.nodeAt(id);
    if (node.id == -1) continue;
    
    int dim_b = std::min((int)node.weight_angle.size(), (int)current_q.size());
    float dist = (node.weight_angle.head(dim_b) - current_q.head(dim_b)).template lpNorm<Eigen::Infinity>();
    if (dist < default_dist_th) {
      active_nodes_.push_back(
          {id, node.weight_angle, nodeDangerLevel(node), default_dist_th});

      for (int nid : gng.getNeighborsAngle(id)) {
        if (active_node_ids_.find(nid) == active_node_ids_.end()) {
          active_node_ids_.insert(nid);
          q.push(nid);
        }
      }
    }
  }

  // Fallback if the entry point was found but BFS failed to collect anything within the threshold
  if (active_nodes_.empty() && curr_dist < default_dist_th) {
    initialize(current_q, gng, default_dist_th);
  }
}

} // namespace simulation
} // namespace robot_sim
