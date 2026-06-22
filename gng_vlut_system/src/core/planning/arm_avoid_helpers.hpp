#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/msg/topological_node.hpp>
#include <ais_gng_feature_msgs/msg/topological_node_feature.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "core/common/manipulability_serialization.hpp"

namespace robot_sim::planning::arm_avoid {

template <typename GNGType>
static inline Eigen::VectorXf currentJointVector(
    const std::vector<std::string> &chain_joint_names,
    const sensor_msgs::msg::JointState &latest_state) {
  Eigen::VectorXf q(static_cast<int>(chain_joint_names.size()));
  q.setZero();

  std::unordered_map<std::string, double> value_by_name;
  value_by_name.reserve(latest_state.name.size());
  for (std::size_t i = 0; i < latest_state.name.size(); ++i) {
    if (i < latest_state.position.size()) {
      value_by_name[latest_state.name[i]] = latest_state.position[i];
    }
  }

  for (std::size_t i = 0; i < chain_joint_names.size(); ++i) {
    const auto it = value_by_name.find(chain_joint_names[i]);
    q[static_cast<int>(i)] =
        (it != value_by_name.end()) ? static_cast<float>(it->second) : 0.0f;
  }
  return q;
}

template <typename GNGPtr>
static inline int findNearestNode(const GNGPtr &gng,
                                  const Eigen::VectorXf &posture) {
  if (!gng) {
    return -1;
  }

  float min_dist = std::numeric_limits<float>::infinity();
  int nearest_id = -1;
  gng->forEachActiveValid([&](int i, const auto &node) {
    const int dim = std::min(static_cast<int>(node.weight_angle.size()),
                             static_cast<int>(posture.size()));
    if (dim <= 0) {
      return;
    }
    const float d = (node.weight_angle.head(dim) - posture.head(dim)).norm();
    if (d < min_dist) {
      min_dist = d;
      nearest_id = i;
    }
  });
  return nearest_id;
}

template <typename GNGPtr>
static inline bool isSafeNode(const GNGPtr &gng, int node_id) {
  if (!gng || node_id < 0 ||
      static_cast<std::size_t>(node_id) >= gng->getMaxNodeNum()) {
    return false;
  }
  const auto &node = gng->nodeAt(node_id);
  return node.id != -1 && node.status.active && node.status.valid &&
         !node.status.is_colliding && !node.status.is_danger;
}

template <typename GNGPtr>
static inline bool hasAllSafeNeighbors(const GNGPtr &gng, int node_id) {
  if (!gng || node_id < 0 ||
      static_cast<std::size_t>(node_id) >= gng->getMaxNodeNum()) {
    return false;
  }
  for (int neighbor_id : gng->getNeighborsAngle(node_id)) {
    if (neighbor_id < 0 ||
        static_cast<std::size_t>(neighbor_id) >= gng->getMaxNodeNum()) {
      return false;
    }
    const auto &neighbor = gng->nodeAt(neighbor_id);
    if (neighbor.id == -1 || !neighbor.status.active || !neighbor.status.valid ||
        neighbor.status.is_colliding || neighbor.status.is_danger) {
      return false;
    }
  }
  return true;
}

template <typename GNGPtr>
static inline std::optional<int> findSafeNodeBfs(const GNGPtr &gng,
                                                 int start_id,
                                                 std::vector<int> &explored_order) {
  if (!gng || start_id < 0 ||
      static_cast<std::size_t>(start_id) >= gng->getMaxNodeNum()) {
    return std::nullopt;
  }

  std::queue<int> bfs_queue;
  std::unordered_set<int> visited;
  visited.reserve(gng->getMaxNodeNum());
  bfs_queue.push(start_id);
  visited.insert(start_id);

  while (!bfs_queue.empty()) {
    const int current_id = bfs_queue.front();
    bfs_queue.pop();
    explored_order.push_back(current_id);

    if (isSafeNode(gng, current_id) && hasAllSafeNeighbors(gng, current_id)) {
      return current_id;
    }

    for (int neighbor_id : gng->getNeighborsAngle(current_id)) {
      if (neighbor_id < 0 ||
          static_cast<std::size_t>(neighbor_id) >= gng->getMaxNodeNum()) {
        continue;
      }
      if (visited.insert(neighbor_id).second) {
        bfs_queue.push(neighbor_id);
      }
    }
  }

  return std::nullopt;
}

template <typename GNGPtr>
static inline ais_gng_msgs::msg::TopologicalMap buildGraphMessage(
    rclcpp::Node &node, const GNGPtr &gng, int selected_id,
    std::vector<ais_gng_feature_msgs::msg::TopologicalNodeFeature> *node_features = nullptr) {
  ais_gng_msgs::msg::TopologicalMap msg;
  msg.header.stamp = node.now();
  msg.header.frame_id = node.get_parameter("root_link").as_string();
  msg.frame_number = 0;

  if (!gng) {
    return msg;
  }

  std::unordered_map<int, uint16_t> id_to_index;
  msg.nodes.reserve(gng->getMaxNodeNum());

  for (std::size_t i = 0; i < gng->getMaxNodeNum(); ++i) {
    const auto &n = gng->nodeAt(static_cast<int>(i));
    if (n.id == -1) {
      continue;
    }
    ais_gng_msgs::msg::TopologicalNode out;
    out.id = static_cast<uint16_t>(n.id);
    out.pos.x = n.weight_coord.x();
    out.pos.y = n.weight_coord.y();
    out.pos.z = n.weight_coord.z();
    out.normal.x = n.status.ee_direction.x();
    out.normal.y = n.status.ee_direction.y();
    out.normal.z = n.status.ee_direction.z();
    out.rho = 0.0f;
    out.label = n.status.is_colliding ? 2 : (n.status.is_danger ? 3 : 1);
    if (n.id == selected_id) {
      out.label = 1;
      if (node_features) {
        ais_gng_feature_msgs::msg::TopologicalNodeFeature feature;
        feature.node_id = static_cast<uint16_t>(n.id);
        feature.is_goal = true;
        node_features->push_back(std::move(feature));
      }
    }
    id_to_index[n.id] = static_cast<uint16_t>(msg.nodes.size());
    msg.nodes.push_back(std::move(out));
  }

  std::unordered_set<uint64_t> seen_edges;
  for (std::size_t i = 0; i < gng->getMaxNodeNum(); ++i) {
    const auto &n = gng->nodeAt(static_cast<int>(i));
    if (n.id == -1) {
      continue;
    }
    for (int neighbor_id : gng->getNeighborsAngle(static_cast<int>(i))) {
      const auto it = id_to_index.find(neighbor_id);
      if (it == id_to_index.end()) {
        continue;
      }
      const int lo = std::min(n.id, neighbor_id);
      const int hi = std::max(n.id, neighbor_id);
      const uint64_t key = (static_cast<uint64_t>(lo) << 32) |
                           static_cast<uint32_t>(hi);
      if (!seen_edges.insert(key).second) {
        continue;
      }
      msg.edges.push_back(id_to_index[n.id]);
      msg.edges.push_back(it->second);
    }
  }
  return msg;
}

} // namespace robot_sim::planning::arm_avoid
