#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/msg/topological_node.hpp>
#include <ais_gng_feature_msgs/msg/topological_node_feature_array.hpp>
#include <ais_gng_feature_msgs/msg/topological_node_feature.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

#include "core/common/manipulability_serialization.hpp"
#include "visualization/visualization_gng.hpp"

namespace robot_sim::bridge::topofuzzy {

constexpr float kDefaultEps = 1e-6f;

inline geometry_msgs::msg::Point32 toPoint32(const Eigen::Vector3f &v) {
  geometry_msgs::msg::Point32 p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

inline Eigen::Vector3f transformPoint(const Eigen::Isometry3d &tf,
                                      const Eigen::Vector3f &point) {
  return (tf * point.cast<double>()).cast<float>();
}

inline Eigen::Vector3f transformVector(const Eigen::Isometry3d &tf,
                                       const Eigen::Vector3f &vec) {
  return (tf.linear() * vec.cast<double>()).cast<float>();
}

inline uint8_t viewerLabelFromStatus(const GNG::Status &status) {
  if (status.is_colliding) {
    return 2;
  }
  if (status.is_danger) {
    return 3;
  }
  return 1;
}

inline int pathLabelSeverity(uint8_t label) {
  if (label == 2) {
    return 3;
  }
  if (label == 3) {
    return 2;
  }
  return label == 1 ? 1 : 0;
}

// A path visualization node is the aggregation of many raw path samples
// that fall in the same cluster. Picking the single worst label among them
// (old behavior) means one brief "danger" sample anywhere in a cluster
// paints the whole cluster yellow, so most clusters end up yellow on any
// realistic path. Tallying votes and taking the majority label reflects
// the cluster's actual dominant state instead.
inline void castPathLabelVote(std::array<int, 4> &votes, uint8_t label) {
  if (label < votes.size()) {
    ++votes[label];
  }
}

inline uint8_t majorityPathLabel(const std::array<int, 4> &votes,
                                 uint8_t fallback) {
  int best_index = -1;
  int best_votes = 0;
  for (int label = 0; label < static_cast<int>(votes.size()); ++label) {
    if (votes[label] <= 0) {
      continue;
    }
    if (votes[label] > best_votes ||
        (votes[label] == best_votes && best_index >= 0 &&
         pathLabelSeverity(static_cast<uint8_t>(label)) >
             pathLabelSeverity(static_cast<uint8_t>(best_index)))) {
      best_votes = votes[label];
      best_index = label;
    }
  }
  return best_index >= 0 ? static_cast<uint8_t>(best_index) : fallback;
}

inline ais_gng_msgs::msg::TopologicalMap buildVisualizationPathMessage(
    const ais_gng_msgs::msg::TopologicalMap &source_path,
    const ais_gng_msgs::msg::TopologicalMap &visualization_graph,
    const std::vector<std::int32_t> &source_to_visual,
    const robot_sim::visualization::VisualizationGngModel &visualization_model,
    const std::unordered_map<std::uint64_t, std::size_t>
        &transition_path_index) {
  ais_gng_msgs::msg::TopologicalMap out;
  out.header = visualization_graph.header;
  out.header.stamp = source_path.header.stamp;
  out.frame_number = source_path.frame_number;

  const bool source_frame_matches =
      source_path.header.frame_id.empty() ||
      source_path.header.frame_id == visualization_graph.header.frame_id;
  std::vector<std::int32_t> input_to_output(source_path.nodes.size(), -1);
  std::vector<std::int32_t> visual_to_output(visualization_graph.nodes.size(),
                                              -1);
  constexpr std::uint16_t kCurrentPoseNodeId =
      std::numeric_limits<std::uint16_t>::max();
  std::int32_t current_pose_output = -1;
  std::uint16_t next_goal_node_id = kCurrentPoseNodeId - 1;
  std::unordered_map<std::uint16_t, std::int32_t> goal_to_output;
  // Per-output-node label vote tally, kept in sync with out.nodes (index i
  // holds the votes for out.nodes[i]). Finalized into out.nodes[i].label
  // after all path samples have been folded in; see majorityPathLabel.
  std::vector<std::array<int, 4>> label_votes;

  const auto ensure_visual_node =
      [&out, &visualization_graph, &visual_to_output,
       &label_votes](std::uint32_t visual_id) {
        if (visual_id >= visualization_graph.nodes.size()) {
          return std::int32_t{-1};
        }
        auto &output_index = visual_to_output[visual_id];
        if (output_index < 0) {
          output_index = static_cast<std::int32_t>(out.nodes.size());
          out.nodes.push_back(visualization_graph.nodes[visual_id]);
          label_votes.push_back({});
        }
        return output_index;
      };

  out.nodes.reserve(source_path.nodes.size());
  for (std::size_t input_index = 0; input_index < source_path.nodes.size();
       ++input_index) {
    const auto &source_node = source_path.nodes[input_index];
    if (source_node.id == kCurrentPoseNodeId) {
      if (!source_frame_matches) {
        continue;
      }
      if (current_pose_output < 0) {
        current_pose_output = static_cast<std::int32_t>(out.nodes.size());
        out.nodes.push_back(source_node);
        label_votes.push_back({});
      }
      input_to_output[input_index] = current_pose_output;
      continue;
    }

    if (source_node.is_goal && source_frame_matches) {
      const auto goal_it = goal_to_output.find(source_node.id);
      if (goal_it != goal_to_output.end()) {
        input_to_output[input_index] = goal_it->second;
        continue;
      }
      if (next_goal_node_id <= visualization_graph.nodes.size()) {
        continue;
      }
      auto goal_node = source_node;
      goal_node.id = next_goal_node_id--;
      goal_node.is_goal = true;
      const std::int32_t output_index =
          static_cast<std::int32_t>(out.nodes.size());
      out.nodes.push_back(std::move(goal_node));
      label_votes.push_back({});
      goal_to_output.emplace(source_node.id, output_index);
      input_to_output[input_index] = output_index;
      continue;
    }

    const std::size_t source_id = source_node.id;
    if (source_id >= source_to_visual.size()) {
      continue;
    }
    const std::int32_t visual_id = source_to_visual[source_id];
    if (visual_id < 0 ||
        static_cast<std::size_t>(visual_id) >= visualization_graph.nodes.size()) {
      continue;
    }

    input_to_output[input_index] =
        ensure_visual_node(static_cast<std::uint32_t>(visual_id));
    if (input_to_output[input_index] >= 0) {
      const auto output_index =
          static_cast<std::size_t>(input_to_output[input_index]);
      auto &output_node = out.nodes[output_index];
      output_node.is_goal = output_node.is_goal || source_node.is_goal;
      castPathLabelVote(label_votes[output_index], source_node.label);
    }
  }

  std::unordered_set<std::uint64_t> seen_edges;
  const auto append_edge = [&out, &seen_edges](std::int32_t source,
                                               std::int32_t target) {
    if (source < 0 || target < 0 || source == target) {
      return;
    }
    const std::uint32_t lo =
        static_cast<std::uint32_t>(std::min(source, target));
    const std::uint32_t hi =
        static_cast<std::uint32_t>(std::max(source, target));
    const std::uint64_t edge_key =
        (static_cast<std::uint64_t>(lo) << 32U) | hi;
    if (seen_edges.insert(edge_key).second) {
      out.edges.push_back(static_cast<std::uint16_t>(source));
      out.edges.push_back(static_cast<std::uint16_t>(target));
    }
  };

  out.edges.reserve(source_path.edges.size() * 2);
  for (std::size_t edge_index = 0; edge_index + 1 < source_path.edges.size();
       edge_index += 2) {
    const std::size_t source_index = source_path.edges[edge_index];
    const std::size_t target_index = source_path.edges[edge_index + 1];
    if (source_index >= input_to_output.size() ||
        target_index >= input_to_output.size()) {
      continue;
    }
    const std::int32_t mapped_source = input_to_output[source_index];
    const std::int32_t mapped_target = input_to_output[target_index];
    if (mapped_source < 0 || mapped_target < 0) {
      continue;
    }

    const auto &source_node = source_path.nodes[source_index];
    const auto &target_node = source_path.nodes[target_index];
    if (source_node.id == kCurrentPoseNodeId ||
        target_node.id == kCurrentPoseNodeId) {
      append_edge(mapped_source, mapped_target);
      continue;
    }

    const auto transition_it = transition_path_index.find(
        robot_sim::visualization::visualizationGngSourceEdgeKey(
            source_node.id, target_node.id));
    if (transition_it == transition_path_index.end() ||
        transition_it->second >= visualization_model.transition_paths.size()) {
      append_edge(mapped_source, mapped_target);
      continue;
    }
    const auto &transition =
        visualization_model.transition_paths[transition_it->second];
    if (!transition.has_visual_connection) {
      continue;
    }
    const bool forward =
        source_node.id == transition.source_node_id &&
        target_node.id == transition.target_node_id;
    const bool reverse =
        source_node.id == transition.target_node_id &&
        target_node.id == transition.source_node_id;
    if (!forward && !reverse) {
      append_edge(mapped_source, mapped_target);
      continue;
    }

    std::int32_t previous = -1;
    const std::size_t path_begin = transition.path_offset;
    const std::size_t path_end = path_begin + transition.path_size;
    if (transition.path_size < 2 ||
        path_end > visualization_model.transition_path_nodes.size()) {
      append_edge(mapped_source, mapped_target);
      continue;
    }
    for (std::size_t path_index = 0; path_index < transition.path_size;
         ++path_index) {
      const std::size_t oriented_index =
          forward ? path_begin + path_index : path_end - 1 - path_index;
      std::int32_t current = -1;
      if (path_index == 0) {
        current = mapped_source;
      } else if (path_index + 1 == transition.path_size) {
        current = mapped_target;
      } else {
        current = ensure_visual_node(
            visualization_model.transition_path_nodes[oriented_index]);
      }
      if (current >= 0) {
        const auto current_index = static_cast<std::size_t>(current);
        castPathLabelVote(label_votes[current_index], source_node.label);
        castPathLabelVote(label_votes[current_index], target_node.label);
      }
      if (previous >= 0) {
        append_edge(previous, current);
      }
      previous = current;
    }
  }

  for (std::size_t i = 0; i < out.nodes.size(); ++i) {
    out.nodes[i].label = majorityPathLabel(label_votes[i], out.nodes[i].label);
  }
  return out;
}

template <typename GNGType>
inline ais_gng_msgs::msg::TopologicalMap buildVisualizationGngMessage(
    rclcpp::Node &node, const std::shared_ptr<GNGType> &source_gng,
    const robot_sim::visualization::VisualizationGngModel &visual_gng,
    const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
    const std::string &frame_id, const std::string &source_frame_id,
    float eps = kDefaultEps) {
  ais_gng_msgs::msg::TopologicalMap msg;
  msg.header.stamp = node.now();
  msg.header.frame_id = frame_id;
  if (!source_gng) {
    return msg;
  }

  Eigen::Isometry3d source_to_target = Eigen::Isometry3d::Identity();
  const bool need_transform = frame_id != source_frame_id;
  if (need_transform) {
    try {
      const auto transform = tf_buffer->lookupTransform(
          frame_id, source_frame_id, tf2::TimePointZero);
      source_to_target = tf2::transformToEigen(transform.transform);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          node.get_logger(), *node.get_clock(), 5000,
          "topofuzzy_bridge: visualization GNG TF lookup failed from '%s' "
          "to '%s': %s",
          source_frame_id.c_str(), frame_id.c_str(), ex.what());
      return msg;
    }
  }

  std::unordered_map<int, std::size_t> source_index_by_id;
  source_index_by_id.reserve(source_gng->getNodes().size());
  for (std::size_t index = 0; index < source_gng->getNodes().size(); ++index) {
    const auto &source = source_gng->getNodes()[index];
    if (source.id >= 0) {
      source_index_by_id.emplace(source.id, index);
    }
  }

  const std::size_t node_count = std::min<std::size_t>(
      visual_gng.nodes.size(), std::numeric_limits<uint16_t>::max());
  msg.nodes.reserve(node_count);
  for (std::size_t visual_index = 0; visual_index < node_count;
       ++visual_index) {
    const auto &visual_node = visual_gng.nodes[visual_index];
    bool has_safe_member = false;
    bool has_danger_member = false;
    Eigen::Vector3f normal_sum = Eigen::Vector3f::Zero();

    for (const int source_id : visual_node.source_node_ids) {
      const auto source_it = source_index_by_id.find(source_id);
      if (source_it == source_index_by_id.end()) {
        continue;
      }
      const auto &source = source_gng->getNodes()[source_it->second];
      if (source.status.ee_direction.allFinite()) {
        normal_sum += source.status.ee_direction;
      }
      const bool usable = source.status.active &&
                          source.status.self_collision_free &&
                          !source.status.is_colliding;
      if (!usable) {
        continue;
      }
      if (source.status.is_danger) {
        has_danger_member = true;
      } else {
        has_safe_member = true;
      }
    }

    ais_gng_msgs::msg::TopologicalNode out;
    out.id = static_cast<uint16_t>(visual_index);
    const Eigen::Vector3f position =
        need_transform
            ? transformPoint(source_to_target, visual_node.position)
            : visual_node.position;
    out.pos = toPoint32(position);

    Eigen::Vector3f normal = normal_sum.norm() > eps
                                 ? normal_sum.normalized()
                                 : Eigen::Vector3f::UnitZ();
    if (need_transform) {
      normal = transformVector(source_to_target, normal);
    }
    out.normal = toPoint32(normal.norm() > eps ? normal.normalized()
                                               : Eigen::Vector3f::UnitZ());
    out.label = has_safe_member ? 1 : (has_danger_member ? 3 : 2);
    msg.nodes.push_back(std::move(out));
  }

  msg.edges.reserve(visual_gng.edges.size() * 2);
  for (const auto &[source, target] : visual_gng.edges) {
    if (source >= node_count || target >= node_count || source == target) {
      continue;
    }
    msg.edges.push_back(static_cast<uint16_t>(source));
    msg.edges.push_back(static_cast<uint16_t>(target));
  }
  return msg;
}

template <typename GNGType>
inline ais_gng_msgs::msg::TopologicalMap buildGraphMessage(
    rclcpp::Node &node, const std::shared_ptr<GNGType> &gng,
    const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
    const std::string &frame_id, const std::string &source_frame_id,
    int edge_mode, float eps = kDefaultEps) {
  ais_gng_msgs::msg::TopologicalMap msg;
  msg.header.stamp = node.now();
  msg.header.frame_id = frame_id;

  if (!gng) {
    return msg;
  }

  Eigen::Isometry3d source_to_target = Eigen::Isometry3d::Identity();
  const bool need_transform = frame_id != source_frame_id;
  if (need_transform) {
    try {
      const auto ts = tf_buffer->lookupTransform(
          frame_id, source_frame_id, tf2::TimePointZero);
      source_to_target = tf2::transformToEigen(ts.transform);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 5000,
                           "topofuzzy_bridge: TF lookup failed from '%s' to '%s': %s",
                           source_frame_id.c_str(), frame_id.c_str(), ex.what());
      return msg;
    }
  }

  std::unordered_map<int, uint16_t> id_to_index;
  msg.nodes.reserve(gng->getNodes().size());
  for (size_t i = 0; i < gng->getNodes().size(); ++i) {
    const auto &node_data = gng->getNodes()[i];
    if (node_data.id == -1) {
      continue;
    }

    if (msg.nodes.size() >= std::numeric_limits<uint16_t>::max()) {
      RCLCPP_WARN(node.get_logger(),
                  "topofuzzy_bridge: too many nodes for uint16 indexing, truncating");
      break;
    }

    ais_gng_msgs::msg::TopologicalNode out;
    out.id = static_cast<uint16_t>(node_data.id);
    const Eigen::Vector3f transformed_pos = need_transform
                                                ? transformPoint(source_to_target,
                                                                 node_data.weight_coord)
                                                : node_data.weight_coord;
    out.pos = toPoint32(transformed_pos);

    const Eigen::Vector3f normal = (node_data.status.ee_direction.norm() > eps)
                                       ? node_data.status.ee_direction.normalized()
                                       : Eigen::Vector3f::UnitZ();
    Eigen::Vector3f transformed_normal = need_transform
                                             ? transformVector(source_to_target, normal)
                                             : normal;
    if (transformed_normal.norm() > eps) {
      transformed_normal.normalize();
    } else {
      transformed_normal = Eigen::Vector3f::UnitZ();
    }
    out.normal = toPoint32(transformed_normal);
    out.label = viewerLabelFromStatus(node_data.status);
    ais_gng_feature_msgs::msg::TopologicalNodeFeature feature;
    feature.node_id = static_cast<uint16_t>(node_data.id);
    robot_sim::common::fillManipulabilityFields(feature, node_data.status.manip_info);
    robot_sim::common::fillRotationalManipulabilityFields(feature, node_data.status.rotational_manip_info);
    robot_sim::common::fillNodeKinematicsFields(
        feature, node_data.weight_angle,
        node_data.weight_coord, node_data.status.ee_orientation);

    const uint16_t published_index = static_cast<uint16_t>(msg.nodes.size());
    id_to_index.emplace(node_data.id, published_index);
    msg.nodes.push_back(std::move(out));
  }

  const auto &gng_nodes = gng->getNodes();
  const std::vector<int> *neighbors_ptr = nullptr;
  std::unordered_set<uint64_t> seen_edges;
  for (size_t i = 0; i < gng_nodes.size(); ++i) {
    const auto &node_data = gng_nodes[i];
    if (node_data.id == -1) {
      continue;
    }

    neighbors_ptr = (edge_mode == 0)
                        ? &gng->getNeighborsAngle(static_cast<int>(i))
                        : &gng->getNeighborsCoord(static_cast<int>(i));
    const auto src_it = id_to_index.find(node_data.id);
    if (src_it == id_to_index.end()) {
      continue;
    }

    for (const int neighbor_id : *neighbors_ptr) {
      if (neighbor_id < 0) {
        continue;
      }
      const auto tgt_it = id_to_index.find(neighbor_id);
      if (tgt_it == id_to_index.end()) {
        continue;
      }
      const int lo = std::min(node_data.id, neighbor_id);
      const int hi = std::max(node_data.id, neighbor_id);
      const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(lo)) << 32) |
                           static_cast<uint32_t>(hi);
      if (!seen_edges.insert(key).second) {
        continue;
      }
      msg.edges.push_back(src_it->second);
      msg.edges.push_back(tgt_it->second);
    }
  }

  return msg;
}

template <typename GNGType>
inline ais_gng_msgs::msg::TopologicalMap buildLayerGraphMessage(
    rclcpp::Node &node, const std::shared_ptr<GNGType> &gng,
    const std::shared_ptr<tf2_ros::Buffer> &tf_buffer, int layer,
    const std::string &frame_id, const std::string &source_frame_id,
    float eps = kDefaultEps) {
  ais_gng_msgs::msg::TopologicalMap msg;
  msg.header.stamp = node.now();
  msg.header.frame_id = frame_id;

  if (!gng) {
    return msg;
  }

  Eigen::Isometry3d source_to_target = Eigen::Isometry3d::Identity();
  const bool need_transform = frame_id != source_frame_id;
  if (need_transform) {
    try {
      const auto ts = tf_buffer->lookupTransform(
          frame_id, source_frame_id, tf2::TimePointZero);
      source_to_target = tf2::transformToEigen(ts.transform);
    } catch (const tf2::TransformException &) {
      return msg;
    }
  }

  std::unordered_map<int, uint16_t> id_to_index;
  msg.nodes.reserve(gng->getNodes().size());
  for (size_t i = 0; i < gng->getNodes().size(); ++i) {
    const auto &node_data = gng->getNodes()[i];
    if (node_data.id == -1 || layer >= static_cast<int>(node_data.weight_coords.size())) {
      continue;
    }
    if (msg.nodes.size() >= std::numeric_limits<uint16_t>::max()) {
      break;
    }

    ais_gng_msgs::msg::TopologicalNode out;
    out.id = static_cast<uint16_t>(node_data.id);
    const Eigen::Vector3f transformed_pos = need_transform
                                                ? transformPoint(source_to_target,
                                                                 node_data.weight_coords[layer])
                                                : node_data.weight_coords[layer];
    out.pos = toPoint32(transformed_pos);

    const Eigen::Vector3f normal = (node_data.status.ee_direction.norm() > eps)
                                       ? node_data.status.ee_direction.normalized()
                                       : Eigen::Vector3f::UnitZ();
    Eigen::Vector3f transformed_normal = need_transform
                                             ? transformVector(source_to_target, normal)
                                             : normal;
    if (transformed_normal.norm() > eps) {
      transformed_normal.normalize();
    } else {
      transformed_normal = Eigen::Vector3f::UnitZ();
    }
    out.normal = toPoint32(transformed_normal);
    out.label = viewerLabelFromStatus(node_data.status);
    ais_gng_feature_msgs::msg::TopologicalNodeFeature feature;
    feature.node_id = static_cast<uint16_t>(node_data.id);
    robot_sim::common::fillManipulabilityFields(feature, node_data.status.manip_info);
    robot_sim::common::fillRotationalManipulabilityFields(feature, node_data.status.rotational_manip_info);
    robot_sim::common::fillNodeKinematicsFields(
        feature, node_data.weight_angle,
        node_data.weight_coords[layer], node_data.status.ee_orientation);

    const uint16_t published_index = static_cast<uint16_t>(msg.nodes.size());
    id_to_index.emplace(node_data.id, published_index);
    msg.nodes.push_back(std::move(out));
  }

  std::unordered_set<uint64_t> seen_edges;
  for (size_t i = 0; i < gng->getNodes().size(); ++i) {
    const auto &node_data = gng->getNodes()[i];
    if (node_data.id == -1 || layer >= static_cast<int>(node_data.weight_coords.size())) {
      continue;
    }
    const auto &neighbors = gng->getNeighborsCoord(static_cast<int>(i), layer);
    const auto src_it = id_to_index.find(node_data.id);
    if (src_it == id_to_index.end()) {
      continue;
    }
    for (const int neighbor_id : neighbors) {
      if (neighbor_id < 0) {
        continue;
      }
      const auto tgt_it = id_to_index.find(neighbor_id);
      if (tgt_it == id_to_index.end()) {
        continue;
      }
      const int lo = std::min(node_data.id, neighbor_id);
      const int hi = std::max(node_data.id, neighbor_id);
      const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(lo)) << 32) |
                           static_cast<uint32_t>(hi);
      if (!seen_edges.insert(key).second) {
        continue;
      }
      msg.edges.push_back(src_it->second);
      msg.edges.push_back(tgt_it->second);
    }
  }

  return msg;
}

template <typename GNGType>
inline ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray buildNodeFeatureArray(
    rclcpp::Node &node, const std::shared_ptr<GNGType> &gng,
    const std::string &frame_id = "") {
  ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray msg;
  msg.header.stamp = node.now();
  msg.header.frame_id = frame_id;

  if (!gng) {
    return msg;
  }

  msg.features.reserve(gng->getNodes().size());
  for (const auto &node_data : gng->getNodes()) {
    if (node_data.id == -1) {
      continue;
    }

    ais_gng_feature_msgs::msg::TopologicalNodeFeature feature;
    feature.node_id = static_cast<uint16_t>(node_data.id);
    robot_sim::common::fillManipulabilityFields(feature, node_data.status.manip_info);
    robot_sim::common::fillRotationalManipulabilityFields(
        feature, node_data.status.rotational_manip_info);
    robot_sim::common::fillNodeKinematicsFields(
        feature, node_data.weight_angle, node_data.weight_coord,
        node_data.status.ee_orientation);
    msg.features.push_back(std::move(feature));
  }

  return msg;
}

} // namespace robot_sim::bridge::topofuzzy
