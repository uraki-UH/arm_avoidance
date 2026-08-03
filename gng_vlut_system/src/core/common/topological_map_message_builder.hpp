#pragma once

#include <algorithm>
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
