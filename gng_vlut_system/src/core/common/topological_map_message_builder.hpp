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
