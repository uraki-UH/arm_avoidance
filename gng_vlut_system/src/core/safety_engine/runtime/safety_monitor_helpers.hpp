#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Dense>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/msg/topological_node.hpp>
#include <ais_gng_feature_msgs/msg/topological_node_feature.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "core/common/manipulability_serialization.hpp"

namespace robot_sim::safety::monitor_helpers {

constexpr float kEps = 1e-6f;

inline geometry_msgs::msg::Point32 toPoint32(const Eigen::Vector3f &v) {
  geometry_msgs::msg::Point32 p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
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

template <typename ContextPtr>
inline ais_gng_msgs::msg::TopologicalMap buildGraphMessage(
    rclcpp::Node &node, const ContextPtr &context,
    const std::string &base_frame) {
  ais_gng_msgs::msg::TopologicalMap msg;
  msg.header.stamp = node.now();
  msg.header.frame_id = base_frame;

  if (!context || !context->gng) {
    return msg;
  }

  const auto &gng = *context->gng;
  std::unordered_map<int, uint16_t> id_to_index;
  msg.nodes.reserve(gng.getNodes().size());

  for (size_t i = 0; i < gng.getNodes().size(); ++i) {
    const auto &node_data = gng.getNodes()[i];
    if (node_data.id == -1) {
      continue;
    }

    ais_gng_msgs::msg::TopologicalNode out;
    out.id = static_cast<uint16_t>(node_data.id);
    out.pos = toPoint32(node_data.weight_coord);
    const Eigen::Vector3f normal =
        (node_data.status.ee_direction.norm() > kEps)
            ? node_data.status.ee_direction.normalized()
            : Eigen::Vector3f::UnitZ();
    out.normal = toPoint32(normal);
    out.label = viewerLabelFromStatus(node_data.status);
    ais_gng_feature_msgs::msg::TopologicalNodeFeature feature;
    feature.node_id = static_cast<uint16_t>(node_data.id);
    robot_sim::common::fillManipulabilityFields(feature, node_data.status.manip_info);
    robot_sim::common::fillNodeKinematicsFields(
        feature, node_data.weight_angle,
        node_data.weight_coord, node_data.status.ee_orientation);
    id_to_index.emplace(node_data.id, static_cast<uint16_t>(msg.nodes.size()));
    msg.nodes.push_back(std::move(out));
  }

  std::unordered_set<uint64_t> seen_edges;
  for (size_t i = 0; i < gng.getNodes().size(); ++i) {
    const auto &node_data = gng.getNodes()[i];
    if (node_data.id == -1) {
      continue;
    }
    const auto &neighbors = gng.getNeighborsCoord(static_cast<int>(i));
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

template <typename ContextPtr>
inline visualization_msgs::msg::MarkerArray buildMarkerArray(
    rclcpp::Node &node, const ContextPtr &context,
    const std::string &base_frame) {
  visualization_msgs::msg::MarkerArray markers;
  if (!context || !context->gng) {
    return markers;
  }

  int id = 0;
  for (const auto &node_data : context->gng->getNodes()) {
    if (node_data.id == -1) {
      continue;
    }

    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame;
    m.header.stamp = node.now();
    m.ns = "gng_safety";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.scale.x = m.scale.y = m.scale.z = 0.01;
    m.pose.position.x = node_data.weight_coord.x();
    m.pose.position.y = node_data.weight_coord.y();
    m.pose.position.z = node_data.weight_coord.z();
    m.pose.orientation.w = 1.0;

    const uint8_t label = viewerLabelFromStatus(node_data.status);
    if (label == 2) {
      m.color.r = 1.0;
      m.color.a = 0.5;
    } else if (label == 3) {
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.a = 0.5;
    } else {
      m.color.g = 1.0;
      m.color.a = 0.2;
    }
    markers.markers.push_back(m);

    if (node_data.weight_coords.size() > 1) {
      for (size_t layer = 1; layer < node_data.weight_coords.size(); ++layer) {
        visualization_msgs::msg::Marker layer_marker;
        layer_marker.header.frame_id = base_frame;
        layer_marker.header.stamp = node.now();
        layer_marker.ns = "gng_safety";
        layer_marker.id = node_data.id * 100 + static_cast<int>(layer);
        layer_marker.type = visualization_msgs::msg::Marker::SPHERE;
        layer_marker.action = visualization_msgs::msg::Marker::ADD;
        layer_marker.scale.x = layer_marker.scale.y = layer_marker.scale.z = 0.008;
        layer_marker.pose.position.x = node_data.weight_coords[layer].x();
        layer_marker.pose.position.y = node_data.weight_coords[layer].y();
        layer_marker.pose.position.z = node_data.weight_coords[layer].z();
        layer_marker.pose.orientation.w = 1.0;
        layer_marker.color.r = m.color.r;
        layer_marker.color.g = m.color.g;
        layer_marker.color.b = m.color.b;
        layer_marker.color.a = 0.35;
        markers.markers.push_back(layer_marker);
      }
    }
  }
  return markers;
}

} // namespace robot_sim::safety::monitor_helpers
