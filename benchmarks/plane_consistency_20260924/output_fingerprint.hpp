#pragma once

#include "ais_gng_msgs/msg/plane_cluster_array.hpp"
#include <cstdint>

// パディングを除いた全出力フィールドのビット列照合用FNV-1a。計測区間外専用。
inline std::uint64_t output_fingerprint(
  const ais_gng_msgs::msg::PlaneClusterArray &message, const bool enable_support_edges = true)
{
  std::uint64_t hash = 14695981039346656037ULL;
  const auto add = [&hash](const auto &value) {
      const auto *bytes = reinterpret_cast<const unsigned char *>(&value);
      for (std::size_t idx = 0U; idx < sizeof(value); ++idx) {
        hash = (hash ^ bytes[idx]) * 1099511628211ULL;
      }
    };
  const auto add_point = [&add](const auto &point) {
      add(point.x); add(point.y); add(point.z);
    };
  add(message.header.stamp.sec); add(message.header.stamp.nanosec);
  add(message.header.frame_id.size());
  for (const auto value : message.header.frame_id) {add(value);}
  add(message.frame_number); add(message.clusters.size());
  for (const auto &cluster : message.clusters) {
    add(cluster.id); add(cluster.source_label);
    add(cluster.node_indices.size());
    for (const auto value : cluster.node_indices) {add(value);}
    add_point(cluster.centroid); add_point(cluster.normal);
    add_point(cluster.tangent_u); add_point(cluster.tangent_v);
    for (const auto value : cluster.position_covariance) {add(value);}
    add(cluster.boundary.size());
    for (const auto &point : cluster.boundary) {add_point(point);}
    if (enable_support_edges) {
      add(cluster.support_edges.size());
      for (const auto &point : cluster.support_edges) {add_point(point);}
    }
    add(cluster.area); add(cluster.extent_u); add(cluster.extent_v);
    add(cluster.local_spacing); add(cluster.planarity); add(cluster.residual_ratio);
  }
  return hash;
}
