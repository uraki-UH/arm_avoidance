#pragma once

#include <graph/gripper_volume_graph_builder.hpp>

#include <ais_gng_msgs/msg/topological_cluster.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/msg/topological_node.hpp>
#include <std_msgs/msg/header.hpp>

#include <Eigen/Geometry>

#include <cstdint>
#include <utility>

namespace grasping_system::graph
{

inline ais_gng_msgs::msg::TopologicalMap toTopologicalMap(
  const GripperVolumeGraph &volume,
  const std_msgs::msg::Header &header,
  std::uint8_t label = ais_gng_msgs::msg::TopologicalMap::DEFAULT,
  std::uint8_t semantic_label = ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT,
  bool include_cluster = true)
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.header = header;
  map.frame_number = 0;

  const auto &orientation = volume.spec.pose_in_frame.orientation;
  Eigen::Quaterniond rotation(
    orientation.w, orientation.x, orientation.y, orientation.z);
  const Eigen::Vector3d normal = rotation * Eigen::Vector3d::UnitZ();

  map.nodes.reserve(volume.graph.nodes().size());
  for (const auto &source : volume.graph.nodes()) {
    ais_gng_msgs::msg::TopologicalNode node;
    node.id = static_cast<std::uint16_t>(source.id);
    node.pos.x = static_cast<float>(source.pose_in_object.position.x);
    node.pos.y = static_cast<float>(source.pose_in_object.position.y);
    node.pos.z = static_cast<float>(source.pose_in_object.position.z);
    node.normal.x = static_cast<float>(normal.x());
    node.normal.y = static_cast<float>(normal.y());
    node.normal.z = static_cast<float>(normal.z());
    node.label = label;
    node.semantic_label = semantic_label;
    node.semantic_reliability = semantic_label == 0 ? 0.0F : 1.0F;
    node.frame = map.frame_number;
    map.nodes.push_back(std::move(node));
  }

  map.edges.reserve(volume.graph.edges().size() * 2U);
  for (const auto &source : volume.graph.edges()) {
    map.edges.push_back(static_cast<std::uint16_t>(source.from));
    map.edges.push_back(static_cast<std::uint16_t>(source.to));
  }

  if (!include_cluster) {
    return map;
  }

  ais_gng_msgs::msg::TopologicalCluster cluster;
  cluster.id = 0;
  cluster.label = label;
  cluster.label_inferred = label;
  cluster.label_reliability = 1.0F;
  cluster.semantic_label = semantic_label;
  cluster.semantic_reliability = semantic_label == 0 ? 0.0F : 1.0F;
  cluster.pos.x = static_cast<float>(volume.spec.pose_in_frame.position.x);
  cluster.pos.y = static_cast<float>(volume.spec.pose_in_frame.position.y);
  cluster.pos.z = static_cast<float>(volume.spec.pose_in_frame.position.z);
  cluster.scale.x = static_cast<float>(volume.spec.dimensions[0]);
  cluster.scale.y = static_cast<float>(volume.spec.dimensions[1]);
  cluster.scale.z = static_cast<float>(volume.spec.dimensions[2]);
  cluster.quat = volume.spec.pose_in_frame.orientation;
  cluster.frame = map.frame_number;
  cluster.match = 1.0F;
  cluster.nodes.reserve(map.nodes.size());
  for (const auto &node : map.nodes) {
    cluster.nodes.push_back(node.id);
  }
  map.clusters.push_back(std::move(cluster));
  return map;
}

}  // namespace grasping_system::graph
