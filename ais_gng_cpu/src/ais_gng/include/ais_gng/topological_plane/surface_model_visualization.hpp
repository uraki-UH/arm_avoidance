#pragma once

#include "ais_gng/topological_plane/surface_model_tracking.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <set>

namespace fuzzrobo::surface_model
{
ais_gng_msgs::msg::TopologicalMap make_graph(
  const result &surfaces, const ais_gng_msgs::msg::TopologicalMap &map);
visualization_msgs::msg::MarkerArray make_markers(
  const result &surfaces, const ais_gng_msgs::msg::TopologicalMap &map,
  bool enable_labels, bool enable_patch_graph,
  std::set<std::pair<std::string, int>> &published,
  std::size_t min_display_plane_patches = 2);
std::string serialize(const result &surfaces, const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes,
  std::size_t min_display_plane_patches = 2);

class publisher
{
public:
  explicit publisher(rclcpp::Node &node);
  void update(const ais_gng_msgs::msg::TopologicalMap &map,
    const ais_gng_msgs::msg::PlaneClusterArray &planes);
private:
  rclcpp::Node &node_;
  options config_;
  retention_options retention_;
  tracker tracker_;
  bool enable_ = true;
  bool enable_labels_ = true;
  bool enable_patch_graph_ = false;
  bool enable_markers_ = true;
  std::size_t min_display_plane_patches_ = 2;
  double period_ = 0.5;
  std::chrono::steady_clock::time_point last_{};
  std::set<std::pair<std::string, int>> published_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr graph_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
};
}  // namespace fuzzrobo::surface_model
