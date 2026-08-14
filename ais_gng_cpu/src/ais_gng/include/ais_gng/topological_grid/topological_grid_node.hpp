#pragma once

#include <ais_gng/topological_grid/topological_grid_assignment.hpp>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>

namespace fuzzrobo::topological_grid
{

class TopologicalGridNode : public rclcpp::Node
{
public:
  explicit TopologicalGridNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void mapCallback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg);
  void publishResult(const ais_gng_msgs::msg::TopologicalMap &map);

  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr voxel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string summary_topic_;
  GridSpec grid_spec_;
  bool origin_shift_half_ = false;
  int x_shift_ = 42;
  int y_shift_ = 21;
  int z_shift_ = 0;
  long offset_ = 1000000L;
  std::unordered_set<std::uint8_t> excluded_labels_;
  std::size_t minimum_observations_ = 1;
  std::size_t maximum_missed_updates_ = 0;
  std::unique_ptr<TemporalVoxelFilter> temporal_filter_;
};

}  // namespace fuzzrobo::topological_grid
