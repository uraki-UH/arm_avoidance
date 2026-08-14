#pragma once

#include <ais_gng/topological_grid/topological_grid_assignment.hpp>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <chrono>
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
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void pointCloudWatchdog();
  void publishResult(
    const ais_gng_msgs::msg::TopologicalMap &map,
    const GridPointCounts &input_point_counts);
  GridPointCounts buildPointCounts(const sensor_msgs::msg::PointCloud2 &msg) const;
  voxel_msgs::msg::Voxel buildVoxelMessage(
    const std_msgs::msg::Header &header,
    const std::vector<LabeledGridVoxel> &voxels) const;
  bool headersMatch(
    const std_msgs::msg::Header &map_header,
    const std_msgs::msg::Header &pointcloud_header) const;

  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr voxel_pub_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr edge_inferred_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::TimerBase::SharedPtr pointcloud_watchdog_;

  std::string input_topic_;
  std::string pointcloud_topic_;
  std::string output_topic_;
  std::string edge_inferred_topic_;
  std::string summary_topic_;
  double pointcloud_timeout_sec_ = 0.5;
  GridSpec grid_spec_;
  bool origin_shift_half_ = false;
  int x_shift_ = 42;
  int y_shift_ = 21;
  int z_shift_ = 0;
  long offset_ = 1000000L;
  VoxelizationOptions voxelization_options_;
  EdgeInferenceOptions edge_inference_options_;
  TemporalVoxelFilterConfig temporal_filter_config_;
  std::unique_ptr<TemporalVoxelFilter> temporal_filter_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr pending_map_;
  std::chrono::steady_clock::time_point pending_map_received_at_;
  GridPointCounts latest_point_counts_;
  std_msgs::msg::Header latest_pointcloud_header_;
  bool has_latest_pointcloud_ = false;
};

}  // namespace fuzzrobo::topological_grid
