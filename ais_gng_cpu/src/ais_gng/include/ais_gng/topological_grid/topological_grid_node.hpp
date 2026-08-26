#pragma once

#include <ais_gng/topological_grid/topological_grid_assignment.hpp>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <voxel_msgs/msg/voxel.hpp>

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
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
  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void pointCloudWatchdog();
  void publishResult(
    const ais_gng_msgs::msg::TopologicalMap &map,
    const GridPointCounts &input_point_counts,
    double point_count_ms);
  GridPointCounts buildPointCounts(const sensor_msgs::msg::PointCloud2 &msg) const;
  NodeLocalStructureStates buildLocalStructureStates(
    const ais_gng_msgs::msg::TopologicalMap &map,
    const GridVoxelizationResult &result,
    const NodeIdentitySet &evaluation_identities);
  GridVisibilityStates buildVisibilityStates(
    const std_msgs::msg::Header &map_header,
    const std::vector<LabeledGridVoxel> &label_voxels);
  void applyNormalDriftScores(
    const ais_gng_msgs::msg::TopologicalMap &map,
    GridVoxelizationResult &result);
  voxel_msgs::msg::Voxel buildVoxelMessage(
    const std_msgs::msg::Header &header,
    const std::vector<LabeledGridVoxel> &voxels,
    std::uint32_t revision) const;
  bool headersMatch(
    const std_msgs::msg::Header &map_header,
    const std_msgs::msg::Header &pointcloud_header) const;

  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr voxel_pub_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr isolated_voxel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr assignment_detail_pub_;
  rclcpp::TimerBase::SharedPtr pointcloud_watchdog_;

  std::string input_topic_;
  std::string pointcloud_topic_;
  std::string output_topic_;
  std::string isolated_topic_;
  std::string summary_topic_;
  std::string assignment_detail_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  double pointcloud_timeout_sec_ = 0.5;
  bool depth_visibility_enabled_ = true;
  double depth_visibility_max_sync_offset_sec_ = 0.005;
  double depth_visibility_unit_scale_ = 0.001;
  double depth_visibility_relative_tolerance_ = 0.02;
  bool depth_visibility_tf_enabled_ = true;
  bool depth_visibility_fallback_transform_enabled_ = true;
  std::string depth_visibility_fallback_map_frame_ = "base_link";
  double camera_to_map_x_ = 0.434;
  double camera_to_map_y_ = -0.693;
  double camera_to_map_z_ = 0.279;
  double camera_to_map_roll_deg_ = -103.8;
  double camera_to_map_pitch_deg_ = -28.9;
  double camera_to_map_yaw_deg_ = -3.4;
  std::size_t depth_visibility_cache_size_ = 16;
  bool normal_drift_filter_enabled_ = true;
  GridSpec grid_spec_;
  int x_shift_ = 42;
  int y_shift_ = 21;
  int z_shift_ = 0;
  long offset_ = 1000000L;
  VoxelizationOptions voxelization_options_;
  EdgeInferenceOptions edge_inference_options_;
  TriangleInferenceOptions triangle_inference_options_;
  TriangleTopologyCache triangle_topology_cache_;
  TemporalVoxelFilterConfig temporal_filter_config_;
  std::unique_ptr<TemporalVoxelFilter> temporal_filter_;
  PointActivitySchedulerConfig point_activity_config_;
  std::unique_ptr<PointActivityScheduler> point_activity_scheduler_;
  double point_activity_cell_size_ = 0.02;
  PointActivityDecision last_point_activity_decision_;
  std::size_t point_activity_processed_update_count_ = 0;
  std::size_t point_activity_skipped_update_count_ = 0;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr pending_map_;
  std::chrono::steady_clock::time_point pending_map_received_at_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> depth_image_cache_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  GridPointCounts latest_point_counts_;
  std_msgs::msg::Header latest_pointcloud_header_;
  double latest_point_count_ms_ = 0.0;
  bool has_latest_pointcloud_ = false;
  std::uint32_t voxel_revision_ = 0;
  rclcpp::Time last_map_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_temporal_filter_stamp_{0, 0, RCL_ROS_TIME};
  std::size_t last_map_node_count_ = 0;
  std::size_t history_reset_count_ = 0;
  std::string last_history_reset_reason_ = "none";
  bool has_last_map_state_ = false;
  bool has_last_temporal_filter_stamp_ = false;
  bool history_reset_on_time_regression_ = true;
  double history_reset_node_count_ratio_ = 0.5;
  struct NormalDriftState
  {
    NodeObservation observation;
    std::size_t last_seen_epoch = 0;
  };
  std::unordered_map<NodeIdentity, NormalDriftState, NodeIdentityHash> normal_drift_states_;
  std::size_t normal_drift_epoch_ = 0;
  std::unordered_map<NodeIdentity, std::vector<NodeIdentity>, NodeIdentityHash>
    previous_node_neighbors_;
  NodeObservationMap previous_node_observations_;
  struct NormalDriftStats
  {
    std::size_t observed_node_count = 0;
    std::size_t valid_normal_node_count = 0;
    std::size_t moving_node_count = 0;
    double mean_score = 0.0;
    double maximum_score = 0.0;
  };
  NormalDriftStats last_normal_drift_stats_;
  struct LocalStructureStats
  {
    std::size_t evaluated_node_count = 0;
    std::size_t static_node_count = 0;
    std::size_t moving_node_count = 0;
    std::size_t ambiguous_node_count = 0;
  };
  LocalStructureStats last_local_structure_stats_;
  struct VisibilityStats
  {
    bool depth_matched = false;
    bool transform_available = false;
    double sync_offset_ms = -1.0;
    std::size_t out_of_view_count = 0;
    std::size_t occluded_count = 0;
    std::size_t free_count = 0;
    std::size_t unknown_count = 0;
  };
  VisibilityStats last_visibility_stats_;
};

}  // namespace fuzzrobo::topological_grid
