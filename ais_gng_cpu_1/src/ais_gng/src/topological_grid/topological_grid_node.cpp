#include <ais_gng/topological_grid/topological_grid_node.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace
{

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::uint8_t parseLabel(const std::string &raw_token)
{
  std::string token = trim(raw_token);
  std::transform(
    token.begin(), token.end(), token.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});

  static const std::unordered_map<std::string, std::uint8_t> kLabels{
    {"DEFAULT", ais_gng_msgs::msg::TopologicalMap::DEFAULT},
    {"SAFE_TERRAIN", ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN},
    {"WALL", ais_gng_msgs::msg::TopologicalMap::WALL},
    {"UNKNOWN_OBJECT", ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT},
    {"HUMAN", ais_gng_msgs::msg::TopologicalMap::HUMAN},
    {"CAR", ais_gng_msgs::msg::TopologicalMap::CAR},
  };
  const auto named = kLabels.find(token);
  if (named != kLabels.end()) {
    return named->second;
  }

  std::size_t parsed_length = 0;
  int value = -1;
  try {
    value = std::stoi(token, &parsed_length);
  } catch (const std::exception &) {
    throw std::invalid_argument("unknown topological label: " + raw_token);
  }
  if (parsed_length != token.size() || value < 0 || value > 255) {
    throw std::invalid_argument("topological label must be in [0, 255]: " + raw_token);
  }
  return static_cast<std::uint8_t>(value);
}

std::unordered_set<std::uint8_t> parseLabels(const std::string &csv)
{
  std::unordered_set<std::uint8_t> labels;
  std::stringstream stream(csv);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (!trim(token).empty()) {
      labels.insert(parseLabel(token));
    }
  }
  return labels;
}

std::string labelSetToString(const std::unordered_set<std::uint8_t> &labels)
{
  std::vector<int> sorted_labels;
  sorted_labels.reserve(labels.size());
  for (const auto label : labels) {
    sorted_labels.push_back(static_cast<int>(label));
  }
  std::sort(sorted_labels.begin(), sorted_labels.end());

  std::ostringstream stream;
  for (std::size_t i = 0; i < sorted_labels.size(); ++i) {
    if (i > 0) {
      stream << ",";
    }
    stream << sorted_labels[i];
  }
  return stream.str();
}

fuzzrobo::topological_grid::PointSupportMode parsePointSupportMode(std::string value)
{
  value = trim(value);
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (value == "same_cell") {
    return fuzzrobo::topological_grid::PointSupportMode::SameCell;
  }
  if (value == "radius") {
    return fuzzrobo::topological_grid::PointSupportMode::Radius;
  }
  if (value == "node_input_ids") {
    return fuzzrobo::topological_grid::PointSupportMode::NodeInputIds;
  }
  if (value == "auto") {
    return fuzzrobo::topological_grid::PointSupportMode::Auto;
  }
  throw std::invalid_argument(
          "point_support_mode must be auto, radius, node_input_ids, or same_cell: " + value);
}

const char *pointSupportModeName(fuzzrobo::topological_grid::PointSupportMode mode)
{
  switch (mode) {
    case fuzzrobo::topological_grid::PointSupportMode::SameCell:
      return "same_cell";
    case fuzzrobo::topological_grid::PointSupportMode::Radius:
      return "radius";
    case fuzzrobo::topological_grid::PointSupportMode::NodeInputIds:
      return "node_input_ids";
    case fuzzrobo::topological_grid::PointSupportMode::Auto:
      return "auto";
  }
  return "unknown";
}

std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time &stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

float depthMeters(
  const sensor_msgs::msg::Image &image, int x, int y, double unit_scale)
{
  if (x < 0 || y < 0 || x >= static_cast<int>(image.width) ||
    y >= static_cast<int>(image.height))
  {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const auto *row = image.data.data() + static_cast<std::size_t>(y) * image.step;
  if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    image.encoding == sensor_msgs::image_encodings::MONO16)
  {
    std::uint16_t value{};
    std::memcpy(&value, row + static_cast<std::size_t>(x) * sizeof(value), sizeof(value));
    return value == 0U ? std::numeric_limits<float>::quiet_NaN() :
           static_cast<float>(static_cast<double>(value) * unit_scale);
  }
  if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    float value{};
    std::memcpy(&value, row + static_cast<std::size_t>(x) * sizeof(value), sizeof(value));
    return std::isfinite(value) && value > 0.0F ? value :
           std::numeric_limits<float>::quiet_NaN();
  }
  return std::numeric_limits<float>::quiet_NaN();
}

}  // namespace

namespace fuzzrobo::topological_grid
{

TopologicalGridNode::TopologicalGridNode(const rclcpp::NodeOptions &options)
: Node("topological_grid_node", options)
{
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/topological_map/merged");
  pointcloud_topic_ = this->declare_parameter<std::string>(
    "pointcloud_topic", "/downsampling/grasp_support");
  output_topic_ = this->declare_parameter<std::string>(
    "output_topic", "/topological_grid_assignments");
  isolated_topic_ = this->declare_parameter<std::string>("isolated_topic", "");
  if (isolated_topic_.empty()) {
    isolated_topic_ = output_topic_ + "/isolated";
  }
  summary_topic_ = this->declare_parameter<std::string>(
    "summary_topic", "");
  if (summary_topic_.empty()) {
    summary_topic_ = output_topic_ + "/summary";
  }
  assignment_detail_topic_ = output_topic_ + "/assignments";
  depth_visibility_enabled_ = this->declare_parameter<bool>(
    "depth_visibility_enabled", true);
  depth_topic_ = this->declare_parameter<std::string>(
    "depth_topic", "/camera/camera/depth/image_rect_raw");
  camera_info_topic_ = this->declare_parameter<std::string>(
    "camera_info_topic", "/camera/camera/depth/camera_info");
  depth_visibility_max_sync_offset_sec_ = this->declare_parameter<double>(
    "depth_visibility_max_sync_offset_sec", 0.005);
  depth_visibility_unit_scale_ = this->declare_parameter<double>(
    "depth_visibility_unit_scale", 0.001);
  depth_visibility_relative_tolerance_ = this->declare_parameter<double>(
    "depth_visibility_relative_tolerance", 0.02);
  depth_visibility_tf_enabled_ = this->declare_parameter<bool>(
    "depth_visibility_tf_enabled", true);
  depth_visibility_fallback_transform_enabled_ = this->declare_parameter<bool>(
    "depth_visibility_fallback_transform_enabled", true);
  depth_visibility_fallback_map_frame_ = this->declare_parameter<std::string>(
    "depth_visibility_fallback_map_frame", "base_link");
  camera_to_map_x_ = this->declare_parameter<double>("camera_to_map.x", 0.434);
  camera_to_map_y_ = this->declare_parameter<double>("camera_to_map.y", -0.693);
  camera_to_map_z_ = this->declare_parameter<double>("camera_to_map.z", 0.279);
  camera_to_map_roll_deg_ = this->declare_parameter<double>(
    "camera_to_map.roll_deg", -103.8);
  camera_to_map_pitch_deg_ = this->declare_parameter<double>(
    "camera_to_map.pitch_deg", -28.9);
  camera_to_map_yaw_deg_ = this->declare_parameter<double>(
    "camera_to_map.yaw_deg", -3.4);
  const int depth_visibility_cache_size = this->declare_parameter<int>(
    "depth_visibility_cache_size", 16);
  normal_drift_filter_enabled_ = this->declare_parameter<bool>(
    "normal_drift_filter_enabled", true);
  pointcloud_timeout_sec_ = this->declare_parameter<double>("pointcloud_timeout_sec", 0.5);
  grid_spec_.cell_size = this->declare_parameter<double>("grid_size", 0.01);
  grid_spec_.origin_x = this->declare_parameter<double>("origin_x", 0.0);
  grid_spec_.origin_y = this->declare_parameter<double>("origin_y", 0.0);
  grid_spec_.origin_z = this->declare_parameter<double>("origin_z", 0.0);
  x_shift_ = this->declare_parameter<int>("x_shift", 42);
  y_shift_ = this->declare_parameter<int>("y_shift", 21);
  z_shift_ = this->declare_parameter<int>("z_shift", 0);
  offset_ = this->declare_parameter<long>("offset", 1000000L);
  voxelization_options_.excluded_labels = parseLabels(this->declare_parameter<std::string>(
    "excluded_labels", "SAFE_TERRAIN,HUMAN,CAR"));
  voxelization_options_.require_input_points = this->declare_parameter<bool>(
    "require_input_points", true);
  voxelization_options_.point_support_mode = parsePointSupportMode(
    this->declare_parameter<std::string>("point_support_mode", "auto"));
  voxelization_options_.point_support_radius_m = this->declare_parameter<double>(
    "point_support_radius_m", 0.02);
  voxelization_options_.unknown_shape_filter_enabled = this->declare_parameter<bool>(
    "unknown_shape_filter_enabled", false);
  voxelization_options_.shape_neighborhood_hops = this->declare_parameter<int>(
    "shape_neighborhood_hops", 2);
  const int shape_minimum_neighbors = this->declare_parameter<int>(
    "shape_minimum_neighbors", 3);
  voxelization_options_.shape_residual_weight = this->declare_parameter<double>(
    "shape_residual_weight", 0.7);
  voxelization_options_.shape_mad_multiplier = this->declare_parameter<double>(
    "shape_mad_multiplier", 3.0);
  voxelization_options_.shape_seed_expansion_scale = this->declare_parameter<double>(
    "shape_seed_expansion_scale", 2.0);
  edge_inference_options_.enabled = this->declare_parameter<bool>(
    "edge_inference_enabled", true);
  edge_inference_options_.maximum_edge_length = this->declare_parameter<double>(
    "edge_max_length", 0.0);
  edge_inference_options_.require_point_support_for_output = this->declare_parameter<bool>(
    "edge_inferred_require_input_points", false);
  triangle_inference_options_.enabled = this->declare_parameter<bool>(
    "triangle_inference_enabled", true);
  triangle_inference_options_.maximum_edge_length = this->declare_parameter<double>(
    "triangle_max_edge_length", 0.05);
  triangle_inference_options_.minimum_area = this->declare_parameter<double>(
    "triangle_min_area", 1.0e-6);
  triangle_inference_options_.minimum_aspect_ratio = this->declare_parameter<double>(
    "triangle_min_aspect_ratio", 0.05);
  triangle_inference_options_.maximum_normal_angle_degrees = this->declare_parameter<double>(
    "triangle_max_normal_angle_deg", 45.0);
  triangle_inference_options_.minimum_point_support_ratio = this->declare_parameter<double>(
    "triangle_min_point_support_ratio", 0.0);
  triangle_inference_options_.require_point_support_for_output = this->declare_parameter<bool>(
    "triangle_inferred_require_input_points", true);
  const int minimum_input_points_per_voxel = this->declare_parameter<int>(
    "minimum_input_points_per_voxel", 1);
  const int neighbor_radius_cells = this->declare_parameter<int>(
    "neighbor_radius_cells", 1);
  voxelization_options_.neighbor_radius_m = this->declare_parameter<double>(
    "neighbor_radius_m", 0.02);
  const int temporal_history_window_size = this->declare_parameter<int>(
    "temporal_history_window_size", 32);
  temporal_filter_config_.time_constant_sec = this->declare_parameter<double>(
    "temporal_time_constant_sec", 0.30);
  temporal_filter_config_.activation_score = this->declare_parameter<double>(
    "temporal_activation_score", 0.65);
  temporal_filter_config_.retention_score = this->declare_parameter<double>(
    "temporal_retention_score", 0.35);
  temporal_filter_config_.node_identity_retention_enabled = this->declare_parameter<bool>(
    "node_identity_retention_enabled", false);
  temporal_filter_config_.node_identity_max_displacement = this->declare_parameter<double>(
    "node_identity_max_displacement", 0.02);
  temporal_filter_config_.node_identity_history_migration_enabled =
    this->declare_parameter<bool>("node_identity_history_migration_enabled", false);
  history_reset_on_time_regression_ = this->declare_parameter<bool>(
    "history_reset_on_time_regression", false);
  history_reset_node_count_ratio_ = this->declare_parameter<double>(
    "history_reset_node_count_ratio", 0.5);
  point_activity_config_.enabled = this->declare_parameter<bool>(
    "point_activity_update_enabled", true);
  point_activity_cell_size_ = this->declare_parameter<double>(
    "point_activity_cell_size", 0.02);
  point_activity_config_.ema_alpha = this->declare_parameter<double>(
    "point_activity_ema_alpha", 0.2);
  point_activity_config_.top_fraction = this->declare_parameter<double>(
    "point_activity_top_fraction", 0.1);
  point_activity_config_.occupancy_change_weight = this->declare_parameter<double>(
    "point_activity_occupancy_weight", 0.7);
  const int point_activity_warmup_updates = this->declare_parameter<int>(
    "point_activity_warmup_updates", 5);
  const int point_activity_minimum_update_interval = this->declare_parameter<int>(
    "point_activity_minimum_update_interval", 1);
  const int point_activity_maximum_update_interval = this->declare_parameter<int>(
    "point_activity_maximum_update_interval", 10);

  if (grid_spec_.cell_size <= 0.0 || pointcloud_timeout_sec_ <= 0.0 ||
    edge_inference_options_.maximum_edge_length < 0.0 ||
    triangle_inference_options_.maximum_edge_length <= 0.0 ||
    triangle_inference_options_.minimum_area < 0.0 ||
    triangle_inference_options_.minimum_aspect_ratio < 0.0 ||
    triangle_inference_options_.minimum_aspect_ratio > 1.0 ||
    triangle_inference_options_.maximum_normal_angle_degrees < 0.0 ||
    triangle_inference_options_.maximum_normal_angle_degrees > 180.0 ||
    triangle_inference_options_.minimum_point_support_ratio < 0.0 ||
    triangle_inference_options_.minimum_point_support_ratio > 1.0 ||
    temporal_filter_config_.time_constant_sec <= 0.0 ||
    temporal_filter_config_.activation_score <= 0.0 ||
    temporal_filter_config_.activation_score > 1.0 ||
    temporal_filter_config_.retention_score < 0.0 ||
    temporal_filter_config_.retention_score > temporal_filter_config_.activation_score ||
    temporal_filter_config_.node_identity_max_displacement < 0.0 ||
    depth_visibility_max_sync_offset_sec_ < 0.0 ||
    depth_visibility_unit_scale_ <= 0.0 ||
    depth_visibility_relative_tolerance_ < 0.0 ||
    voxelization_options_.neighbor_radius_m < 0.0 ||
    voxelization_options_.point_support_radius_m < 0.0 ||
    history_reset_node_count_ratio_ < 0.0 || history_reset_node_count_ratio_ > 1.0 ||
    point_activity_cell_size_ <= 0.0 || point_activity_config_.ema_alpha <= 0.0 ||
    point_activity_config_.ema_alpha > 1.0 || point_activity_config_.top_fraction <= 0.0 ||
    point_activity_config_.top_fraction > 1.0 ||
    point_activity_config_.occupancy_change_weight < 0.0 ||
    point_activity_config_.occupancy_change_weight > 1.0 ||
    voxelization_options_.shape_residual_weight < 0.0 ||
    voxelization_options_.shape_residual_weight > 1.0 ||
    voxelization_options_.shape_mad_multiplier < 0.0 ||
    voxelization_options_.shape_seed_expansion_scale < 0.0)
  {
    throw std::invalid_argument(
            "grid, inference, point-support, and node-identity parameters are invalid");
  }
  if (minimum_input_points_per_voxel <= 0 || neighbor_radius_cells < 0 ||
    temporal_history_window_size <= 0 ||
    depth_visibility_cache_size <= 0 ||
    point_activity_warmup_updates < 0 ||
    voxelization_options_.shape_neighborhood_hops <= 0 || shape_minimum_neighbors <= 0 ||
    point_activity_minimum_update_interval <= 0 ||
    point_activity_maximum_update_interval < point_activity_minimum_update_interval)
  {
    throw std::invalid_argument(
            "point, temporal history, and activity interval counts must be valid");
  }
  voxelization_options_.minimum_input_points_per_voxel =
    static_cast<std::size_t>(minimum_input_points_per_voxel);
  voxelization_options_.neighbor_radius_cells = neighbor_radius_cells;
  voxelization_options_.shape_minimum_neighbors =
    static_cast<std::size_t>(shape_minimum_neighbors);
  temporal_filter_config_.history_window_size =
    static_cast<std::size_t>(temporal_history_window_size);
  depth_visibility_cache_size_ = static_cast<std::size_t>(depth_visibility_cache_size);
  temporal_filter_ = std::make_unique<TemporalVoxelFilter>(temporal_filter_config_);
  point_activity_config_.minimum_update_interval =
    static_cast<std::size_t>(point_activity_minimum_update_interval);
  point_activity_config_.warmup_update_count =
    static_cast<std::size_t>(point_activity_warmup_updates);
  point_activity_config_.maximum_update_interval =
    static_cast<std::size_t>(point_activity_maximum_update_interval);
  point_activity_scheduler_ = std::make_unique<PointActivityScheduler>(point_activity_config_);

  voxel_pub_ = this->create_publisher<voxel_msgs::msg::Voxel>(
    output_topic_,
    rclcpp::QoS(1).reliable().transient_local());
  isolated_voxel_pub_ = this->create_publisher<voxel_msgs::msg::Voxel>(
    isolated_topic_,
    rclcpp::QoS(1).reliable().transient_local());
  summary_pub_ = this->create_publisher<std_msgs::msg::String>(summary_topic_, 10);
  // Per-cell JSON is useful for debugging, but constructing it for thousands
  // of cells every update is much more expensive than the Voxel message.
  // Keep it on a separate best-effort topic and build it only on demand.
  assignment_detail_pub_ = this->create_publisher<std_msgs::msg::String>(
    assignment_detail_topic_, rclcpp::QoS(1).best_effort());
  if (depth_visibility_enabled_ && depth_visibility_tf_enabled_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  map_sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
    input_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&TopologicalGridNode::mapCallback, this, std::placeholders::_1));
  if (voxelization_options_.require_input_points) {
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TopologicalGridNode::pointCloudCallback, this, std::placeholders::_1));
    pointcloud_watchdog_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&TopologicalGridNode::pointCloudWatchdog, this));
  }
  if (depth_visibility_enabled_) {
    depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS().keep_last(depth_visibility_cache_size_),
      std::bind(&TopologicalGridNode::depthImageCallback, this, std::placeholders::_1));
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TopologicalGridNode::cameraInfoCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(
    this->get_logger(),
    "TopologicalGridNode ready: input=%s pointcloud=%s timeout=%.2fs output=%s "
    "isolated=%s edge_fill=%s/%s grid_size=%.3f "
    "origin=(%.3f, %.3f, %.3f) "
    "excluded=[%s] point_support=%s/%s/%zu neighbors=%d/%.3fm history=%zu "
    "stability=tau:%.3fs activate:%.2f retain:%.2f "
    "node_identity=%s/%.3fm migration=%s triangle=%s/%.3fm "
    "point_activity=%s/%.3fm/warmup=%zu/interval=%zu-%zu "
    "unknown_shape=%s/hops=%d/mad=%.2f/expand=%.2f",
    input_topic_.c_str(),
    pointcloud_topic_.c_str(),
    pointcloud_timeout_sec_,
    output_topic_.c_str(),
    isolated_topic_.c_str(),
    edge_inference_options_.enabled ? "enabled" : "disabled",
    edge_inference_options_.maximum_edge_length > 0.0 ? "metric" : "adaptive",
    grid_spec_.cell_size,
    grid_spec_.origin_x,
    grid_spec_.origin_y,
    grid_spec_.origin_z,
    labelSetToString(voxelization_options_.excluded_labels).c_str(),
    voxelization_options_.require_input_points ? "required" : "disabled",
    pointSupportModeName(voxelization_options_.point_support_mode),
    voxelization_options_.minimum_input_points_per_voxel,
    voxelization_options_.neighbor_radius_cells,
    voxelization_options_.neighbor_radius_m,
    temporal_filter_config_.history_window_size,
    temporal_filter_config_.time_constant_sec,
    temporal_filter_config_.activation_score,
    temporal_filter_config_.retention_score,
    temporal_filter_config_.node_identity_retention_enabled ? "enabled" : "disabled",
    temporal_filter_config_.node_identity_max_displacement,
    temporal_filter_config_.node_identity_history_migration_enabled ? "enabled" : "disabled",
    triangle_inference_options_.enabled ? "enabled" : "disabled",
    triangle_inference_options_.maximum_edge_length,
    point_activity_config_.enabled ? "enabled" : "disabled",
    point_activity_cell_size_,
    point_activity_config_.warmup_update_count,
    point_activity_config_.minimum_update_interval,
    point_activity_config_.maximum_update_interval,
    voxelization_options_.unknown_shape_filter_enabled ? "enabled" : "disabled",
    voxelization_options_.shape_neighborhood_hops,
    voxelization_options_.shape_mad_multiplier,
    voxelization_options_.shape_seed_expansion_scale);
  RCLCPP_INFO(
    this->get_logger(),
    "Depth visibility: %s depth=%s camera_info=%s sync=%.1fms tolerance=%.3f tf=%s fallback=%s",
    depth_visibility_enabled_ ? "enabled" : "disabled",
    depth_topic_.c_str(), camera_info_topic_.c_str(),
    depth_visibility_max_sync_offset_sec_ * 1000.0,
    depth_visibility_relative_tolerance_,
    depth_visibility_tf_enabled_ ? "enabled" : "disabled",
    depth_visibility_fallback_transform_enabled_ ? "enabled" : "disabled");
  RCLCPP_INFO(
    this->get_logger(),
    "Normal drift filter: %s (GNG normal displacement / local edge spacing)",
    normal_drift_filter_enabled_ ? "enabled" : "disabled");
}

void TopologicalGridNode::mapCallback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (!voxelization_options_.require_input_points) {
    publishResult(*msg, GridPointCounts{}, 0.0);
    return;
  }

  if (has_latest_pointcloud_ && headersMatch(msg->header, latest_pointcloud_header_)) {
    publishResult(*msg, latest_point_counts_, latest_point_count_ms_);
    has_latest_pointcloud_ = false;
    latest_point_counts_.clear();
    latest_point_count_ms_ = 0.0;
    return;
  }

  if (!pending_map_) {
    pending_map_received_at_ = std::chrono::steady_clock::now();
  }
  pending_map_ = msg;
}

void TopologicalGridNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const auto point_count_started = std::chrono::steady_clock::now();
  auto point_counts = buildPointCounts(*msg);
  const double point_count_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - point_count_started).count();
  if (pending_map_ && headersMatch(pending_map_->header, msg->header)) {
    publishResult(*pending_map_, point_counts, point_count_ms);
    pending_map_.reset();
    return;
  }

  latest_point_counts_ = std::move(point_counts);
  latest_pointcloud_header_ = msg->header;
  latest_point_count_ms_ = point_count_ms;
  has_latest_pointcloud_ = true;
}

void TopologicalGridNode::depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  depth_image_cache_.push_back(msg);
  while (depth_image_cache_.size() > depth_visibility_cache_size_) {
    depth_image_cache_.pop_front();
  }
}

void TopologicalGridNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  latest_camera_info_ = msg;
}

GridVisibilityStates TopologicalGridNode::buildVisibilityStates(
  const std_msgs::msg::Header &map_header,
  const std::vector<LabeledGridVoxel> &label_voxels)
{
  last_visibility_stats_ = VisibilityStats{};
  GridVisibilityStates states;
  if (!depth_visibility_enabled_ || !latest_camera_info_ || depth_image_cache_.empty()) {
    return states;
  }

  const std::int64_t map_stamp_ns = stampNanoseconds(map_header.stamp);
  const std::int64_t max_offset_ns = static_cast<std::int64_t>(
    depth_visibility_max_sync_offset_sec_ * 1000000000.0);
  sensor_msgs::msg::Image::SharedPtr depth_image;
  std::int64_t best_offset_ns = std::numeric_limits<std::int64_t>::max();
  for (const auto &candidate : depth_image_cache_) {
    const auto offset_ns = std::llabs(stampNanoseconds(candidate->header.stamp) - map_stamp_ns);
    if (offset_ns <= max_offset_ns && offset_ns < best_offset_ns) {
      depth_image = candidate;
      best_offset_ns = offset_ns;
    }
  }
  if (!depth_image) {
    return states;
  }
  last_visibility_stats_.depth_matched = true;
  last_visibility_stats_.sync_offset_ms = static_cast<double>(best_offset_ns) / 1000000.0;
  if (depth_image->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
    depth_image->encoding != sensor_msgs::image_encodings::MONO16 &&
    depth_image->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Depth visibility disabled for frame: unsupported encoding '%s'",
      depth_image->encoding.c_str());
    return states;
  }

  const double fx = latest_camera_info_->k[0];
  const double fy = latest_camera_info_->k[4];
  const double cx = latest_camera_info_->k[2];
  const double cy = latest_camera_info_->k[5];
  if (fx <= 0.0 || fy <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Depth visibility disabled for frame: CameraInfo has invalid focal length");
    return states;
  }

  tf2::Transform map_to_depth;
  bool have_transform = map_header.frame_id == depth_image->header.frame_id;
  if (have_transform) {
    map_to_depth.setIdentity();
  } else if (depth_visibility_tf_enabled_ && tf_buffer_) {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        depth_image->header.frame_id, map_header.frame_id,
        rclcpp::Time(map_header.stamp, RCL_ROS_TIME));
      tf2::fromMsg(transform.transform, map_to_depth);
      have_transform = true;
    } catch (const tf2::TransformException &) {
      have_transform = false;
    }
  }
  if (!have_transform && depth_visibility_fallback_transform_enabled_ &&
    map_header.frame_id == depth_visibility_fallback_map_frame_)
  {
    constexpr double kDegToRad = 0.01745329251994329577;
    tf2::Quaternion camera_to_map_rotation;
    camera_to_map_rotation.setRPY(
      camera_to_map_roll_deg_ * kDegToRad,
      camera_to_map_pitch_deg_ * kDegToRad,
      camera_to_map_yaw_deg_ * kDegToRad);
    const tf2::Transform camera_to_map(
      camera_to_map_rotation,
      tf2::Vector3(camera_to_map_x_, camera_to_map_y_, camera_to_map_z_));
    map_to_depth = camera_to_map.inverse();
    have_transform = true;
  }
  if (!have_transform) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Depth visibility skipped: no transform %s -> %s",
      map_header.frame_id.c_str(), depth_image->header.frame_id.c_str());
    return states;
  }
  last_visibility_stats_.transform_available = true;

  std::unordered_set<GridCell, GridCellHash> cells;
  cells.reserve(label_voxels.size() + temporal_filter_->trackedVoxelCount());
  for (const auto &voxel : label_voxels) {
    cells.insert(voxel.cell);
  }
  for (const auto &cell : temporal_filter_->trackedVoxels()) {
    cells.insert(cell);
  }
  states.reserve(cells.size());
  for (const auto &cell : cells) {
    const tf2::Vector3 map_point(
      grid_spec_.origin_x + (static_cast<double>(cell.x) + 0.5) * grid_spec_.cell_size,
      grid_spec_.origin_y + (static_cast<double>(cell.y) + 0.5) * grid_spec_.cell_size,
      grid_spec_.origin_z + (static_cast<double>(cell.z) + 0.5) * grid_spec_.cell_size);
    const tf2::Vector3 camera_point = map_to_depth * map_point;
    VoxelVisibilityState state = VoxelVisibilityState::Unknown;
    if (camera_point.z() <= 1.0e-6) {
      state = VoxelVisibilityState::OutOfView;
    } else {
      const int pixel_x = static_cast<int>(std::lround(
        fx * camera_point.x() / camera_point.z() + cx));
      const int pixel_y = static_cast<int>(std::lround(
        fy * camera_point.y() / camera_point.z() + cy));
      if (pixel_x < 1 || pixel_x >= static_cast<int>(depth_image->width) - 1 ||
        pixel_y < 1 || pixel_y >= static_cast<int>(depth_image->height) - 1)
      {
        state = VoxelVisibilityState::OutOfView;
      } else {
        std::array<float, 9> samples{};
        std::size_t sample_count = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const float sample = depthMeters(
              *depth_image, pixel_x + dx, pixel_y + dy, depth_visibility_unit_scale_);
            if (std::isfinite(sample)) {
              samples[sample_count++] = sample;
            }
          }
        }
        if (sample_count > 0) {
          std::sort(samples.begin(), samples.begin() + sample_count);
          const float median_depth = samples[sample_count / 2];
          const double near_limit = camera_point.z() *
            (1.0 - depth_visibility_relative_tolerance_);
          const double far_limit = camera_point.z() *
            (1.0 + depth_visibility_relative_tolerance_);
          if (median_depth < near_limit) {
            state = VoxelVisibilityState::Occluded;
          } else if (median_depth > far_limit) {
            state = VoxelVisibilityState::Free;
          }
        }
      }
    }
    states.emplace(cell, state);
    switch (state) {
      case VoxelVisibilityState::OutOfView:
        ++last_visibility_stats_.out_of_view_count;
        break;
      case VoxelVisibilityState::Occluded:
        ++last_visibility_stats_.occluded_count;
        break;
      case VoxelVisibilityState::Free:
        ++last_visibility_stats_.free_count;
        break;
      case VoxelVisibilityState::Unknown:
        ++last_visibility_stats_.unknown_count;
        break;
    }
  }
  return states;
}

void TopologicalGridNode::applyNormalDriftScores(
  const ais_gng_msgs::msg::TopologicalMap &map,
  GridVoxelizationResult &result)
{
  last_normal_drift_stats_ = NormalDriftStats{};
  if (!normal_drift_filter_enabled_ || result.eligible_nodes.empty()) {
    return;
  }

  ++normal_drift_epoch_;
  const std::size_t node_count = map.nodes.size();
  std::vector<double> incident_length_sum(node_count, 0.0);
  std::vector<std::size_t> incident_edge_count(node_count, 0U);
  for (std::size_t edge = 0; edge + 1U < map.edges.size(); edge += 2U) {
    const std::size_t first = map.edges[edge];
    const std::size_t second = map.edges[edge + 1U];
    if (first >= node_count || second >= node_count || first == second) {
      continue;
    }
    const auto &a = map.nodes[first].pos;
    const auto &b = map.nodes[second].pos;
    const double dx = static_cast<double>(a.x) - b.x;
    const double dy = static_cast<double>(a.y) - b.y;
    const double dz = static_cast<double>(a.z) - b.z;
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!std::isfinite(length) || length <= 1.0e-9) {
      continue;
    }
    incident_length_sum[first] += length;
    incident_length_sum[second] += length;
    ++incident_edge_count[first];
    ++incident_edge_count[second];
  }

  std::unordered_map<NodeIdentity, std::size_t, NodeIdentityHash> node_indices;
  std::unordered_set<NodeIdentity, NodeIdentityHash> ambiguous_identities;
  node_indices.reserve(node_count);
  for (std::size_t index = 0; index < node_count; ++index) {
    const NodeIdentity identity{map.nodes[index].id, map.nodes[index].frame};
    if (ambiguous_identities.find(identity) != ambiguous_identities.end()) {
      continue;
    }
    const auto [existing, inserted] = node_indices.emplace(identity, index);
    if (!inserted) {
      node_indices.erase(existing);
      ambiguous_identities.insert(identity);
    }
  }

  std::unordered_map<NodeIdentity, double, NodeIdentityHash> score_by_identity;
  score_by_identity.reserve(result.eligible_nodes.size());
  double score_sum = 0.0;
  for (const auto &[identity, observation] : result.eligible_nodes) {
    ++last_normal_drift_stats_.observed_node_count;
    double score = 0.0;
    const auto node_index = node_indices.find(identity);
    if (node_index != node_indices.end() &&
      incident_edge_count[node_index->second] > 0U)
    {
      const double spacing = incident_length_sum[node_index->second] /
        static_cast<double>(incident_edge_count[node_index->second]);
      const double normal_norm = std::sqrt(
        observation.normal_x * observation.normal_x +
        observation.normal_y * observation.normal_y +
        observation.normal_z * observation.normal_z);
      if (std::isfinite(spacing) && spacing > 1.0e-9 &&
        std::isfinite(normal_norm) && normal_norm > 1.0e-9)
      {
        ++last_normal_drift_stats_.valid_normal_node_count;
        const auto previous = normal_drift_states_.find(identity);
        if (previous != normal_drift_states_.end() &&
          previous->second.last_seen_epoch + 1U == normal_drift_epoch_)
        {
          const double dx = observation.x - previous->second.observation.x;
          const double dy = observation.y - previous->second.observation.y;
          const double dz = observation.z - previous->second.observation.z;
          const double normal_displacement = std::abs(
            dx * observation.normal_x + dy * observation.normal_y + dz * observation.normal_z) /
            normal_norm;
          score = std::clamp(normal_displacement / spacing, 0.0, 1.0);
        }
      }
    }
    normal_drift_states_.insert_or_assign(
      identity, NormalDriftState{observation, normal_drift_epoch_});
    score_by_identity.emplace(identity, score);
    score_sum += score;
    last_normal_drift_stats_.maximum_score = std::max(
      last_normal_drift_stats_.maximum_score, score);
    if (score > 0.0) {
      ++last_normal_drift_stats_.moving_node_count;
    }
  }
  last_normal_drift_stats_.mean_score = result.eligible_nodes.empty() ? 0.0 :
    score_sum / static_cast<double>(result.eligible_nodes.size());

  for (auto &voxel : result.label_voxels) {
    if (voxel.node_observations.empty()) {
      continue;
    }
    double voxel_score = 0.0;
    for (const auto &observation : voxel.node_observations) {
      const auto score = score_by_identity.find(observation.identity);
      if (score != score_by_identity.end()) {
        voxel_score += score->second;
      }
    }
    voxel.normal_drift_score = std::clamp(
      voxel_score / static_cast<double>(voxel.node_observations.size()), 0.0, 1.0);
  }

  const std::size_t cleanup_limit = std::max<std::size_t>(
    64U, result.eligible_nodes.size() * 2U);
  if (normal_drift_states_.size() > cleanup_limit) {
    for (auto it = normal_drift_states_.begin(); it != normal_drift_states_.end();) {
      if (it->second.last_seen_epoch != normal_drift_epoch_) {
        it = normal_drift_states_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

NodeLocalStructureStates TopologicalGridNode::buildLocalStructureStates(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridVoxelizationResult &result,
  const NodeIdentitySet &evaluation_identities)
{
  // Both thresholds are normalized by the former GNG neighbour spacing, not
  // by grid_size or a metric distance.  A cluster must have moved by at least
  // half its local spacing, with much smaller disagreement between neighbours,
  // before its former cells and current cells are treated as a moving-object
  // trail.  Coherent translation is used as current-cell negative evidence.
  constexpr std::size_t kMinimumSharedNeighbors = 2U;
  constexpr double kStaticShiftRatio = 0.25;
  constexpr double kCoherentDisagreementRatio = 0.25;

  last_local_structure_stats_ = LocalStructureStats{};
  NodeLocalStructureStates states;
  std::unordered_map<NodeIdentity, std::vector<NodeIdentity>, NodeIdentityHash>
    current_neighbors;
  NodeIdentitySet tracked_identities = evaluation_identities;
  tracked_identities.reserve(
    tracked_identities.size() + previous_node_neighbors_.size());
  for (const auto &[identity, neighbors] : previous_node_neighbors_) {
    static_cast<void>(neighbors);
    tracked_identities.insert(identity);
  }
  current_neighbors.reserve(tracked_identities.size());

  for (std::size_t edge = 0; edge + 1U < map.edges.size(); edge += 2U) {
    const std::size_t first_index = static_cast<std::size_t>(map.edges[edge]);
    const std::size_t second_index = static_cast<std::size_t>(map.edges[edge + 1U]);
    if (first_index >= map.nodes.size() || second_index >= map.nodes.size() ||
      first_index == second_index)
    {
      continue;
    }
    const NodeIdentity first{map.nodes[first_index].id, map.nodes[first_index].frame};
    const NodeIdentity second{map.nodes[second_index].id, map.nodes[second_index].frame};
    const bool first_tracked = tracked_identities.find(first) != tracked_identities.end();
    const bool second_tracked = tracked_identities.find(second) != tracked_identities.end();
    if (!first_tracked && !second_tracked) {
      continue;
    }
    if (result.eligible_nodes.find(first) == result.eligible_nodes.end() ||
      result.eligible_nodes.find(second) == result.eligible_nodes.end())
    {
      continue;
    }
    if (first_tracked) {
      current_neighbors[first].push_back(second);
    }
    if (second_tracked) {
      current_neighbors[second].push_back(first);
    }
  }

  if (!previous_node_observations_.empty()) {
    states.reserve(previous_node_neighbors_.size());
    // Reuse this scratch buffer for every center node.  The prior version
    // allocated one displacement vector per node even though only one center
    // is evaluated at a time.  Keeping its capacity preserves the calculation
    // and neighbour order while avoiding thousands of short-lived allocations.
    std::vector<std::array<double, 3>> displacements;
    for (const auto &[identity, neighbors] : previous_node_neighbors_) {
      const auto previous_center = previous_node_observations_.find(identity);
      if (previous_center == previous_node_observations_.end()) {
        continue;
      }
      double mean_dx = 0.0;
      double mean_dy = 0.0;
      double mean_dz = 0.0;
      double spacing_sum = 0.0;
      displacements.clear();
      if (displacements.capacity() < neighbors.size()) {
        displacements.reserve(neighbors.size());
      }
      for (const auto &neighbor_identity : neighbors) {
        const auto previous_neighbor = previous_node_observations_.find(neighbor_identity);
        const auto current_neighbor = result.eligible_nodes.find(neighbor_identity);
        if (previous_neighbor == previous_node_observations_.end() ||
          current_neighbor == result.eligible_nodes.end())
        {
          continue;
        }
        const double offset_x = previous_neighbor->second.x - previous_center->second.x;
        const double offset_y = previous_neighbor->second.y - previous_center->second.y;
        const double offset_z = previous_neighbor->second.z - previous_center->second.z;
        const double spacing = std::sqrt(
          offset_x * offset_x + offset_y * offset_y + offset_z * offset_z);
        if (!std::isfinite(spacing) || spacing <= 1.0e-9) {
          continue;
        }
        const double dx = current_neighbor->second.x - previous_neighbor->second.x;
        const double dy = current_neighbor->second.y - previous_neighbor->second.y;
        const double dz = current_neighbor->second.z - previous_neighbor->second.z;
        if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz)) {
          continue;
        }
        displacements.push_back({dx, dy, dz});
        mean_dx += dx;
        mean_dy += dy;
        mean_dz += dz;
        spacing_sum += spacing;
      }
      if (displacements.size() < kMinimumSharedNeighbors) {
        continue;
      }
      const double inverse_count = 1.0 / static_cast<double>(displacements.size());
      mean_dx *= inverse_count;
      mean_dy *= inverse_count;
      mean_dz *= inverse_count;
      const double local_spacing = spacing_sum * inverse_count;
      const double translation_ratio = std::sqrt(
        mean_dx * mean_dx + mean_dy * mean_dy + mean_dz * mean_dz) / local_spacing;
      double disagreement_sum = 0.0;
      for (const auto &displacement : displacements) {
        const double dx = displacement[0] - mean_dx;
        const double dy = displacement[1] - mean_dy;
        const double dz = displacement[2] - mean_dz;
        disagreement_sum += std::sqrt(dx * dx + dy * dy + dz * dz);
      }
      const double disagreement_ratio = disagreement_sum * inverse_count / local_spacing;
      ++last_local_structure_stats_.evaluated_node_count;
      NodeLocalStructureState state = NodeLocalStructureState::Ambiguous;
      if (translation_ratio < kStaticShiftRatio) {
        state = NodeLocalStructureState::Static;
        ++last_local_structure_stats_.static_node_count;
      } else if (disagreement_ratio <= kCoherentDisagreementRatio) {
        state = NodeLocalStructureState::Moving;
        ++last_local_structure_stats_.moving_node_count;
      } else {
        ++last_local_structure_stats_.ambiguous_node_count;
      }
      states.emplace(identity, state);
    }
  }

  NodeObservationMap next_node_observations;
  next_node_observations.reserve(current_neighbors.size() * 2U);
  for (const auto &[identity, neighbors] : current_neighbors) {
    const auto center = result.eligible_nodes.find(identity);
    if (center != result.eligible_nodes.end()) {
      next_node_observations.emplace(identity, center->second);
    }
    for (const auto &neighbor_identity : neighbors) {
      const auto neighbor = result.eligible_nodes.find(neighbor_identity);
      if (neighbor != result.eligible_nodes.end()) {
        next_node_observations.emplace(neighbor_identity, neighbor->second);
      }
    }
  }
  previous_node_observations_ = std::move(next_node_observations);
  previous_node_neighbors_ = std::move(current_neighbors);
  return states;
}

void TopologicalGridNode::pointCloudWatchdog()
{
  if (!pending_map_) {
    return;
  }
  const double elapsed_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - pending_map_received_at_).count();
  if (elapsed_sec < pointcloud_timeout_sec_) {
    return;
  }

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "No matching pointcloud received within %.2fs for map frame=%s; publishing empty voxels",
    pointcloud_timeout_sec_, pending_map_->header.frame_id.c_str());
  publishResult(*pending_map_, GridPointCounts{}, 0.0);
  pending_map_.reset();
}

GridPointCounts TopologicalGridNode::buildPointCounts(
  const sensor_msgs::msg::PointCloud2 &msg) const
{
  GridPointCounts point_counts;
  point_counts.reserve(msg.width * msg.height);
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }
      ++point_counts[positionToGridCell(*iter_x, *iter_y, *iter_z, grid_spec_)];
    }
  } catch (const std::runtime_error &error) {
    RCLCPP_ERROR(get_logger(), "Invalid pointcloud layout: %s", error.what());
    point_counts.clear();
  }
  return point_counts;
}

bool TopologicalGridNode::headersMatch(
  const std_msgs::msg::Header &map_header,
  const std_msgs::msg::Header &pointcloud_header) const
{
  return map_header.frame_id == pointcloud_header.frame_id &&
    map_header.stamp.sec == pointcloud_header.stamp.sec &&
    map_header.stamp.nanosec == pointcloud_header.stamp.nanosec;
}

voxel_msgs::msg::Voxel TopologicalGridNode::buildVoxelMessage(
  const std_msgs::msg::Header &header,
  const std::vector<LabeledGridVoxel> &voxels,
  std::uint32_t revision) const
{
  voxel_msgs::msg::Voxel voxel_msg;
  voxel_msg.header = header;
  voxel_msg.voxel_size = static_cast<float>(grid_spec_.cell_size);
  voxel_msg.origin_x = static_cast<float>(grid_spec_.origin_x);
  voxel_msg.origin_y = static_cast<float>(grid_spec_.origin_y);
  voxel_msg.origin_z = static_cast<float>(grid_spec_.origin_z);
  voxel_msg.x_shift = x_shift_;
  voxel_msg.y_shift = y_shift_;
  voxel_msg.z_shift = z_shift_;
  voxel_msg.offset = offset_;
  voxel_msg.revision = revision;
  voxel_msg.data.reserve(voxels.size());
  voxel_msg.labels.reserve(voxels.size());
  for (const auto &voxel : voxels) {
    const std::uint64_t flat_id =
      (static_cast<std::uint64_t>(static_cast<std::int64_t>(voxel.cell.x) + offset_) << x_shift_) |
      (static_cast<std::uint64_t>(static_cast<std::int64_t>(voxel.cell.y) + offset_) << y_shift_) |
      (static_cast<std::uint64_t>(static_cast<std::int64_t>(voxel.cell.z) + offset_) << z_shift_);
    voxel_msg.data.push_back(flat_id);
    voxel_msg.labels.push_back(voxel.label);
  }
  return voxel_msg;
}

void TopologicalGridNode::publishResult(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridPointCounts &input_point_counts,
  double point_count_ms)
{
  const auto update_started = std::chrono::steady_clock::now();
  const rclcpp::Time current_stamp(map.header.stamp, RCL_ROS_TIME);
  std::string reset_reason;
  if (has_last_map_state_ && history_reset_on_time_regression_ &&
    current_stamp < last_map_stamp_)
  {
    reset_reason = "time_regression";
  } else if (has_last_map_state_ && history_reset_node_count_ratio_ > 0.0 &&
    last_map_node_count_ > 0 &&
    static_cast<double>(map.nodes.size()) <
    static_cast<double>(last_map_node_count_) * history_reset_node_count_ratio_)
  {
    reset_reason = "node_count_drop";
  }
  if (!reset_reason.empty()) {
    temporal_filter_->clear();
    has_last_temporal_filter_stamp_ = false;
    point_activity_scheduler_->clear();
    normal_drift_states_.clear();
    normal_drift_epoch_ = 0;
    previous_node_neighbors_.clear();
    previous_node_observations_.clear();
    last_local_structure_stats_ = LocalStructureStats{};
    ++history_reset_count_;
    last_history_reset_reason_ = reset_reason;
    RCLCPP_WARN(
      get_logger(), "Cleared topological-grid history: reason=%s nodes=%zu->%zu",
      reset_reason.c_str(), last_map_node_count_, map.nodes.size());
  }
  last_map_stamp_ = current_stamp;
  last_map_node_count_ = map.nodes.size();
  has_last_map_state_ = true;

  if (voxelization_options_.require_input_points && point_activity_config_.enabled) {
    GridSpec activity_spec = grid_spec_;
    activity_spec.cell_size = point_activity_cell_size_;
    const auto activity_point_counts = aggregatePointCounts(
      input_point_counts, grid_spec_, activity_spec);
    last_point_activity_decision_ = point_activity_scheduler_->update(activity_point_counts);
    if (!last_point_activity_decision_.should_process) {
      ++point_activity_skipped_update_count_;
      const double update_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - update_started).count();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Topological grid: skipped point_activity=%.3f interval=%zu elapsed=%zu cells=%zu "
        "point_count=%.2fms update=%.2fms total=%.2fms",
        last_point_activity_decision_.activity_score,
        last_point_activity_decision_.desired_update_interval,
        last_point_activity_decision_.updates_since_process,
        last_point_activity_decision_.tracked_cell_count,
        point_count_ms, update_ms, point_count_ms + update_ms);
      return;
    }
  } else {
    last_point_activity_decision_ = PointActivityDecision{};
  }
  ++point_activity_processed_update_count_;

  // SAFE_TERRAIN must remain in the structural graph.  It is deliberately
  // prevented from creating a candidate by TemporalVoxelFilter, but treating
  // a terrain reclassification as an absent node destroys the history of an
  // object resting on the floor. HUMAN and CAR remain hard exclusions.
  auto structural_voxelization_options = voxelization_options_;
  structural_voxelization_options.excluded_labels.erase(
    ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN);
  GridCellSet safe_terrain_required_cells;
  temporal_filter_->appendTrackedCells(safe_terrain_required_cells);
  const auto assignment_started = std::chrono::steady_clock::now();
  auto result = voxelizeNodes(
    map, grid_spec_, structural_voxelization_options, &input_point_counts,
    &safe_terrain_required_cells);
  applyNormalDriftScores(map, result);
  NodeIdentitySet evaluation_identities;
  evaluation_identities.reserve(result.label_voxels.size());
  for (const auto &voxel : result.label_voxels) {
    for (const auto &observation : voxel.node_observations) {
      if (observation.label != ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN) {
        evaluation_identities.insert(observation.identity);
      }
    }
  }
  temporal_filter_->appendTrackedNodeIdentities(evaluation_identities);
  const auto local_structure_states = buildLocalStructureStates(
    map, result, evaluation_identities);
  const double assignment_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - assignment_started).count();
  double temporal_elapsed_seconds = 0.0;
  if (has_last_temporal_filter_stamp_ && current_stamp > last_temporal_filter_stamp_) {
    temporal_elapsed_seconds =
      (current_stamp - last_temporal_filter_stamp_).seconds();
  }
  const auto temporal_started = std::chrono::steady_clock::now();
  const auto visibility_states = buildVisibilityStates(map.header, result.label_voxels);
  const auto stable_voxels = temporal_filter_->update(
    result.label_voxels,
    voxelization_options_.require_input_points,
    voxelization_options_.minimum_input_points_per_voxel,
    &input_point_counts,
    &result.eligible_nodes,
    temporal_elapsed_seconds,
    visibility_states.empty() ? nullptr : &visibility_states,
    local_structure_states.empty() ? nullptr : &local_structure_states);
  const double temporal_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - temporal_started).count();
  last_temporal_filter_stamp_ = current_stamp;
  has_last_temporal_filter_stamp_ = true;
  const auto retained_without_current_points = static_cast<std::size_t>(std::count_if(
    stable_voxels.begin(), stable_voxels.end(),
    [this](const LabeledGridVoxel &voxel) {
      return voxelization_options_.require_input_points &&
             voxel.input_point_count < voxelization_options_.minimum_input_points_per_voxel;
    }));
  const auto retained_without_current_labels = static_cast<std::size_t>(std::count_if(
    stable_voxels.begin(), stable_voxels.end(),
    [](const LabeledGridVoxel &voxel) {return voxel.node_count == 0;}));
  const auto retained_by_node_identity = static_cast<std::size_t>(std::count_if(
    stable_voxels.begin(), stable_voxels.end(),
    [](const LabeledGridVoxel &voxel) {return voxel.retained_by_node_identity;}));
  const auto retained_by_gng_structure = static_cast<std::size_t>(std::count_if(
    stable_voxels.begin(), stable_voxels.end(),
    [](const LabeledGridVoxel &voxel) {return voxel.retained_by_gng_structure;}));
  const auto retained_by_local_structure = static_cast<std::size_t>(std::count_if(
    stable_voxels.begin(), stable_voxels.end(),
    [](const LabeledGridVoxel &voxel) {return voxel.retained_by_local_structure;}));
  const auto local_motion_suppressed_voxel_count = static_cast<std::size_t>(std::count_if(
    stable_voxels.begin(), stable_voxels.end(),
    [](const LabeledGridVoxel &voxel) {return voxel.local_motion_score > 0.0;}));
  const auto isolated_temporal_decay_count = static_cast<std::size_t>(std::count_if(
    stable_voxels.begin(), stable_voxels.end(),
    [](const LabeledGridVoxel &voxel) {return voxel.isolated_temporal_decay_applied;}));
  double triangle_topology_ms = 0.0;
  bool triangle_topology_rebuilt = false;
  std::size_t triangle_topology_cycle_count = 0U;
  const std::vector<std::array<std::size_t, 3>> *triangle_indices = nullptr;
  if (triangle_inference_options_.enabled) {
    const auto triangle_topology_started = std::chrono::steady_clock::now();
    triangle_indices = &triangle_topology_cache_.update(map);
    triangle_topology_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - triangle_topology_started).count();
    triangle_topology_rebuilt = triangle_topology_cache_.wasRebuilt();
    triangle_topology_cycle_count = triangle_indices->size();
  }
  const auto inference_started = std::chrono::steady_clock::now();
  const auto edge_result = inferVoxelsFromStableVoxelEdges(
    map, grid_spec_, stable_voxels,
    structural_voxelization_options.excluded_labels,
    edge_inference_options_, &input_point_counts);
  const auto isolation_split = splitVoxelsByGngConnectivity(stable_voxels, edge_result);
  const auto triangle_result = inferVoxelsFromStableVoxelTriangles(
    map, grid_spec_, isolation_split.connected_voxels,
    structural_voxelization_options.excluded_labels,
    triangle_inference_options_, &input_point_counts, triangle_indices);
  const auto direct_and_edge_voxels = mergeDirectAndInferredVoxels(
    isolation_split.connected_voxels, edge_result.voxels);
  const auto combined_voxels = mergeDirectAndInferredVoxels(
    direct_and_edge_voxels, triangle_result.voxels);
  const double inference_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - inference_started).count();
  const auto message_started = std::chrono::steady_clock::now();
  const std::uint32_t revision = ++voxel_revision_;
  const auto voxel_msg = buildVoxelMessage(map.header, combined_voxels, revision);
  const auto isolated_voxel_msg = buildVoxelMessage(
    map.header, isolation_split.isolated_voxels, revision);
  const double message_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - message_started).count();
  const auto pub_started = std::chrono::steady_clock::now();
  voxel_pub_->publish(voxel_msg);
  isolated_voxel_pub_->publish(isolated_voxel_msg);

  std_msgs::msg::String summary_msg;
  std::ostringstream oss;
  oss << "{";
  oss << "\"point_count_ms\":" << point_count_ms << ",";
  oss << "\"assignment_ms\":" << assignment_ms << ",";
  oss << "\"temporal_ms\":" << temporal_ms << ",";
  oss << "\"triangle_topology_ms\":" << triangle_topology_ms << ",";
  oss << "\"inference_ms\":" << inference_ms << ",";
  oss << "\"message_ms\":" << message_ms << ",";
  oss << "\"node_count\":" << result.included_node_count << ",";
  oss << "\"input_node_count\":" << map.nodes.size() << ",";
  oss << "\"included_node_count\":" << result.included_node_count << ",";
  oss << "\"excluded_node_count\":" << result.excluded_node_count << ",";
  oss << "\"unsupported_node_count\":" << result.unsupported_node_count << ",";
  oss << "\"shape_candidate_node_count\":" << result.shape_candidate_node_count << ",";
  oss << "\"shape_seed_node_count\":" << result.shape_seed_node_count << ",";
  oss << "\"shape_retained_node_count\":" << result.shape_retained_node_count << ",";
  oss << "\"shape_rejected_node_count\":" << result.shape_rejected_node_count << ",";
  oss << "\"shape_score_median\":" << result.shape_score_median << ",";
  oss << "\"shape_score_mad\":" << result.shape_score_mad << ",";
  oss << "\"shape_score_threshold\":" << result.shape_score_threshold << ",";
  oss << "\"unknown_component_count\":" << result.unknown_component_count << ",";
  oss << "\"unknown_component_event_count\":"
      << result.unknown_component_event_count << ",";
  oss << "\"unknown_component_event_node_count\":"
      << result.unknown_component_event_node_count << ",";
  oss << "\"unknown_component_event_voxel_count\":"
      << result.unknown_component_event_voxel_count << ",";
  oss << "\"insufficient_point_voxel_count\":"
      << result.insufficient_point_voxel_count << ",";
  oss << "\"grid_neighbor_isolated_voxel_count\":"
      << result.isolated_voxel_count << ",";
  oss << "\"isolated_voxel_count\":" << isolation_split.isolated_voxels.size() << ",";
  oss << "\"voxel_count\":" << combined_voxels.size() << ",";
  oss << "\"stable_direct_voxel_count\":" << stable_voxels.size() << ",";
  oss << "\"direct_voxel_count\":" << isolation_split.connected_voxels.size() << ",";
  oss << "\"edge_inferred_voxel_count\":" << edge_result.voxels.size() << ",";
  oss << "\"triangle_inferred_voxel_count\":" << triangle_result.voxels.size() << ",";
  oss << "\"input_edge_count\":" << edge_result.input_edge_count << ",";
  oss << "\"voxel_edge_count\":" << edge_result.voxel_edge_count << ",";
  oss << "\"invalid_edge_count\":" << edge_result.invalid_edge_count << ",";
  oss << "\"excluded_edge_count\":" << edge_result.excluded_edge_count << ",";
  oss << "\"inactive_endpoint_edge_count\":"
      << edge_result.inactive_endpoint_edge_count << ",";
  oss << "\"same_voxel_edge_count\":" << edge_result.same_voxel_edge_count << ",";
  oss << "\"overlength_edge_count\":" << edge_result.overlength_edge_count << ",";
  oss << "\"duplicate_voxel_edge_count\":"
      << edge_result.duplicate_voxel_edge_count << ",";
  oss << "\"candidate_triangle_count\":"
      << triangle_result.candidate_triangle_count << ",";
  oss << "\"triangle_topology_cycle_count\":"
      << triangle_topology_cycle_count << ",";
  oss << "\"triangle_topology_rebuilt\":"
      << (triangle_topology_rebuilt ? "true" : "false") << ",";
  oss << "\"triangle_topology_rebuild_count\":"
      << triangle_topology_cache_.rebuildCount() << ",";
  oss << "\"accepted_triangle_count\":"
      << triangle_result.accepted_triangle_count << ",";
  oss << "\"inactive_vertex_triangle_count\":"
      << triangle_result.inactive_vertex_triangle_count << ",";
  oss << "\"excluded_triangle_count\":"
      << triangle_result.excluded_triangle_count << ",";
  oss << "\"overlength_triangle_count\":"
      << triangle_result.overlength_triangle_count << ",";
  oss << "\"degenerate_triangle_count\":"
      << triangle_result.degenerate_triangle_count << ",";
  oss << "\"normal_rejected_triangle_count\":"
      << triangle_result.normal_rejected_triangle_count << ",";
  oss << "\"point_support_rejected_triangle_count\":"
      << triangle_result.point_support_rejected_triangle_count << ",";
  oss << "\"retained_without_current_points\":"
      << retained_without_current_points << ",";
  oss << "\"retained_without_current_labels\":"
      << retained_without_current_labels << ",";
  oss << "\"retained_by_gng_structure\":" << retained_by_gng_structure << ",";
  oss << "\"isolated_temporal_decay_count\":" << isolated_temporal_decay_count << ",";
  oss << "\"retained_by_local_structure\":" << retained_by_local_structure << ",";
  oss << "\"local_motion_suppressed_voxel_count\":"
      << local_motion_suppressed_voxel_count << ",";
  oss << "\"retained_by_node_identity\":" << retained_by_node_identity << ",";
  oss << "\"label_voxel_count\":" << result.label_voxels.size() << ",";
  oss << "\"observed_voxel_count\":" << result.voxels.size() << ",";
  oss << "\"tracked_voxel_count\":" << temporal_filter_->trackedVoxelCount() << ",";
  oss << "\"history_window_size\":"
      << temporal_filter_config_.history_window_size << ",";
  oss << "\"temporal_time_constant_sec\":"
      << temporal_filter_config_.time_constant_sec << ",";
  oss << "\"temporal_activation_score\":"
      << temporal_filter_config_.activation_score << ",";
  oss << "\"temporal_retention_score\":"
      << temporal_filter_config_.retention_score << ",";
  oss << "\"depth_visibility_enabled\":"
      << (depth_visibility_enabled_ ? "true" : "false") << ",";
  oss << "\"depth_visibility_matched\":"
      << (last_visibility_stats_.depth_matched ? "true" : "false") << ",";
  oss << "\"depth_visibility_transform_available\":"
      << (last_visibility_stats_.transform_available ? "true" : "false") << ",";
  oss << "\"depth_visibility_sync_offset_ms\":"
      << last_visibility_stats_.sync_offset_ms << ",";
  oss << "\"depth_visibility_out_of_view_count\":"
      << last_visibility_stats_.out_of_view_count << ",";
  oss << "\"depth_visibility_occluded_count\":"
      << last_visibility_stats_.occluded_count << ",";
  oss << "\"depth_visibility_free_count\":"
      << last_visibility_stats_.free_count << ",";
  oss << "\"depth_visibility_unknown_count\":"
      << last_visibility_stats_.unknown_count << ",";
  oss << "\"normal_drift_filter_enabled\":"
      << (normal_drift_filter_enabled_ ? "true" : "false") << ",";
  oss << "\"normal_drift_observed_node_count\":"
      << last_normal_drift_stats_.observed_node_count << ",";
  oss << "\"normal_drift_valid_normal_node_count\":"
      << last_normal_drift_stats_.valid_normal_node_count << ",";
  oss << "\"normal_drift_moving_node_count\":"
      << last_normal_drift_stats_.moving_node_count << ",";
  oss << "\"normal_drift_mean_score\":"
      << last_normal_drift_stats_.mean_score << ",";
  oss << "\"normal_drift_maximum_score\":"
      << last_normal_drift_stats_.maximum_score << ",";
  oss << "\"local_structure_evaluated_node_count\":"
      << last_local_structure_stats_.evaluated_node_count << ",";
  oss << "\"local_structure_static_node_count\":"
      << last_local_structure_stats_.static_node_count << ",";
  oss << "\"local_structure_moving_node_count\":"
      << last_local_structure_stats_.moving_node_count << ",";
  oss << "\"local_structure_ambiguous_node_count\":"
      << last_local_structure_stats_.ambiguous_node_count << ",";
  oss << "\"node_identity_retention_enabled\":"
      << (temporal_filter_config_.node_identity_retention_enabled ? "true" : "false")
      << ",";
  oss << "\"node_identity_max_displacement\":"
      << temporal_filter_config_.node_identity_max_displacement << ",";
  oss << "\"node_identity_history_migration_enabled\":"
      << (temporal_filter_config_.node_identity_history_migration_enabled ? "true" : "false")
      << ",";
  oss << "\"require_input_points\":"
      << (voxelization_options_.require_input_points ? "true" : "false") << ",";
  oss << "\"point_support_mode\":\""
      << pointSupportModeName(voxelization_options_.point_support_mode) << "\",";
  oss << "\"point_support_radius_m\":"
      << voxelization_options_.point_support_radius_m << ",";
  oss << "\"unknown_shape_filter_enabled\":"
      << (voxelization_options_.unknown_shape_filter_enabled ? "true" : "false") << ",";
  oss << "\"shape_neighborhood_hops\":"
      << voxelization_options_.shape_neighborhood_hops << ",";
  oss << "\"shape_minimum_neighbors\":"
      << voxelization_options_.shape_minimum_neighbors << ",";
  oss << "\"shape_residual_weight\":"
      << voxelization_options_.shape_residual_weight << ",";
  oss << "\"shape_mad_multiplier\":"
      << voxelization_options_.shape_mad_multiplier << ",";
  oss << "\"shape_seed_expansion_scale\":"
      << voxelization_options_.shape_seed_expansion_scale << ",";
  oss << "\"minimum_input_points_per_voxel\":"
      << voxelization_options_.minimum_input_points_per_voxel << ",";
  oss << "\"neighbor_radius_cells\":"
      << voxelization_options_.neighbor_radius_cells << ",";
  oss << "\"neighbor_radius_m\":"
      << voxelization_options_.neighbor_radius_m << ",";
  oss << "\"history_reset_count\":" << history_reset_count_ << ",";
  oss << "\"last_history_reset_reason\":\"" << last_history_reset_reason_ << "\",";
  oss << "\"point_activity_update_enabled\":"
      << (point_activity_config_.enabled ? "true" : "false") << ",";
  oss << "\"point_activity_cell_size\":" << point_activity_cell_size_ << ",";
  oss << "\"point_activity_warmup_updates\":"
      << point_activity_config_.warmup_update_count << ",";
  oss << "\"point_activity_score\":"
      << last_point_activity_decision_.activity_score << ",";
  oss << "\"point_activity_mean_hit_frequency\":"
      << last_point_activity_decision_.mean_hit_frequency << ",";
  oss << "\"point_activity_desired_update_interval\":"
      << last_point_activity_decision_.desired_update_interval << ",";
  oss << "\"point_activity_tracked_cell_count\":"
      << last_point_activity_decision_.tracked_cell_count << ",";
  oss << "\"point_activity_processed_update_count\":"
      << point_activity_processed_update_count_ << ",";
  oss << "\"point_activity_skipped_update_count\":"
      << point_activity_skipped_update_count_ << ",";
  oss << "\"edge_inference_enabled\":"
      << (edge_inference_options_.enabled ? "true" : "false") << ",";
  oss << "\"edge_inferred_require_input_points\":"
      << (edge_inference_options_.require_point_support_for_output ? "true" : "false")
      << ",";
  oss << "\"triangle_inferred_require_input_points\":"
      << (triangle_inference_options_.require_point_support_for_output ? "true" : "false")
      << ",";
  oss << "\"edge_max_length\":" << edge_inference_options_.maximum_edge_length << ",";
  oss << "\"edge_length_gate\":\""
      << (edge_inference_options_.maximum_edge_length > 0.0 ? "metric" : "adaptive")
      << "\",";
  oss << "\"gng_connected_direct_voxel_count\":"
      << isolation_split.connected_voxels.size() << ",";
  oss << "\"isolated_topic\":\"" << isolated_topic_ << "\",";
  oss << "\"isolated_excluded_from_grasp_candidates\":true,";
  oss << "\"triangle_inference_enabled\":"
      << (triangle_inference_options_.enabled ? "true" : "false") << ",";
  oss << "\"triangle_max_edge_length\":"
      << triangle_inference_options_.maximum_edge_length << ",";
  oss << "\"triangle_min_area\":" << triangle_inference_options_.minimum_area << ",";
  oss << "\"triangle_min_aspect_ratio\":"
      << triangle_inference_options_.minimum_aspect_ratio << ",";
  oss << "\"triangle_max_normal_angle_deg\":"
      << triangle_inference_options_.maximum_normal_angle_degrees << ",";
  oss << "\"triangle_min_point_support_ratio\":"
      << triangle_inference_options_.minimum_point_support_ratio << ",";
  oss << "\"input_point_voxel_count\":" << input_point_counts.size() << ",";
  oss << "\"grid_size\":" << grid_spec_.cell_size << ",";
  oss << "\"voxel_size\":" << voxel_msg.voxel_size << ",";
  oss << "\"revision\":" << voxel_msg.revision << ",";
  oss << "\"origin_x\":" << voxel_msg.origin_x << ",";
  oss << "\"origin_y\":" << voxel_msg.origin_y << ",";
  oss << "\"origin_z\":" << voxel_msg.origin_z << ",";
  oss << "\"origin\":[" << grid_spec_.origin_x << "," << grid_spec_.origin_y << ","
      << grid_spec_.origin_z << "],";
  oss << "\"x_shift\":" << x_shift_ << ",";
  oss << "\"y_shift\":" << y_shift_ << ",";
  oss << "\"z_shift\":" << z_shift_ << ",";
  oss << "\"offset\":" << offset_ << ",";
  oss << "\"excluded_labels\":[";
  std::vector<int> excluded_labels;
  excluded_labels.reserve(voxelization_options_.excluded_labels.size());
  for (const auto label : voxelization_options_.excluded_labels) {
    excluded_labels.push_back(static_cast<int>(label));
  }
  std::sort(excluded_labels.begin(), excluded_labels.end());
  for (std::size_t i = 0; i < excluded_labels.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << excluded_labels[i];
  }
  oss << "],";
  std::unordered_map<std::uint8_t, std::size_t> label_counts;
  for (const auto &voxel : combined_voxels) {
    ++label_counts[voxel.label];
  }
  std::vector<std::pair<int, std::size_t>> sorted_label_counts;
  sorted_label_counts.reserve(label_counts.size());
  for (const auto &[label, count] : label_counts) {
    sorted_label_counts.emplace_back(static_cast<int>(label), count);
  }
  std::sort(sorted_label_counts.begin(), sorted_label_counts.end());
  oss << "\"label_counts\":{";
  for (std::size_t i = 0; i < sorted_label_counts.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << "\"" << sorted_label_counts[i].first << "\":" << sorted_label_counts[i].second;
  }
  oss << "},";
  oss << "\"assignments_topic\":\"" << assignment_detail_topic_ << "\"";
  oss << "}";
  summary_msg.data = oss.str();
  summary_pub_->publish(summary_msg);

  if (assignment_detail_pub_->get_subscription_count() > 0U) {
    std_msgs::msg::String assignment_detail_msg;
    std::ostringstream detail;
    detail << "{\"revision\":" << revision
           << ",\"voxel_count\":" << combined_voxels.size()
           << ",\"assignments\":[";
    for (std::size_t i = 0; i < combined_voxels.size(); ++i) {
      if (i > 0) {
        detail << ",";
      }
      const auto &voxel = combined_voxels[i];
      detail << "{\"cell\":[" << voxel.cell.x << "," << voxel.cell.y << "," << voxel.cell.z << "]"
             << ",\"voxel_id\":" << voxel_msg.data[i]
             << ",\"label\":" << static_cast<int>(voxel.label)
             << ",\"node_count\":" << voxel.node_count
             << ",\"input_point_count\":" << voxel.input_point_count
             << ",\"neighbor_count\":" << voxel.neighbor_count
             << ",\"local_motion_score\":" << voxel.local_motion_score
             << ",\"has_cross_cell_gng_edge\":"
             << (voxel.has_cross_cell_gng_edge ? "true" : "false")
             << ",\"edge_support_count\":" << voxel.edge_support_count
             << ",\"triangle_support_count\":" << voxel.triangle_support_count
             << ",\"unknown_component_event\":"
             << (voxel.unknown_component_event ? "true" : "false")
             << ",\"unknown_component_active_cell_count\":"
             << voxel.unknown_component_active_cell_count
             << ",\"retained_by_gng_structure\":"
             << (voxel.retained_by_gng_structure ? "true" : "false")
             << ",\"retained_by_local_structure\":"
             << (voxel.retained_by_local_structure ? "true" : "false")
             << ",\"retained_by_node_identity\":"
             << (voxel.retained_by_node_identity ? "true" : "false")
             << ",\"isolated_temporal_decay_applied\":"
             << (voxel.isolated_temporal_decay_applied ? "true" : "false")
             << ",\"source\":\"";
      const bool direct = voxel.history_sample_count > 0 || !voxel.node_observations.empty();
      if (direct) {
        detail << "direct";
      } else if (voxel.edge_support_count > 0 && voxel.triangle_support_count > 0) {
        detail << "edge_and_triangle_inferred";
      } else if (voxel.triangle_support_count > 0) {
        detail << "triangle_inferred";
      } else {
        detail << "edge_inferred";
      }
      detail << "\"}";
    }
    detail << "]}";
    assignment_detail_msg.data = detail.str();
    assignment_detail_pub_->publish(assignment_detail_msg);
  }
  const double pub_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - pub_started).count();
  const double update_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - update_started).count();

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Topological grid: input_nodes=%zu included=%zu excluded=%zu unsupported=%zu "
    "observed_voxels=%zu isolated=%zu grasp_direct=%zu voxel_edges=%zu edge_inferred=%zu "
    "triangles=%zu triangle_inferred=%zu combined=%zu point_count=%.2fms assignment=%.2fms "
    "temporal=%.2fms inference=%.2fms message=%.2fms pub=%.2fms update=%.2fms total=%.2fms",
    map.nodes.size(), result.included_node_count, result.excluded_node_count,
    result.unsupported_node_count, result.voxels.size(),
    isolation_split.isolated_voxels.size(),
    isolation_split.connected_voxels.size(), edge_result.voxel_edge_count,
    edge_result.voxels.size(),
    triangle_result.accepted_triangle_count, triangle_result.voxels.size(),
    combined_voxels.size(), point_count_ms, assignment_ms, temporal_ms, inference_ms,
    message_ms, pub_ms, update_ms, point_count_ms + update_ms);
}

}  // namespace fuzzrobo::topological_grid

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fuzzrobo::topological_grid::TopologicalGridNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
