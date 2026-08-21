#include <ais_gng/topological_grid/topological_grid_node.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
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

}  // namespace

namespace fuzzrobo::topological_grid
{

TopologicalGridNode::TopologicalGridNode(const rclcpp::NodeOptions &options)
: Node("topological_grid_node", options)
{
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/topological_map/merged");
  pointcloud_topic_ = this->declare_parameter<std::string>(
    "pointcloud_topic", "/downsampling/unknown");
  output_topic_ = this->declare_parameter<std::string>(
    "output_topic", "/topological_grid_assignments");
  delta_topic_ = this->declare_parameter<std::string>("delta_topic", "");
  if (delta_topic_.empty()) {
    delta_topic_ = output_topic_ + "/delta";
  }
  isolated_topic_ = this->declare_parameter<std::string>("isolated_topic", "");
  if (isolated_topic_.empty()) {
    isolated_topic_ = output_topic_ + "/isolated";
  }
  edge_inferred_topic_ = this->declare_parameter<std::string>("edge_inferred_topic", "");
  if (edge_inferred_topic_.empty()) {
    edge_inferred_topic_ = output_topic_ + "/edge_inferred";
  }
  triangle_inferred_topic_ = this->declare_parameter<std::string>(
    "triangle_inferred_topic", "");
  if (triangle_inferred_topic_.empty()) {
    triangle_inferred_topic_ = output_topic_ + "/triangle_inferred";
  }
  summary_topic_ = this->declare_parameter<std::string>(
    "summary_topic", "/topological_grid_assignments/summary");
  pointcloud_timeout_sec_ = this->declare_parameter<double>("pointcloud_timeout_sec", 0.5);
  grid_spec_.cell_size = this->declare_parameter<double>("grid_size", 0.01);
  grid_spec_.origin_x = this->declare_parameter<double>("origin_x", 0.0);
  grid_spec_.origin_y = this->declare_parameter<double>("origin_y", 0.0);
  grid_spec_.origin_z = this->declare_parameter<double>("origin_z", 0.0);
  origin_shift_half_ = this->declare_parameter<bool>("origin_shift_half", false);
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
  edge_inference_options_.enabled = this->declare_parameter<bool>(
    "edge_inference_enabled", true);
  edge_inference_options_.maximum_edge_length = this->declare_parameter<double>(
    "edge_max_length", 0.10);
  const bool inferred_require_input_points = this->declare_parameter<bool>(
    "inferred_require_input_points", true);
  edge_inference_options_.require_point_support_for_output = inferred_require_input_points;
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
  triangle_inference_options_.require_point_support_for_output = inferred_require_input_points;
  const int minimum_input_points_per_voxel = this->declare_parameter<int>(
    "minimum_input_points_per_voxel", 1);
  const int neighbor_radius_cells = this->declare_parameter<int>(
    "neighbor_radius_cells", 1);
  voxelization_options_.neighbor_radius_m = this->declare_parameter<double>(
    "neighbor_radius_m", 0.02);
  const int history_window_size = this->declare_parameter<int>("history_window_size", 100);
  const int minimum_label_history_count = this->declare_parameter<int>(
    "minimum_label_history_count", 3);
  const int minimum_point_input_history_count = this->declare_parameter<int>(
    "minimum_point_input_history_count", 3);
  const int isolated_minimum_label_history_count = this->declare_parameter<int>(
    "isolated_minimum_label_history_count", 5);
  const int isolated_minimum_point_input_history_count = this->declare_parameter<int>(
    "isolated_minimum_point_input_history_count", 5);
  const int maximum_missing_label_updates = this->declare_parameter<int>(
    "maximum_missing_label_updates", 2);
  temporal_filter_config_.node_identity_retention_enabled = this->declare_parameter<bool>(
    "node_identity_retention_enabled", false);
  temporal_filter_config_.node_identity_max_displacement = this->declare_parameter<double>(
    "node_identity_max_displacement", 0.02);
  temporal_filter_config_.node_identity_history_migration_enabled =
    this->declare_parameter<bool>("node_identity_history_migration_enabled", true);
  history_reset_on_time_regression_ = this->declare_parameter<bool>(
    "history_reset_on_time_regression", false);
  history_reset_node_count_ratio_ = this->declare_parameter<double>(
    "history_reset_node_count_ratio", 0.5);

  if (grid_spec_.cell_size <= 0.0 || pointcloud_timeout_sec_ <= 0.0 ||
    edge_inference_options_.maximum_edge_length <= 0.0 ||
    triangle_inference_options_.maximum_edge_length <= 0.0 ||
    triangle_inference_options_.minimum_area < 0.0 ||
    triangle_inference_options_.minimum_aspect_ratio < 0.0 ||
    triangle_inference_options_.minimum_aspect_ratio > 1.0 ||
    triangle_inference_options_.maximum_normal_angle_degrees < 0.0 ||
    triangle_inference_options_.maximum_normal_angle_degrees > 180.0 ||
    triangle_inference_options_.minimum_point_support_ratio < 0.0 ||
    triangle_inference_options_.minimum_point_support_ratio > 1.0 ||
    temporal_filter_config_.node_identity_max_displacement < 0.0 ||
    voxelization_options_.neighbor_radius_m < 0.0 ||
    voxelization_options_.point_support_radius_m < 0.0 ||
    history_reset_node_count_ratio_ < 0.0 || history_reset_node_count_ratio_ > 1.0)
  {
    throw std::invalid_argument(
            "grid, inference, point-support, and node-identity parameters are invalid");
  }
  if (minimum_input_points_per_voxel <= 0 || neighbor_radius_cells < 0 ||
    history_window_size <= 0 || minimum_label_history_count <= 0 ||
    minimum_point_input_history_count <= 0 ||
    isolated_minimum_label_history_count <= 0 ||
    isolated_minimum_point_input_history_count <= 0 ||
    maximum_missing_label_updates < 0 ||
    minimum_label_history_count > history_window_size ||
    minimum_point_input_history_count > history_window_size ||
    isolated_minimum_label_history_count > history_window_size ||
    isolated_minimum_point_input_history_count > history_window_size ||
    maximum_missing_label_updates > history_window_size)
  {
    throw std::invalid_argument(
            "point and history counts must be positive and history counts must not exceed "
            "history_window_size");
  }
  voxelization_options_.minimum_input_points_per_voxel =
    static_cast<std::size_t>(minimum_input_points_per_voxel);
  voxelization_options_.neighbor_radius_cells = neighbor_radius_cells;
  temporal_filter_config_.history_window_size =
    static_cast<std::size_t>(history_window_size);
  temporal_filter_config_.minimum_label_history_count =
    static_cast<std::size_t>(minimum_label_history_count);
  temporal_filter_config_.minimum_point_input_history_count =
    static_cast<std::size_t>(minimum_point_input_history_count);
  temporal_filter_config_.isolated_minimum_label_history_count =
    static_cast<std::size_t>(isolated_minimum_label_history_count);
  temporal_filter_config_.isolated_minimum_point_input_history_count =
    static_cast<std::size_t>(isolated_minimum_point_input_history_count);
  temporal_filter_config_.maximum_missing_label_updates =
    static_cast<std::size_t>(maximum_missing_label_updates);
  temporal_filter_ = std::make_unique<TemporalVoxelFilter>(temporal_filter_config_);

  if (origin_shift_half_) {
    const double half = grid_spec_.cell_size * 0.5;
    grid_spec_.origin_x = half;
    grid_spec_.origin_y = half;
    grid_spec_.origin_z = half;
  }

  voxel_pub_ = this->create_publisher<voxel_msgs::msg::Voxel>(
    output_topic_,
    rclcpp::QoS(1).reliable().transient_local());
  voxel_delta_pub_ = this->create_publisher<voxel_msgs::msg::VoxelLabelDelta>(
    delta_topic_, rclcpp::QoS(10).reliable());
  isolated_voxel_pub_ = this->create_publisher<voxel_msgs::msg::Voxel>(
    isolated_topic_,
    rclcpp::QoS(1).reliable().transient_local());
  edge_inferred_pub_ = this->create_publisher<voxel_msgs::msg::Voxel>(
    edge_inferred_topic_,
    rclcpp::QoS(1).reliable().transient_local());
  triangle_inferred_pub_ = this->create_publisher<voxel_msgs::msg::Voxel>(
    triangle_inferred_topic_,
    rclcpp::QoS(1).reliable().transient_local());
  summary_pub_ = this->create_publisher<std_msgs::msg::String>(summary_topic_, 10);
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

  RCLCPP_INFO(
    this->get_logger(),
    "TopologicalGridNode ready: input=%s pointcloud=%s timeout=%.2fs output=%s delta=%s "
    "isolated=%s edge_inferred=%s edge_fill=%s/%.3fm grid_size=%.3f "
    "origin=(%.3f, %.3f, %.3f) "
    "shifted=%s excluded=[%s] point_support=%s/%s/%zu neighbors=%d/%.3fm history=%zu "
    "label/point=%zu/%zu isolated=%zu/%zu missing_label_grace=%zu "
    "node_identity=%s/%.3fm migration=%s triangle=%s/%s/%.3fm",
    input_topic_.c_str(),
    pointcloud_topic_.c_str(),
    pointcloud_timeout_sec_,
    output_topic_.c_str(),
    delta_topic_.c_str(),
    isolated_topic_.c_str(),
    edge_inferred_topic_.c_str(),
    edge_inference_options_.enabled ? "enabled" : "disabled",
    edge_inference_options_.maximum_edge_length,
    grid_spec_.cell_size,
    grid_spec_.origin_x,
    grid_spec_.origin_y,
    grid_spec_.origin_z,
    origin_shift_half_ ? "true" : "false",
    labelSetToString(voxelization_options_.excluded_labels).c_str(),
    voxelization_options_.require_input_points ? "required" : "disabled",
    pointSupportModeName(voxelization_options_.point_support_mode),
    voxelization_options_.minimum_input_points_per_voxel,
    voxelization_options_.neighbor_radius_cells,
    voxelization_options_.neighbor_radius_m,
    temporal_filter_config_.history_window_size,
    temporal_filter_config_.minimum_label_history_count,
    temporal_filter_config_.minimum_point_input_history_count,
    temporal_filter_config_.isolated_minimum_label_history_count,
    temporal_filter_config_.isolated_minimum_point_input_history_count,
    temporal_filter_config_.maximum_missing_label_updates,
    temporal_filter_config_.node_identity_retention_enabled ? "enabled" : "disabled",
    temporal_filter_config_.node_identity_max_displacement,
    temporal_filter_config_.node_identity_history_migration_enabled ? "enabled" : "disabled",
    triangle_inference_options_.enabled ? "enabled" : "disabled",
    triangle_inferred_topic_.c_str(),
    triangle_inference_options_.maximum_edge_length);
}

void TopologicalGridNode::mapCallback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (!voxelization_options_.require_input_points) {
    publishResult(*msg, GridPointCounts{});
    return;
  }

  if (has_latest_pointcloud_ && headersMatch(msg->header, latest_pointcloud_header_)) {
    publishResult(*msg, latest_point_counts_);
    has_latest_pointcloud_ = false;
    latest_point_counts_.clear();
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
  auto point_counts = buildPointCounts(*msg);
  if (pending_map_ && headersMatch(pending_map_->header, msg->header)) {
    publishResult(*pending_map_, point_counts);
    pending_map_.reset();
    return;
  }

  latest_point_counts_ = std::move(point_counts);
  latest_pointcloud_header_ = msg->header;
  has_latest_pointcloud_ = true;
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
  publishResult(*pending_map_, GridPointCounts{});
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

voxel_msgs::msg::VoxelLabelDelta TopologicalGridNode::buildVoxelDelta(
  const voxel_msgs::msg::Voxel &voxel_msg)
{
  constexpr std::uint8_t kAbsentLabel = 255U;
  std::unordered_map<std::int64_t, std::uint8_t> current_labels;
  current_labels.reserve(voxel_msg.data.size());
  for (std::size_t index = 0; index < voxel_msg.data.size(); ++index) {
    current_labels.insert_or_assign(voxel_msg.data[index], voxel_msg.labels[index]);
  }

  std::vector<std::int64_t> changed_ids;
  changed_ids.reserve(current_labels.size() + last_published_labels_.size());
  for (const auto &[id, label] : current_labels) {
    const auto previous = last_published_labels_.find(id);
    if (previous == last_published_labels_.end() || previous->second != label) {
      changed_ids.push_back(id);
    }
  }
  for (const auto &[id, label] : last_published_labels_) {
    (void)label;
    if (current_labels.find(id) == current_labels.end()) {
      changed_ids.push_back(id);
    }
  }
  std::sort(changed_ids.begin(), changed_ids.end());

  voxel_msgs::msg::VoxelLabelDelta delta;
  delta.header = voxel_msg.header;
  delta.voxel_size = voxel_msg.voxel_size;
  delta.origin_x = voxel_msg.origin_x;
  delta.origin_y = voxel_msg.origin_y;
  delta.origin_z = voxel_msg.origin_z;
  delta.x_shift = voxel_msg.x_shift;
  delta.y_shift = voxel_msg.y_shift;
  delta.z_shift = voxel_msg.z_shift;
  delta.offset = voxel_msg.offset;
  delta.revision = voxel_msg.revision;
  delta.data.reserve(changed_ids.size());
  delta.old_labels.reserve(changed_ids.size());
  delta.new_labels.reserve(changed_ids.size());
  for (const auto id : changed_ids) {
    const auto previous = last_published_labels_.find(id);
    const auto current = current_labels.find(id);
    delta.data.push_back(id);
    delta.old_labels.push_back(previous == last_published_labels_.end() ?
      kAbsentLabel : previous->second);
    delta.new_labels.push_back(current == current_labels.end() ?
      kAbsentLabel : current->second);
  }
  last_published_labels_ = std::move(current_labels);
  return delta;
}

void TopologicalGridNode::publishResult(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridPointCounts &input_point_counts)
{
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
    ++history_reset_count_;
    last_history_reset_reason_ = reset_reason;
    RCLCPP_WARN(
      get_logger(), "Cleared topological-grid history: reason=%s nodes=%zu->%zu",
      reset_reason.c_str(), last_map_node_count_, map.nodes.size());
  }
  last_map_stamp_ = current_stamp;
  last_map_node_count_ = map.nodes.size();
  has_last_map_state_ = true;

  const auto result = voxelizeNodes(
    map, grid_spec_, voxelization_options_, &input_point_counts);
  const auto stable_voxels = temporal_filter_->update(
    result.label_voxels,
    voxelization_options_.require_input_points,
    voxelization_options_.minimum_input_points_per_voxel,
    &input_point_counts,
    &result.eligible_nodes);
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
  const auto isolation_split = splitVoxelsByIsolation(stable_voxels);
  const auto edge_result = inferVoxelsFromStableVoxelEdges(
    map, grid_spec_, isolation_split.connected_voxels,
    voxelization_options_.excluded_labels,
    edge_inference_options_, &input_point_counts);
  const auto triangle_result = inferVoxelsFromStableVoxelTriangles(
    map, grid_spec_, isolation_split.connected_voxels,
    voxelization_options_.excluded_labels,
    triangle_inference_options_, &input_point_counts);
  const auto direct_and_edge_voxels = mergeDirectAndInferredVoxels(
    isolation_split.connected_voxels, edge_result.voxels);
  const auto combined_voxels = mergeDirectAndInferredVoxels(
    direct_and_edge_voxels, triangle_result.voxels);
  const std::uint32_t revision = ++voxel_revision_;
  const auto voxel_msg = buildVoxelMessage(map.header, combined_voxels, revision);
  const auto isolated_voxel_msg = buildVoxelMessage(
    map.header, isolation_split.isolated_voxels, revision);
  const auto edge_inferred_msg = buildVoxelMessage(
    map.header, edge_result.voxels, revision);
  const auto triangle_inferred_msg = buildVoxelMessage(
    map.header, triangle_result.voxels, revision);
  const auto voxel_delta = buildVoxelDelta(voxel_msg);
  voxel_pub_->publish(voxel_msg);
  voxel_delta_pub_->publish(voxel_delta);
  isolated_voxel_pub_->publish(isolated_voxel_msg);
  edge_inferred_pub_->publish(edge_inferred_msg);
  triangle_inferred_pub_->publish(triangle_inferred_msg);

  std_msgs::msg::String summary_msg;
  std::ostringstream oss;
  oss << "{";
  oss << "\"node_count\":" << result.included_node_count << ",";
  oss << "\"input_node_count\":" << map.nodes.size() << ",";
  oss << "\"included_node_count\":" << result.included_node_count << ",";
  oss << "\"excluded_node_count\":" << result.excluded_node_count << ",";
  oss << "\"unsupported_node_count\":" << result.unsupported_node_count << ",";
  oss << "\"insufficient_point_voxel_count\":"
      << result.insufficient_point_voxel_count << ",";
  oss << "\"observed_isolated_voxel_count\":" << result.isolated_voxel_count << ",";
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
  oss << "\"retained_by_node_identity\":" << retained_by_node_identity << ",";
  oss << "\"label_voxel_count\":" << result.label_voxels.size() << ",";
  oss << "\"observed_voxel_count\":" << result.voxels.size() << ",";
  oss << "\"tracked_voxel_count\":" << temporal_filter_->trackedVoxelCount() << ",";
  oss << "\"history_window_size\":"
      << temporal_filter_config_.history_window_size << ",";
  oss << "\"minimum_label_history_count\":"
      << temporal_filter_config_.minimum_label_history_count << ",";
  oss << "\"minimum_point_input_history_count\":"
      << temporal_filter_config_.minimum_point_input_history_count << ",";
  oss << "\"isolated_minimum_label_history_count\":"
      << temporal_filter_config_.isolated_minimum_label_history_count << ",";
  oss << "\"isolated_minimum_point_input_history_count\":"
      << temporal_filter_config_.isolated_minimum_point_input_history_count << ",";
  oss << "\"maximum_missing_label_updates\":"
      << temporal_filter_config_.maximum_missing_label_updates << ",";
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
  oss << "\"minimum_input_points_per_voxel\":"
      << voxelization_options_.minimum_input_points_per_voxel << ",";
  oss << "\"neighbor_radius_cells\":"
      << voxelization_options_.neighbor_radius_cells << ",";
  oss << "\"neighbor_radius_m\":"
      << voxelization_options_.neighbor_radius_m << ",";
  oss << "\"history_reset_count\":" << history_reset_count_ << ",";
  oss << "\"last_history_reset_reason\":\"" << last_history_reset_reason_ << "\",";
  oss << "\"edge_inference_enabled\":"
      << (edge_inference_options_.enabled ? "true" : "false") << ",";
  oss << "\"inferred_require_input_points\":"
      << (edge_inference_options_.require_point_support_for_output ? "true" : "false")
      << ",";
  oss << "\"edge_max_length\":" << edge_inference_options_.maximum_edge_length << ",";
  oss << "\"edge_inferred_topic\":\"" << edge_inferred_topic_ << "\",";
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
  oss << "\"triangle_inferred_topic\":\"" << triangle_inferred_topic_ << "\",";
  oss << "\"input_point_voxel_count\":" << input_point_counts.size() << ",";
  oss << "\"grid_size\":" << grid_spec_.cell_size << ",";
  oss << "\"voxel_size\":" << voxel_msg.voxel_size << ",";
  oss << "\"revision\":" << voxel_msg.revision << ",";
  oss << "\"delta_topic\":\"" << delta_topic_ << "\",";
  oss << "\"delta_voxel_count\":" << voxel_delta.data.size() << ",";
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
  oss << "\"assignments\":[";
  for (std::size_t i = 0; i < combined_voxels.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    const auto &voxel = combined_voxels[i];
    oss << "{\"cell\":[" << voxel.cell.x << "," << voxel.cell.y << "," << voxel.cell.z << "]"
        << ",\"voxel_id\":" << voxel_msg.data[i]
        << ",\"label\":" << static_cast<int>(voxel.label)
        << ",\"node_count\":" << voxel.node_count
        << ",\"input_point_count\":" << voxel.input_point_count
        << ",\"neighbor_count\":" << voxel.neighbor_count
        << ",\"edge_support_count\":" << voxel.edge_support_count
        << ",\"triangle_support_count\":" << voxel.triangle_support_count
        << ",\"retained_by_node_identity\":"
        << (voxel.retained_by_node_identity ? "true" : "false")
        << ",\"source\":\"";
    const bool direct = voxel.history_sample_count > 0 || !voxel.node_observations.empty();
    if (direct) {
      oss << "direct";
    } else if (voxel.edge_support_count > 0 && voxel.triangle_support_count > 0) {
      oss << "edge_and_triangle_inferred";
    } else if (voxel.triangle_support_count > 0) {
      oss << "triangle_inferred";
    } else {
      oss << "edge_inferred";
    }
    oss
        << "\"}";
  }
  oss << "]";
  oss << "}";
  summary_msg.data = oss.str();
  summary_pub_->publish(summary_msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Topological grid: input_nodes=%zu included=%zu excluded=%zu unsupported=%zu "
    "observed_voxels=%zu isolated=%zu grasp_direct=%zu voxel_edges=%zu edge_inferred=%zu "
    "triangles=%zu triangle_inferred=%zu combined=%zu",
    map.nodes.size(), result.included_node_count, result.excluded_node_count,
    result.unsupported_node_count, result.voxels.size(),
    isolation_split.isolated_voxels.size(),
    isolation_split.connected_voxels.size(), edge_result.voxel_edge_count,
    edge_result.voxels.size(),
    triangle_result.accepted_triangle_count, triangle_result.voxels.size(),
    combined_voxels.size());
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
