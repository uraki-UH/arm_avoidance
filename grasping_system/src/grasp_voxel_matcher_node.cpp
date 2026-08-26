#include <candidate/grasp_voxel_matcher.hpp>

#include <ais_gng_msgs/msg/planar_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <voxel_msgs/msg/voxel.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace grasping_system::nodes
{

namespace candidate = grasping_system::candidate;

class GraspVoxelMatcherNode : public rclcpp::Node
{
public:
  GraspVoxelMatcherNode()
  : Node("grasp_voxel_matcher_node")
  {
    object_voxels_topic_ = declare_parameter<std::string>(
      "object_voxels_topic", "/topological_grid_voxels");
    environment_voxels_topic_ = declare_parameter<std::string>(
      "environment_voxels_topic", "");
    required_graph_topic_ = declare_parameter<std::string>(
      "required_graph_topic", "grip_V_topological_map");
    undersize_graph_topic_ = declare_parameter<std::string>(
      "undersize_graph_topic", "grip_minV_topological_map");
    forbidden_graph_topic_ = declare_parameter<std::string>(
      "forbidden_graph_topic", "grip_baseV_topological_map");
    const std::string candidate_topic = declare_parameter<std::string>(
      "candidate_topic", "/grasp_pose_cands");
    const std::string candidate_voxels_topic = declare_parameter<std::string>(
      "candidate_voxels_topic", "/grasp_pose_cand_cells");
    const std::string summary_topic = declare_parameter<std::string>(
      "summary_topic", "/grasp_pose_cands/summary");
    topological_map_topic_ = declare_parameter<std::string>(
      "topological_map_topic", "/topological_map");
    planar_clusters_topic_ = declare_parameter<std::string>(
      "planar_clusters_topic", "/topological_planar_clusters_incremental");
    enable_depth_visibility_ = declare_parameter<bool>("enable_depth_visibility", false);
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/camera/camera/depth/image_rect_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera/depth/camera_info");

    match_config_.minimum_required_occupancy_ratio = declare_parameter<double>(
      "minimum_required_occupancy_ratio", 0.1);
    match_config_.minimum_required_hits = positiveSizeParameter(
      "minimum_required_hits", 3, true);
    match_config_.minimum_outside_undersize_hits = positiveSizeParameter(
      "minimum_outside_undersize_hits", 1, true);
    match_config_.maximum_forbidden_hits = positiveSizeParameter(
      "maximum_forbidden_hits", 0, true);
    match_config_.maximum_anchor_voxels = positiveSizeParameter(
      "maximum_anchor_voxels", 500, false);
    match_config_.maximum_candidates = positiveSizeParameter(
      "maximum_candidates", 50, false);
    max_depth_cache_num_ = positiveSizeParameter("max_depth_cache_num", 64, false);
    const auto closing_axis = declare_parameter<std::vector<double>>(
      "contact_closing_axis", {0.0, 1.0, 0.0});
    if (closing_axis.size() != 3U) {
      throw std::invalid_argument("contact_closing_axis must contain exactly three values");
    }
    contact_closing_axis_ = Eigen::Vector3d(
      closing_axis[0], closing_axis[1], closing_axis[2]);
    if (!contact_closing_axis_.allFinite() || contact_closing_axis_.norm() < 1e-12) {
      throw std::invalid_argument("contact_closing_axis must be finite and non-zero");
    }
    contact_closing_axis_.normalize();

    const int update_period_ms = declare_parameter<int>("update_period_ms", 500);
    if (update_period_ms <= 0) {
      throw std::invalid_argument("update_period_ms must be positive");
    }
    orientations_ = loadOrientations();

    const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    candidate_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      candidate_topic, transient_qos);
    candidate_voxels_pub_ = create_publisher<voxel_msgs::msg::Voxel>(
      candidate_voxels_topic, transient_qos);
    summary_pub_ = create_publisher<std_msgs::msg::String>(summary_topic, transient_qos);

    if (enable_depth_visibility_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::SharedPtr msg) {
          const std::int64_t stamp_ns = stampNanoseconds(msg->header.stamp);
          depth_image_cache_[stamp_ns] = std::move(msg);
          while (depth_image_cache_.size() > max_depth_cache_num_) {
            depth_image_cache_.erase(depth_image_cache_.begin());
          }
          is_depth_dirty_ = true;
        });
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          camera_info_ = std::move(msg);
          is_depth_dirty_ = true;
        });
    }

    object_voxels_sub_ = create_subscription<voxel_msgs::msg::Voxel>(
      object_voxels_topic_, transient_qos,
      [this](voxel_msgs::msg::Voxel::SharedPtr msg) {
        objectSnapshotCallback(std::move(msg));
      });
    if (!environment_voxels_topic_.empty() &&
      environment_voxels_topic_ != object_voxels_topic_)
    {
      environment_voxels_sub_ = create_subscription<voxel_msgs::msg::Voxel>(
        environment_voxels_topic_, transient_qos,
        [this](voxel_msgs::msg::Voxel::SharedPtr msg) {
          environmentSnapshotCallback(std::move(msg));
        });
    }

    required_graph_sub_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      required_graph_topic_, transient_qos,
      [this](ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
        required_graph_ = std::move(msg);
        template_dirty_ = true;
        dirty_ = true;
      });
    if (!undersize_graph_topic_.empty()) {
      undersize_graph_sub_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
        undersize_graph_topic_, transient_qos,
        [this](ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
          undersize_graph_ = std::move(msg);
          template_dirty_ = true;
          dirty_ = true;
        });
    }
    if (!forbidden_graph_topic_.empty()) {
      forbidden_graph_sub_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
        forbidden_graph_topic_, transient_qos,
        [this](ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
          forbidden_graph_ = std::move(msg);
          template_dirty_ = true;
          dirty_ = true;
      });
    }
    if (!topological_map_topic_.empty()) {
      topological_map_sub_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
        topological_map_topic_, transient_qos,
        [this](ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
          topological_map_ = std::move(msg);
          surface_normals_dirty_ = true;
          planar_collision_dirty_ = true;
        });
    }
    if (!planar_clusters_topic_.empty()) {
      planar_clusters_sub_ = create_subscription<ais_gng_msgs::msg::PlanarClusterArray>(
        planar_clusters_topic_, transient_qos,
        [this](ais_gng_msgs::msg::PlanarClusterArray::SharedPtr msg) {
          planar_clusters_ = std::move(msg);
          surface_normals_dirty_ = true;
          planar_collision_dirty_ = true;
        });
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(update_period_ms),
      std::bind(&GraspVoxelMatcherNode::updateMatches, this));

    RCLCPP_INFO(
      get_logger(),
      "Grasp voxel matcher ready: objects=%s environment=%s required=%s undersize=%s forbidden=%s map=%s planes=%s depth=%s camera_info=%s orientations=%zu period=%dms",
      object_voxels_topic_.c_str(),
      environment_voxels_topic_.empty() ? "<object voxels>" : environment_voxels_topic_.c_str(),
      required_graph_topic_.c_str(),
      undersize_graph_topic_.empty() ? "<disabled>" : undersize_graph_topic_.c_str(),
      forbidden_graph_topic_.empty() ? "<disabled>" : forbidden_graph_topic_.c_str(),
      topological_map_topic_.empty() ? "<disabled>" : topological_map_topic_.c_str(),
      planar_clusters_topic_.empty() ? "<disabled>" : planar_clusters_topic_.c_str(),
      enable_depth_visibility_ ? depth_topic_.c_str() : "<disabled>",
      enable_depth_visibility_ ? camera_info_topic_.c_str() : "<disabled>",
      orientations_.size(), update_period_ms);
  }

private:
  std::size_t positiveSizeParameter(
    const std::string &name, int default_value, bool allow_zero)
  {
    const int value = declare_parameter<int>(name, default_value);
    if (value < 0 || (!allow_zero && value == 0)) {
      throw std::invalid_argument(
              name + (allow_zero ? " must be non-negative" : " must be positive"));
    }
    return static_cast<std::size_t>(value);
  }

  std::vector<Eigen::Quaterniond> loadOrientations()
  {
    constexpr double kTwoPi = 6.28318530717958647692;
    const auto explicit_rpy = declare_parameter<std::vector<double>>(
      "orientation_rpy", std::vector<double>{});
    const int yaw_samples = declare_parameter<int>("yaw_samples", 12);
    const double roll = declare_parameter<double>("roll", 0.0);
    const double pitch = declare_parameter<double>("pitch", 0.0);
    const double yaw_offset = declare_parameter<double>("yaw_offset", 0.0);
    std::vector<Eigen::Quaterniond> orientations;
    if (!explicit_rpy.empty()) {
      if (explicit_rpy.size() % 3U != 0U) {
        throw std::invalid_argument("orientation_rpy must contain roll,pitch,yaw triples");
      }
      orientations.reserve(explicit_rpy.size() / 3U);
      for (std::size_t index = 0; index < explicit_rpy.size(); index += 3U) {
        orientations.push_back(fromRpy(
          explicit_rpy[index], explicit_rpy[index + 1U], explicit_rpy[index + 2U]));
      }
      return orientations;
    }

    if (yaw_samples <= 0 || !std::isfinite(roll) || !std::isfinite(pitch) ||
      !std::isfinite(yaw_offset))
    {
      throw std::invalid_argument("yaw_samples must be positive and orientation values finite");
    }
    orientations.reserve(static_cast<std::size_t>(yaw_samples));
    for (int index = 0; index < yaw_samples; ++index) {
      const double yaw = yaw_offset + kTwoPi * static_cast<double>(index) /
        static_cast<double>(yaw_samples);
      orientations.push_back(fromRpy(roll, pitch, yaw));
    }
    return orientations;
  }

  static Eigen::Quaterniond fromRpy(double roll, double pitch, double yaw)
  {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
           Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())) *
           Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  }

  static std::vector<Eigen::Vector3d> graphPoints(
    const ais_gng_msgs::msg::TopologicalMap &graph)
  {
    std::vector<Eigen::Vector3d> points;
    points.reserve(graph.nodes.size());
    for (const auto &node : graph.nodes) {
      points.emplace_back(node.pos.x, node.pos.y, node.pos.z);
    }
    return points;
  }

  struct OpposingContactBands
  {
    std::vector<candidate::GraspVoxelContactPair> pairs;
  };

  static double minimumProjectionSpacing(
    const std::vector<Eigen::Vector3d> &points,
    const Eigen::Vector3d &axis)
  {
    std::vector<double> projections;
    projections.reserve(points.size());
    for (const auto &point : points) {
      projections.push_back(point.dot(axis));
    }
    std::sort(projections.begin(), projections.end());
    double spacing = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index < projections.size(); ++index) {
      const double difference = projections[index] - projections[index - 1U];
      if (difference > 1e-9) {
        spacing = std::min(spacing, difference);
      }
    }
    if (!std::isfinite(spacing)) {
      throw std::invalid_argument(
              "cannot derive opposing contact bands from a zero-thickness grasp volume");
    }
    return spacing;
  }

  static OpposingContactBands makeOpposingContactBands(
    const std::vector<Eigen::Vector3d> &open_volume,
    const std::vector<Eigen::Vector3d> &closed_volume,
    const Eigen::Vector3d &closing_axis)
  {
    if (open_volume.empty() || closed_volume.empty()) {
      throw std::invalid_argument("opposing contact bands require open and closed grasp volumes");
    }
    const auto projection_range = [&closing_axis](const std::vector<Eigen::Vector3d> &points) {
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = -std::numeric_limits<double>::infinity();
        for (const auto &point : points) {
          const double projection = point.dot(closing_axis);
          minimum = std::min(minimum, projection);
          maximum = std::max(maximum, projection);
        }
        return std::pair<double, double>{minimum, maximum};
      };
    const auto [open_minimum, open_maximum] = projection_range(open_volume);
    const auto [closed_minimum, closed_maximum] = projection_range(closed_volume);
    const double spacing = minimumProjectionSpacing(open_volume, closing_axis);
    const double band_half_width = 0.5 * spacing + 1e-9;

    std::vector<Eigen::Vector3d> open_positive;
    std::vector<Eigen::Vector3d> open_negative;
    for (const auto &point : open_volume) {
      const double projection = point.dot(closing_axis);
      if (projection >= open_maximum - band_half_width) {
        open_positive.push_back(point);
      }
      if (projection <= open_minimum + band_half_width) {
        open_negative.push_back(point);
      }
    }
    if (open_positive.empty() || open_negative.empty()) {
      throw std::logic_error("failed to extract both open-gripper contact bands");
    }

    const double maximum_stroke = std::max(
      std::abs(closed_maximum - open_maximum),
      std::abs(closed_minimum - open_minimum));
    const std::size_t closure_samples = std::max<std::size_t>(
      2U, static_cast<std::size_t>(std::ceil(maximum_stroke / spacing)) + 1U);

    OpposingContactBands bands;
    bands.pairs.reserve(closure_samples);
    for (std::size_t sample = 0; sample < closure_samples; ++sample) {
      const double fraction = static_cast<double>(sample) /
        static_cast<double>(closure_samples - 1U);
      const double positive_plane =
        (1.0 - fraction) * open_maximum + fraction * closed_maximum;
      const double negative_plane =
        (1.0 - fraction) * open_minimum + fraction * closed_minimum;
      const Eigen::Vector3d positive_shift =
        (positive_plane - open_maximum) * closing_axis;
      const Eigen::Vector3d negative_shift =
        (negative_plane - open_minimum) * closing_axis;
      candidate::GraspVoxelContactPair contact_pair;
      contact_pair.positive.reserve(open_positive.size());
      contact_pair.negative.reserve(open_negative.size());
      for (const auto &point : open_positive) {
        contact_pair.positive.push_back(point + positive_shift);
      }
      for (const auto &point : open_negative) {
        contact_pair.negative.push_back(point + negative_shift);
      }
      bands.pairs.push_back(std::move(contact_pair));
    }
    return bands;
  }

  static candidate::GraspVoxelContactPair makeBoundaryPair(
    const std::vector<Eigen::Vector3d> &volume,
    const Eigen::Vector3d &axis)
  {
    if (volume.empty()) {
      throw std::invalid_argument("lateral continuation requires a non-empty grasp volume");
    }
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    for (const auto &point : volume) {
      const double projection = point.dot(axis);
      minimum = std::min(minimum, projection);
      maximum = std::max(maximum, projection);
    }
    const double band_half_width = 0.5 * minimumProjectionSpacing(volume, axis) + 1e-9;
    candidate::GraspVoxelContactPair boundary_pair;
    for (const auto &point : volume) {
      const double projection = point.dot(axis);
      if (projection >= maximum - band_half_width) {
        boundary_pair.positive.push_back(point);
      }
      if (projection <= minimum + band_half_width) {
        boundary_pair.negative.push_back(point);
      }
    }
    if (boundary_pair.positive.empty() || boundary_pair.negative.empty()) {
      throw std::logic_error("failed to extract a grasp-volume boundary pair");
    }
    return boundary_pair;
  }

  static std::vector<candidate::GraspVoxelContactPair> makeLateralContinuationPairs(
    const std::vector<Eigen::Vector3d> &open_volume,
    const Eigen::Vector3d &closing_axis)
  {
    const Eigen::Vector3d reference =
      std::abs(closing_axis.dot(Eigen::Vector3d::UnitX())) < 0.9 ?
      Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitZ();
    Eigen::Vector3d first_axis = reference - reference.dot(closing_axis) * closing_axis;
    first_axis.normalize();
    Eigen::Vector3d second_axis = closing_axis.cross(first_axis);
    second_axis.normalize();
    return {
      makeBoundaryPair(open_volume, first_axis),
      makeBoundaryPair(open_volume, second_axis)};
  }

  bool usesSeparateEnvironment() const noexcept
  {
    return !environment_voxels_topic_.empty() &&
      environment_voxels_topic_ != object_voxels_topic_;
  }

  void objectSnapshotCallback(voxel_msgs::msg::Voxel::SharedPtr msg)
  {
    object_voxels_ = std::move(msg);
    surface_normals_dirty_ = true;
    planar_collision_dirty_ = true;
    dirty_ = true;
  }

  void environmentSnapshotCallback(voxel_msgs::msg::Voxel::SharedPtr msg)
  {
    environment_voxels_ = std::move(msg);
    dirty_ = true;
  }

  static voxel_idx::VoxelIndexingSchema makeIndexing(
    float voxel_size,
    int x_shift,
    int y_shift,
    int z_shift,
    std::int64_t offset)
  {
    voxel_idx::VoxelIndexingSchema indexing;
    indexing.x_shift = x_shift;
    indexing.y_shift = y_shift;
    indexing.z_shift = z_shift;
    indexing.offset = offset;
    indexing.voxel_size = voxel_size;
    if (!indexing.isValid()) {
      throw std::invalid_argument("voxel message has an invalid indexing schema");
    }
    return indexing;
  }

  static std::unique_ptr<candidate::OccupiedVoxelGrid> makeGrid(
    const voxel_msgs::msg::Voxel &msg)
  {
    if (msg.voxel_size <= 0.0F || !std::isfinite(msg.voxel_size)) {
      throw std::invalid_argument("voxel message has an invalid voxel_size");
    }
    if (!msg.labels.empty() && msg.labels.size() != msg.data.size()) {
      throw std::invalid_argument("voxel labels must be empty or aligned with data");
    }

    const auto indexing = makeIndexing(
      msg.voxel_size, msg.x_shift, msg.y_shift, msg.z_shift, msg.offset);

    candidate::VoxelGridGeometry geometry;
    geometry.voxel_size = msg.voxel_size;
    geometry.origin = Eigen::Vector3d(msg.origin_x, msg.origin_y, msg.origin_z);
    auto grid = std::make_unique<candidate::OccupiedVoxelGrid>(geometry);
    for (const auto raw_id : msg.data) {
      grid->add(indexing.unpack(static_cast<std::uint64_t>(raw_id)));
    }
    return grid;
  }

  bool refreshPlanarCollisionCells()
  {
    if (!planar_collision_dirty_) {
      return false;
    }
    planar_collision_dirty_ = false;
    has_planar_collision_context_ = false;
    std::unordered_set<candidate::VoxelIndex, candidate::VoxelIndexHash> cells;
    if (planar_clusters_ && object_voxels_ &&
      planar_clusters_->header.frame_id == object_voxels_->header.frame_id)
    {
      const candidate::VoxelGridGeometry geometry{
        object_voxels_->voxel_size,
        Eigen::Vector3d(
          object_voxels_->origin_x, object_voxels_->origin_y, object_voxels_->origin_z)};
      for (const auto &cluster : planar_clusters_->clusters) {
        for (const auto &point : cluster.support_edges) {
          cells.insert(geometry.pointToCell(Eigen::Vector3d(point.x, point.y, point.z)));
        }
        if (cluster.support_edges.empty() && topological_map_ &&
          topological_map_->header.frame_id == planar_clusters_->header.frame_id &&
          topological_map_->frame_number == planar_clusters_->frame_number)
        {
          for (const auto node_idx : cluster.node_indices) {
            if (node_idx >= topological_map_->nodes.size()) {
              continue;
            }
            const auto &node = topological_map_->nodes[node_idx];
            cells.insert(geometry.pointToCell(Eigen::Vector3d(
                node.pos.x, node.pos.y, node.pos.z)));
          }
        }
      }
      has_planar_collision_context_ = true;
    }
    const bool is_same = cells.size() == planar_collision_cells_.size() &&
      std::all_of(
      cells.begin(), cells.end(),
      [this](const candidate::VoxelIndex &cell) {
        return planar_collision_cells_.find(cell) != planar_collision_cells_.end();
      });
    planar_collision_cell_num_ = cells.size();
    if (is_same) {
      return false;
    }
    planar_collision_cells_ = std::move(cells);
    return true;
  }

  std::unique_ptr<candidate::OccupiedVoxelGrid> makeCollisionGrid(
    const voxel_msgs::msg::Voxel &msg) const
  {
    auto grid = makeGrid(msg);
    for (const auto &cell : planar_collision_cells_) {
      grid->add(cell);
    }
    return grid;
  }

  void rebuildSurfaceNormals(const candidate::VoxelGridGeometry &geometry)
  {
    surface_normals_.reset();
    has_surface_normal_context_ = false;
    has_planar_cluster_context_ = false;
    surface_normal_cell_num_ = 0;
    surface_normals_dirty_ = false;
    if (!topological_map_ || !object_voxels_) {
      return;
    }
    if (topological_map_->header.frame_id != object_voxels_->header.frame_id) {
      return;
    }

    std::unordered_set<std::uint32_t> planar_node_indices;
    if (planar_clusters_ &&
      planar_clusters_->header.frame_id == topological_map_->header.frame_id &&
      planar_clusters_->frame_number == topological_map_->frame_number)
    {
      for (const auto &cluster : planar_clusters_->clusters) {
        planar_node_indices.insert(cluster.node_indices.begin(), cluster.node_indices.end());
      }
      has_planar_cluster_context_ = true;
    }

    const auto indexing = makeIndexing(
      object_voxels_->voxel_size,
      object_voxels_->x_shift,
      object_voxels_->y_shift,
      object_voxels_->z_shift,
      object_voxels_->offset);
    std::unordered_set<candidate::VoxelIndex, candidate::VoxelIndexHash> object_cells;
    object_cells.reserve(object_voxels_->data.size());
    for (const auto raw_id : object_voxels_->data) {
      object_cells.insert(indexing.unpack(static_cast<std::uint64_t>(raw_id)));
    }

    auto surface_normals = std::make_unique<candidate::VoxelSurfaceNormalGrid>(geometry);
    for (std::size_t node_idx = 0; node_idx < topological_map_->nodes.size(); ++node_idx) {
      const auto &node = topological_map_->nodes[node_idx];
      const Eigen::Vector3d position(node.pos.x, node.pos.y, node.pos.z);
      const Eigen::Vector3d normal(node.normal.x, node.normal.y, node.normal.z);
      const auto cell = geometry.pointToCell(position);
      if (object_cells.find(cell) == object_cells.end()) {
        continue;
      }
      surface_normals->add(
        cell, normal,
        planar_node_indices.find(static_cast<std::uint32_t>(node_idx)) !=
        planar_node_indices.end());
    }
    surface_normal_cell_num_ = surface_normals->size();
    has_surface_normal_context_ = surface_normal_cell_num_ > 0U;
    surface_normals_ = std::move(surface_normals);
  }

  static bool isSupportedDepthEncoding(const sensor_msgs::msg::Image &image)
  {
    return image.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
           image.encoding == sensor_msgs::image_encodings::MONO16 ||
           image.encoding == sensor_msgs::image_encodings::TYPE_32FC1;
  }

  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time &stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
  }

  sensor_msgs::msg::Image::SharedPtr findMatchingDepthImage(
    const voxel_msgs::msg::Voxel &object_voxels) const
  {
    const auto iterator = depth_image_cache_.find(stampNanoseconds(object_voxels.header.stamp));
    return iterator == depth_image_cache_.end() ? nullptr : iterator->second;
  }

  bool isDepthVisibilityReady() const
  {
    return object_voxels_ && camera_info_ && findMatchingDepthImage(*object_voxels_);
  }

  static double depthMetersAt(
    const sensor_msgs::msg::Image &image,
    int column,
    int row)
  {
    if (column < 0 || row < 0 ||
      column >= static_cast<int>(image.width) || row >= static_cast<int>(image.height) ||
      image.is_bigendian != 0U)
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      image.encoding == sensor_msgs::image_encodings::MONO16)
    {
      const std::size_t pixel_idx = static_cast<std::size_t>(row) * image.step +
        static_cast<std::size_t>(column) * sizeof(std::uint16_t);
      if (pixel_idx + sizeof(std::uint16_t) > image.data.size()) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      std::uint16_t raw_depth = 0U;
      std::memcpy(&raw_depth, image.data.data() + pixel_idx, sizeof(raw_depth));
      return raw_depth == 0U ? std::numeric_limits<double>::quiet_NaN() :
             static_cast<double>(raw_depth) * 0.001;
    }
    if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      const std::size_t pixel_idx = static_cast<std::size_t>(row) * image.step +
        static_cast<std::size_t>(column) * sizeof(float);
      if (pixel_idx + sizeof(float) > image.data.size()) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      float raw_depth = std::numeric_limits<float>::quiet_NaN();
      std::memcpy(&raw_depth, image.data.data() + pixel_idx, sizeof(raw_depth));
      return raw_depth;
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  candidate::GraspVoxelMatcher::CandidateFilter makeDepthVisibilityFilter(
    const voxel_msgs::msg::Voxel &object_voxels,
    const sensor_msgs::msg::Image::SharedPtr &depth_image) const
  {
    if (!depth_image || !camera_info_) {
      throw std::logic_error("depth visibility context is unavailable");
    }
    if (!isSupportedDepthEncoding(*depth_image)) {
      throw std::invalid_argument("depth image encoding is unsupported");
    }
    if (depth_image->is_bigendian != 0U) {
      throw std::invalid_argument("big-endian depth images are unsupported");
    }
    if (depth_image->header.frame_id.empty() || object_voxels.header.frame_id.empty()) {
      throw std::invalid_argument("depth and object voxel frame ids are required");
    }
    if (!camera_info_->header.frame_id.empty() &&
      camera_info_->header.frame_id != depth_image->header.frame_id)
    {
      throw std::invalid_argument("camera info frame does not match the depth image frame");
    }
    const double focal_x = camera_info_->k[0U];
    const double focal_y = camera_info_->k[4U];
    const double center_x = camera_info_->k[2U];
    const double center_y = camera_info_->k[5U];
    if (!std::isfinite(focal_x) || !std::isfinite(focal_y) ||
      !std::isfinite(center_x) || !std::isfinite(center_y) ||
      focal_x <= 0.0 || focal_y <= 0.0)
    {
      throw std::invalid_argument("camera info has invalid intrinsics");
    }

    tf2::Transform object_to_depth;
    object_to_depth.setIdentity();
    if (object_voxels.header.frame_id != depth_image->header.frame_id) {
      const auto transform = tf_buffer_->lookupTransform(
        depth_image->header.frame_id,
        object_voxels.header.frame_id,
        rclcpp::Time(depth_image->header.stamp));
      tf2::fromMsg(transform.transform, object_to_depth);
    }
    const candidate::VoxelGridGeometry geometry{
      object_voxels.voxel_size,
      Eigen::Vector3d(
        object_voxels.origin_x, object_voxels.origin_y, object_voxels.origin_z)};
    if (!geometry.isValid()) {
      throw std::invalid_argument("object voxel geometry is invalid");
    }
    return [
      depth_image, geometry, object_to_depth,
      focal_x, focal_y, center_x, center_y](
      const candidate::GraspVoxelMatchCandidate &candidate_result,
      const candidate::CompiledGraspOrientation &orientation) {
        if (orientation.visibility_offsets.empty()) {
          return false;
        }
        for (const auto &offset : orientation.visibility_offsets) {
          const candidate::VoxelIndex cell{
            candidate_result.anchor.x + offset.x,
            candidate_result.anchor.y + offset.y,
            candidate_result.anchor.z + offset.z};
          const Eigen::Vector3d point = geometry.origin + geometry.voxel_size *
            Eigen::Vector3d(
            static_cast<double>(cell.x) + 0.5,
            static_cast<double>(cell.y) + 0.5,
            static_cast<double>(cell.z) + 0.5);
          const tf2::Vector3 depth_point = object_to_depth * tf2::Vector3(
            point.x(), point.y(), point.z());
          if (!std::isfinite(depth_point.x()) || !std::isfinite(depth_point.y()) ||
            !std::isfinite(depth_point.z()) || depth_point.z() <= 0.0)
          {
            return false;
          }
          const double projected_column =
            focal_x * depth_point.x() / depth_point.z() + center_x;
          const double projected_row =
            focal_y * depth_point.y() / depth_point.z() + center_y;
          if (!std::isfinite(projected_column) || !std::isfinite(projected_row) ||
            projected_column < static_cast<double>(std::numeric_limits<int>::min()) ||
            projected_column > static_cast<double>(std::numeric_limits<int>::max()) ||
            projected_row < static_cast<double>(std::numeric_limits<int>::min()) ||
            projected_row > static_cast<double>(std::numeric_limits<int>::max()))
          {
            return false;
          }
          const int column = static_cast<int>(std::lround(projected_column));
          const int row = static_cast<int>(std::lround(projected_row));
          const double observed_depth = depthMetersAt(*depth_image, column, row);
          if (!std::isfinite(observed_depth) || observed_depth <= 0.0 ||
            observed_depth - depth_point.z() <= geometry.voxel_size)
          {
            return false;
          }
        }
        return true;
      };
  }

  std::string waitingReason() const
  {
    if (!object_voxels_) {
      return "object_voxels";
    }
    if (!environment_voxels_topic_.empty() &&
      environment_voxels_topic_ != object_voxels_topic_ && !environment_voxels_)
    {
      return "environment_voxels";
    }
    if (!required_graph_) {
      return "required_graph";
    }
    if (!undersize_graph_topic_.empty() && !undersize_graph_) {
      return "undersize_graph";
    }
    if (!forbidden_graph_topic_.empty() && !forbidden_graph_) {
      return "forbidden_graph";
    }
    if (enable_depth_visibility_ && !isDepthVisibilityReady()) {
      return "depth_visibility";
    }
    return {};
  }

  void publishEmptyCandidates()
  {
    if (!object_voxels_) {
      return;
    }
    const candidate::GraspVoxelMatchResult result;
    publishCandidates(result);
    publishCandidateVoxels(result);
  }

  void updateMatches()
  {
    if (!dirty_ && !planar_collision_dirty_ &&
      !(enable_depth_visibility_ && is_depth_dirty_))
    {
      return;
    }
    const std::string waiting_for = waitingReason();
    if (!waiting_for.empty()) {
      if (waiting_for == "depth_visibility") {
        publishEmptyCandidates();
      }
      publishStatus("waiting", waiting_for);
      return;
    }
    const bool voxel_state_changed = dirty_;
    const bool depth_state_changed = enable_depth_visibility_ && is_depth_dirty_;
    const bool planar_collision_changed = refreshPlanarCollisionCells();
    if (!voxel_state_changed && !planar_collision_changed && !depth_state_changed) {
      return;
    }
    dirty_ = false;

    try {
      const auto started = std::chrono::steady_clock::now();
      const auto environment_msg = environment_voxels_ ? environment_voxels_ : object_voxels_;
      const auto depth_visibility_filter = enable_depth_visibility_ ?
        makeDepthVisibilityFilter(*object_voxels_, findMatchingDepthImage(*object_voxels_)) :
        candidate::GraspVoxelMatcher::CandidateFilter{};
      if (!compiled_template_ || template_dirty_ ||
        std::abs(compiled_template_->voxel_size - object_voxels_->voxel_size) > 1e-9)
      {
        candidate::GraspVoxelTemplate grasp_template;
        grasp_template.required_occupied = graphPoints(*required_graph_);
        // 最大開口内部の把持対象予定体積
        // 指との接触を許容する開口内部の掃引衝突判定除外
        // 開口外側の掃引体積だけによる衝突制約
        grasp_template.collision_exempt = grasp_template.required_occupied;
        grasp_template.lateral_continuation_pairs = makeLateralContinuationPairs(
          grasp_template.required_occupied, contact_closing_axis_);
        if (undersize_graph_) {
          grasp_template.optional_not_sole_support = graphPoints(*undersize_graph_);
          const auto contacts = makeOpposingContactBands(
            grasp_template.required_occupied,
            grasp_template.optional_not_sole_support,
            contact_closing_axis_);
          grasp_template.opposing_contact_pairs = std::move(contacts.pairs);
        }
        if (forbidden_graph_) {
          grasp_template.required_empty = graphPoints(*forbidden_graph_);
        }
        compiled_template_ = std::make_unique<candidate::CompiledGraspVoxelTemplate>(
          candidate::GraspVoxelMatcher::compile(
            grasp_template, orientations_, object_voxels_->voxel_size));
        template_dirty_ = false;
      }
      auto target_grid = makeGrid(*object_voxels_);
      auto collision_grid = makeCollisionGrid(*environment_msg);
      auto result = candidate::GraspVoxelMatcher::match(
        *target_grid, *collision_grid, *compiled_template_, match_config_,
        depth_visibility_filter);
      const std::size_t object_voxel_count = target_grid->size();
      const std::size_t environment_voxel_count = environment_msg->data.size();
      const std::size_t collision_voxel_count = collision_grid->size();
      const candidate::VoxelGridGeometry geometry{
        object_voxels_->voxel_size,
        Eigen::Vector3d(
          object_voxels_->origin_x, object_voxels_->origin_y, object_voxels_->origin_z)};
      if (surface_normals_dirty_ || !surface_normals_ ||
        !surface_normals_->geometry().matches(geometry))
      {
        rebuildSurfaceNormals(geometry);
      }
      if (has_surface_normal_context_) {
        candidate::GraspVoxelMatcher::annotateContactNormals(
          result, *compiled_template_, *surface_normals_, contact_closing_axis_);
      }
      is_depth_dirty_ = false;
      const double calc_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();

      const auto pub_started = std::chrono::steady_clock::now();
      publishCandidates(result);
      publishCandidateVoxels(result);
      publishSummary(
        result, object_voxel_count, environment_voxel_count, collision_voxel_count,
        calc_ms);
      const double pub_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - pub_started).count();
      const double total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Grasp voxel match: objects=%zu anchors=%zu poses=%zu accepted=%zu calc=%.2fms pub=%.2fms total=%.2fms",
        object_voxel_count,
        result.anchors_considered, result.poses_evaluated,
        result.candidates.size(), calc_ms, pub_ms, total_ms);
    } catch (const std::exception &error) {
      if (enable_depth_visibility_) {
        publishEmptyCandidates();
      }
      publishStatus("error", error.what());
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Grasp voxel matching failed: %s", error.what());
    }
  }

  void publishCandidates(const candidate::GraspVoxelMatchResult &result)
  {
    geometry_msgs::msg::PoseArray msg;
    msg.header = object_voxels_->header;
    msg.poses.reserve(result.candidates.size());
    for (const auto &candidate_result : result.candidates) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = candidate_result.tcp_position.x();
      pose.position.y = candidate_result.tcp_position.y();
      pose.position.z = candidate_result.tcp_position.z();
      pose.orientation.x = candidate_result.tcp_orientation.x();
      pose.orientation.y = candidate_result.tcp_orientation.y();
      pose.orientation.z = candidate_result.tcp_orientation.z();
      pose.orientation.w = candidate_result.tcp_orientation.w();
      msg.poses.push_back(std::move(pose));
    }
    candidate_pub_->publish(msg);
  }

  void publishCandidateVoxels(const candidate::GraspVoxelMatchResult &result)
  {
    voxel_msgs::msg::Voxel msg;
    msg.header = object_voxels_->header;
    msg.voxel_size = object_voxels_->voxel_size;
    msg.origin_x = object_voxels_->origin_x;
    msg.origin_y = object_voxels_->origin_y;
    msg.origin_z = object_voxels_->origin_z;
    msg.x_shift = object_voxels_->x_shift;
    msg.y_shift = object_voxels_->y_shift;
    msg.z_shift = object_voxels_->z_shift;
    msg.offset = object_voxels_->offset;
    msg.revision = object_voxels_->revision;
    msg.data.reserve(result.candidates.size());
    msg.labels.reserve(result.candidates.size());
    const auto indexing = makeIndexing(
      msg.voxel_size, msg.x_shift, msg.y_shift, msg.z_shift, msg.offset);
    for (const auto &candidate_result : result.candidates) {
      msg.data.push_back(static_cast<std::int64_t>(indexing.pack(candidate_result.tcp_cell)));
      msg.labels.push_back(1U);
    }
    candidate_voxels_pub_->publish(msg);
  }

  void publishSummary(
    const candidate::GraspVoxelMatchResult &result,
    std::size_t object_voxel_count,
    std::size_t environment_voxel_count,
    std::size_t collision_voxel_count,
    double processing_ms)
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "{\"status\":\"ready\""
           << ",\"object_voxel_count\":" << object_voxel_count
           << ",\"environment_voxel_count\":" << environment_voxel_count
           << ",\"collision_voxel_count\":" << collision_voxel_count
           << ",\"orientation_count\":" << orientations_.size()
           << ",\"anchors_considered\":" << result.anchors_considered
           << ",\"poses_evaluated\":" << result.poses_evaluated
           << ",\"raw_candidate_num\":" << result.raw_candidate_num
           << ",\"candidate_cell_num\":" << result.candidate_cell_num
           << ",\"suppressed_same_tcp_cell_num\":"
           << result.suppressed_same_tcp_cell_num
           << ",\"candidate_count\":" << result.candidates.size()
           << ",\"surface_normal_context\":"
           << (has_surface_normal_context_ ? "true" : "false")
           << ",\"surface_normal_cell_num\":" << surface_normal_cell_num_
           << ",\"planar_cluster_context\":"
           << (has_planar_cluster_context_ ? "true" : "false")
           << ",\"planar_collision_context\":"
           << (has_planar_collision_context_ ? "true" : "false")
           << ",\"planar_collision_cell_num\":" << planar_collision_cell_num_
           << ",\"depth_visibility_enabled\":"
           << (enable_depth_visibility_ ? "true" : "false")
           << ",\"depth_visibility_ready\":"
           << (isDepthVisibilityReady() ? "true" : "false")
           << ",\"rejected_required_occupancy\":"
           << result.rejected_required_occupancy
           << ",\"rejected_undersize_only\":" << result.rejected_undersize_only
           << ",\"rejected_missing_opposing_contact\":"
           << result.rejected_missing_opposing_contact
           << ",\"rejected_lateral_continuation\":"
           << result.rejected_lateral_continuation
           << ",\"rejected_forbidden_occupancy\":"
           << result.rejected_forbidden_occupancy
           << ",\"rejected_visibility\":" << result.rejected_visibility
           << ",\"processing_ms\":" << processing_ms
           << ",\"candidates\":[";
    for (std::size_t index = 0; index < result.candidates.size(); ++index) {
      if (index > 0) {
        stream << ",";
      }
      const auto &candidate_result = result.candidates[index];
      stream << "{\"index\":" << index
             << ",\"score\":" << candidate_result.score
             << ",\"required_hits\":" << candidate_result.required_hits
             << ",\"required_samples\":" << candidate_result.required_samples
             << ",\"required_occupancy_ratio\":"
             << candidate_result.required_occupancy_ratio
             << ",\"outside_undersize_hits\":"
             << candidate_result.outside_undersize_hits
             << ",\"positive_contact_hits\":"
             << candidate_result.positive_contact_hits
             << ",\"negative_contact_hits\":"
             << candidate_result.negative_contact_hits
             << ",\"selected_contact_pair\":"
             << candidate_result.selected_contact_pair
             << ",\"lateral_continuation_axes\":"
             << candidate_result.lateral_continuation_axes
             << ",\"forbidden_hits\":" << candidate_result.forbidden_hits
             << ",\"anchor\":[" << candidate_result.anchor.x << ","
             << candidate_result.anchor.y << "," << candidate_result.anchor.z << "]"
             << ",\"tcp_cell\":[" << candidate_result.tcp_cell.x << ","
             << candidate_result.tcp_cell.y << "," << candidate_result.tcp_cell.z << "]"
             << ",\"orientation_index\":" << candidate_result.orientation_index
             << ",\"tcp_position\":[" << candidate_result.tcp_position.x() << ","
             << candidate_result.tcp_position.y() << ","
             << candidate_result.tcp_position.z() << "]"
             << ",\"contact_normal_alignment\":"
             << candidate_result.contact_normal_alignment
             << ",\"planar_contact_ratio\":" << candidate_result.planar_contact_ratio
             << ",\"positive_normal_num\":" << candidate_result.positive_normal_num
             << ",\"negative_normal_num\":" << candidate_result.negative_normal_num
             << ",\"positive_planar_normal_num\":"
             << candidate_result.positive_planar_normal_num
             << ",\"negative_planar_normal_num\":"
             << candidate_result.negative_planar_normal_num
             << "}";
    }
    stream << "]}";
    msg.data = stream.str();
    summary_pub_->publish(msg);
  }

  void publishStatus(const char *status, const std::string &detail)
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "{\"status\":\"" << status << "\",\"detail\":\"";
    for (const char value : detail) {
      if (value == '\\' || value == '"') {
        stream << '\\';
      }
      stream << value;
    }
    stream << "\"}";
    msg.data = stream.str();
    summary_pub_->publish(msg);
  }

  std::string object_voxels_topic_;
  std::string environment_voxels_topic_;
  std::string required_graph_topic_;
  std::string undersize_graph_topic_;
  std::string forbidden_graph_topic_;
  std::string topological_map_topic_;
  std::string planar_clusters_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  candidate::GraspVoxelMatchConfig match_config_;
  Eigen::Vector3d contact_closing_axis_ = Eigen::Vector3d::UnitY();
  std::vector<Eigen::Quaterniond> orientations_;
  bool dirty_ = true;
  bool template_dirty_ = true;
  bool surface_normals_dirty_ = true;
  bool planar_collision_dirty_ = true;
  bool has_surface_normal_context_ = false;
  bool has_planar_cluster_context_ = false;
  bool has_planar_collision_context_ = false;
  bool enable_depth_visibility_ = true;
  bool is_depth_dirty_ = true;
  std::size_t surface_normal_cell_num_ = 0;
  std::size_t planar_collision_cell_num_ = 0;
  std::size_t max_depth_cache_num_ = 0;
  std::unique_ptr<candidate::CompiledGraspVoxelTemplate> compiled_template_;
  std::unique_ptr<candidate::VoxelSurfaceNormalGrid> surface_normals_;
  std::unordered_set<candidate::VoxelIndex, candidate::VoxelIndexHash> planar_collision_cells_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  voxel_msgs::msg::Voxel::SharedPtr object_voxels_;
  voxel_msgs::msg::Voxel::SharedPtr environment_voxels_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr required_graph_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr undersize_graph_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr forbidden_graph_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr topological_map_;
  ais_gng_msgs::msg::PlanarClusterArray::SharedPtr planar_clusters_;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  std::map<std::int64_t, sensor_msgs::msg::Image::SharedPtr> depth_image_cache_;

  rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr object_voxels_sub_;
  rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr environment_voxels_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr required_graph_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr undersize_graph_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr forbidden_graph_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr topological_map_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::PlanarClusterArray>::SharedPtr planar_clusters_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr candidate_pub_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr candidate_voxels_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // grasping_system::nodes 名前空間

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grasping_system::nodes::GraspVoxelMatcherNode>());
  rclcpp::shutdown();
  return 0;
}
