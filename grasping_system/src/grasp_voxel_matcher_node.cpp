#include <candidate/grasp_voxel_matcher.hpp>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <voxel_msgs/msg/voxel.hpp>
#include <voxel_msgs/msg/voxel_label_delta.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
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
    incremental_matching_enabled_ = declare_parameter<bool>(
      "incremental_matching_enabled", true);
    object_delta_topic_ = declare_parameter<std::string>("object_delta_topic", "");
    if (object_delta_topic_.empty()) {
      object_delta_topic_ = object_voxels_topic_ + "/delta";
    }
    environment_delta_topic_ = declare_parameter<std::string>("environment_delta_topic", "");
    if (environment_delta_topic_.empty() && !environment_voxels_topic_.empty()) {
      environment_delta_topic_ = environment_voxels_topic_ + "/delta";
    }
    required_graph_topic_ = declare_parameter<std::string>(
      "required_graph_topic", "grip_V_topological_map");
    undersize_graph_topic_ = declare_parameter<std::string>(
      "undersize_graph_topic", "grip_minV_topological_map");
    forbidden_graph_topic_ = declare_parameter<std::string>(
      "forbidden_graph_topic", "grip_baseV_topological_map");
    const std::string candidate_topic = declare_parameter<std::string>(
      "candidate_topic", "/grasp_voxel_candidates");
    const std::string summary_topic = declare_parameter<std::string>(
      "summary_topic", "/grasp_voxel_candidates/summary");

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
    summary_pub_ = create_publisher<std_msgs::msg::String>(summary_topic, transient_qos);

    object_voxels_sub_ = create_subscription<voxel_msgs::msg::Voxel>(
      object_voxels_topic_, transient_qos,
      [this](voxel_msgs::msg::Voxel::SharedPtr msg) {
        objectSnapshotCallback(std::move(msg));
      });
    if (incremental_matching_enabled_) {
      object_delta_sub_ = create_subscription<voxel_msgs::msg::VoxelLabelDelta>(
        object_delta_topic_, rclcpp::QoS(10).reliable(),
        [this](voxel_msgs::msg::VoxelLabelDelta::SharedPtr msg) {
          applyDelta(*msg, true, !usesSeparateEnvironment());
        });
    }
    if (!environment_voxels_topic_.empty() &&
      environment_voxels_topic_ != object_voxels_topic_)
    {
      environment_voxels_sub_ = create_subscription<voxel_msgs::msg::Voxel>(
        environment_voxels_topic_, transient_qos,
        [this](voxel_msgs::msg::Voxel::SharedPtr msg) {
          environmentSnapshotCallback(std::move(msg));
        });
      if (incremental_matching_enabled_) {
        environment_delta_sub_ = create_subscription<voxel_msgs::msg::VoxelLabelDelta>(
          environment_delta_topic_, rclcpp::QoS(10).reliable(),
          [this](voxel_msgs::msg::VoxelLabelDelta::SharedPtr msg) {
            applyDelta(*msg, false, true);
          });
      }
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

    timer_ = create_wall_timer(
      std::chrono::milliseconds(update_period_ms),
      std::bind(&GraspVoxelMatcherNode::updateMatches, this));

    RCLCPP_INFO(
      get_logger(),
      "Grasp voxel matcher ready: objects=%s delta=%s environment=%s required=%s undersize=%s forbidden=%s orientations=%zu period=%dms incremental=%s",
      object_voxels_topic_.c_str(),
      object_delta_topic_.c_str(),
      environment_voxels_topic_.empty() ? "<object voxels>" : environment_voxels_topic_.c_str(),
      required_graph_topic_.c_str(),
      undersize_graph_topic_.empty() ? "<disabled>" : undersize_graph_topic_.c_str(),
      forbidden_graph_topic_.empty() ? "<disabled>" : forbidden_graph_topic_.c_str(),
      orientations_.size(), update_period_ms,
      incremental_matching_enabled_ ? "enabled" : "disabled");
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
    if (!incremental_matching_enabled_ || object_voxels_->revision == 0U ||
      !object_snapshot_initialized_ ||
      object_voxels_->revision < object_revision_ ||
      object_voxels_->revision > object_revision_ + 1U)
    {
      object_snapshot_needs_reset_ = true;
      dirty_ = true;
    }
  }

  void environmentSnapshotCallback(voxel_msgs::msg::Voxel::SharedPtr msg)
  {
    environment_voxels_ = std::move(msg);
    if (!incremental_matching_enabled_ || environment_voxels_->revision == 0U ||
      !environment_snapshot_initialized_ ||
      environment_voxels_->revision < environment_revision_ ||
      environment_voxels_->revision > environment_revision_ + 1U)
    {
      environment_snapshot_needs_reset_ = true;
      dirty_ = true;
    }
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

  void applyDelta(
    const voxel_msgs::msg::VoxelLabelDelta &msg,
    bool update_target,
    bool update_collision)
  {
    constexpr std::uint8_t kAbsentLabel = 255U;
    try {
      if (msg.revision == 0U || msg.data.size() != msg.old_labels.size() ||
        msg.data.size() != msg.new_labels.size())
      {
        throw std::invalid_argument("voxel delta fields are not aligned");
      }
      if (!incremental_matcher_ ||
        (update_target && (!object_snapshot_initialized_ || object_snapshot_needs_reset_)) ||
        (update_collision && usesSeparateEnvironment() &&
        (!environment_snapshot_initialized_ || environment_snapshot_needs_reset_)))
      {
        dirty_ = true;
        return;
      }

      std::uint32_t &last_revision = update_target ? object_revision_ : environment_revision_;
      if (msg.revision <= last_revision) {
        return;
      }
      if (msg.revision != last_revision + 1U) {
        if (update_target) {
          object_snapshot_needs_reset_ = true;
        }
        if (update_collision && usesSeparateEnvironment()) {
          environment_snapshot_needs_reset_ = true;
        }
        dirty_ = true;
        return;
      }

      const auto indexing = makeIndexing(
        msg.voxel_size, msg.x_shift, msg.y_shift, msg.z_shift, msg.offset);
      bool occupancy_changed = false;
      for (std::size_t index = 0; index < msg.data.size(); ++index) {
        const auto cell = indexing.unpack(static_cast<std::uint64_t>(msg.data[index]));
        const bool old_occupied = msg.old_labels[index] != kAbsentLabel;
        const bool new_occupied = msg.new_labels[index] != kAbsentLabel;
        if (update_target) {
          occupancy_changed = incremental_matcher_->applyTargetDelta(
            cell, old_occupied, new_occupied) || occupancy_changed;
        }
        if (update_collision) {
          occupancy_changed = incremental_matcher_->applyCollisionDelta(
            cell, old_occupied, new_occupied) || occupancy_changed;
        }
      }
      last_revision = msg.revision;
      dirty_ = dirty_ || occupancy_changed;
    } catch (const std::exception &error) {
      if (update_target) {
        object_snapshot_needs_reset_ = true;
      }
      if (update_collision && usesSeparateEnvironment()) {
        environment_snapshot_needs_reset_ = true;
      }
      dirty_ = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Incremental voxel update discarded; a full snapshot will resynchronize: %s",
        error.what());
    }
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
    return {};
  }

  void updateMatches()
  {
    if (!dirty_) {
      return;
    }
    const std::string waiting_for = waitingReason();
    if (!waiting_for.empty()) {
      publishStatus("waiting", waiting_for);
      return;
    }
    dirty_ = false;

    try {
      const auto started = std::chrono::steady_clock::now();
      const auto environment_msg = environment_voxels_ ? environment_voxels_ : object_voxels_;
      bool template_rebuilt = false;
      if (!compiled_template_ || template_dirty_ ||
        std::abs(compiled_template_->voxel_size - object_voxels_->voxel_size) > 1e-9)
      {
        candidate::GraspVoxelTemplate grasp_template;
        grasp_template.required_occupied = graphPoints(*required_graph_);
        // The maximum-open interior is the prospective object volume.  It is
        // intentionally exempt from a finger-closing sweep: an object is
        // allowed to meet a finger there.  Only the swept volume outside this
        // gap remains a collision constraint.
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
        template_rebuilt = true;
      }
      const bool incremental_stream = incremental_matching_enabled_ &&
        object_voxels_->revision != 0U &&
        (!usesSeparateEnvironment() || environment_msg->revision != 0U);
      bool incremental_active = false;
      std::size_t object_voxel_count = 0;
      std::size_t environment_voxel_count = 0;
      candidate::GraspVoxelMatchResult result;
      if (incremental_stream) {
        if (!incremental_matcher_ || template_rebuilt || object_snapshot_needs_reset_ ||
          (usesSeparateEnvironment() && environment_snapshot_needs_reset_))
        {
          auto target_grid = makeGrid(*object_voxels_);
          auto collision_grid = makeGrid(*environment_msg);
          incremental_matcher_ = std::make_unique<candidate::IncrementalGraspVoxelMatcher>(
            target_grid->geometry(), *compiled_template_, match_config_);
          incremental_matcher_->reset(*target_grid, *collision_grid);
          object_revision_ = object_voxels_->revision;
          object_snapshot_initialized_ = true;
          object_snapshot_needs_reset_ = false;
          if (usesSeparateEnvironment()) {
            environment_revision_ = environment_msg->revision;
            environment_snapshot_initialized_ = true;
            environment_snapshot_needs_reset_ = false;
          } else {
            environment_revision_ = object_revision_;
          }
        }
        result = incremental_matcher_->match();
        object_voxel_count = incremental_matcher_->targetVoxelCount();
        environment_voxel_count = incremental_matcher_->collisionVoxelCount();
        incremental_active = true;
      } else {
        auto target_grid = makeGrid(*object_voxels_);
        auto collision_grid = makeGrid(*environment_msg);
        result = candidate::GraspVoxelMatcher::match(
          *target_grid, *collision_grid, *compiled_template_, match_config_);
        object_voxel_count = target_grid->size();
        environment_voxel_count = collision_grid->size();
      }
      const double processing_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();

      publishCandidates(result);
      publishSummary(
        result, object_voxel_count, environment_voxel_count, processing_ms, incremental_active);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Grasp voxel match: mode=%s objects=%zu anchors=%zu poses=%zu changed_states=%zu accepted=%zu processing=%.2fms",
        incremental_active ? "incremental" : "full", object_voxel_count,
        result.anchors_considered, result.poses_evaluated, result.candidate_states_updated,
        result.candidates.size(), processing_ms);
    } catch (const std::exception &error) {
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

  void publishSummary(
    const candidate::GraspVoxelMatchResult &result,
    std::size_t object_voxel_count,
    std::size_t environment_voxel_count,
    double processing_ms,
    bool incremental_active)
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "{\"status\":\"ready\""
           << ",\"object_voxel_count\":" << object_voxel_count
           << ",\"environment_voxel_count\":" << environment_voxel_count
           << ",\"matching_mode\":\""
           << (incremental_active ? "incremental" : "full") << "\""
           << ",\"orientation_count\":" << orientations_.size()
           << ",\"anchors_considered\":" << result.anchors_considered
           << ",\"poses_evaluated\":" << result.poses_evaluated
           << ",\"candidate_states_tracked\":" << result.candidate_states_tracked
           << ",\"candidate_states_updated\":" << result.candidate_states_updated
           << ",\"candidate_count\":" << result.candidates.size()
           << ",\"rejected_required_occupancy\":"
           << result.rejected_required_occupancy
           << ",\"rejected_undersize_only\":" << result.rejected_undersize_only
           << ",\"rejected_missing_opposing_contact\":"
           << result.rejected_missing_opposing_contact
           << ",\"rejected_lateral_continuation\":"
           << result.rejected_lateral_continuation
           << ",\"rejected_forbidden_occupancy\":"
           << result.rejected_forbidden_occupancy
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
             << ",\"orientation_index\":" << candidate_result.orientation_index
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
  std::string object_delta_topic_;
  std::string environment_delta_topic_;
  std::string required_graph_topic_;
  std::string undersize_graph_topic_;
  std::string forbidden_graph_topic_;
  candidate::GraspVoxelMatchConfig match_config_;
  Eigen::Vector3d contact_closing_axis_ = Eigen::Vector3d::UnitY();
  std::vector<Eigen::Quaterniond> orientations_;
  bool incremental_matching_enabled_ = true;
  bool dirty_ = true;
  bool template_dirty_ = true;
  bool object_snapshot_initialized_ = false;
  bool environment_snapshot_initialized_ = false;
  bool object_snapshot_needs_reset_ = true;
  bool environment_snapshot_needs_reset_ = true;
  std::uint32_t object_revision_ = 0;
  std::uint32_t environment_revision_ = 0;
  std::unique_ptr<candidate::CompiledGraspVoxelTemplate> compiled_template_;
  std::unique_ptr<candidate::IncrementalGraspVoxelMatcher> incremental_matcher_;

  voxel_msgs::msg::Voxel::SharedPtr object_voxels_;
  voxel_msgs::msg::Voxel::SharedPtr environment_voxels_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr required_graph_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr undersize_graph_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr forbidden_graph_;

  rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr object_voxels_sub_;
  rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr environment_voxels_sub_;
  rclcpp::Subscription<voxel_msgs::msg::VoxelLabelDelta>::SharedPtr object_delta_sub_;
  rclcpp::Subscription<voxel_msgs::msg::VoxelLabelDelta>::SharedPtr environment_delta_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr required_graph_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr undersize_graph_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr forbidden_graph_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr candidate_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace grasping_system::nodes

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grasping_system::nodes::GraspVoxelMatcherNode>());
  rclcpp::shutdown();
  return 0;
}
