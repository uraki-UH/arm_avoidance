#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "nodes/bridge/persistent_depth_world_index.hpp"
#include "nodes/bridge/reachability_voxel_accumulator.hpp"
#include "safety_engine/indexing/voxel_id_codec.hpp"

namespace robot_sim::bridge
{

class depth_world_index_benchmark_node : public rclcpp::Node
{
public:
  depth_world_index_benchmark_node()
  : Node("depth_world_index_benchmark"),
    codec_(declare_parameter<double>("voxel_size", 0.02))
  {
    const std::string depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera/camera/depth/image_rect_raw");
    const std::string camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera/depth/camera_info");
    frame_num_ = static_cast<int>(
      std::max<std::int64_t>(0, declare_parameter<int>("frame_num", 60)));
    robot_spacing_x_ = declare_parameter<double>("robot_spacing_x", 0.0);
    robot_yaw_step_rad_ =
      declare_parameter<double>("robot_yaw_step_deg", 0.0) * std::acos(-1.0) / 180.0;
    robot_num_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
      declare_parameter<int>("robot_num", 1), 1, static_cast<std::int64_t>(max_robot_num)));
    enable_comparison_benchmark_ = declare_parameter<bool>(
      "enable_comparison_benchmark", false);
    enable_runtime_log_ = declare_parameter<bool>("enable_runtime_log", false);
    enable_debug_publish_ = declare_parameter<bool>("enable_debug_publish", false);
    debug_roi_voxel_topic_ = declare_parameter<std::string>(
      "debug_roi_voxel_topic", "/depth_world_index/debug/roi_voxels");
    debug_world_bucket_voxel_topic_ = declare_parameter<std::string>(
      "debug_world_bucket_voxel_topic", "/depth_world_index/debug/world_buckets_voxels");
    debug_frame_id_ = declare_parameter<std::string>("debug_frame_id", "");

    codec_.setIndexingParams(
      declare_parameter<int>("x_shift", 42),
      declare_parameter<int>("y_shift", 21),
      declare_parameter<int>("z_shift", 0),
      declare_parameter<long>("offset", 1000000L));

    bounds_.enable_filter = true;
    bounds_.min_corner = Eigen::Vector3d(
      declare_parameter<double>("min_reachability_x", -0.25),
      declare_parameter<double>("min_reachability_y", -0.25),
      declare_parameter<double>("min_reachability_z", 0.4));
    bounds_.max_corner = Eigen::Vector3d(
      declare_parameter<double>("max_reachability_x", 0.25),
      declare_parameter<double>("max_reachability_y", 0.25),
      declare_parameter<double>("max_reachability_z", 1.6));
    bounds_.margin = Eigen::Vector3d(
      declare_parameter<double>("reachability_margin_x", 0.1),
      declare_parameter<double>("reachability_margin_y", 0.1),
      declare_parameter<double>("reachability_margin_z", 0.1));
    bounds_.validate();

    const std::size_t max_dense_voxel_num = static_cast<std::size_t>(
      std::max<std::int64_t>(
        0, declare_parameter<int>("max_dense_voxel_num", 8000000)));
    const std::size_t query_robot_num =
      enable_comparison_benchmark_ ? max_robot_num : robot_num_;
    for (std::size_t robot_idx = 0U; robot_idx < query_robot_num; ++robot_idx) {
      persistent_accumulators_.push_back(std::make_unique<reachability_voxel_accumulator>(
        codec_, bounds_, max_dense_voxel_num));
      if (enable_comparison_benchmark_) {
        direct_accumulators_.push_back(std::make_unique<reachability_voxel_accumulator>(
          codec_, bounds_, max_dense_voxel_num));
      }
    }

    persistent_depth_world_index_config index_config;
    index_config.bucket_size = declare_parameter<double>("bucket_size", 0.2);
    index_config.depth_scale = static_cast<float>(
      declare_parameter<double>("depth_scale", 0.001));
    index_config.depth_update_mm_th = static_cast<std::uint16_t>(std::clamp<std::int64_t>(
      declare_parameter<int>("depth_update_mm_th", 1), 0, 65535));
    index_config.free_confirmation_num = static_cast<std::uint16_t>(
      std::clamp<std::int64_t>(
        declare_parameter<int>("free_confirmation_num", 3), 1, 65535));
    index_ = std::make_unique<persistent_depth_world_index>(index_config);
    world_bucket_codec_.setVoxelSize(index_->bucket_size());
    world_bucket_codec_.setIndexingParams(
      codec_.xShift(), codec_.yShift(), codec_.zShift(), codec_.offset());

    camera_to_world_ = make_transform(
      declare_parameter<double>("camera_world_x", 0.0),
      declare_parameter<double>("camera_world_y", 0.0),
      declare_parameter<double>("camera_world_z", 0.0),
      declare_parameter<double>("camera_world_roll_deg", 0.0),
      declare_parameter<double>("camera_world_pitch_deg", 0.0),
      declare_parameter<double>("camera_world_yaw_deg", 0.0));

    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      std::bind(&depth_world_index_benchmark_node::depth_callback, this, std::placeholders::_1));
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      std::bind(
        &depth_world_index_benchmark_node::camera_info_callback, this,
        std::placeholders::_1));
    if (enable_debug_publish_) {
      rclcpp::QoS debug_qos(1);
      debug_roi_voxel_publisher_ = create_publisher<voxel_msgs::msg::Voxel>(
        debug_roi_voxel_topic_, debug_qos.reliable().transient_local());
      debug_world_bucket_voxel_publisher_ = create_publisher<voxel_msgs::msg::Voxel>(
        debug_world_bucket_voxel_topic_, debug_qos.reliable().transient_local());
    }

    RCLCPP_INFO(
      get_logger(),
      "depth persistent index開始: depth=%s camera_info=%s frame_num=%d robot_num=%zu comparison_benchmark=%s runtime_log=%s bucket_size=%.3f voxel_size=%.3f update_mm_th=%u free_confirmation_num=%u debug_publish=%s",
      depth_topic.c_str(), camera_info_topic.c_str(), frame_num_, robot_num_,
      enable_comparison_benchmark_ ? "true" : "false", enable_runtime_log_ ? "true" : "false",
      index_->bucket_size(), codec_.voxelSize(), index_config.depth_update_mm_th,
      index_config.free_confirmation_num,
      enable_debug_publish_ ? "true" : "false");
  }

private:
  using steady_clock = std::chrono::steady_clock;

  static double elapsed_ms(steady_clock::time_point start)
  {
    return std::chrono::duration<double, std::milli>(steady_clock::now() - start).count();
  }

  static Eigen::Isometry3f make_transform(
    double x,
    double y,
    double z,
    double roll_deg,
    double pitch_deg,
    double yaw_deg)
  {
    const float degree_to_rad = static_cast<float>(std::acos(-1.0) / 180.0);
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    transform.translation() = Eigen::Vector3f(
      static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    transform.linear() = (
      Eigen::AngleAxisf(static_cast<float>(yaw_deg) * degree_to_rad, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(static_cast<float>(pitch_deg) * degree_to_rad, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(static_cast<float>(roll_deg) * degree_to_rad, Eigen::Vector3f::UnitX())
    ).toRotationMatrix();
    return transform;
  }

  Eigen::Isometry3d robot_to_world(std::size_t robot_idx) const
  {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation().x() = static_cast<double>(robot_idx) * robot_spacing_x_;
    transform.linear() = Eigen::AngleAxisd(
      static_cast<double>(robot_idx) * robot_yaw_step_rad_,
      Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return transform;
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> world_query_bounds(
    const Eigen::Isometry3d &robot_to_world_transform) const
  {
    const Eigen::Vector3d min_local = bounds_.min_corner - bounds_.margin;
    const Eigen::Vector3d max_local = bounds_.max_corner + bounds_.margin;
    Eigen::Vector3d min_world = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
    Eigen::Vector3d max_world = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
    for (int corner_idx = 0; corner_idx < 8; ++corner_idx) {
      Eigen::Vector3d corner;
      for (int axis_idx = 0; axis_idx < 3; ++axis_idx) {
        corner[axis_idx] = (corner_idx & (1 << axis_idx)) ?
          max_local[axis_idx] : min_local[axis_idx];
      }
      const Eigen::Vector3d world_corner = robot_to_world_transform * corner;
      min_world = min_world.cwiseMin(world_corner);
      max_world = max_world.cwiseMax(world_corner);
    }
    return {min_world, max_world};
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    depth_camera_intrinsics intrinsics;
    intrinsics.width = msg->width;
    intrinsics.height = msg->height;
    intrinsics.fx = static_cast<float>(msg->k[0]);
    intrinsics.fy = static_cast<float>(msg->k[4]);
    intrinsics.cx = static_cast<float>(msg->k[2]);
    intrinsics.cy = static_cast<float>(msg->k[5]);
    try {
      const bool is_changed = index_->set_camera_intrinsics(intrinsics);
      if (is_changed) {
        RCLCPP_INFO(
          get_logger(),
          "depth camera model設定: width=%u height=%u fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
          intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy,
          intrinsics.cx, intrinsics.cy);
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "camera_info設定失敗: %s", error.what());
    }
  }

  static bool extract_depth_mm(
    const sensor_msgs::msg::Image &image,
    std::vector<std::uint16_t> &depth_mm)
  {
    if (image.encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
      image.encoding != sensor_msgs::image_encodings::MONO16)
    {
      return false;
    }
    const std::size_t min_step = static_cast<std::size_t>(image.width) * sizeof(std::uint16_t);
    if (image.step < min_step || image.data.size() < image.step * image.height) {
      return false;
    }
    depth_mm.resize(static_cast<std::size_t>(image.width) * image.height);
    for (std::size_t y = 0U; y < image.height; ++y) {
      const std::uint8_t *row = image.data.data() + y * image.step;
      for (std::size_t x = 0U; x < image.width; ++x) {
        std::memcpy(
          &depth_mm[y * image.width + x], row + x * sizeof(std::uint16_t),
          sizeof(std::uint16_t));
      }
    }
    return true;
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!index_->intrinsics().pixel_num()) {
      if (!has_reported_missing_camera_info_) {
        RCLCPP_WARN(get_logger(), "camera_info待機中のためdepth画像を保留");
        has_reported_missing_camera_info_ = true;
      }
      return;
    }
    if (msg->width != index_->intrinsics().width || msg->height != index_->intrinsics().height) {
      RCLCPP_WARN(
        get_logger(),
        "depth画像サイズ不一致: depth=%ux%u camera_info=%ux%u",
        msg->width, msg->height, index_->intrinsics().width, index_->intrinsics().height);
      return;
    }

    std::vector<std::uint16_t> depth_mm;
    if (!extract_depth_mm(*msg, depth_mm)) {
      RCLCPP_WARN(get_logger(), "未対応または破損したdepth画像: encoding=%s", msg->encoding.c_str());
      return;
    }

    try {
      const bool is_timing_enabled =
        frame_num_ > 0 || enable_comparison_benchmark_ || enable_runtime_log_;
      const auto update_start = is_timing_enabled ? steady_clock::now() : steady_clock::time_point{};
      const persistent_depth_world_index_update_stats update_stats = index_->update(
        depth_mm, camera_to_world_);
      const double update_ms = is_timing_enabled ? elapsed_ms(update_start) : 0.0;
      const std::size_t changed_point_num =
        update_stats.inserted_point_num + update_stats.updated_point_num +
        update_stats.removed_point_num;
      if (frame_num_ > 0) {
        update_ms_samples_.push_back(update_ms);
        changed_point_num_samples_.push_back(changed_point_num);
        deferred_free_point_num_ += update_stats.deferred_free_point_num;
        accumulated_inserted_point_num_ += update_stats.inserted_point_num;
        accumulated_updated_point_num_ += update_stats.updated_point_num;
        accumulated_removed_point_num_ += update_stats.removed_point_num;
      }

      std::vector<std::vector<long>> direct_voxel_ids;
      if (enable_comparison_benchmark_) {
        direct_voxel_ids.resize(max_robot_num);
        const auto direct_start = steady_clock::now();
        std::size_t count_idx = 0U;
        for (std::size_t robot_idx = 0U; robot_idx < max_robot_num; ++robot_idx) {
          const Eigen::Isometry3d world_to_robot = robot_to_world(robot_idx).inverse();
          auto &accumulator = *direct_accumulators_[robot_idx];
          accumulator.begin_frame(depth_mm.size());
          for (std::size_t pixel_idx = 0U; pixel_idx < depth_mm.size(); ++pixel_idx) {
            if (depth_mm[pixel_idx] == 0U) {
              continue;
            }
            accumulator.add_point(
              index_->deproject(pixel_idx, depth_mm[pixel_idx], camera_to_world_).cast<double>(),
              world_to_robot);
          }
          direct_voxel_ids[robot_idx] = accumulator.finish_voxel_ids();
          if (robot_idx + 1U == robot_counts_[count_idx]) {
            if (frame_num_ > 0) {
              direct_ms_samples_[count_idx].push_back(elapsed_ms(direct_start));
            }
            ++count_idx;
          }
        }
      }

      const auto query_start = is_timing_enabled ? steady_clock::now() : steady_clock::time_point{};
      std::size_t count_idx = 0U;
      std::size_t candidate_point_num = 0U;
      std::size_t accepted_point_num = 0U;
      std::vector<long> debug_roi_voxel_ids;
      const std::size_t query_robot_num =
        enable_comparison_benchmark_ ? max_robot_num : robot_num_;
      for (std::size_t robot_idx = 0U; robot_idx < query_robot_num; ++robot_idx) {
        const Eigen::Isometry3d robot_to_world_transform = robot_to_world(robot_idx);
        const Eigen::Isometry3d world_to_robot = robot_to_world_transform.inverse();
        const auto [min_world, max_world] = world_query_bounds(robot_to_world_transform);
        auto &accumulator = *persistent_accumulators_[robot_idx];
        accumulator.begin_frame(index_->point_num());
        const world_bucket_query_stats query_stats = index_->query_aabb(
          min_world, max_world,
          [&accumulator, &world_to_robot](const Eigen::Vector3f &point) {
            accumulator.add_point(point.cast<double>(), world_to_robot);
          });
        if (enable_runtime_log_) {
          candidate_point_num += query_stats.candidate_point_num;
          accepted_point_num += query_stats.accepted_point_num;
        }
        const std::vector<long> &persistent_voxel_ids = accumulator.finish_voxel_ids();
        if (enable_debug_publish_ && robot_idx == 0U) {
          debug_roi_voxel_ids = persistent_voxel_ids;
        }
        if (enable_comparison_benchmark_) {
          const auto [false_negative_num, false_positive_num] = count_voxel_difference(
            direct_voxel_ids[robot_idx], persistent_voxel_ids);
          false_negative_num_ += false_negative_num;
          false_positive_num_ += false_positive_num;
          direct_voxel_num_ += direct_voxel_ids[robot_idx].size();
          persistent_voxel_num_ += persistent_voxel_ids.size();
          if (false_negative_num != 0U || false_positive_num != 0U) {
            ++mismatch_num_;
          }
        }
        if (enable_comparison_benchmark_ && robot_idx + 1U == robot_counts_[count_idx]) {
          const double query_ms = elapsed_ms(query_start);
          if (frame_num_ > 0) {
            persistent_query_ms_samples_[count_idx].push_back(query_ms);
            persistent_total_ms_samples_[count_idx].push_back(update_ms + query_ms);
          }
          ++count_idx;
        }
      }
      const double query_ms = is_timing_enabled ? elapsed_ms(query_start) : 0.0;
      if (frame_num_ > 0) {
        query_ms_samples_.push_back(query_ms);
        total_ms_samples_.push_back(update_ms + query_ms);
      }
      if (enable_debug_publish_) {
        publish_debug_outputs(msg->header, debug_roi_voxel_ids);
      }

      if (frame_num_ > 0 || enable_runtime_log_) {
        ++measured_frame_num_;
      }
      if (enable_runtime_log_) {
        const double changed_ratio = depth_mm.empty() ? 0.0 :
          static_cast<double>(changed_point_num) / static_cast<double>(depth_mm.size());
        RCLCPP_INFO(
          get_logger(),
          "計測 %zu/%d: rebuild=%s pixels=%zu index_points=%zu buckets=%zu update=%.3fms query=%.3fms total=%.3fms robot_num=%zu changed=%zu changed_ratio=%.4f candidates=%zu accepted=%zu",
          measured_frame_num_, frame_num_, update_stats.is_rebuild ? "true" : "false",
          depth_mm.size(), index_->point_num(), index_->bucket_num(), update_ms,
          query_ms, update_ms + query_ms, query_robot_num, changed_point_num,
          changed_ratio, candidate_point_num, accepted_point_num);
      }

      if (frame_num_ > 0 && measured_frame_num_ >= static_cast<std::size_t>(frame_num_)) {
        report_results();
        depth_subscription_.reset();
        camera_info_subscription_.reset();
        rclcpp::shutdown();
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "depth persistent index計測失敗: %s", error.what());
      rclcpp::shutdown();
    }
  }

  void publish_debug_outputs(
    const std_msgs::msg::Header &source_header,
    const std::vector<long> &roi_voxel_ids)
  {
    std_msgs::msg::Header header = source_header;
    if (!debug_frame_id_.empty()) {
      header.frame_id = debug_frame_id_;
    }
    debug_roi_voxel_publisher_->publish(codec_.makeMessage(header, roi_voxel_ids));

    std::vector<long> world_bucket_voxel_ids;
    world_bucket_voxel_ids.reserve(index_->bucket_num());
    index_->visit_buckets([&](const world_bucket_key &key, std::size_t) {
      world_bucket_voxel_ids.push_back(world_bucket_codec_.toFlatId(
        Eigen::Vector3i(key.x, key.y, key.z)));
    });
    debug_world_bucket_voxel_publisher_->publish(
      world_bucket_codec_.makeMessage(header, world_bucket_voxel_ids));
  }

  static double percentile(std::vector<double> values, double ratio)
  {
    if (values.empty()) {
      return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t idx = static_cast<std::size_t>(
      std::round(ratio * static_cast<double>(values.size() - 1U)));
    return values[idx];
  }

  static double percentile(std::vector<std::size_t> values, double ratio)
  {
    if (values.empty()) {
      return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t idx = static_cast<std::size_t>(
      std::round(ratio * static_cast<double>(values.size() - 1U)));
    return static_cast<double>(values[idx]);
  }

  static std::pair<std::size_t, std::size_t> count_voxel_difference(
    const std::vector<long> &expected,
    const std::vector<long> &actual)
  {
    std::size_t expected_idx = 0U;
    std::size_t actual_idx = 0U;
    std::size_t false_negative_num = 0U;
    std::size_t false_positive_num = 0U;
    while (expected_idx < expected.size() && actual_idx < actual.size()) {
      if (expected[expected_idx] < actual[actual_idx]) {
        ++false_negative_num;
        ++expected_idx;
      } else if (actual[actual_idx] < expected[expected_idx]) {
        ++false_positive_num;
        ++actual_idx;
      } else {
        ++expected_idx;
        ++actual_idx;
      }
    }
    false_negative_num += expected.size() - expected_idx;
    false_positive_num += actual.size() - actual_idx;
    return {false_negative_num, false_positive_num};
  }

  void report_results() const
  {
    const std::size_t query_robot_num =
      enable_comparison_benchmark_ ? max_robot_num : robot_num_;
    RCLCPP_INFO(
      get_logger(),
      "DEPTH_PERSISTENT_RUNTIME frames=%zu robot_num=%zu update_p50_ms=%.3f update_p95_ms=%.3f query_p50_ms=%.3f query_p95_ms=%.3f total_p50_ms=%.3f total_p95_ms=%.3f changed_point_p50=%.0f changed_point_p95=%.0f inserted=%zu updated=%zu removed=%zu deferred_free=%zu",
      measured_frame_num_, query_robot_num, percentile(update_ms_samples_, 0.50),
      percentile(update_ms_samples_, 0.95), percentile(query_ms_samples_, 0.50),
      percentile(query_ms_samples_, 0.95), percentile(total_ms_samples_, 0.50),
      percentile(total_ms_samples_, 0.95), percentile(changed_point_num_samples_, 0.50),
      percentile(changed_point_num_samples_, 0.95), accumulated_inserted_point_num_,
      accumulated_updated_point_num_, accumulated_removed_point_num_, deferred_free_point_num_);
    if (!enable_comparison_benchmark_) {
      return;
    }
    const double recall = direct_voxel_num_ == 0U ? 1.0 :
      1.0 - static_cast<double>(false_negative_num_) / static_cast<double>(direct_voxel_num_);
    const double precision = persistent_voxel_num_ == 0U ? 1.0 :
      1.0 - static_cast<double>(false_positive_num_) /
      static_cast<double>(persistent_voxel_num_);
    RCLCPP_INFO(
      get_logger(),
      "DEPTH_PERSISTENT_RESULT frames=%zu update_p50_ms=%.3f update_p95_ms=%.3f changed_point_p50=%.0f changed_point_p95=%.0f inserted=%zu updated=%zu removed=%zu deferred_free=%zu mismatch_num=%zu false_negative_num=%zu false_positive_num=%zu recall=%.6f precision=%.6f",
      measured_frame_num_, percentile(update_ms_samples_, 0.50), percentile(update_ms_samples_, 0.95),
      percentile(changed_point_num_samples_, 0.50), percentile(changed_point_num_samples_, 0.95),
      accumulated_inserted_point_num_, accumulated_updated_point_num_, accumulated_removed_point_num_,
      deferred_free_point_num_, mismatch_num_, false_negative_num_, false_positive_num_, recall,
      precision);
    for (std::size_t count_idx = 0U; count_idx < robot_counts_.size(); ++count_idx) {
      RCLCPP_INFO(
        get_logger(),
        "DEPTH_PERSISTENT_RESULT robot_num=%zu direct_p50_ms=%.3f direct_p95_ms=%.3f persistent_query_p50_ms=%.3f persistent_query_p95_ms=%.3f persistent_total_p50_ms=%.3f persistent_total_p95_ms=%.3f",
        robot_counts_[count_idx],
        percentile(direct_ms_samples_[count_idx], 0.50),
        percentile(direct_ms_samples_[count_idx], 0.95),
        percentile(persistent_query_ms_samples_[count_idx], 0.50),
        percentile(persistent_query_ms_samples_[count_idx], 0.95),
        percentile(persistent_total_ms_samples_[count_idx], 0.50),
        percentile(persistent_total_ms_samples_[count_idx], 0.95));
    }
  }

  static constexpr std::size_t max_robot_num{8U};
  static constexpr std::array<std::size_t, 4> robot_counts_{{1U, 2U, 4U, 8U}};

  int frame_num_{60};
  double robot_spacing_x_{0.0};
  double robot_yaw_step_rad_{0.0};
  std::size_t robot_num_{1U};
  bool enable_comparison_benchmark_{false};
  bool enable_runtime_log_{false};
  bool enable_debug_publish_{false};
  std::string debug_roi_voxel_topic_;
  std::string debug_world_bucket_voxel_topic_;
  std::string debug_frame_id_;
  reachability_bounds bounds_;
  robot_sim::analysis::VoxelIdCodec codec_;
  robot_sim::analysis::VoxelIdCodec world_bucket_codec_;
  Eigen::Isometry3f camera_to_world_{Eigen::Isometry3f::Identity()};
  std::unique_ptr<persistent_depth_world_index> index_;
  std::vector<std::unique_ptr<reachability_voxel_accumulator>> direct_accumulators_;
  std::vector<std::unique_ptr<reachability_voxel_accumulator>> persistent_accumulators_;
  std::vector<double> update_ms_samples_;
  std::vector<double> query_ms_samples_;
  std::vector<double> total_ms_samples_;
  std::vector<std::size_t> changed_point_num_samples_;
  std::array<std::vector<double>, robot_counts_.size()> direct_ms_samples_;
  std::array<std::vector<double>, robot_counts_.size()> persistent_query_ms_samples_;
  std::array<std::vector<double>, robot_counts_.size()> persistent_total_ms_samples_;
  std::size_t measured_frame_num_{0U};
  std::size_t mismatch_num_{0U};
  std::size_t false_negative_num_{0U};
  std::size_t false_positive_num_{0U};
  std::size_t direct_voxel_num_{0U};
  std::size_t persistent_voxel_num_{0U};
  std::size_t accumulated_inserted_point_num_{0U};
  std::size_t accumulated_updated_point_num_{0U};
  std::size_t accumulated_removed_point_num_{0U};
  std::size_t deferred_free_point_num_{0U};
  bool has_reported_missing_camera_info_{false};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr debug_roi_voxel_publisher_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr debug_world_bucket_voxel_publisher_;
};

}  // robot_sim::bridge namespace終端

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_sim::bridge::depth_world_index_benchmark_node>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
