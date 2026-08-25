#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
#include <vector>

#include <Eigen/Geometry>
#include <omp.h>

#include "nodes/bridge/reachability_voxel_accumulator.hpp"
#include "nodes/bridge/world_point_bucket_index.hpp"
#include "safety_engine/indexing/voxel_id_codec.hpp"

namespace robot_sim::bridge
{

class world_bucket_benchmark_node : public rclcpp::Node
{
public:
  world_bucket_benchmark_node()
  : Node("world_bucket_benchmark"),
    codec_(declare_parameter<double>("voxel_size", 0.02)),
    bucket_index_(declare_parameter<double>("bucket_size", 0.2))
  {
    const std::string input_topic = declare_parameter<std::string>(
      "input_topic", "/camera/camera/depth/color/points");
    const std::string depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera/camera/depth/image_rect_raw");
    frame_num_ = static_cast<int>(
      std::max<std::int64_t>(1, declare_parameter<int>("frame_num", 30)));
    parallel_thread_num_ = static_cast<int>(
      std::max<std::int64_t>(1, declare_parameter<int>("parallel_thread_num", 8)));
    index_refresh_frame_num_ = static_cast<int>(
      std::max<std::int64_t>(1, declare_parameter<int>("index_refresh_frame_num", 1)));
    robot_spacing_x_ = declare_parameter<double>("robot_spacing_x", 0.15);
    robot_yaw_step_rad_ =
      declare_parameter<double>("robot_yaw_step_deg", 10.0) * std::acos(-1.0) / 180.0;

    codec_.setIndexingParams(
      declare_parameter<int>("x_shift", 42),
      declare_parameter<int>("y_shift", 21),
      declare_parameter<int>("z_shift", 0),
      declare_parameter<long>("offset", 1000000L));

    bounds_.enable_filter = true;
    bounds_.min_corner = Eigen::Vector3d(
      declare_parameter<double>("min_reachability_x", -0.1),
      declare_parameter<double>("min_reachability_y", -1.0),
      declare_parameter<double>("min_reachability_z", -1.0));
    bounds_.max_corner = Eigen::Vector3d(
      declare_parameter<double>("max_reachability_x", 0.5),
      declare_parameter<double>("max_reachability_y", 1.0),
      declare_parameter<double>("max_reachability_z", 1.0));
    bounds_.margin = Eigen::Vector3d(
      declare_parameter<double>("reachability_margin_x", 0.2),
      declare_parameter<double>("reachability_margin_y", 0.2),
      declare_parameter<double>("reachability_margin_z", 0.2));
    bounds_.validate();

    const std::size_t max_dense_voxel_num = static_cast<std::size_t>(
      std::max<std::int64_t>(
        0, declare_parameter<int>("max_dense_voxel_num", 8000000)));
    for (std::size_t robot_idx = 0; robot_idx < max_robot_num; ++robot_idx) {
      direct_accumulators_.push_back(std::make_unique<reachability_voxel_accumulator>(
        codec_, bounds_, max_dense_voxel_num));
      bucket_accumulators_.push_back(std::make_unique<reachability_voxel_accumulator>(
        codec_, bounds_, max_dense_voxel_num));
      parallel_accumulators_.push_back(std::make_unique<reachability_voxel_accumulator>(
        codec_, bounds_, max_dense_voxel_num));
    }

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&world_bucket_benchmark_node::point_cloud_callback, this, std::placeholders::_1));
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      std::bind(&world_bucket_benchmark_node::depth_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "world bucket benchmark開始: input=%s frame_num=%d bucket_size=%.3f voxel_size=%.3f parallel_thread_num=%d index_refresh_frame_num=%d",
      input_topic.c_str(), frame_num_, bucket_index_.bucket_size(), codec_.voxelSize(),
      parallel_thread_num_, index_refresh_frame_num_);
  }

private:
  using steady_clock = std::chrono::steady_clock;

  static double elapsed_ms(steady_clock::time_point start)
  {
    return std::chrono::duration<double, std::milli>(steady_clock::now() - start).count();
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

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<Eigen::Vector3f> source_points;
    source_points.reserve(static_cast<std::size_t>(msg->width) * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> point_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> point_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> point_z(*msg, "z");
    Eigen::Vector3f min_environment = Eigen::Vector3f::Constant(
      std::numeric_limits<float>::infinity());
    Eigen::Vector3f max_environment = Eigen::Vector3f::Constant(
      -std::numeric_limits<float>::infinity());
    for (; point_x != point_x.end(); ++point_x, ++point_y, ++point_z) {
      const Eigen::Vector3f point(*point_x, *point_y, *point_z);
      source_points.push_back(point);
      if (point.allFinite()) {
        min_environment = min_environment.cwiseMin(point);
        max_environment = max_environment.cwiseMax(point);
      }
    }
    if (measured_frame_num_ == 0U && min_environment.allFinite()) {
      RCLCPP_INFO(
        get_logger(),
        "環境点群AABB: min=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f) extent=(%.3f, %.3f, %.3f)",
        min_environment.x(), min_environment.y(), min_environment.z(),
        max_environment.x(), max_environment.y(), max_environment.z(),
        max_environment.x() - min_environment.x(),
        max_environment.y() - min_environment.y(),
        max_environment.z() - min_environment.z());
    }
    measure_point_change_ratios(source_points);

    const bool is_index_refresh_frame =
      measured_frame_num_ % static_cast<std::size_t>(index_refresh_frame_num_) == 0U;
    double build_ms = 0.0;
    if (is_index_refresh_frame) {
      const auto build_start = steady_clock::now();
      bucket_index_.begin_frame(source_points.size());
      for (const Eigen::Vector3f &point : source_points) {
        bucket_index_.add_point(point);
      }
      build_ms = elapsed_ms(build_start);
      build_ms_samples_.push_back(build_ms);
    }

    std::array<std::vector<long>, max_robot_num> direct_voxel_ids;
    const auto direct_start = steady_clock::now();
    std::size_t count_idx = 0;
    for (std::size_t robot_idx = 0; robot_idx < max_robot_num; ++robot_idx) {
      const Eigen::Isometry3d world_to_robot = robot_to_world(robot_idx).inverse();
      auto &accumulator = *direct_accumulators_[robot_idx];
      accumulator.begin_frame(source_points.size());
      for (const Eigen::Vector3f &point : source_points) {
        accumulator.add_point(point.cast<double>(), world_to_robot);
      }
      direct_voxel_ids[robot_idx] = accumulator.finish_voxel_ids();

      if (robot_idx + 1U == robot_counts_[count_idx]) {
        direct_ms_samples_[count_idx].push_back(elapsed_ms(direct_start));
        ++count_idx;
      }
    }

    std::array<std::vector<long>, max_robot_num> parallel_voxel_ids;
    const auto parallel_start = steady_clock::now();
#pragma omp parallel for num_threads(parallel_thread_num_) schedule(static)
    for (int robot_idx = 0; robot_idx < static_cast<int>(max_robot_num); ++robot_idx) {
      const Eigen::Isometry3d world_to_robot =
        robot_to_world(static_cast<std::size_t>(robot_idx)).inverse();
      auto &accumulator = *parallel_accumulators_[static_cast<std::size_t>(robot_idx)];
      accumulator.begin_frame(source_points.size());
      for (const Eigen::Vector3f &point : source_points) {
        accumulator.add_point(point.cast<double>(), world_to_robot);
      }
      parallel_voxel_ids[static_cast<std::size_t>(robot_idx)] =
        accumulator.finish_voxel_ids();
    }
    const double parallel_ms = elapsed_ms(parallel_start);
    parallel_ms_samples_.push_back(parallel_ms);
    for (std::size_t robot_idx = 0; robot_idx < max_robot_num; ++robot_idx) {
      if (parallel_voxel_ids[robot_idx] != direct_voxel_ids[robot_idx]) {
        ++mismatch_num_;
        RCLCPP_ERROR(
          get_logger(),
          "並列VLUT ID不一致: frame=%zu robot_idx=%zu direct=%zu parallel=%zu",
          measured_frame_num_ + 1U, robot_idx,
          direct_voxel_ids[robot_idx].size(), parallel_voxel_ids[robot_idx].size());
      }
    }

    const auto bucket_query_start = steady_clock::now();
    count_idx = 0;
    world_bucket_query_stats last_query_stats;
    for (std::size_t robot_idx = 0; robot_idx < max_robot_num; ++robot_idx) {
      const Eigen::Isometry3d robot_to_world_transform = robot_to_world(robot_idx);
      const Eigen::Isometry3d world_to_robot = robot_to_world_transform.inverse();
      const auto [min_world, max_world] = world_query_bounds(robot_to_world_transform);
      auto &accumulator = *bucket_accumulators_[robot_idx];
      accumulator.begin_frame(source_points.size());
      last_query_stats = bucket_index_.query_aabb(
        min_world, max_world,
        [&accumulator, &world_to_robot](const Eigen::Vector3f &point) {
          accumulator.add_point(point.cast<double>(), world_to_robot);
        });
      const std::vector<long> &bucket_voxel_ids = accumulator.finish_voxel_ids();
      const auto [false_negative_num, false_positive_num] = count_voxel_difference(
        direct_voxel_ids[robot_idx], bucket_voxel_ids);
      if (bucket_voxel_ids != direct_voxel_ids[robot_idx]) {
        ++mismatch_num_;
        if (is_index_refresh_frame) {
          RCLCPP_ERROR(
            get_logger(),
            "VLUT ID不一致: frame=%zu robot_idx=%zu direct=%zu bucket=%zu",
            measured_frame_num_ + 1U, robot_idx,
            direct_voxel_ids[robot_idx].size(), bucket_voxel_ids.size());
        }
      }
      if (!is_index_refresh_frame) {
        reuse_direct_voxel_num_ += direct_voxel_ids[robot_idx].size();
        reuse_bucket_voxel_num_ += bucket_voxel_ids.size();
        reuse_false_negative_num_ += false_negative_num;
        reuse_false_positive_num_ += false_positive_num;
      }

      if (robot_idx + 1U == robot_counts_[count_idx]) {
        const double query_ms = elapsed_ms(bucket_query_start);
        bucket_query_ms_samples_[count_idx].push_back(query_ms);
        bucket_total_ms_samples_[count_idx].push_back(build_ms + query_ms);
        ++count_idx;
      }
    }

    if (!is_index_refresh_frame) {
      ++reuse_frame_num_;
    }
    ++measured_frame_num_;
    const double accepted_ratio = source_points.empty() ? 0.0 :
      static_cast<double>(last_query_stats.accepted_point_num) /
      static_cast<double>(source_points.size());
    accepted_ratio_samples_.push_back(accepted_ratio);
    RCLCPP_INFO(
      get_logger(),
      "計測 %zu/%d: refresh_index=%s points=%zu buckets=%zu build=%.3fms direct8=%.3fms parallel8=%.3fms bucket8_total=%.3fms candidate8=%zu accepted8=%zu accepted_ratio8=%.3f",
      measured_frame_num_, frame_num_, is_index_refresh_frame ? "true" : "false",
      source_points.size(), bucket_index_.bucket_num(), build_ms,
      direct_ms_samples_.back().back(), parallel_ms, bucket_total_ms_samples_.back().back(),
      last_query_stats.candidate_point_num, last_query_stats.accepted_point_num, accepted_ratio);

    if (measured_frame_num_ >= static_cast<std::size_t>(frame_num_)) {
      report_results();
      subscription_.reset();
      rclcpp::shutdown();
    }
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

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
      msg->encoding != sensor_msgs::image_encodings::MONO16)
    {
      return;
    }
    const std::size_t pixel_num = static_cast<std::size_t>(msg->width) * msg->height;
    std::vector<std::uint16_t> current_depth_mm(pixel_num, 0U);
    for (std::size_t y = 0; y < msg->height; ++y) {
      const std::uint8_t *row = msg->data.data() + y * msg->step;
      for (std::size_t x = 0; x < msg->width; ++x) {
        std::memcpy(
          &current_depth_mm[y * msg->width + x],
          row + x * sizeof(std::uint16_t), sizeof(std::uint16_t));
      }
    }

    if (previous_depth_mm_.size() == current_depth_mm.size()) {
      std::array<std::size_t, depth_change_mm_th_.size()> changed_pixel_nums{};
      for (std::size_t pixel_idx = 0; pixel_idx < pixel_num; ++pixel_idx) {
        const std::uint16_t current_depth = current_depth_mm[pixel_idx];
        const std::uint16_t previous_depth = previous_depth_mm_[pixel_idx];
        if ((current_depth == 0U) != (previous_depth == 0U)) {
          for (std::size_t th_idx = 0; th_idx < depth_change_mm_th_.size(); ++th_idx) {
            ++changed_pixel_nums[th_idx];
          }
          continue;
        }
        if (current_depth == 0U) {
          continue;
        }
        const int depth_change_mm = std::abs(
          static_cast<int>(current_depth) - static_cast<int>(previous_depth));
        for (std::size_t th_idx = 0; th_idx < depth_change_mm_th_.size(); ++th_idx) {
          if (depth_change_mm > depth_change_mm_th_[th_idx]) {
            ++changed_pixel_nums[th_idx];
          }
        }
      }
      for (std::size_t th_idx = 0; th_idx < depth_change_mm_th_.size(); ++th_idx) {
        depth_change_ratio_samples_[th_idx].push_back(
          pixel_num == 0U ? 0.0 :
          static_cast<double>(changed_pixel_nums[th_idx]) / static_cast<double>(pixel_num));
      }
    }
    previous_depth_mm_ = std::move(current_depth_mm);
  }

  void measure_point_change_ratios(const std::vector<Eigen::Vector3f> &source_points)
  {
    if (previous_source_points_.size() == source_points.size()) {
      std::array<std::size_t, point_change_dist_th_.size()> changed_point_nums{};
      for (std::size_t point_idx = 0; point_idx < source_points.size(); ++point_idx) {
        const Eigen::Vector3f &current_point = source_points[point_idx];
        const Eigen::Vector3f &previous_point = previous_source_points_[point_idx];
        const bool has_current_point = current_point.allFinite();
        const bool had_previous_point = previous_point.allFinite();
        if (has_current_point != had_previous_point) {
          for (std::size_t th_idx = 0; th_idx < point_change_dist_th_.size(); ++th_idx) {
            ++changed_point_nums[th_idx];
          }
          continue;
        }
        if (!has_current_point) {
          continue;
        }
        const float point_change_dist = (current_point - previous_point).norm();
        for (std::size_t th_idx = 0; th_idx < point_change_dist_th_.size(); ++th_idx) {
          if (point_change_dist > point_change_dist_th_[th_idx]) {
            ++changed_point_nums[th_idx];
          }
        }
      }
      for (std::size_t th_idx = 0; th_idx < point_change_dist_th_.size(); ++th_idx) {
        point_change_ratio_samples_[th_idx].push_back(
          source_points.empty() ? 0.0 :
          static_cast<double>(changed_point_nums[th_idx]) /
          static_cast<double>(source_points.size()));
      }
    }
    previous_source_points_ = source_points;
  }

  void report_results() const
  {
    RCLCPP_INFO(
      get_logger(),
      "BUCKET_RESULT frames=%zu mismatch_num=%zu build_p50_ms=%.3f build_p95_ms=%.3f accepted_ratio8_p50=%.3f",
      measured_frame_num_, mismatch_num_,
      percentile(build_ms_samples_, 0.50), percentile(build_ms_samples_, 0.95),
      percentile(accepted_ratio_samples_, 0.50));
    RCLCPP_INFO(
      get_logger(),
      "PARALLEL_RESULT robot_num=%zu thread_num=%d direct_p50_ms=%.3f direct_p95_ms=%.3f parallel_p50_ms=%.3f parallel_p95_ms=%.3f",
      max_robot_num, parallel_thread_num_,
      percentile(direct_ms_samples_.back(), 0.50),
      percentile(direct_ms_samples_.back(), 0.95),
      percentile(parallel_ms_samples_, 0.50),
      percentile(parallel_ms_samples_, 0.95));
    const double recall = reuse_direct_voxel_num_ == 0U ? 1.0 :
      1.0 - static_cast<double>(reuse_false_negative_num_) /
      static_cast<double>(reuse_direct_voxel_num_);
    const double precision = reuse_bucket_voxel_num_ == 0U ? 1.0 :
      1.0 - static_cast<double>(reuse_false_positive_num_) /
      static_cast<double>(reuse_bucket_voxel_num_);
    RCLCPP_INFO(
      get_logger(),
      "REUSE_RESULT index_refresh_frame_num=%d reuse_frame_num=%zu false_negative_num=%zu false_positive_num=%zu recall=%.6f precision=%.6f",
      index_refresh_frame_num_, reuse_frame_num_, reuse_false_negative_num_,
      reuse_false_positive_num_, recall, precision);
    if (point_change_ratio_samples_.front().empty()) {
      RCLCPP_INFO(
        get_logger(),
        "POINT_CHANGE_RESULT unavailable=true reason=unorganized_point_cloud");
    } else {
      for (std::size_t th_idx = 0; th_idx < point_change_dist_th_.size(); ++th_idx) {
        RCLCPP_INFO(
          get_logger(),
          "POINT_CHANGE_RESULT dist_th_m=%.3f change_ratio_p50=%.6f change_ratio_p95=%.6f",
          point_change_dist_th_[th_idx],
          percentile(point_change_ratio_samples_[th_idx], 0.50),
          percentile(point_change_ratio_samples_[th_idx], 0.95));
      }
    }
    if (depth_change_ratio_samples_.front().empty()) {
      RCLCPP_INFO(
        get_logger(),
        "DEPTH_CHANGE_RESULT unavailable=true reason=no_depth_image");
    } else {
      for (std::size_t th_idx = 0; th_idx < depth_change_mm_th_.size(); ++th_idx) {
        RCLCPP_INFO(
          get_logger(),
          "DEPTH_CHANGE_RESULT depth_change_mm_th=%d change_ratio_p50=%.6f change_ratio_p95=%.6f",
          depth_change_mm_th_[th_idx],
          percentile(depth_change_ratio_samples_[th_idx], 0.50),
          percentile(depth_change_ratio_samples_[th_idx], 0.95));
      }
    }
    for (std::size_t count_idx = 0; count_idx < robot_counts_.size(); ++count_idx) {
      RCLCPP_INFO(
        get_logger(),
        "BUCKET_RESULT robot_num=%zu direct_p50_ms=%.3f direct_p95_ms=%.3f bucket_query_p50_ms=%.3f bucket_query_p95_ms=%.3f bucket_total_p50_ms=%.3f bucket_total_p95_ms=%.3f",
        robot_counts_[count_idx],
        percentile(direct_ms_samples_[count_idx], 0.50),
        percentile(direct_ms_samples_[count_idx], 0.95),
        percentile(bucket_query_ms_samples_[count_idx], 0.50),
        percentile(bucket_query_ms_samples_[count_idx], 0.95),
        percentile(bucket_total_ms_samples_[count_idx], 0.50),
        percentile(bucket_total_ms_samples_[count_idx], 0.95));
    }
  }

  static constexpr std::size_t max_robot_num{8U};
  static constexpr std::array<std::size_t, 4> robot_counts_{{1U, 2U, 4U, 8U}};
  static constexpr std::array<float, 4> point_change_dist_th_{{0.005F, 0.01F, 0.02F, 0.05F}};
  static constexpr std::array<int, 4> depth_change_mm_th_{{5, 10, 20, 50}};

  int frame_num_{30};
  int parallel_thread_num_{8};
  int index_refresh_frame_num_{1};
  double robot_spacing_x_{0.15};
  double robot_yaw_step_rad_{0.0};
  reachability_bounds bounds_;
  robot_sim::analysis::VoxelIdCodec codec_;
  world_point_bucket_index bucket_index_;
  std::vector<std::unique_ptr<reachability_voxel_accumulator>> direct_accumulators_;
  std::vector<std::unique_ptr<reachability_voxel_accumulator>> bucket_accumulators_;
  std::vector<std::unique_ptr<reachability_voxel_accumulator>> parallel_accumulators_;
  std::vector<double> build_ms_samples_;
  std::array<std::vector<double>, robot_counts_.size()> direct_ms_samples_;
  std::array<std::vector<double>, robot_counts_.size()> bucket_query_ms_samples_;
  std::array<std::vector<double>, robot_counts_.size()> bucket_total_ms_samples_;
  std::vector<double> parallel_ms_samples_;
  std::vector<double> accepted_ratio_samples_;
  std::vector<Eigen::Vector3f> previous_source_points_;
  std::array<std::vector<double>, point_change_dist_th_.size()> point_change_ratio_samples_;
  std::vector<std::uint16_t> previous_depth_mm_;
  std::array<std::vector<double>, depth_change_mm_th_.size()> depth_change_ratio_samples_;
  std::size_t measured_frame_num_{0U};
  std::size_t mismatch_num_{0U};
  std::size_t reuse_frame_num_{0U};
  std::size_t reuse_direct_voxel_num_{0U};
  std::size_t reuse_bucket_voxel_num_{0U};
  std::size_t reuse_false_negative_num_{0U};
  std::size_t reuse_false_positive_num_{0U};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
};

}  // robot_sim::bridge namespace終端

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_sim::bridge::world_bucket_benchmark_node>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
