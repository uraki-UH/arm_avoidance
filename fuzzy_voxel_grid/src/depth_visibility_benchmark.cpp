#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ais_gng_msgs/msg/topological_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{

using Clock = std::chrono::steady_clock;

int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

double elapsedMilliseconds(const Clock::time_point & begin, const Clock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

double percentile(std::vector<double> values, const double fraction)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(std::llround(
    fraction * static_cast<double>(values.size() - 1U)));
  return values[index];
}

struct NodeState
{
  uint32_t last_epoch{0U};
  uint16_t free_streak{0U};
};

struct ModeMetrics
{
  std::vector<double> pooling_ms;
  std::vector<double> visibility_ms;
  std::vector<double> streak_ms;
  std::vector<double> total_ms;
};

struct DepthGrid
{
  int width{0};
  int height{0};
  int scale{1};
  const sensor_msgs::msg::Image * image{nullptr};
  std::vector<float> pooled_depth_m;
};

class DepthVisibilityBenchmark final : public rclcpp::Node
{
public:
  DepthVisibilityBenchmark()
  : Node("depth_visibility_benchmark")
  {
    depth_topic_ = this->declare_parameter<std::string>(
      "depth_topic", "/camera/camera/depth/image_rect_raw");
    camera_info_topic_ = this->declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera/depth/camera_info");
    topological_map_topic_ = this->declare_parameter<std::string>(
      "topological_map_topic", "/topological_map");
    frame_count_target_ = this->declare_parameter<int>("frame_count", 30);
    max_sync_offset_ms_ = this->declare_parameter<double>("max_sync_offset_ms", 5.0);
    depth_unit_scale_ = this->declare_parameter<double>("depth_unit_scale", 0.001);
    relative_depth_tolerance_ = this->declare_parameter<double>(
      "relative_depth_tolerance", 0.02);
    free_support_required_ = this->declare_parameter<int>("free_support_required", 6);

    base_from_camera_translation_[0] = this->declare_parameter<double>("camera_to_base.x", 0.434);
    base_from_camera_translation_[1] = this->declare_parameter<double>("camera_to_base.y", -0.693);
    base_from_camera_translation_[2] = this->declare_parameter<double>("camera_to_base.z", 0.279);
    const double roll_deg = this->declare_parameter<double>("camera_to_base.roll_deg", -103.8);
    const double pitch_deg = this->declare_parameter<double>("camera_to_base.pitch_deg", -28.9);
    const double yaw_deg = this->declare_parameter<double>("camera_to_base.yaw_deg", -3.4);

    if (frame_count_target_ <= 0 || max_sync_offset_ms_ < 0.0 || depth_unit_scale_ <= 0.0 ||
      relative_depth_tolerance_ < 0.0 || free_support_required_ < 1 || free_support_required_ > 9)
    {
      throw std::invalid_argument("Invalid depth_visibility_benchmark parameter.");
    }

    makeBaseFromCameraRotation(roll_deg, pitch_deg, yaw_deg);
    for (auto & states : states_by_mode_) {
      states.resize(65536U);
    }

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DepthVisibilityBenchmark::depthCallback, this, std::placeholders::_1));
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DepthVisibilityBenchmark::cameraInfoCallback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      topological_map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&DepthVisibilityBenchmark::mapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Depth visibility benchmark: depth=%s camera_info=%s map=%s frames=%d",
      depth_topic_.c_str(), camera_info_topic_.c_str(), topological_map_topic_.c_str(),
      frame_count_target_);
  }

private:
  static constexpr std::array<int, 3> kScales{1, 2, 4};

  void makeBaseFromCameraRotation(double roll_deg, double pitch_deg, double yaw_deg)
  {
    constexpr double kPi = 3.14159265358979323846;
    const double roll = roll_deg * kPi / 180.0;
    const double pitch = pitch_deg * kPi / 180.0;
    const double yaw = yaw_deg * kPi / 180.0;

    const std::array<std::array<double, 3>, 3> rx{{
      {{1.0, 0.0, 0.0}},
      {{0.0, std::cos(roll), -std::sin(roll)}},
      {{0.0, std::sin(roll), std::cos(roll)}}}};
    const std::array<std::array<double, 3>, 3> ry{{
      {{std::cos(pitch), 0.0, std::sin(pitch)}},
      {{0.0, 1.0, 0.0}},
      {{-std::sin(pitch), 0.0, std::cos(pitch)}}}};
    const std::array<std::array<double, 3>, 3> rz{{
      {{std::cos(yaw), -std::sin(yaw), 0.0}},
      {{std::sin(yaw), std::cos(yaw), 0.0}},
      {{0.0, 0.0, 1.0}}}};

    std::array<std::array<double, 3>, 3> rx_ry{};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        for (int inner = 0; inner < 3; ++inner) {
          rx_ry[row][column] += rx[row][inner] * ry[inner][column];
        }
      }
    }
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        for (int inner = 0; inner < 3; ++inner) {
          base_from_camera_rotation_[row][column] += rx_ry[row][inner] * rz[inner][column];
        }
      }
    }
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr message)
  {
    depth_by_stamp_[stampNanoseconds(message->header.stamp)] = message;
    while (depth_by_stamp_.size() > 64U) {
      depth_by_stamp_.erase(depth_by_stamp_.begin());
    }
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr message)
  {
    camera_info_ = message;
  }

  const sensor_msgs::msg::Image::SharedPtr findMatchingDepth(const int64_t map_stamp_ns, double * offset_ms)
  {
    auto exact = depth_by_stamp_.find(map_stamp_ns);
    if (exact != depth_by_stamp_.end()) {
      *offset_ms = 0.0;
      return exact->second;
    }

    const int64_t max_offset_ns = static_cast<int64_t>(max_sync_offset_ms_ * 1000000.0);
    auto upper = depth_by_stamp_.lower_bound(map_stamp_ns);
    const sensor_msgs::msg::Image::SharedPtr * best = nullptr;
    int64_t best_offset_ns = std::numeric_limits<int64_t>::max();
    for (auto candidate = upper; candidate != depth_by_stamp_.end(); ++candidate) {
      const int64_t offset = std::llabs(candidate->first - map_stamp_ns);
      if (offset > max_offset_ns) {
        break;
      }
      if (offset < best_offset_ns) {
        best = &candidate->second;
        best_offset_ns = offset;
      }
    }
    if (upper != depth_by_stamp_.begin()) {
      auto candidate = std::prev(upper);
      const int64_t offset = std::llabs(candidate->first - map_stamp_ns);
      if (offset <= max_offset_ns && offset < best_offset_ns) {
        best = &candidate->second;
        best_offset_ns = offset;
      }
    }
    if (best == nullptr) {
      return nullptr;
    }
    *offset_ms = static_cast<double>(best_offset_ns) / 1000000.0;
    return *best;
  }

  float sourceDepthMeters(const sensor_msgs::msg::Image & image, int x, int y) const
  {
    if (x < 0 || x >= static_cast<int>(image.width) || y < 0 || y >= static_cast<int>(image.height)) {
      return std::numeric_limits<float>::infinity();
    }
    const auto * row = image.data.data() + static_cast<std::size_t>(y) * image.step;
    if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      image.encoding == sensor_msgs::image_encodings::MONO16)
    {
      uint16_t value{};
      std::memcpy(&value, row + static_cast<std::size_t>(x) * sizeof(uint16_t), sizeof(value));
      return value == 0U ? std::numeric_limits<float>::infinity() :
             static_cast<float>(value * depth_unit_scale_);
    }
    if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      float value{};
      std::memcpy(&value, row + static_cast<std::size_t>(x) * sizeof(float), sizeof(value));
      return std::isfinite(value) && value > 0.0F ? value : std::numeric_limits<float>::infinity();
    }
    return std::numeric_limits<float>::infinity();
  }

  DepthGrid makeDepthGrid(const sensor_msgs::msg::Image & image, const int scale)
  {
    DepthGrid grid;
    grid.width = static_cast<int>(image.width) / scale;
    grid.height = static_cast<int>(image.height) / scale;
    grid.scale = scale;
    grid.image = &image;
    if (scale == 1) {
      return grid;
    }

    grid.pooled_depth_m.assign(
      static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height),
      std::numeric_limits<float>::infinity());
    for (int grid_y = 0; grid_y < grid.height; ++grid_y) {
      for (int grid_x = 0; grid_x < grid.width; ++grid_x) {
        float nearest = std::numeric_limits<float>::infinity();
        for (int dy = 0; dy < scale; ++dy) {
          for (int dx = 0; dx < scale; ++dx) {
            nearest = std::min(nearest, sourceDepthMeters(
              image, grid_x * scale + dx, grid_y * scale + dy));
          }
        }
        grid.pooled_depth_m[static_cast<std::size_t>(grid_y) * grid.width + grid_x] = nearest;
      }
    }
    return grid;
  }

  float depthAt(const DepthGrid & grid, const int x, const int y) const
  {
    if (x < 0 || x >= grid.width || y < 0 || y >= grid.height) {
      return std::numeric_limits<float>::infinity();
    }
    if (grid.scale == 1) {
      return sourceDepthMeters(*grid.image, x, y);
    }
    return grid.pooled_depth_m[static_cast<std::size_t>(y) * grid.width + x];
  }

  ModeMetrics & processMode(
    const std::size_t mode_index,
    const sensor_msgs::msg::Image & depth_image,
    const ais_gng_msgs::msg::TopologicalMap & map)
  {
    ModeMetrics & metrics = metrics_by_mode_[mode_index];
    const int scale = kScales[mode_index];
    const auto total_begin = Clock::now();

    const auto pooling_begin = Clock::now();
    const DepthGrid grid = makeDepthGrid(depth_image, scale);
    const auto pooling_end = Clock::now();

    const auto visibility_begin = Clock::now();
    const double fx = camera_info_->k[0] / scale;
    const double fy = camera_info_->k[4] / scale;
    const double cx = camera_info_->k[2] / scale;
    const double cy = camera_info_->k[5] / scale;
    std::vector<uint16_t> visible_ids;
    std::vector<bool> free_evidence;
    visible_ids.reserve(map.nodes.size());
    free_evidence.reserve(map.nodes.size());
    int free_count = 0;

    for (const auto & node : map.nodes) {
      const double base_x = node.pos.x - base_from_camera_translation_[0];
      const double base_y = node.pos.y - base_from_camera_translation_[1];
      const double base_z = node.pos.z - base_from_camera_translation_[2];
      // p_camera = R_base_camera^T * (p_base - t_base_camera).
      const double camera_x = base_from_camera_rotation_[0][0] * base_x +
        base_from_camera_rotation_[1][0] * base_y + base_from_camera_rotation_[2][0] * base_z;
      const double camera_y = base_from_camera_rotation_[0][1] * base_x +
        base_from_camera_rotation_[1][1] * base_y + base_from_camera_rotation_[2][1] * base_z;
      const double camera_z = base_from_camera_rotation_[0][2] * base_x +
        base_from_camera_rotation_[1][2] * base_y + base_from_camera_rotation_[2][2] * base_z;
      if (camera_z <= 1.0e-6) {
        continue;
      }
      const int pixel_x = static_cast<int>(std::lround(fx * camera_x / camera_z + cx));
      const int pixel_y = static_cast<int>(std::lround(fy * camera_y / camera_z + cy));
      if (pixel_x < 1 || pixel_x >= grid.width - 1 || pixel_y < 1 || pixel_y >= grid.height - 1) {
        continue;
      }

      const float minimum_behind_depth = static_cast<float>(
        camera_z * (1.0 + relative_depth_tolerance_));
      int behind_support = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const float observed_depth = depthAt(grid, pixel_x + dx, pixel_y + dy);
          if (std::isfinite(observed_depth) && observed_depth > minimum_behind_depth) {
            ++behind_support;
          }
        }
      }
      const bool free = behind_support >= free_support_required_;
      free_count += free ? 1 : 0;
      visible_ids.push_back(node.id);
      free_evidence.push_back(free);
    }
    const auto visibility_end = Clock::now();

    const auto streak_begin = Clock::now();
    auto & states = states_by_mode_[mode_index];
    int delete_ready_count = 0;
    for (std::size_t i = 0; i < visible_ids.size(); ++i) {
      auto & state = states[visible_ids[i]];
      const bool consecutive = state.last_epoch + 1U == epoch_;
      if (free_evidence[i]) {
        state.free_streak = consecutive ?
          static_cast<uint16_t>(std::min<unsigned int>(65535U, state.free_streak + 1U)) : 1U;
      } else {
        state.free_streak = 0U;
      }
      state.last_epoch = epoch_;
      delete_ready_count += state.free_streak >= 3U ? 1 : 0;
    }
    const auto streak_end = Clock::now();
    const auto total_end = Clock::now();

    metrics.pooling_ms.push_back(elapsedMilliseconds(pooling_begin, pooling_end));
    metrics.visibility_ms.push_back(elapsedMilliseconds(visibility_begin, visibility_end));
    metrics.streak_ms.push_back(elapsedMilliseconds(streak_begin, streak_end));
    metrics.total_ms.push_back(elapsedMilliseconds(total_begin, total_end));
    RCLCPP_INFO(
      get_logger(),
      "scale=1/%d pool=%.3fms visibility=%.3fms streak=%.3fms total=%.3fms free=%d delete_ready=%d",
      scale, metrics.pooling_ms.back(), metrics.visibility_ms.back(), metrics.streak_ms.back(),
      metrics.total_ms.back(), free_count, delete_ready_count);
    return metrics;
  }

  void mapCallback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr map)
  {
    if (camera_info_ == nullptr || reported_) {
      return;
    }
    double sync_offset_ms{};
    const auto depth_image = findMatchingDepth(stampNanoseconds(map->header.stamp), &sync_offset_ms);
    if (depth_image == nullptr) {
      ++unmatched_maps_;
      return;
    }
    if (depth_image->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
      depth_image->encoding != sensor_msgs::image_encodings::MONO16 &&
      depth_image->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unsupported depth encoding '%s'. Expected 16UC1, mono16, or 32FC1.",
        depth_image->encoding.c_str());
      return;
    }
    if (depth_image->width != camera_info_->width || depth_image->height != camera_info_->height) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth image %ux%u differs from CameraInfo %ux%u.",
        depth_image->width, depth_image->height, camera_info_->width, camera_info_->height);
    }

    RCLCPP_DEBUG(get_logger(), "Matched map-depth offset=%.3f ms", sync_offset_ms);
    ++epoch_;
    for (std::size_t mode = 0; mode < kScales.size(); ++mode) {
      processMode(mode, *depth_image, *map);
    }
    ++processed_frames_;
    if (processed_frames_ >= frame_count_target_) {
      reportSummary();
      reported_ = true;
      rclcpp::shutdown();
    }
  }

  void reportSummary() const
  {
    RCLCPP_INFO(
      get_logger(),
      "Depth visibility benchmark summary: frames=%d unmatched_maps=%d", processed_frames_, unmatched_maps_);
    for (std::size_t mode = 0; mode < kScales.size(); ++mode) {
      const auto & metrics = metrics_by_mode_[mode];
      const auto mean = [](const std::vector<double> & values) {
          return std::accumulate(values.begin(), values.end(), 0.0) /
                 static_cast<double>(values.size());
        };
      RCLCPP_INFO(
        get_logger(),
        "scale=1/%d: pool mean/p50/p95=%.3f/%.3f/%.3f ms, "
        "visibility=%.3f/%.3f/%.3f ms, streak=%.3f/%.3f/%.3f ms, total=%.3f/%.3f/%.3f ms",
        kScales[mode],
        mean(metrics.pooling_ms), percentile(metrics.pooling_ms, 0.50), percentile(metrics.pooling_ms, 0.95),
        mean(metrics.visibility_ms), percentile(metrics.visibility_ms, 0.50), percentile(metrics.visibility_ms, 0.95),
        mean(metrics.streak_ms), percentile(metrics.streak_ms, 0.50), percentile(metrics.streak_ms, 0.95),
        mean(metrics.total_ms), percentile(metrics.total_ms, 0.50), percentile(metrics.total_ms, 0.95));
    }
  }

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string topological_map_topic_;
  int frame_count_target_{30};
  double max_sync_offset_ms_{5.0};
  double depth_unit_scale_{0.001};
  double relative_depth_tolerance_{0.02};
  int free_support_required_{6};
  std::array<double, 3> base_from_camera_translation_{};
  std::array<std::array<double, 3>, 3> base_from_camera_rotation_{};
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  std::map<int64_t, sensor_msgs::msg::Image::SharedPtr> depth_by_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  std::array<std::vector<NodeState>, 3> states_by_mode_;
  std::array<ModeMetrics, 3> metrics_by_mode_;
  uint32_t epoch_{0U};
  int processed_frames_{0};
  int unmatched_maps_{0};
  bool reported_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthVisibilityBenchmark>());
  rclcpp::shutdown();
  return 0;
}
