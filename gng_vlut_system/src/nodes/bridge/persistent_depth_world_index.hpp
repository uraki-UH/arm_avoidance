#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "nodes/bridge/world_point_bucket_index.hpp"

namespace robot_sim::bridge
{

struct depth_camera_intrinsics
{
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  float fx{0.0F};
  float fy{0.0F};
  float cx{0.0F};
  float cy{0.0F};

  bool operator==(const depth_camera_intrinsics &other) const
  {
    return width == other.width && height == other.height &&
           fx == other.fx && fy == other.fy && cx == other.cx && cy == other.cy;
  }

  bool operator!=(const depth_camera_intrinsics &other) const
  {
    return !(*this == other);
  }

  void validate() const
  {
    if (width == 0U || height == 0U || !std::isfinite(fx) || !std::isfinite(fy) ||
      !std::isfinite(cx) || !std::isfinite(cy) || fx <= 0.0F || fy <= 0.0F)
    {
      throw std::invalid_argument("depth camera intrinsics are invalid");
    }
  }

  std::size_t pixel_num() const
  {
    return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  }
};

struct persistent_depth_world_index_config
{
  double bucket_size{0.2};
  float depth_scale{0.001F};
  std::uint16_t depth_update_mm_th{1U};
  std::uint16_t free_confirmation_num{3U};

  void validate() const
  {
    if (!std::isfinite(bucket_size) || bucket_size <= 0.0 ||
      !std::isfinite(depth_scale) || depth_scale <= 0.0F || free_confirmation_num == 0U)
    {
      throw std::invalid_argument("persistent depth world index config is invalid");
    }
  }
};

struct persistent_depth_world_index_update_stats
{
  std::size_t valid_depth_pixel_num{0U};
  std::size_t inserted_point_num{0U};
  std::size_t updated_point_num{0U};
  std::size_t removed_point_num{0U};
  std::size_t deferred_free_point_num{0U};
  std::size_t unchanged_point_num{0U};
  std::size_t invalid_depth_pixel_num{0U};
  bool is_rebuild{false};
};

// depth画素を安定handleとして保持するworld座標系の永続bucket索引
class persistent_depth_world_index
{
public:
  explicit persistent_depth_world_index(persistent_depth_world_index_config config)
  : config_(std::move(config))
  {
    config_.validate();
    inverse_bucket_size_ = 1.0 / config_.bucket_size;
  }

  bool set_camera_intrinsics(const depth_camera_intrinsics &intrinsics)
  {
    intrinsics.validate();
    if (has_intrinsics_ && intrinsics == intrinsics_) {
      return false;
    }

    intrinsics_ = intrinsics;
    has_intrinsics_ = true;
    rays_.resize(intrinsics_.pixel_num());
    for (std::uint32_t y = 0U; y < intrinsics_.height; ++y) {
      for (std::uint32_t x = 0U; x < intrinsics_.width; ++x) {
        const std::size_t pixel_idx =
          static_cast<std::size_t>(y) * intrinsics_.width + x;
        rays_[pixel_idx] = Eigen::Vector3f(
          (static_cast<float>(x) - intrinsics_.cx) / intrinsics_.fx,
          (static_cast<float>(y) - intrinsics_.cy) / intrinsics_.fy,
          1.0F);
      }
    }
    reset();
    return true;
  }

  void reset()
  {
    buckets_.clear();
    pixels_.assign(rays_.size(), pixel_state{});
    active_bucket_num_ = 0U;
    active_point_num_ = 0U;
    has_camera_to_world_ = false;
  }

  persistent_depth_world_index_update_stats update(
    const std::vector<std::uint16_t> &depth_mm,
    const Eigen::Isometry3f &camera_to_world)
  {
    if (!has_intrinsics_) {
      throw std::logic_error("camera intrinsics are not configured");
    }
    if (depth_mm.size() != pixels_.size()) {
      throw std::invalid_argument("depth image size does not match camera intrinsics");
    }
    if (!camera_to_world.matrix().allFinite()) {
      throw std::invalid_argument("camera to world transform is invalid");
    }

    persistent_depth_world_index_update_stats stats;
    if (!has_camera_to_world_ || !camera_to_world.isApprox(camera_to_world_, 1.0e-6F)) {
      // 固定カメラ姿勢の変更時に古いworld座標点を破棄する全再構築
      clear_points();
      camera_to_world_ = camera_to_world;
      has_camera_to_world_ = true;
      stats.is_rebuild = true;
    }

    for (std::size_t pixel_idx = 0U; pixel_idx < depth_mm.size(); ++pixel_idx) {
      const std::uint16_t current_depth_mm = depth_mm[pixel_idx];
      pixel_state &pixel = pixels_[pixel_idx];
      if (current_depth_mm == 0U) {
        ++stats.invalid_depth_pixel_num;
        if (!pixel.has_point) {
          continue;
        }
        pixel.free_confirmation_count = static_cast<std::uint16_t>(std::min<std::uint32_t>(
          static_cast<std::uint32_t>(pixel.free_confirmation_count) + 1U,
          std::numeric_limits<std::uint16_t>::max()));
        if (pixel.free_confirmation_count >= config_.free_confirmation_num) {
          remove_point(pixel_idx);
          ++stats.removed_point_num;
        } else {
          ++stats.deferred_free_point_num;
        }
        continue;
      }

      ++stats.valid_depth_pixel_num;
      const Eigen::Vector3f current_point = deproject(pixel_idx, current_depth_mm, camera_to_world_);
      if (!pixel.has_point) {
        add_point(pixel_idx, current_depth_mm, current_point);
        ++stats.inserted_point_num;
        continue;
      }

      pixel.free_confirmation_count = 0U;
      const std::uint16_t depth_change_mm = static_cast<std::uint16_t>(std::abs(
        static_cast<int>(current_depth_mm) - static_cast<int>(pixel.depth_mm)));
      if (stats.is_rebuild || depth_change_mm >= config_.depth_update_mm_th) {
        move_point(pixel_idx, current_depth_mm, current_point);
        ++stats.updated_point_num;
      } else {
        pixel.depth_mm = current_depth_mm;
        ++stats.unchanged_point_num;
      }
    }
    return stats;
  }

  Eigen::Vector3f deproject(
    std::size_t pixel_idx,
    std::uint16_t depth_mm,
    const Eigen::Isometry3f &camera_to_world) const
  {
    if (!has_intrinsics_ || pixel_idx >= rays_.size() || depth_mm == 0U) {
      throw std::out_of_range("depth pixel is unavailable");
    }
    const float depth_m = static_cast<float>(depth_mm) * config_.depth_scale;
    return camera_to_world * (rays_[pixel_idx] * depth_m);
  }

  template<typename Visitor>
  world_bucket_query_stats query_aabb(
    const Eigen::Vector3d &min_corner,
    const Eigen::Vector3d &max_corner,
    Visitor visitor) const
  {
    if (!min_corner.allFinite() || !max_corner.allFinite() ||
      (min_corner.array() > max_corner.array()).any())
    {
      throw std::invalid_argument("query bounds are invalid");
    }

    const world_bucket_key min_key = to_key(min_corner);
    const world_bucket_key max_key = to_key(max_corner);
    world_bucket_query_stats stats;
    const std::uint64_t span_x = inclusive_span(min_key.x, max_key.x);
    const std::uint64_t span_y = inclusive_span(min_key.y, max_key.y);
    const std::uint64_t span_z = inclusive_span(min_key.z, max_key.z);
    constexpr std::uint64_t max_candidate_bucket_num = 100000000ULL;
    if (span_x > max_candidate_bucket_num ||
      span_y > max_candidate_bucket_num / span_x ||
      span_z > max_candidate_bucket_num / (span_x * span_y))
    {
      throw std::length_error("query covers too many buckets");
    }
    stats.candidate_bucket_num = static_cast<std::size_t>(span_x * span_y * span_z);

    for (std::int64_t z = min_key.z; z <= static_cast<std::int64_t>(max_key.z); ++z) {
      for (std::int64_t y = min_key.y; y <= static_cast<std::int64_t>(max_key.y); ++y) {
        for (std::int64_t x = min_key.x; x <= static_cast<std::int64_t>(max_key.x); ++x) {
          const world_bucket_key key{
            static_cast<std::int32_t>(x),
            static_cast<std::int32_t>(y),
            static_cast<std::int32_t>(z)};
          const auto bucket_it = buckets_.find(key);
          if (bucket_it == buckets_.end() || bucket_it->second.empty()) {
            continue;
          }
          ++stats.existing_bucket_num;
          stats.candidate_point_num += bucket_it->second.size();
          for (const std::size_t pixel_idx : bucket_it->second) {
            const pixel_state &pixel = pixels_[pixel_idx];
            const Eigen::Vector3d point = pixel.point.cast<double>();
            if ((point.array() >= min_corner.array()).all() &&
              (point.array() <= max_corner.array()).all())
            {
              visitor(pixel.point);
              ++stats.accepted_point_num;
            }
          }
        }
      }
    }
    return stats;
  }

  template<typename Visitor>
  void visit_points(Visitor visitor) const
  {
    for (const auto &bucket_entry : buckets_) {
      for (const std::size_t pixel_idx : bucket_entry.second) {
        visitor(pixels_[pixel_idx].point);
      }
    }
  }

  template<typename Visitor>
  void visit_buckets(Visitor visitor) const
  {
    for (const auto &bucket_entry : buckets_) {
      if (!bucket_entry.second.empty()) {
        visitor(bucket_entry.first, bucket_entry.second.size());
      }
    }
  }

  const depth_camera_intrinsics &intrinsics() const
  {
    return intrinsics_;
  }

  double bucket_size() const
  {
    return config_.bucket_size;
  }

  std::size_t bucket_num() const
  {
    return active_bucket_num_;
  }

  std::size_t point_num() const
  {
    return active_point_num_;
  }

private:
  struct pixel_state
  {
    bool has_point{false};
    std::uint16_t depth_mm{0U};
    std::uint16_t free_confirmation_count{0U};
    Eigen::Vector3f point{Eigen::Vector3f::Zero()};
    world_bucket_key bucket_key{};
    std::size_t bucket_offset{0U};
  };

  world_bucket_key to_key(const Eigen::Vector3d &point) const
  {
    return world_bucket_key{
      checked_floor_to_int(point.x() * inverse_bucket_size_),
      checked_floor_to_int(point.y() * inverse_bucket_size_),
      checked_floor_to_int(point.z() * inverse_bucket_size_)};
  }

  static std::int32_t checked_floor_to_int(double value)
  {
    const double floored = std::floor(value);
    if (floored < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
      floored > static_cast<double>(std::numeric_limits<std::int32_t>::max()))
    {
      throw std::out_of_range("bucket coordinate is outside int32 range");
    }
    return static_cast<std::int32_t>(floored);
  }

  static std::uint64_t inclusive_span(std::int32_t min_value, std::int32_t max_value)
  {
    return static_cast<std::uint64_t>(
      static_cast<std::int64_t>(max_value) - static_cast<std::int64_t>(min_value) + 1);
  }

  void clear_points()
  {
    buckets_.clear();
    for (pixel_state &pixel : pixels_) {
      pixel = {};
    }
    active_bucket_num_ = 0U;
    active_point_num_ = 0U;
  }

  void add_point(std::size_t pixel_idx, std::uint16_t depth_mm, const Eigen::Vector3f &point)
  {
    pixel_state &pixel = pixels_[pixel_idx];
    const world_bucket_key key = to_key(point.cast<double>());
    auto &bucket = buckets_[key];
    if (bucket.empty()) {
      ++active_bucket_num_;
    }
    pixel.has_point = true;
    pixel.depth_mm = depth_mm;
    pixel.free_confirmation_count = 0U;
    pixel.point = point;
    pixel.bucket_key = key;
    pixel.bucket_offset = bucket.size();
    bucket.push_back(pixel_idx);
    ++active_point_num_;
  }

  void remove_point(std::size_t pixel_idx)
  {
    pixel_state &pixel = pixels_[pixel_idx];
    auto bucket_it = buckets_.find(pixel.bucket_key);
    if (bucket_it == buckets_.end() || pixel.bucket_offset >= bucket_it->second.size()) {
      throw std::logic_error("persistent bucket state is invalid");
    }
    std::vector<std::size_t> &bucket = bucket_it->second;
    const std::size_t moved_pixel_idx = bucket.back();
    bucket[pixel.bucket_offset] = moved_pixel_idx;
    pixels_[moved_pixel_idx].bucket_offset = pixel.bucket_offset;
    bucket.pop_back();
    if (bucket.empty()) {
      buckets_.erase(bucket_it);
      --active_bucket_num_;
    }
    pixel = {};
    --active_point_num_;
  }

  void move_point(std::size_t pixel_idx, std::uint16_t depth_mm, const Eigen::Vector3f &point)
  {
    pixel_state &pixel = pixels_[pixel_idx];
    const world_bucket_key new_key = to_key(point.cast<double>());
    if (new_key == pixel.bucket_key) {
      pixel.depth_mm = depth_mm;
      pixel.point = point;
      return;
    }
    remove_point(pixel_idx);
    add_point(pixel_idx, depth_mm, point);
  }

  persistent_depth_world_index_config config_;
  double inverse_bucket_size_{0.0};
  bool has_intrinsics_{false};
  depth_camera_intrinsics intrinsics_;
  std::vector<Eigen::Vector3f> rays_;
  std::vector<pixel_state> pixels_;
  std::unordered_map<
    world_bucket_key, std::vector<std::size_t>, world_bucket_key_hash> buckets_;
  Eigen::Isometry3f camera_to_world_{Eigen::Isometry3f::Identity()};
  bool has_camera_to_world_{false};
  std::size_t active_bucket_num_{0U};
  std::size_t active_point_num_{0U};
};

}  // robot_sim::bridge namespace終端
