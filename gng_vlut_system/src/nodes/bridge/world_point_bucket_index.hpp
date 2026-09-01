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

#include <Eigen/Core>

namespace robot_sim::bridge
{

struct world_bucket_key
{
  std::int32_t x{0};
  std::int32_t y{0};
  std::int32_t z{0};

  bool operator==(const world_bucket_key &other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct world_bucket_key_hash
{
  std::size_t operator()(const world_bucket_key &key) const noexcept
  {
    // 符号付き座標を含む3軸hashの混合
    std::size_t seed = std::hash<std::int32_t>{}(key.x);
    seed ^= std::hash<std::int32_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<std::int32_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct world_bucket_query_stats
{
  std::size_t candidate_bucket_num{0};
  std::size_t existing_bucket_num{0};
  std::size_t candidate_point_num{0};
  std::size_t accepted_point_num{0};
};

// world座標系の固定幅bucketによる現フレーム点群索引
class world_point_bucket_index
{
public:
  explicit world_point_bucket_index(double bucket_size)
  : bucket_size_(bucket_size), inverse_bucket_size_(1.0 / bucket_size)
  {
    if (!std::isfinite(bucket_size_) || bucket_size_ <= 0.0) {
      throw std::invalid_argument("bucket size must be finite and positive");
    }
  }

  void begin_frame(std::size_t expected_point_num)
  {
    // 固定カメラで安定するbucket構造と各点配列の容量の再利用
    for (auto &bucket_entry : buckets_) {
      bucket_entry.second.clear();
    }
    active_bucket_num_ = 0;
    point_num_ = 0;
    nonfinite_point_num_ = 0;
    const std::size_t expected_bucket_num = std::max<std::size_t>(1U, expected_point_num / 64U);
    if (buckets_.bucket_count() < expected_bucket_num) {
      buckets_.reserve(expected_bucket_num);
    }
  }

  void add_point(const Eigen::Vector3f &point)
  {
    if (!point.allFinite()) {
      ++nonfinite_point_num_;
      return;
    }
    auto &bucket = buckets_[to_key(point.cast<double>())];
    if (bucket.empty()) {
      ++active_bucket_num_;
    }
    bucket.push_back(point);
    ++point_num_;
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
    const std::uint64_t candidate_bucket_num = span_x * span_y * span_z;
    stats.candidate_bucket_num = static_cast<std::size_t>(candidate_bucket_num);

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
          for (const Eigen::Vector3f &point : bucket_it->second) {
            const Eigen::Vector3d point_double = point.cast<double>();
            if ((point_double.array() >= min_corner.array()).all() &&
              (point_double.array() <= max_corner.array()).all())
            {
              visitor(point);
              ++stats.accepted_point_num;
            }
          }
        }
      }
    }
    return stats;
  }

  double bucket_size() const
  {
    return bucket_size_;
  }

  std::size_t bucket_num() const
  {
    return active_bucket_num_;
  }

  std::size_t point_num() const
  {
    return point_num_;
  }

  std::size_t nonfinite_point_num() const
  {
    return nonfinite_point_num_;
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

private:
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

  double bucket_size_;
  double inverse_bucket_size_;
  std::unordered_map<
    world_bucket_key, std::vector<Eigen::Vector3f>, world_bucket_key_hash> buckets_;
  std::size_t active_bucket_num_{0};
  std::size_t point_num_{0};
  std::size_t nonfinite_point_num_{0};
};

}  // robot_sim::bridge namespace終端
