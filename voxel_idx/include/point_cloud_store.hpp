#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Geometry>

namespace voxel_idx
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
  // 座標と元点番号の同居。ROI照会後の点群再読出し不要
  struct indexed_point
  {
    Eigen::Vector3f position;
    std::uint32_t source_idx;
  };

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

  void add_point(const Eigen::Vector3f &point,
    std::uint32_t source_idx = std::numeric_limits<std::uint32_t>::max())
  {
    if (!point.allFinite()) {
      ++nonfinite_point_num_;
      return;
    }
    auto &bucket = buckets_[to_key(point.cast<double>())];
    if (bucket.empty()) {
      ++active_bucket_num_;
    }
    bucket.push_back({point, source_idx});
    ++point_num_;
  }

  template<typename Visitor>
  world_bucket_query_stats query_aabb(
    const Eigen::Vector3d &min_corner,
    const Eigen::Vector3d &max_corner,
    Visitor visitor) const
  {
    return query_aabb_with_source(min_corner, max_corner,
      [&](const Eigen::Vector3f &point, std::uint32_t) {visitor(point);});
  }

  template<typename Visitor>
  world_bucket_query_stats query_aabb_with_source(
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
    const long double candidate_bucket_num = static_cast<long double>(span_x) * span_y * span_z;
    // 巨大な疎領域でも走査量は保持bucket数。統計値だけsize_t上限で飽和。
    stats.candidate_bucket_num = candidate_bucket_num >= std::numeric_limits<std::size_t>::max()
      ? std::numeric_limits<std::size_t>::max() : static_cast<std::size_t>(candidate_bucket_num);

    const auto visit_bucket = [&](const auto &points) {
      if (points.empty()) {return;}
      ++stats.existing_bucket_num;
      stats.candidate_point_num += points.size();
      for (const auto &point : points) {
        const Eigen::Vector3d value = point.position.template cast<double>();
        if ((value.array() >= min_corner.array()).all() &&
          (value.array() <= max_corner.array()).all())
        {
          visitor(point.position, point.source_idx);
          ++stats.accepted_point_num;
        }
      }
    };
    // 広域は保持bucketの走査。空間全体の空bucket検索の省略。
    if (candidate_bucket_num > buckets_.size()) {
      for (const auto &entry : buckets_) {
        const auto &key = entry.first;
        if (key.x >= min_key.x && key.x <= max_key.x &&
          key.y >= min_key.y && key.y <= max_key.y && key.z >= min_key.z && key.z <= max_key.z)
        {
          visit_bucket(entry.second);
        }
      }
      return stats;
    }

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
          visit_bucket(bucket_it->second);
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

  template<typename Visitor>
  void visit_points(Visitor visitor) const
  {
    visit_points_with_source([&](const Eigen::Vector3f &point, std::uint32_t) {visitor(point);});
  }

  template<typename Visitor>
  void visit_points_with_source(Visitor visitor) const
  {
    for (const auto &entry : buckets_) {
      for (const auto &point : entry.second) {visitor(point.position, point.source_idx);}
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
    world_bucket_key, std::vector<indexed_point>, world_bucket_key_hash> buckets_;
  std::size_t active_bucket_num_{0};
  std::size_t point_num_{0};
  std::size_t nonfinite_point_num_{0};
};

// ROI登録と自己判定の共通結果。元点の細セルIDと上位ビットの自己ラベル
struct roi_point_membership
{
  struct cell
  {
    std::int64_t id;
    bool is_self;
    bool has_roi_point;
  };
  static constexpr std::uint64_t self_flag = std::uint64_t{1} << 63;
  static constexpr std::uint64_t no_cell = std::numeric_limits<std::uint64_t>::max();
  std::vector<std::uint64_t> point_cells;
  std::vector<cell> cells;

  bool is_self_point(std::uint32_t source_idx) const
  {
    const auto slot = point_cells.at(source_idx);
    return slot != no_cell && (slot & self_flag) != 0;
  }
};

// 不変の公開スナップショット。ROI専用フレームでは任意のworld索引なし
struct point_frame
{
  std::shared_ptr<const world_point_bucket_index> point_idx;
  std::shared_ptr<const void> source_owner;
  std::string source_type;
  Eigen::Isometry3d source_to_world{Eigen::Isometry3d::Identity()};
  std::string frame_id;
  std::int64_t stamp_ns{0};
  std::uint64_t revision{0};
  std::shared_ptr<const roi_point_membership> roi_points;
  std::int64_t self_mask_stamp_ns{0};
  std::chrono::steady_clock::time_point received_at{std::chrono::steady_clock::now()};
};

// world座標のセル集計条件。fuzzy属性・ROS・利用者固有ラベルへの依存なし。
struct point_cell_spec
{
  Eigen::Vector3d size{Eigen::Vector3d::Constant(.1)};
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  Eigen::Vector3d min_corner{Eigen::Vector3d::Constant(-1)};
  Eigen::Vector3d max_corner{Eigen::Vector3d::Constant(1)};
  Eigen::Vector3d exclude_min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d exclude_max{Eigen::Vector3d::Zero()};
  bool enable_exclusion{false};
  std::size_t max_dense_voxel_num{8000000};
  bool operator==(const point_cell_spec &other) const;
  bool contains(const Eigen::Vector3f &point) const;
  world_bucket_key key(const Eigen::Vector3d &point) const;
};

struct point_cell
{
  world_bucket_key key;
  std::uint32_t num_points{0};
};

// 有界セルの連続配列索引。過大な範囲だけhashへ切替、全空間の毎フレーム初期化なし。
class point_cell_counts
{
public:
  explicit point_cell_counts(point_cell_spec spec);
  void begin_frame();
  void add_point(const Eigen::Vector3f &point);
  std::size_t find(const world_bucket_key &key) const;
  const std::vector<point_cell> &cells() const {return cells_;}
  const point_cell_spec &spec() const {return spec_;}
  bool has_dense_lookup() const {return !dense_lookup_.empty();}
  static constexpr std::size_t no_cell = std::numeric_limits<std::size_t>::max();
private:
  std::size_t dense_idx(const world_bucket_key &key) const;
  point_cell_spec spec_;
  world_bucket_key min_key_, max_key_;
  std::size_t num_x_{0}, num_y_{0};
  std::vector<std::uint32_t> dense_lookup_;
  std::unordered_map<world_bucket_key, std::uint32_t, world_bucket_key_hash> sparse_lookup_;
  std::vector<point_cell> cells_;
};

// 点単位経路のインライン化。DSO境界の反復呼出しなし。
inline bool point_cell_spec::contains(const Eigen::Vector3f &point) const
{
  const Eigen::Vector3d value = point.cast<double>();
  return value.allFinite() && (value.array() >= min_corner.array()).all() &&
    (value.array() <= max_corner.array()).all() &&
    (!enable_exclusion || !((value.array() >= exclude_min.array()).all() &&
    (value.array() <= exclude_max.array()).all()));
}

inline world_bucket_key point_cell_spec::key(const Eigen::Vector3d &point) const
{
  const Eigen::Array3d value = ((point-origin).array() / size.array()).floor();
  if (!value.isFinite().all() ||
    (value < static_cast<double>(std::numeric_limits<std::int32_t>::min())).any() ||
    (value > static_cast<double>(std::numeric_limits<std::int32_t>::max())).any())
  {
    throw std::out_of_range("セル座標がint32範囲外");
  }
  return {static_cast<std::int32_t>(value.x()), static_cast<std::int32_t>(value.y()),
    static_cast<std::int32_t>(value.z())};
}

inline std::size_t point_cell_counts::dense_idx(const world_bucket_key &key) const
{
  return static_cast<std::size_t>(static_cast<std::int64_t>(key.x)-min_key_.x) + num_x_ *
    (static_cast<std::size_t>(static_cast<std::int64_t>(key.y)-min_key_.y) + num_y_ *
    static_cast<std::size_t>(static_cast<std::int64_t>(key.z)-min_key_.z));
}

inline void point_cell_counts::add_point(const Eigen::Vector3f &point)
{
  if (!spec_.contains(point)) {return;}
  const auto key = spec_.key(point.cast<double>());
  auto &slot = has_dense_lookup() ? dense_lookup_[dense_idx(key)] : sparse_lookup_[key];
  if (!slot) {
    if (cells_.size() == std::numeric_limits<std::uint32_t>::max()) {
      throw std::length_error("セル数がuint32範囲外");
    }
    cells_.push_back({key, 0U});
    slot = static_cast<std::uint32_t>(cells_.size());
  }
  ++cells_[slot-1].num_points;
}

// 同一条件・同一snapshotの集計は一度だけ。読取中の集計結果は不変。
class point_cell_query
{
public:
  explicit point_cell_query(point_cell_spec spec) : spec_(std::move(spec)) {}
  const point_cell_spec &spec() const {return spec_;}
  std::shared_ptr<const point_cell_counts> read(const std::shared_ptr<const point_frame> &frame);
private:
  point_cell_spec spec_;
  std::mutex mutex_;
  std::weak_ptr<const point_frame> frame_;
  std::shared_ptr<point_cell_counts> current_, spare_;
};

// 同一プロセス内の共有窓口。fuzzy評価・ROS通信・点群コピーへの依存なし。
class point_frame_channel
{
public:
  void claim_writer(const void *writer);
  void release_writer(const void *writer);
  void publish(const void *writer, point_frame frame);
  std::shared_ptr<const point_frame> latest() const;
  std::shared_ptr<point_cell_query> cell_query(const point_cell_spec &spec);
private:
  mutable std::mutex mutex_;
  const void *writer_{nullptr};
  std::uint64_t revision_{0};
  std::shared_ptr<const point_frame> frame_;
  std::vector<std::weak_ptr<point_cell_query>> cell_queries_;
};

// 共有ライブラリ内で一元管理するchannel。別コンポーネント間でも同じ実体。
std::shared_ptr<point_frame_channel> shared_point_frames(const std::string &name);

}  // 名前空間voxel_idx
