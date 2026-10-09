#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

namespace pointcloud_sampling {

namespace detail {
// レイアウト検査はフレーム単位。XYZ読み出しは抽出方式に応じた候補点のみ
struct xyz_reader {
  const sensor_msgs::msg::PointCloud2 &cloud;
  uint32_t num_points{0};
  std::array<uint32_t, 3> offsets{};
  std::array<uint32_t, 3> sizes{};
  bool has_byte_swap{false};
  bool has_row_padding{false};
  bool is_native_float32{false};

  explicit xyz_reader(const sensor_msgs::msg::PointCloud2 &input) : cloud(input) {
    const uint64_t count = uint64_t(cloud.width) * cloud.height;
    if (count == 0) return;
    if (count > std::numeric_limits<uint32_t>::max() || cloud.point_step == 0 ||
        uint64_t(cloud.width) * cloud.point_step > cloud.row_step ||
        uint64_t(cloud.height - 1) * cloud.row_step + uint64_t(cloud.width) * cloud.point_step > cloud.data.size()) {
      throw std::invalid_argument("Invalid PointCloud2 layout");
    }
    num_points = count;
    for (uint32_t axis = 0; axis < 3; ++axis) {
      const std::string name(1, "xyz"[axis]);
      const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
          [&](const auto &value) { return value.name == name; });
      if (field == cloud.fields.end() || field->count != 1 ||
          (field->datatype != 7 && field->datatype != 8)) {
        throw std::invalid_argument("Missing floating point XYZ field");
      }
      offsets[axis] = field->offset;
      sizes[axis] = field->datatype == 7 ? 4 : 8;
      if (uint64_t(offsets[axis]) + sizes[axis] > cloud.point_step)
        throw std::invalid_argument("Invalid XYZ field offset");
    }
    const uint16_t endian_probe = 1;
    const bool is_native_big_endian = *reinterpret_cast<const uint8_t *>(&endian_probe) == 0;
    has_byte_swap = cloud.is_bigendian != is_native_big_endian;
    has_row_padding = uint64_t(cloud.width) * cloud.point_step != cloud.row_step;
    is_native_float32 = !has_byte_swap && sizes == std::array<uint32_t, 3>{4, 4, 4};
  }

  bool read(uint32_t idx, std::array<double, 3> &xyz) const {
    const uint64_t byte_offset = has_row_padding
        ? uint64_t(idx / cloud.width) * cloud.row_step + uint64_t(idx % cloud.width) * cloud.point_step
        : uint64_t(idx) * cloud.point_step;
    const auto *point = cloud.data.data() + byte_offset;
    // 通常のfloat32入力は固定長コピー。非整列フィールドにも対応
    if (is_native_float32) {
      for (uint32_t axis = 0; axis < 3; ++axis) {
        float value;
        std::memcpy(&value, point + offsets[axis], sizeof(value));
        if (!std::isfinite(value)) return false;
        xyz[axis] = value;
      }
      return true;
    }
    for (uint32_t axis = 0; axis < 3; ++axis) {
      std::array<uint8_t, 8> bytes{};
      std::memcpy(bytes.data(), point + offsets[axis], sizes[axis]);
      if (has_byte_swap) std::reverse(bytes.begin(), bytes.begin() + sizes[axis]);
      if (sizes[axis] == 4) {
        float value;
        std::memcpy(&value, bytes.data(), 4);
        xyz[axis] = value;
      } else {
        double value;
        std::memcpy(&value, bytes.data(), 8);
        xyz[axis] = value;
      }
      if (!std::isfinite(xyz[axis])) return false;
    }
    return true;
  }
};
}  // namespace detail

// 16×16画素ごとの有効点数に比例した割当と領域内reservoir抽出。
// 非organized点群では連続256点ごとの領域。返値は間引き前の画素番号。
// max_points=0は無制限。同じ入力とseedに対して同じ選択。
template<class AcceptPoint>
inline std::vector<uint32_t> select_stratified_if(
    const sensor_msgs::msg::PointCloud2 &cloud, uint32_t max_points, uint32_t seed,
    AcceptPoint accept_point) {
  const detail::xyz_reader reader(cloud);
  if (reader.num_points == 0) return {};
  const bool is_organized = cloud.height > 1;
  const uint32_t num_columns = is_organized ? (cloud.width - 1) / 16 + 1 : 1;
  const uint32_t num_tiles = is_organized
      ? num_columns * ((cloud.height - 1) / 16 + 1) : (cloud.width - 1) / 256 + 1;
  const auto tile_of = [&](uint32_t idx) {
    return is_organized ? (idx / cloud.width / 16) * num_columns + idx % cloud.width / 16 : idx / 256;
  };
  std::vector<uint32_t> valid_indices;
  valid_indices.reserve(reader.num_points);
  std::vector<uint32_t> counts(num_tiles, 0);
  for (uint32_t idx = 0; idx < reader.num_points; ++idx) {
    std::array<double, 3> xyz{};
    if (reader.read(idx, xyz) && accept_point(idx, xyz)) {
      valid_indices.push_back(idx);
      ++counts[tile_of(idx)];
    }
  }
  if (max_points == 0 || valid_indices.size() <= max_points) return valid_indices;

  // 空領域に割当を消費しない累積比例配分。合計は厳密にmax_points。
  // 少数点の割当時にも固定領域だけが残らない共通乱数オフセット。
  std::mt19937 random(seed);
  const uint32_t allocation_offset = std::uniform_int_distribution<uint32_t>(0, valid_indices.size() - 1)(random);
  std::vector<uint32_t> starts(num_tiles + 1, 0);
  uint64_t cumulative = 0;
  for (uint32_t tile = 0; tile < num_tiles; ++tile) {
    cumulative += counts[tile];
    starts[tile + 1] = (cumulative * max_points + allocation_offset) / valid_indices.size();
  }
  std::fill(counts.begin(), counts.end(), 0);
  std::vector<uint32_t> selected(max_points);
  for (const uint32_t idx : valid_indices) {
    const uint32_t tile = tile_of(idx);
    const uint32_t capacity = starts[tile + 1] - starts[tile];
    if (capacity == 0) continue;
    const uint32_t seen = counts[tile]++;
    const uint32_t slot = seen < capacity ? seen : std::uniform_int_distribution<uint32_t>(0, seen)(random);
    if (slot < capacity) selected[starts[tile] + slot] = idx;
  }
  return selected;
}

// 条件なしの従来経路。最適化時の判定・座標保存の除去対象
inline std::vector<uint32_t> select_stratified(
    const sensor_msgs::msg::PointCloud2 &cloud, uint32_t max_points, uint32_t seed) {
  return select_stratified_if(cloud, max_points, seed,
      [](uint32_t, const std::array<double, 3> &) { return true; });
}

// 部分Fisher–Yatesによる重複なし候補抽出。少数抽出時は必要数までの条件判定。
// 大量抽出・大量除外時は連続走査の1 byte判定キャッシュへ切替。全点XYZ複製なし。
// enable_full_scanは全点ラベル等の用途。走査経路によらず同じ選択点・順序。
template<class AcceptPoint>
inline std::vector<uint32_t> select_random_if(
    const sensor_msgs::msg::PointCloud2 &cloud, uint32_t max_points, uint32_t seed,
    AcceptPoint accept_point, bool enable_full_scan = false) {
  const detail::xyz_reader reader(cloud);
  std::vector<uint32_t> selected(reader.num_points);
  std::iota(selected.begin(), selected.end(), 0U);
  const uint32_t limit = max_points ? std::min(max_points, reader.num_points) : reader.num_points;
  uint32_t num_selected = 0;
  std::vector<uint8_t> acceptance;
  const auto cache_remaining = [&](uint32_t first_pending) {
    // 0=未判定、1=採用、2=除外／訪問済み。訪問済み点の二重評価なし
    acceptance.assign(reader.num_points, first_pending ? 2 : 0);
    if (first_pending) for (uint32_t pos = first_pending; pos < reader.num_points; ++pos)
      acceptance[selected[pos]] = 0;
    for (uint32_t idx = 0; idx < reader.num_points; ++idx) {
      if (acceptance[idx]) continue;
      std::array<double, 3> xyz{};
      acceptance[idx] = reader.read(idx, xyz) && accept_point(idx, xyz) ? 1 : 2;
    }
  };
  const uint32_t max_direct_checks = reader.num_points / 4;
  if (enable_full_scan || limit > max_direct_checks) cache_remaining(0);
  std::mt19937 random(seed);
  for (uint32_t visited = 0; visited < reader.num_points; ++visited) {
    // 最初の1024候補での採用率による経路選択。抽選確率・順序への影響なし
    if (visited == 1024 && acceptance.empty() &&
        uint64_t(limit) * visited > uint64_t(num_selected) * max_direct_checks)
      cache_remaining(visited);
    const uint32_t slot = std::uniform_int_distribution<uint32_t>(visited, reader.num_points - 1)(random);
    const uint32_t idx = selected[slot];
    selected[slot] = selected[visited];
    std::array<double, 3> xyz{};
    const bool is_accepted = acceptance.empty()
        ? reader.read(idx, xyz) && accept_point(idx, xyz) : acceptance[idx] == 1;
    // 未訪問部分と重ならない選択済みプレフィックスへの格納。添字配列の追加確保なし
    if (is_accepted && num_selected < limit) selected[num_selected++] = idx;
    if (num_selected == limit) break;
  }
  selected.resize(num_selected);
  return selected;
}

inline std::vector<uint32_t> select_random(
    const sensor_msgs::msg::PointCloud2 &cloud, uint32_t max_points, uint32_t seed) {
  return select_random_if(cloud, max_points, seed,
      [](uint32_t, const std::array<double, 3> &) { return true; });
}

}  // namespace pointcloud_sampling
