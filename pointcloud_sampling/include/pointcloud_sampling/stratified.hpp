#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace pointcloud_sampling {

// 16×16画素ごとの有効点数に比例した割当と領域内reservoir抽出。
// 非organized点群では連続256点ごとの領域。返値は間引き前の画素番号。
// max_points=0は無制限。同じ入力とseedに対して同じ選択。
inline std::vector<uint32_t> select_stratified(
    const sensor_msgs::msg::PointCloud2 &cloud, uint32_t max_points, uint32_t seed) {
  const uint64_t num_points = uint64_t(cloud.width) * cloud.height;
  if (num_points == 0) return {};
  if (num_points > std::numeric_limits<uint32_t>::max() || cloud.point_step == 0 ||
      uint64_t(cloud.width) * cloud.point_step > cloud.row_step ||
      uint64_t(cloud.height - 1) * cloud.row_step + uint64_t(cloud.width) * cloud.point_step > cloud.data.size()) {
    throw std::invalid_argument("Invalid PointCloud2 layout");
  }
  std::array<uint32_t, 3> offsets{};
  std::array<uint32_t, 3> sizes{};
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
  const bool has_byte_swap = cloud.is_bigendian != is_native_big_endian;
  const bool is_organized = cloud.height > 1;
  const uint32_t num_columns = is_organized ? (cloud.width - 1) / 16 + 1 : 1;
  const uint32_t num_tiles = is_organized
      ? num_columns * ((cloud.height - 1) / 16 + 1) : (cloud.width - 1) / 256 + 1;
  const auto tile_of = [&](uint32_t idx) {
    return is_organized ? (idx / cloud.width / 16) * num_columns + idx % cloud.width / 16 : idx / 256;
  };
  std::vector<uint32_t> valid_indices;
  valid_indices.reserve(num_points);
  std::vector<uint32_t> counts(num_tiles, 0);
  for (uint32_t idx = 0; idx < num_points; ++idx) {
    const auto *point = cloud.data.data() + uint64_t(idx / cloud.width) * cloud.row_step +
        uint64_t(idx % cloud.width) * cloud.point_step;
    bool is_valid = true;
    for (uint32_t axis = 0; axis < 3 && is_valid; ++axis) {
      std::array<uint8_t, 8> bytes{};
      std::memcpy(bytes.data(), point + offsets[axis], sizes[axis]);
      if (has_byte_swap) std::reverse(bytes.begin(), bytes.begin() + sizes[axis]);
      if (sizes[axis] == 4) {
        float value;
        std::memcpy(&value, bytes.data(), 4);
        is_valid = std::isfinite(value);
      } else {
        double value;
        std::memcpy(&value, bytes.data(), 8);
        is_valid = std::isfinite(value);
      }
    }
    if (is_valid) { valid_indices.push_back(idx); ++counts[tile_of(idx)]; }
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

// 領域割当なしの全有効点シャッフルと先頭抽出。上限以下でも順序変更。
inline std::vector<uint32_t> select_random(
    const sensor_msgs::msg::PointCloud2 &cloud, uint32_t max_points, uint32_t seed) {
  auto selected = select_stratified(cloud, 0, seed);
  std::mt19937 random(seed);
  std::shuffle(selected.begin(), selected.end(), random);
  if (max_points != 0 && selected.size() > max_points) selected.resize(max_points);
  return selected;
}

}  // namespace pointcloud_sampling
