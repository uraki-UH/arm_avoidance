#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace voxel_idx
{

struct VoxelIndex
{
  int x{0};
  int y{0};
  int z{0};

  constexpr bool operator==(const VoxelIndex &other) const noexcept
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelIndexingSchema
{
  int x_shift{42};
  int y_shift{21};
  int z_shift{0};
  std::int64_t offset{1000000};
  double voxel_size{0.0};

  constexpr bool isValid() const noexcept
  {
    return x_shift > y_shift && y_shift > z_shift && z_shift >= 0 && offset >= 0;
  }

  constexpr bool matches(const VoxelIndexingSchema &other) const noexcept
  {
    return x_shift == other.x_shift && y_shift == other.y_shift &&
           z_shift == other.z_shift && offset == other.offset &&
           voxel_size == other.voxel_size;
  }

  std::uint64_t pack(const VoxelIndex &idx) const
  {
    if (!isValid()) {
      throw std::runtime_error("Invalid voxel indexing schema");
    }
    return (static_cast<std::uint64_t>(static_cast<std::int64_t>(idx.x) + offset) << x_shift) |
           (static_cast<std::uint64_t>(static_cast<std::int64_t>(idx.y) + offset) << y_shift) |
           (static_cast<std::uint64_t>(static_cast<std::int64_t>(idx.z) + offset) << z_shift);
  }

  VoxelIndex unpack(std::uint64_t id) const
  {
    if (!isValid()) {
      throw std::runtime_error("Invalid voxel indexing schema");
    }

    const std::uint64_t y_mask = (1ULL << (x_shift - y_shift)) - 1ULL;
    const std::uint64_t z_mask = (1ULL << (y_shift - z_shift)) - 1ULL;

    const auto x = static_cast<std::int64_t>(id >> x_shift) - offset;
    const auto y = static_cast<std::int64_t>((id >> y_shift) & y_mask) - offset;
    const auto z = static_cast<std::int64_t>((id >> z_shift) & z_mask) - offset;

    return VoxelIndex{static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)};
  }

  static VoxelIndex worldToIndex(double x, double y, double z, double voxel_size)
  {
    return VoxelIndex{
      static_cast<int>(std::floor(x / voxel_size)),
      static_cast<int>(std::floor(y / voxel_size)),
      static_cast<int>(std::floor(z / voxel_size))};
  }

  static std::array<double, 3> indexToWorldCenter(const VoxelIndex &idx, double voxel_size)
  {
    return {
      (static_cast<double>(idx.x) + 0.5) * voxel_size,
      (static_cast<double>(idx.y) + 0.5) * voxel_size,
      (static_cast<double>(idx.z) + 0.5) * voxel_size};
  }
};

}  // namespace voxel_idx