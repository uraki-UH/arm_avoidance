#pragma once

#include <core/grasp_geometry.hpp>
#include <voxel_idx.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace grasping_system::voxel
{

using VoxelKey = voxel_idx::VoxelIndex;

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey &key) const noexcept
  {
    std::size_t seed = static_cast<std::size_t>(key.x);
    seed ^= static_cast<std::size_t>(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= static_cast<std::size_t>(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct VoxelCell
{
  VoxelKey key{};
  double occupancy{1.0};
  bool active{true};
};

class GraspVoxelModel : public core::GraspGeometryModel
{
public:
  static GraspVoxelModel makeBox(
    const std::array<double, 3> &dimensions,
    const voxel_idx::VoxelIndexingSchema &indexing)
  {
    GraspVoxelModel model;
    model.indexing() = indexing;
    if (indexing.voxel_size <= 0.0 ||
        std::any_of(dimensions.begin(), dimensions.end(),
                    [](double size) { return !std::isfinite(size) || size <= 0.0; })) {
      return model;
    }

    std::array<int, 3> cell_counts{};
    for (std::size_t axis = 0; axis < cell_counts.size(); ++axis) {
      const double cell_ratio = dimensions[axis] / indexing.voxel_size;
      const double nearest_integer = std::round(cell_ratio);
      const double stable_ratio = std::abs(cell_ratio - nearest_integer) < 1e-6
        ? nearest_integer
        : cell_ratio;
      cell_counts[axis] = std::max(1, static_cast<int>(std::ceil(stable_ratio)));
    }
    const std::array<int, 3> starts{
      -cell_counts[0] / 2, -cell_counts[1] / 2, -cell_counts[2] / 2};

    for (int x = starts[0]; x < starts[0] + cell_counts[0]; ++x) {
      for (int y = starts[1]; y < starts[1] + cell_counts[1]; ++y) {
        for (int z = starts[2]; z < starts[2] + cell_counts[2]; ++z) {
          VoxelCell cell;
          cell.key = {x, y, z};
          cell.occupancy = 1.0;
          cell.active = true;
          model.addCell(cell);
        }
      }
    }
    return model;
  }

  core::GraspGeometryKind kind() const noexcept override
  {
    return core::GraspGeometryKind::kVoxel;
  }
  std::string typeName() const override { return "voxel"; }

  std::unique_ptr<core::GraspGeometryModel> clone() const override
  {
    return std::make_unique<GraspVoxelModel>(*this);
  }

  void clear()
  {
    cells_.clear();
  }

  void addCell(const VoxelCell &cell)
  {
    cells_[cell.key] = cell;
  }

  const std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash> &cells() const noexcept
  {
    return cells_;
  }

  const VoxelCell *findCell(const VoxelKey &key) const noexcept
  {
    const auto it = cells_.find(key);
    return it == cells_.end() ? nullptr : &it->second;
  }

  voxel_idx::VoxelIndexingSchema &indexing() noexcept { return indexing_; }
  const voxel_idx::VoxelIndexingSchema &indexing() const noexcept { return indexing_; }

private:
  voxel_idx::VoxelIndexingSchema indexing_;
  std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash> cells_;
};

}  // namespace grasping_system::voxel
