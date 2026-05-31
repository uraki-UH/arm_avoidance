#pragma once

#include <grasping_system/core/grasp_geometry.hpp>
#include <voxel_indexing_common/voxel_indexing.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace grasping_system::voxel
{

using VoxelKey = voxel_indexing_common::VoxelIndex;

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

  voxel_indexing_common::VoxelIndexingSchema &indexing() noexcept { return indexing_; }
  const voxel_indexing_common::VoxelIndexingSchema &indexing() const noexcept { return indexing_; }

private:
  voxel_indexing_common::VoxelIndexingSchema indexing_;
  std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash> cells_;
};

}  // namespace grasping_system::voxel
