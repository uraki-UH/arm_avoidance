#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "voxel_indexing_common/voxel_indexing.hpp"

namespace GNG
{
namespace Analysis
{

/**
 * @brief Shared voxel indexing wrapper for legacy GNG code.
 *
 * This keeps the existing API used across gng_vlut_system while delegating
 * the actual voxel id contract to voxel_indexing_common.
 */
class IndexVoxelGrid
{
public:
  explicit IndexVoxelGrid(double voxel_size) : schema_{}
  {
    schema_.voxel_size = voxel_size;
  }

  void setIndexingParams(int x, int y, int z, long off)
  {
    schema_.x_shift = x;
    schema_.y_shift = y;
    schema_.z_shift = z;
    schema_.offset = off;
  }

  void setVoxelSize(double size) { schema_.voxel_size = size; }
  double getVoxelSize() const { return schema_.voxel_size; }

  long getFlatVoxelId(const Eigen::Vector3i &idx) const
  {
    const auto flat = schema_.pack(
      voxel_indexing_common::VoxelIndex{idx.x(), idx.y(), idx.z()});
    return static_cast<long>(flat);
  }

  Eigen::Vector3i getIndexFromFlatId(long id) const
  {
    const auto idx = schema_.unpack(static_cast<std::uint64_t>(id));
    return Eigen::Vector3i(idx.x, idx.y, idx.z);
  }

  int getXShift() const { return schema_.x_shift; }
  int getYShift() const { return schema_.y_shift; }
  int getZShift() const { return schema_.z_shift; }
  long getOffset() const { return static_cast<long>(schema_.offset); }

  const voxel_indexing_common::VoxelIndexingSchema &schema() const noexcept
  {
    return schema_;
  }

  void setSchema(const voxel_indexing_common::VoxelIndexingSchema &schema)
  {
    schema_ = schema;
  }

private:
  voxel_indexing_common::VoxelIndexingSchema schema_;

  // LUTとしての役割（将来的に使用）
  std::unordered_map<long, std::vector<int>> voxel_to_ids_;
  std::unordered_map<int, std::vector<long>> id_to_voxels_;
};

}  // namespace Analysis
}  // namespace GNG
