#include "collision/voxel_grid.hpp"

namespace simulation {

VoxelGrid::VoxelGrid(double voxel_size) : voxel_size_(voxel_size) {}

void VoxelGrid::clear() {
  grid_.clear();
}

Eigen::Vector3i VoxelGrid::getIndex(const Eigen::Vector3d &p) const {
  return (p / voxel_size_).array().floor().cast<int>();
}

void VoxelGrid::insert(const Eigen::Vector3d &p) {
  grid_[getIndex(p)].push_back(p);
}

void VoxelGrid::insert(const std::vector<Eigen::Vector3d> &points) {
  for (const auto &p : points) {
    insert(p);
  }
}

void VoxelGrid::getPointsInAABB(const collision::AABB &aabb,
                               std::vector<Eigen::Vector3d> &out_points) const {
  out_points.clear();

  Eigen::Vector3i min_idx = getIndex(aabb.min);
  Eigen::Vector3i max_idx = getIndex(aabb.max);

  for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
    for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
      for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
        auto it = grid_.find(Eigen::Vector3i(x, y, z));
        if (it != grid_.end()) {
          out_points.insert(out_points.end(), it->second.begin(), it->second.end());
        }
      }
    }
  }
}

} // namespace simulation
