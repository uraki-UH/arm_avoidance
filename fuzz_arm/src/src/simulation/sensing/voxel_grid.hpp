#pragma once

#include "collision/collision_detector.hpp"
#include <Eigen/Dense>
#include <map>
#include <vector>

namespace simulation {

/**
 * @brief A simple voxel grid for spatial partitioning of point clouds.
 *
 * Allows for fast $O(1)$ point insertion and fast range queries by voxel.
 */
class VoxelGrid {
public:
  VoxelGrid(double voxel_size) : voxel_size_(voxel_size) {}

  void clear() { grid_.clear(); }

  /**
   * @brief Insert a point into the grid.
   */
  void insert(const Eigen::Vector3d &p) {
    Eigen::Vector3i index = (p / voxel_size_).array().floor().cast<int>();
    grid_[index].push_back(p);
  }

  /**
   * @brief Find all points that might intersect with the given AABB.
   *
   * Extends the AABB by voxel boundaries and returns points in those voxels.
   */
  void getPointsInAABB(const collision::AABB &bounds,
                       std::vector<Eigen::Vector3d> &result) const {
    Eigen::Vector3i min_idx =
        (bounds.min / voxel_size_).array().floor().cast<int>();
    Eigen::Vector3i max_idx =
        (bounds.max / voxel_size_).array().floor().cast<int>();

    for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
      for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
        for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
          auto it = grid_.find({x, y, z});
          if (it != grid_.end()) {
            result.insert(result.end(), it->second.begin(), it->second.end());
          }
        }
      }
    }
  }

private:
  struct Vector3iComparator {
    bool operator()(const Eigen::Vector3i &a, const Eigen::Vector3i &b) const {
      if (a.x() != b.x())
        return a.x() < b.x();
      if (a.y() != b.y())
        return a.y() < b.y();
      return a.z() < b.z();
    }
  };

  double voxel_size_;
  std::map<Eigen::Vector3i, std::vector<Eigen::Vector3d>, Vector3iComparator>
      grid_;
};

} // namespace simulation
