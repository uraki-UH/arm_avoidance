#pragma once

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include "collision/collision_detector.hpp" // For AABB

namespace simulation {

/**
 * @brief A simple 3D Voxel Grid for efficient Point Cloud queries.
 * Replaces the legacy simulation sensing version.
 */
class VoxelGrid {
public:
  VoxelGrid(double voxel_size = 0.05);
  ~VoxelGrid() = default;

  void clear();
  void insert(const Eigen::Vector3d &p);
  void insert(const std::vector<Eigen::Vector3d> &points);

  /**
   * @brief Get all points contained within a given axis-aligned bounding box.
   */
  void getPointsInAABB(const collision::AABB &aabb, std::vector<Eigen::Vector3d> &out_points) const;

  double getVoxelSize() const { return voxel_size_; }

private:
  struct Vector3iHash {
    std::size_t operator()(const Eigen::Vector3i &v) const {
      std::size_t h = 0;
      h ^= std::hash<int>{}(v.x()) + 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= std::hash<int>{}(v.y()) + 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= std::hash<int>{}(v.z()) + 0x9e3779b9 + (h << 6) + (h >> 2);
      return h;
    }
  };

  Eigen::Vector3i getIndex(const Eigen::Vector3d &p) const;

  double voxel_size_;
  std::unordered_map<Eigen::Vector3i, std::vector<Eigen::Vector3d>, Vector3iHash> grid_;
};

} // namespace simulation
