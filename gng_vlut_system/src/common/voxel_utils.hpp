#pragma once

#include "core_safety/spatial/index_voxel_grid.hpp"
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

namespace common {
namespace geometry {

/**
 * @brief Utilities for conversion between world coordinates and voxel indices.
 * Simulation-independent version for Core Analysis.
 */
class VoxelUtils {
public:
  static Eigen::Vector3i worldToVoxel(const Eigen::Vector3f &pos,
                                      float voxel_size) {
    return (pos.array() / voxel_size).floor().cast<int>();
  }

  static Eigen::Vector3f voxelToWorld(const Eigen::Vector3i &idx,
                                      float voxel_size) {
    return idx.cast<float>() * voxel_size +
           Eigen::Vector3f::Constant(voxel_size * 0.5f);
  }

  /**
   * @brief Get a set of flat voxel IDs covered by a sphere.
   */
  template <typename IdProvider>
  static std::vector<long> getSphereVoxels(const Eigen::Vector3f &center,
                                           float radius, float voxel_size,
                                           IdProvider id_provider) {
    std::vector<long> voxels;
    Eigen::Vector3i min_idx =
        worldToVoxel(center - Eigen::Vector3f::Constant(radius), voxel_size);
    Eigen::Vector3i max_idx =
        worldToVoxel(center + Eigen::Vector3f::Constant(radius), voxel_size);

    int nx = max_idx.x() - min_idx.x() + 1;
    int ny = max_idx.y() - min_idx.y() + 1;
    int nz = max_idx.z() - min_idx.z() + 1;
    if (nx > 0 && ny > 0 && nz > 0) {
      voxels.reserve(nx * ny * nz);
    }

    float r_sq = radius * radius;
    float half_v = voxel_size * 0.5f;

    float z_world_base = min_idx.z() * voxel_size + half_v - center.z();
    float y_world_base = min_idx.y() * voxel_size + half_v - center.y();
    float x_world_base = min_idx.x() * voxel_size + half_v - center.x();

    for (int z = min_idx.z(), k = 0; z <= max_idx.z(); ++z, ++k) {
      float dz = z_world_base + k * voxel_size;
      float dz_sq = dz * dz;

      for (int y = min_idx.y(), j = 0; y <= max_idx.y(); ++y, ++j) {
        float dy = y_world_base + j * voxel_size;
        float dzy_sq = dz_sq + dy * dy;

        for (int x = min_idx.x(), i = 0; x <= max_idx.x(); ++x, ++i) {
          float dx = x_world_base + i * voxel_size;
          if (dzy_sq + dx * dx <= r_sq) {
            Eigen::Vector3i current_idx(x, y, z);
            long flat_id = id_provider(current_idx);
            if (flat_id >= 0)
              voxels.push_back(flat_id);
          }
        }
      }
    }
    return voxels;
  }

  /**
   * @brief Compute the centroid of a set of voxels.
   */
  static Eigen::Vector3f calculateCentroid(const std::vector<long> &voxels,
                                           float voxel_size) {
    if (voxels.empty())
      return Eigen::Vector3f::Zero();
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    for (long vid : voxels) {
      Eigen::Vector3i idx =
          ::GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);
      sum += (idx.cast<float>() + Eigen::Vector3f::Constant(0.5f)) * voxel_size;
    }
    return sum / (float)voxels.size();
  }

  /**
   * @brief Compute added and removed voxels between two sets (Vector-Sort-Unique ready).
   */
  static void computeDiff(const std::vector<long> &old_voxels,
                          const std::vector<long> &new_voxels,
                          std::vector<long> &added,
                          std::vector<long> &removed) {
    std::vector<long> sorted_old = old_voxels;
    std::vector<long> sorted_new = new_voxels;
    std::sort(sorted_old.begin(), sorted_old.end());
    std::sort(sorted_new.begin(), sorted_new.end());

    std::set_difference(sorted_old.begin(), sorted_old.end(),
                        sorted_new.begin(), sorted_new.end(),
                        std::back_inserter(removed));
    std::set_difference(sorted_new.begin(), sorted_new.end(),
                        sorted_old.begin(), sorted_old.end(),
                        std::back_inserter(added));
  }
};

} // namespace geometry
} // namespace common
