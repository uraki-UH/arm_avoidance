#pragma once

#include <Eigen/Dense>
#include <utility>
#include <vector>

namespace GNG {
namespace Analysis {

/**
 * @brief Utility to generate wireframe segments from a set of voxels.
 */
class VoxelWireframeGenerator {
public:
  /**
   * @brief Generates a list of line segments (start, end) that represent the
   * voxel grid.
   * @param voxels List of voxel indices.
   * @param voxel_size Size of each voxel.
   * @return Vector of pairs of points representing line segments.
   */
  static std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>
  generate(const std::vector<Eigen::Vector3i> &voxels, double voxel_size);

  /**
   * @brief Generates only the exterior surface edges of the voxel cloud.
   * (More complex, but cleaner visualization)
   */
  static std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>
  generateSurface(const std::vector<Eigen::Vector3i> &voxels,
                  double voxel_size);
};

} // namespace Analysis
} // namespace GNG
