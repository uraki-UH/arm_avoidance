#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace robot_sim {
namespace analysis {

/**
 * @brief Interface for spatial indexing of GNG nodes.
 * This abstraction allows switching between different implementations
 * (e.g., Sparse Hash Map vs. Dense CSR Grid) without changing the simulation
 * logic.
 */
class ISpatialIndex {
public:
  virtual ~ISpatialIndex() = default;

  /**
   * @brief Get the minimum bounds of the workspace covered by this index.
   */
  virtual Eigen::Vector3d getWorldMin() const = 0;

  /**
   * @brief Get the voxel resolution (size) used by this index.
   */
  virtual double getVoxelSize() const = 0;

  /**
   * @brief Load the pre-computed spatial correlation data.
   */
  virtual bool load(const std::string &filename) = 0;

  /**
   * @brief Convert a world coordinate to a flat voxel ID used by this index.
   */
  virtual long getVoxelId(const Eigen::Vector3d &pos) const = 0;

  /**
   * @brief Query an Axis-Aligned Bounding Box (AABB) and update node collision
   * counters. This is the primary interface for dynamic obstacle collision
   * detection.
   *
   * @param min_pt Minimum world coordinates of the AABB.
   * @param max_pt Maximum world coordinates of the AABB.
   * @param counts Global vector to update (increment/decrement) collision
   * counts.
   * @param delta The value to add to the counter (typically +1 for entry, -1
   * for exit).
   */
  virtual void clear() = 0;
  virtual void insert(int id, const Eigen::Vector3d &pos) = 0;
  virtual void build() {} // Optional build step for CSR/static structures

  virtual void queryAABB(const Eigen::Vector3d &min_pt,
                         const Eigen::Vector3d &max_pt,
                         std::vector<int> &collision_counts,
                         std::vector<int> &danger_counts, float threshold,
                         int delta) const = 0;

  virtual void queryAABB(const Eigen::Vector3d &min_pt,
                         const Eigen::Vector3d &max_pt,
                         std::vector<int> &counts, int delta) const = 0;

  /**
   * @brief Update node collision/danger counters for a list of voxels.
   * This is a low-level, high-performance batch operation used for
   * differential updates (incremental changes).
   *
   * @param voxel_ids List of flat voxel IDs to process.
   * @param counts Global counter vector to update (typically
   * node_collision_counts).
   * @param delta The value to add (increment) to the counters (typically +1 or
   * -1).
   * @param threshold Distance threshold to update node counts (if < 0, uses
   * strict containment).
   */
  virtual void updateCounts(const std::vector<long> &voxel_ids,
                            std::vector<int> &counts, int delta,
                            float threshold = -1.0f) const = 0;

  /**
   * @brief Get all node IDs located in the voxel containing the given point.
   */
  virtual std::vector<int>
  getNodesInVoxel(const Eigen::Vector3d &point) const = 0;

  /**
   * @brief Get all node IDs located in the voxel specified by its flat ID.
   */
  virtual std::vector<int> getNodesInVoxel(long voxel_id) const = 0;
};

} // namespace analysis
} // namespace robot_sim
