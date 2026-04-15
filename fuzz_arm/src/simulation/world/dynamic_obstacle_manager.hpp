#pragma once

#include <unordered_set>
#include <vector>

#include "common/geometry_utils.hpp"
#include "spatial/index_voxel_grid.hpp"
#include "spatial/ispatial_index.hpp"

namespace robot_sim {
namespace simulation {

struct DynamicObstacle {
  int id;
  Eigen::Vector3d position;
  double radius;
  std::vector<long> occupied_voxels;
  std::vector<long> danger_voxels;

  std::vector<long> prev_occupied_voxels;
  std::vector<long> prev_danger_voxels;

  Eigen::Vector3d filtered_velocity = Eigen::Vector3d::Zero();
  bool first_update = true;
  float prev_threshold = -1.0f;
};

/**
 * @brief Represents the change in voxel occupancy between two frames.
 */
struct VoxelDelta {
  std::vector<long> added;
  std::vector<long> removed;
};

/**
 * @brief Aggregated delta for both collision and danger voxels.
 */
struct GlobalVoxelDelta {
  VoxelDelta occupation;
  VoxelDelta danger;
};

class DynamicObstacleManager {
public:
  enum class PredictionMode { NONE, GT_VELOCITY, DIFF_CENTROID };

  DynamicObstacleManager(std::shared_ptr<analysis::ISpatialIndex> spatial_index,
                         double sensing_voxel_size, double lookup_voxel_size,
                         const std::vector<double> &min_bounds)
      : spatial_index_(spatial_index), voxel_size_(sensing_voxel_size),
        lookup_resolution_(lookup_voxel_size), min_bounds_(min_bounds) {
    if (lookup_resolution_ < 1e-6)
      lookup_resolution_ = voxel_size_;
  }

  void updateObstacle(int id, const Eigen::Vector3d &position, double radius,
                      std::vector<int> &collision_counts,
                      std::vector<int> &danger_counts, float threshold) {
    auto it =
        std::find_if(obstacles_.begin(), obstacles_.end(),
                     [id](const DynamicObstacle &obs) { return obs.id == id; });

    if (it == obstacles_.end()) {
      obstacles_.push_back({id,
                            position,
                            radius,
                            {},
                            {},
                            {},
                            {},
                            Eigen::Vector3d::Zero(),
                            true,
                            -1.0f});
      it = obstacles_.end() - 1;
    } else {
      // Optimization: Skip update if obstacle hasn't moved or changed size
      // significantly
      double dist_sq = (it->position - position).squaredNorm();
      double r_diff = std::abs(it->radius - radius);
      double t_diff = std::abs(it->prev_threshold - threshold);

      if (dist_sq < 1e-8 && r_diff < 1e-6 && t_diff < 1e-6) {
        return;
      }
    }

    std::vector<long> new_coll_voxels =
        ::common::geometry::VoxelUtils::getSphereVoxels(
            position.cast<float>(), (float)radius, (float)voxel_size_,
            [](const Eigen::Vector3i &idx) {
              return ::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);
            });
    std::vector<long> new_danger_voxels =
        ::common::geometry::VoxelUtils::getSphereVoxels(
            position.cast<float>(), (float)(radius + threshold),
            (float)voxel_size_, [](const Eigen::Vector3i &idx) {
              return ::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);
            });

    updateVoxelDiff(it->occupied_voxels, new_coll_voxels, collision_counts);
    updateVoxelDiff(it->danger_voxels, new_danger_voxels, danger_counts);

    it->position = position;
    it->radius = radius;
    it->prev_threshold = threshold;
    it->prev_occupied_voxels = it->occupied_voxels;
    it->occupied_voxels = new_coll_voxels;
    it->prev_danger_voxels = it->danger_voxels;
    it->danger_voxels = new_danger_voxels;
  }

  void clearObstacles(std::vector<int> &collision_counts,
                      std::vector<int> &danger_counts) {
    for (const auto &obs : obstacles_) {
      updateCountersOnly(obs.occupied_voxels, collision_counts, -1);
      updateCountersOnly(obs.danger_voxels, danger_counts, -1);
    }
    obstacles_.clear();
  }

  const std::vector<DynamicObstacle> &getObstacles() const {
    return obstacles_;
  }

  /**
   * @brief Update obstacle parameters WITHOUT updating global counters.
   * This is used when we want to handle unified delta updates later.
   */
  void updateObstacleParameters(int id, const Eigen::Vector3d &position,
                                double radius, float threshold) {
    auto it =
        std::find_if(obstacles_.begin(), obstacles_.end(),
                     [id](const DynamicObstacle &obs) { return obs.id == id; });

    if (it == obstacles_.end()) {
      DynamicObstacle obs;
      obs.id = id;
      obs.position = position;
      obs.radius = radius;
      obs.prev_threshold = threshold;
      obstacles_.push_back(obs);
      it = obstacles_.end() - 1;
    }

    it->position = position;
    it->radius = radius;
    it->prev_threshold = threshold;

    // Recalculate local voxels
    it->occupied_voxels = ::common::geometry::VoxelUtils::getSphereVoxels(
        position.cast<float>(), (float)radius, (float)voxel_size_,
        [](const Eigen::Vector3i &idx) {
          return ::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);
        });
    it->danger_voxels = ::common::geometry::VoxelUtils::getSphereVoxels(
        position.cast<float>(), (float)(radius + threshold), (float)voxel_size_,
        [](const Eigen::Vector3i &idx) {
          return ::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);
        });
  }

  /**
   * @brief Computes the global delta between current obstacle state and the
   * previous frame. This handles overlapping obstacles correctly by aggregating
   * them first.
   */
  GlobalVoxelDelta updateUnifiedOccupancy() {
    std::unordered_set<long> curr_occ_set;
    std::unordered_set<long> curr_dan_set;

    size_t total_occ = 0;
    size_t total_dan = 0;
    for (const auto &obs : obstacles_) {
      total_occ += obs.occupied_voxels.size();
      total_dan += obs.danger_voxels.size();
    }
    // Estimate capacities to minimize rehashes
    curr_occ_set.reserve(total_occ);
    curr_dan_set.reserve(total_dan);

    for (const auto &obs : obstacles_) {
      for (long vid : obs.occupied_voxels)
        curr_occ_set.insert(vid);
      for (long vid : obs.danger_voxels)
        curr_dan_set.insert(vid);
    }

    GlobalVoxelDelta delta;

    // Compute occupation deltas
    for (long vid : curr_occ_set) {
      if (unified_occupied_voxels_.find(vid) ==
          unified_occupied_voxels_.end()) {
        delta.occupation.added.push_back(vid);
      }
    }
    for (long vid : unified_occupied_voxels_) {
      if (curr_occ_set.find(vid) == curr_occ_set.end()) {
        delta.occupation.removed.push_back(vid);
      }
    }

    // Compute danger deltas
    for (long vid : curr_dan_set) {
      if (unified_danger_voxels_.find(vid) == unified_danger_voxels_.end()) {
        delta.danger.added.push_back(vid);
      }
    }
    for (long vid : unified_danger_voxels_) {
      if (curr_dan_set.find(vid) == curr_dan_set.end()) {
        delta.danger.removed.push_back(vid);
      }
    }

    // Update internal state
    unified_occupied_voxels_ = std::move(curr_occ_set);
    unified_danger_voxels_ = std::move(curr_dan_set);

    return delta;
  }

  double getVoxelSize() const { return voxel_size_; }

  const std::unordered_set<long> &getUnifiedOccupiedVoxels() const {
    return unified_occupied_voxels_;
  }

  Eigen::Vector3d calculateCentroid(const std::vector<long> &voxels) {
    return ::common::geometry::VoxelUtils::calculateCentroid(voxels,
                                                             (float)voxel_size_)
        .cast<double>();
  }

  Eigen::Vector3d estimateVelocityGlobal(DynamicObstacle &obs, double dt) {
    if (obs.prev_occupied_voxels.empty() || obs.occupied_voxels.empty() ||
        dt <= 1e-6)
      return Eigen::Vector3d::Zero();
    Eigen::Vector3d c_curr = calculateCentroid(obs.occupied_voxels);
    Eigen::Vector3d c_prev = calculateCentroid(obs.prev_occupied_voxels);
    Eigen::Vector3d raw_v = (c_curr - c_prev) / dt;
    double alpha = 0.05;
    if (obs.first_update) {
      obs.filtered_velocity = raw_v;
      obs.first_update = false;
    } else {
      if (raw_v.norm() < 0.01)
        raw_v = Eigen::Vector3d::Zero();
      obs.filtered_velocity =
          alpha * raw_v + (1.0 - alpha) * obs.filtered_velocity;
    }
    return obs.filtered_velocity;
  }

private:
  std::shared_ptr<analysis::ISpatialIndex> spatial_index_;
  std::vector<DynamicObstacle> obstacles_;
  double voxel_size_;
  double lookup_resolution_;
  std::vector<double> min_bounds_;

  std::unordered_set<long> unified_occupied_voxels_;
  std::unordered_set<long> unified_danger_voxels_;

  void updateVoxelDiff(std::vector<long> &old_voxels,
                       std::vector<long> &new_voxels,
                       std::vector<int> &counts) {
    std::vector<long> added, removed;
    ::common::geometry::VoxelUtils::computeDiff(old_voxels, new_voxels, added,
                                                removed);
    updateCountersOnly(removed, counts, -1);
    updateCountersOnly(added, counts, +1);
  }

  void updateCountersOnly(const std::vector<long> &voxels,
                          std::vector<int> &counts, int delta) {
    if (spatial_index_ && !voxels.empty()) {
      spatial_index_->updateCounts(voxels, counts, delta, 1e-6f);
    }
  }
};

} // namespace simulation
} // namespace robot_sim
