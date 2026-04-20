#pragma once

#include "collision/point_cloud_collision_checker.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include "collision/voxel_grid.hpp"
#include <chrono>
#include <vector>

namespace robot_sim {
namespace planner {

/**
 * @brief Validity checker that uses a VoxelGrid and PointCloudCollisionChecker.
 */
class VoxelValidityChecker : public StateValidityChecker {
public:
  VoxelValidityChecker(const simulation::PointCloudCollisionChecker &checker,
                       const simulation::VoxelGrid &grid,
                       const collision::AABB &pc_bounds)
      : checker_(checker), grid_(grid), pc_bounds_(pc_bounds) {}

  bool isValid(const Eigen::VectorXd &q) const override {
    auto start = std::chrono::high_resolution_clock::now();
    check_count_++;

    // Convert Eigen::VectorXd to std::vector<double> for the checker
    std::vector<double> joint_vals(q.data(), q.data() + q.size());

    // PointCloudCollisionChecker returns true if colliding
    bool collided = checker_.checkCollision(joint_vals, grid_, pc_bounds_);

    auto end = std::chrono::high_resolution_clock::now();
    total_check_time_ms_ +=
        std::chrono::duration<double, std::milli>(end - start).count();

    return !collided;
  }

  void resetMetrics() const override {
    check_count_ = 0;
    total_check_time_ms_ = 0.0;
  }

  size_t getCheckCount() const override { return check_count_; }

  double getTotalCheckTimeMs() const override { return total_check_time_ms_; }

private:
  const simulation::PointCloudCollisionChecker &checker_;
  const simulation::VoxelGrid &grid_;
  const collision::AABB pc_bounds_;

  mutable size_t check_count_ = 0;
  mutable double total_check_time_ms_ = 0.0;
};

} // namespace planner
} // namespace robot_sim
