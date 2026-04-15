#include "simulation/planning/ode_state_validity_checker.hpp"
#include <vector>

namespace robot_sim {
namespace simulation {

OdeStateValidityChecker::OdeStateValidityChecker(
    ::simulation::OdeRobotCollisionModel *model,
    ::simulation::PointCloudCollisionChecker *pc_checker,
    const ::simulation::VoxelGrid *grid,
    const ::simulation::EnvironmentManager *env_manager,
    const ::kinematics::KinematicChain *chain)
    : collision_model_(model), pc_checker_(pc_checker), grid_(grid),
      env_manager_(env_manager), chain_(chain) {}

bool OdeStateValidityChecker::isValid(const Eigen::VectorXd &q) const {
  return isValid(q, 0.0);
}

bool OdeStateValidityChecker::isValid(const Eigen::VectorXd &q,
                                      double margin) const {
  auto start_time = std::chrono::high_resolution_clock::now();
  check_count_++;

  if (!chain_) {
    auto end_time = std::chrono::high_resolution_clock::now();
    total_check_time_us_ +=
        std::chrono::duration<double, std::micro>(end_time - start_time)
            .count();
    return true;
  }

  int dof = chain_->getTotalDOF();
  int provided_size = (int)q.size();
  int actual_size = std::min(provided_size, dof);
  std::vector<double> q_vec(actual_size);
  for (int i = 0; i < actual_size; ++i) {
    q_vec[i] = q(i);
  }

  // 0. Joint limits check
  if (!chain_->isWithinLimits(q_vec)) {
    auto end_time = std::chrono::high_resolution_clock::now();
    total_check_time_us_ +=
        std::chrono::duration<double, std::micro>(end_time - start_time)
            .count();
    return false;
  }

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      positions;
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
      orientations;
  chain_->forwardKinematicsAt(q_vec, positions, orientations);
  // Note: margin is currently only applied to Point Cloud check
  if (collision_model_) {
    collision_model_->updateBodyPoses(positions, orientations);
    if (collision_model_->updateCollisionStatus()) {
      if (collision_model_->hasSelfCollision() ||
          collision_model_->hasEnvironmentCollision()) {
        auto end_time = std::chrono::high_resolution_clock::now();
        total_check_time_us_ +=
            std::chrono::duration<double, std::micro>(end_time - start_time)
                .count();
        return false;
      }
    }
  }

  // 2. Point Cloud Collision (with margin)
  if (pc_checker_ && grid_ && env_manager_) {
    if (has_exclusion_ && !positions.empty()) {
      Eigen::Vector3d eef_p = positions.back();
      if ((eef_p - exclusion_pos_).norm() < exclusion_radius_) {
        auto end_time = std::chrono::high_resolution_clock::now();
        total_check_time_us_ +=
            std::chrono::duration<double, std::micro>(end_time - start_time)
                .count();
        return true;
      }
    }

    if (pc_checker_->checkCollision(
            q_vec, *grid_, env_manager_->getPointCloudBounds(), margin)) {
      auto end_time = std::chrono::high_resolution_clock::now();
      total_check_time_us_ +=
          std::chrono::duration<double, std::micro>(end_time - start_time)
              .count();
      return false;
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  total_check_time_us_ +=
      std::chrono::duration<double, std::micro>(end_time - start_time).count();
  return true;
}

void OdeStateValidityChecker::resetMetrics() const {
  check_count_ = 0;
  total_check_time_us_ = 0.0;
}

size_t OdeStateValidityChecker::getCheckCount() const { return check_count_; }

double OdeStateValidityChecker::getTotalCheckTimeUs() const {
  return total_check_time_us_;
}

} // namespace simulation
} // namespace robot_sim
