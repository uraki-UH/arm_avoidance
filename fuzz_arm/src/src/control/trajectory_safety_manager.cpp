#include "control/trajectory_safety_manager.hpp"
namespace robot_sim {
namespace control {

bool TrajectorySafetyManager::checkPathSafety(
    const std::vector<Eigen::VectorXd> &path,
    const planner::StateValidityChecker *validity_checker,
    size_t start_index) const {
  if (!validity_checker || path.empty() || start_index >= path.size()) {
    return true; // Empty path or invalid index is trivially "safe" in context
                 // of "no collision found"
  }

  // Iterate through path waypoints
  for (size_t i = start_index; i < path.size(); ++i) {
    // 1. Check waypoint itself
    // Optimization: In strict mode with skip_step > 1, we might skip some
    // waypoints, but for now let's assume we want to check at least the
    // waypoints if skip_step is small.

    if (config_.strict_mode || (i % config_.skip_step == 0) ||
        i == path.size() - 1) {
      if (!validity_checker->isValid(path[i])) {
        return false;
      }
    }

    // 2. Check segment to next waypoint (Interpolation)
    if (i < path.size() - 1) {
      if (config_.strict_mode) {
        if (!checkSegmentSafety(path[i], path[i + 1], validity_checker)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool TrajectorySafetyManager::checkSegmentSafety(
    const Eigen::VectorXd &q1, const Eigen::VectorXd &q2,
    const planner::StateValidityChecker *validity_checker) const {
  if (!validity_checker)
    return true;

  double dist = (q2 - q1).norm();
  if (dist < 1e-6)
    return true;

  int steps = std::max(1, static_cast<int>(dist / config_.check_resolution));

  for (int k = 1; k < steps; ++k) {
    double t = static_cast<double>(k) / steps;
    Eigen::VectorXd q_interp = q1 + t * (q2 - q1);
    if (!validity_checker->isValid(q_interp)) {
      return false;
    }
  }
  return true;
}

} // namespace control
} // namespace robot_sim
