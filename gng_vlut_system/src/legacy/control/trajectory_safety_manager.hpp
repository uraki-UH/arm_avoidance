#pragma once

#include "planner/RRT/state_validity_checker.hpp"
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace robot_sim {
namespace control {

/**
 * @brief Manages trajectory safety checks with configurable resolution and
 * lookahead.
 */
class TrajectorySafetyManager {
public:
  struct Config {
    double check_resolution;  // rad
    double min_safe_distance; // meters
    bool strict_mode;
    int skip_step;

    Config()
        : check_resolution(0.05), min_safe_distance(0.02), strict_mode(true),
          skip_step(1) {}
  };

  TrajectorySafetyManager(const Config &config = Config()) : config_(config) {}

  void setConfig(const Config &config) { config_ = config; }

  /**
   * @brief Checks if a path is valid given the validity checker.
   * @param path List of joint configurations.
   * @param validity_checker Checker instance.
   * @param start_index Index to start checking from (default 0).
   * @return true if safe, false if collision detected.
   */
  bool checkPathSafety(const std::vector<Eigen::VectorXd> &path,
                       const planner::StateValidityChecker *validity_checker,
                       size_t start_index = 0) const;

  /**
   * @brief Checks if a segment between two configs is safe.
   */
  bool checkSegmentSafety(
      const Eigen::VectorXd &q1, const Eigen::VectorXd &q2,
      const planner::StateValidityChecker *validity_checker) const;

private:
  Config config_;
};

} // namespace control
} // namespace robot_sim
