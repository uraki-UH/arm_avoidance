#pragma once

#include <Eigen/Dense>

namespace robot_sim {
namespace planner {

/**
 * @brief Abstract interface for state validity checking in configuration space.
 */
class StateValidityChecker {
public:
  virtual ~StateValidityChecker() = default;

  /**
   * @brief Check if a given state (joint angles) is valid.
   * @param q Joint angles vector.
   * @return true if the state is valid (no collisions, within limits, etc.)
   */
  virtual bool isValid(const Eigen::VectorXd &q) const = 0;

  /**
   * @brief Reset metrics (count, time).
   */
  virtual void resetMetrics() const {}

  /**
   * @brief Get the total number of validity checks performed.
   */
  virtual size_t getCheckCount() const { return 0; }

  /**
   * @brief Get the total time spent on validity checks in microseconds.
   */
  virtual double getTotalCheckTimeUs() const { return 0.0; }
};

} // namespace planner
} // namespace robot_sim
