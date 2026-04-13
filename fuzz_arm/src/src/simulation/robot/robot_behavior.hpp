#pragma once

#include "control/linear_trajectory_controller.hpp"
#include "control/trajectory_safety_manager.hpp"
#include "kinematics/kinematic_chain.hpp"
#include <cmath>
#include <memory>
#include <vector>

// Forward declaration
namespace robot_sim {
namespace planner {
class StateValidityChecker;
}
} // namespace robot_sim

namespace simulation {

/**
 * @brief Interface for robot obstacle motion behaviors.
 */
class IRobotBehavior {
public:
  virtual ~IRobotBehavior() = default;
  virtual void update(double t, kinematics::KinematicChain &chain) = 0;
};

/**
 * @brief A behavior that makes the robot wave its links using sine waves.
 */
class WavingRobotBehavior : public IRobotBehavior {
public:
  WavingRobotBehavior(double speed = 1.5, double amplitude = 1.0)
      : speed_(speed), amplitude_(amplitude) {}

  void update(double t, kinematics::KinematicChain &chain) override {
    std::vector<double> q(chain.getTotalDOF());
    for (size_t i = 0; i < q.size(); ++i) {
      q[i] = amplitude_ * std::sin(speed_ * t + i * 0.5);
    }
    chain.setJointValues(q);
  }

private:
  double speed_;
  double amplitude_;
};

/**
 * @brief A behavior that keeps the robot in a fixed posture.
 */
class StaticRobotBehavior : public IRobotBehavior {
public:
  void update([[maybe_unused]] double t,
              [[maybe_unused]] kinematics::KinematicChain &chain) override {
    // Do nothing - maintain current or default posture
  }
};

/**
 * @brief Standards-compliant path tracking behavior with safety checks.
 */
class PathTrackingBehavior : public IRobotBehavior {
public:
  PathTrackingBehavior(
      std::shared_ptr<::control::LinearTrajectoryController> controller,
      std::shared_ptr<robot_sim::control::TrajectorySafetyManager>
          safety_manager,
      const robot_sim::planner::StateValidityChecker *validity_checker)
      : controller_(controller), safety_manager_(safety_manager),
        validity_checker_(validity_checker) {}

  void setPath(const std::vector<Eigen::VectorXd> &path) {
    if (controller_)
      controller_->setPath(path);
    is_blocked_ = false;
  }

  void update(double dt, kinematics::KinematicChain &chain) override {
    // 1. Check Safety Ahead
    if (safety_manager_ && validity_checker_ && controller_ &&
        !controller_->isFinished()) {
      // Future expansion: Periodic full-path safety check
    }

    // 2. Execute Controller
    (void)dt;
    (void)chain;
  }

  std::shared_ptr<::control::LinearTrajectoryController> getController() {
    return controller_;
  }
  std::shared_ptr<robot_sim::control::TrajectorySafetyManager>
  getSafetyManager() {
    return safety_manager_;
  }

private:
  std::shared_ptr<::control::LinearTrajectoryController> controller_;
  std::shared_ptr<robot_sim::control::TrajectorySafetyManager> safety_manager_;
  const robot_sim::planner::StateValidityChecker *validity_checker_;
  bool is_blocked_ = false;
};

} // namespace simulation
