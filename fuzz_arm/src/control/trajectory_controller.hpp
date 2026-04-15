#pragma once

#include "control/robot_interface.hpp"
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace control {

/**
 * Priority for robot control commands.
 */
enum class CommandPriority {
  LOW_TRAJECTORY = 0,  // Normal autonomous path following
  MEDIUM_ADAPTIVE = 1, // Dynamic adjustments (e.g., smoothing, minor avoidance)
  HIGH_SAFETY = 2,     // Safety overrides (emergency stops, collision evasion)
  CRITICAL_HALT = 3    // System-level halt
};

/**
 * Container for a control command with priority metadata.
 */
struct ControlCommand {
  Eigen::VectorXd joint_positions;
  CommandPriority priority = CommandPriority::LOW_TRAJECTORY;
  std::string source = "unknown";

  bool canOverride(const ControlCommand &next) const {
    return static_cast<int>(next.priority) >= static_cast<int>(priority);
  }
};

/**
 * Interface for following a path and managing command dispatch to the robot.
 */
class ITrajectoryController {
public:
  virtual ~ITrajectoryController() = default;

  /**
   * Start following a specific path.
   * @param path Sequence of joint configurations.
   */
  virtual void setPath(const std::vector<Eigen::VectorXd> &path) = 0;

  /**
   * Set a manual override command (e.g., from an avoidance module).
   */
  virtual void setOverrideCommand(const ControlCommand &cmd) = 0;

  /**
   * Update the controller. Should be called periodically.
   * @param dt Time step in seconds.
   * @param robot The robot interface to command.
   */
  virtual void update(double dt, IRobotInterface &robot) = 0;

  /**
   * Check if the robot has reached the end of the current path.
   */
  virtual bool isFinished() const = 0;

  /**
   * Get the current index in the path being followed.
   */
  virtual size_t getCurrentIndex() const = 0;

  /**
   * Stop all movement.
   */
  virtual void stop() = 0;

  /**
   * Set global speed scale.
   */
  virtual void setSpeedScale(double scale) = 0;

  /**
   * Get the last commanded joint positions.
   * Useful for ensuring continuity during replanning.
   */
  virtual Eigen::VectorXd getLastCommand() const = 0;
};

} // namespace control
