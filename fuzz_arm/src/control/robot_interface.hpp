#pragma once

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace control {

/**
 * Robot control modes
 */
enum class ControlMode {
  EFFORT,   // Torque/Force control
  VELOCITY, // Velocity control
  POSITION  // Position control
};

/**
 * Snapshot of a joint's state
 */
struct JointState {
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
};

/**
 * Simulator-Agnostic Robot Interface
 * This interface is pure C++ and uses only Eigen.
 * Any simulation specific code (ODE/PhysX) or hardware communication
 * must be hidden in the implementation classes.
 */
class IRobotInterface {
public:
  virtual ~IRobotInterface() = default;

  // --- Configuration ---
  virtual std::string getRobotName() const = 0;
  virtual std::vector<std::string> getJointNames() const = 0;

  // --- State Acquisition ---
  /**
   * Get the state of a specific joint by name.
   */
  virtual JointState getJointState(const std::string &joint_name) const = 0;

  /**
   * Helper to get all joint positions as a vector.
   */
  virtual Eigen::VectorXd getJointPositions() const = 0;

  // --- Actuation ---
  /**
   * Send a command to a specific joint.
   * @param mode What type of command (Effort, Velocity, or Position)
   */
  virtual void setJointCommand(const std::string &joint_name, double value,
                               ControlMode mode) = 0;

  /**
   * Update/Step the interface.
   * In simulation, this might do nothing (simulation is stepped in main loop).
   * For a real robot, this might trigger the actual hardware/read-write.
   */
  virtual void update(double dt) = 0;

  // --- Collision/Contact (Simulator Agnostic) ---
  /**
   * Check if a specific link is in contact with anything.
   * Reports the names of the other objects (Link Names or Asset Names).
   */
  virtual std::vector<std::string>
  getContacts(const std::string &link_name) const = 0;
};

} // namespace control
