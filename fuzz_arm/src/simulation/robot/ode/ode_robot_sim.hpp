#pragma once

#include "control/robot_interface.hpp"
#include "simulation/robot/ode/ode_collision_manager.hpp"
#include "simulation/robot/ode/ode_robot_builder.hpp" // For OdeRobotComponent struct
#include <Eigen/Dense>
#include <map>
#include <memory>
#include <ode/ode.h>
#include <string>
#include <vector>

namespace simulation {

// Forward declaration if needed, but OdeRobotComponent is from
// ode_robot_builder.hpp

class OdeRobotSim : public control::IRobotInterface {
public:
  OdeRobotSim(dWorldID world, CollisionManager *collision_manager,
              const RobotModel &model,
              const std::map<std::string, OdeRobotComponent> &components,
              const std::vector<std::string> &joint_order,
              bool kinematic_mode = false,
              const std::string &robot_name = "robot");

  // IRobotInterface implementation
  std::vector<std::string> getJointNames() const override;
  std::string getRobotName() const override;
  control::JointState
  getJointState(const std::string &joint_name) const override;
  Eigen::VectorXd getJointPositions() const override;
  void setJointCommand(const std::string &joint_name, double value,
                       control::ControlMode mode) override;
  void update(double dt) override;
  std::vector<std::string>
  getContacts(const std::string &link_name) const override;

  // Visual-Only support
  void setLinkPose(const std::string &link_name, const Eigen::Vector3d &pos,
                   const Eigen::Quaterniond &ori);
  void setKinematicMode(bool enabled) { kinematic_mode_ = enabled; }
  void setJointPositions(const Eigen::VectorXd &pos);

private:
  CollisionManager *collision_manager_;
  std::string robot_name_;
  const std::map<std::string, OdeRobotComponent>
      &components_; // Reference to all robot components

  std::vector<std::string> joint_names_; // List of names of controlled joints
  std::map<std::string, const OdeRobotComponent *>
      joint_to_component_; // Map to easily access OdeRobotComponent by joint
                           // name

  // PID state variables for each joint
  std::map<std::string, double> prev_errors_;
  std::map<std::string, double> integral_errors_;
  std::map<std::string, double>
      virtual_joint_positions_; // Added for Visual-Only
  std::map<std::string, double>
      virtual_joint_velocities_; // Added for Visual-Only
  std::map<std::string, Eigen::Isometry3d>
      com_offsets_; // Added for Visual-Only
  std::map<std::string, double>
      velocity_limits_; // Joint velocity limits from URDF

  // ODE Joint Feedback storage
  // Must be stable in memory, so use unique_ptr or vector with reserved
  // capacity Using map of unique_ptr to key by joint name matches other
  // structures
  std::map<std::string, std::unique_ptr<dJointFeedback>> feedbacks_;

  bool kinematic_mode_ = false;
};

} // namespace simulation