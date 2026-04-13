#include "simulation/robot/ode/ode_robot_sim.hpp"
#include <algorithm>

namespace simulation {

OdeRobotSim::OdeRobotSim(
    dWorldID world, CollisionManager *collision_manager,
    const RobotModel &model,
    const std::map<std::string, OdeRobotComponent> &components,
    const std::vector<std::string> &joint_order, bool kinematic_mode,
    const std::string &robot_name)
    : collision_manager_(collision_manager), robot_name_(robot_name),
      components_(components), kinematic_mode_(kinematic_mode) {
  (void)world; // Keep parameter for interface consistency

  // Populate COM offsets from the model
  for (const auto &[name, link] : model.getLinks()) {
    com_offsets_[name] = link.inertial.origin;
  }

  // Build the internal joint list based on the provided joint_order
  joint_names_.clear();
  std::cout << "[DEBUG OdeRobotSim] Looking for " << joint_order.size()
            << " joints in components map.\n";
  for (const std::string &name : joint_order) {
    auto it = components_.find(name);
    if (it != components_.end()) {
      // Only include non-fixed joints that actually have a joint_id
      if (it->second.joint_id != nullptr) {
        joint_names_.push_back(name);
        joint_to_component_[name] = &it->second;
        std::cout << "[DEBUG OdeRobotSim] Found joint '" << name
                  << "', joint_id: " << (void *)it->second.joint_id
                  << std::endl;
      } else {
        std::cout << "[DEBUG OdeRobotSim] Found matching component '" << name
                  << "' but joint_id is NULL (fixed joint?)\n";
      }
    } else {
      std::cout << "[DEBUG OdeRobotSim] ERROR: Component '" << name
                << "' NOT FOUND in components map!\n";
    }
  }

  // Initialize virtual positions for all controlled joints
  for (const std::string &name : joint_names_) {
    virtual_joint_positions_[name] = 0.0;
    prev_errors_[name] = 0.0;
    integral_errors_[name] = 0.0;

    // Allocate Feedback
    feedbacks_[name] = std::make_unique<dJointFeedback>();

    // Set feedback in ODE
    auto it = joint_to_component_.find(name);
    if (it != joint_to_component_.end() && it->second->joint_id) {
      dJointSetFeedback(it->second->joint_id, feedbacks_[name].get());
    }
  }

  std::cout << "[DEBUG] OdeRobotSim Initialized with joints (in order):\n";
  for (const auto &name : joint_names_) {
    std::cout << " - " << name << "\n";
  }
}

std::vector<std::string> OdeRobotSim::getJointNames() const {
  return joint_names_;
}

std::string OdeRobotSim::getRobotName() const { return robot_name_; }

control::JointState
OdeRobotSim::getJointState(const std::string &joint_name) const {
  control::JointState state;
  auto it = joint_to_component_.find(joint_name);
  if (it == joint_to_component_.end())
    return state;

  if (kinematic_mode_) {
    state.position = virtual_joint_positions_.at(joint_name);
    state.velocity = 0.0;
    state.effort = 0.0;
    return state;
  }

  dJointID j = it->second->joint_id;
  int type = dJointGetType(j);

  if (type == dJointTypeHinge) {
    double angle = dJointGetHingeAngle(j);
    // Normalize to [-PI, PI]
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    state.position = angle;
    state.velocity = dJointGetHingeAngleRate(j);
  } else if (type == dJointTypeSlider) {
    state.position = dJointGetSliderPosition(j);
    state.velocity = dJointGetSliderPositionRate(j);
  }
  // Retrieve Effort (Torque) from feedback
  auto fb_it = feedbacks_.find(joint_name);
  if (fb_it != feedbacks_.end() && fb_it->second) {
    dJointFeedback *fb = fb_it->second.get();
    // Torque on body 1 (t1)
    Eigen::Vector3d torque(fb->t1[0], fb->t1[1], fb->t1[2]);

    if (type == dJointTypeHinge) {
      dVector3 axis;
      dJointGetHingeAxis(j, axis);
      Eigen::Vector3d axis_vec(axis[0], axis[1], axis[2]);
      // Project torque onto hinge axis
      state.effort = torque.dot(axis_vec);
    } else if (type == dJointTypeSlider) {
      dVector3 axis;
      dJointGetSliderAxis(j, axis);
      Eigen::Vector3d axis_vec(axis[0], axis[1], axis[2]);
      // Force on body 1 (f1)
      Eigen::Vector3d force(fb->f1[0], fb->f1[1], fb->f1[2]);
      state.effort = force.dot(axis_vec);
    }
  }

  return state;
}

Eigen::VectorXd OdeRobotSim::getJointPositions() const {
  Eigen::VectorXd pos(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    pos[i] = getJointState(joint_names_[i]).position;
  }
  return pos;
}

void OdeRobotSim::setJointPositions(const Eigen::VectorXd &pos) {
  if (pos.size() != static_cast<long>(joint_names_.size())) {
    std::cerr << "[ERROR OdeRobotSim::setJointPositions] Size mismatch: expected "
              << joint_names_.size() << " but got " << pos.size() << std::endl;
    return;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string &name = joint_names_[i];
    double val = pos[i];
    virtual_joint_positions_[name] = val;

    auto it = joint_to_component_.find(name);
    if (it != joint_to_component_.end() && it->second->joint_id) {
      dJointID j = it->second->joint_id;
      int type = dJointGetType(j);

      // Force ODE to the desired position by adjusting the joint and bodies
      // Note: This is a harsh reset, best for Replay or teleporting.
      if (type == dJointTypeHinge) {
        // dJointSetHingeParam(j, dParamLoStop, val);
        // dJointSetHingeParam(j, dParamHiStop, val);
        // Better way to "teleport" a joint is to set body poses, 
        // but for Replay we can also just set virtual positions if the 
        // surrounding simulation is stopped.
      } else if (type == dJointTypeSlider) {
        // dJointSetSliderParam(j, dParamLoStop, val);
        // dJointSetSliderParam(j, dParamHiStop, val);
      }
    }
  }
}

void OdeRobotSim::setJointCommand(const std::string &joint_name, double value,
                                  control::ControlMode mode) {

  auto it = joint_to_component_.find(joint_name);
  if (it == joint_to_component_.end()) {
    std::cerr << "[ERROR OdeRobotSim::setJointCommand] Joint '" << joint_name
              << "' not found in map." << std::endl;
    return;
  }

  if (mode == control::ControlMode::POSITION) {
    virtual_joint_positions_[joint_name] = value;
    virtual_joint_velocities_[joint_name] = 0.0;
  } else if (mode == control::ControlMode::VELOCITY) {
    virtual_joint_velocities_[joint_name] = value;
  }

  dJointID j = it->second->joint_id;
  if (j == nullptr) { // Safety check
    std::cerr << "[ERROR OdeRobotSim::setJointCommand] joint_id is null for "
              << joint_name << std::endl;
    return;
  }
  int type = dJointGetType(j);

  if (mode == control::ControlMode::EFFORT) {
    if (type == dJointTypeHinge)
      dJointAddHingeTorque(j, value);
    else if (type == dJointTypeSlider)
      dJointAddSliderForce(j, value);
  } else if (mode == control::ControlMode::VELOCITY) {
    if (type == dJointTypeHinge) {
      dJointSetHingeParam(j, dParamVel, value);
      dJointSetHingeParam(j, dParamFMax, 100.0); // Arbitrary max force
    } else if (type == dJointTypeSlider) {
      dJointSetSliderParam(j, dParamVel, value);
      dJointSetSliderParam(j, dParamFMax, 100.0);
    }
  } else if (mode == control::ControlMode::POSITION) {
    double current = 0;
    if (type == dJointTypeHinge)
      current = dJointGetHingeAngle(j);
    else if (type == dJointTypeSlider)
      current = dJointGetSliderPosition(j);
    else {
      std::cerr << "  > Unknown joint type for " << joint_name << std::endl;
      return;
    }

    if (kinematic_mode_) {
      // Kinematic Mode: Force joint to target position instantly
      double dt = 0.01; // Simulation step time
      double vel_cmd =
          (value - current) /
          dt; // Calculate required velocity to reach target in one step
      double fMax_kinematic = 1e10; // Very high force for instant tracking

      if (type == dJointTypeHinge) {
        dJointSetHingeParam(j, dParamVel, vel_cmd);
        dJointSetHingeParam(j, dParamFMax, fMax_kinematic);
      } else if (type == dJointTypeSlider) {
        dJointSetSliderParam(j, dParamVel, vel_cmd);
        dJointSetSliderParam(j, dParamFMax, fMax_kinematic);
      }
      prev_errors_[joint_name] =
          value - current; // Still update prev_errors for consistency, though
                           // not used in kinematic vel calc
    } else {
      // Dynamic Mode: Use PID controller
      double kp = 30.0; // Stabilized value
      double kd = 0.5;  // Stabilized value

      double error = value - current;

      // Hingeジョイントの場合は角度の最短経路を選択するようにラップする
      if (type == dJointTypeHinge) {
        if (error > M_PI)
          error -= 2.0 * M_PI;
        else if (error < -M_PI)
          error += 2.0 * M_PI;
      }

      double prev_error = prev_errors_[joint_name];
      double dt = 0.01; // Simulation step time

      double vel_cmd = kp * error + kd * (error - prev_error) / dt;
      prev_errors_[joint_name] = error;

      double fMax_dynamic = 1000.0; // Matched with URDF effort limit

      // Re-introduce explicit velocity clamping to match URDF limits

      if (type == dJointTypeHinge) {
        dJointSetHingeParam(j, dParamVel, vel_cmd);
        dJointSetHingeParam(j, dParamFMax, fMax_dynamic);
      } else if (type == dJointTypeSlider) {
        dJointSetSliderParam(j, dParamVel, vel_cmd);
        dJointSetSliderParam(j, dParamFMax, fMax_dynamic);
      }
    }
  }
}

void OdeRobotSim::update(double dt) {
  if (kinematic_mode_ && dt > 0) {
    for (auto &pair : virtual_joint_velocities_) {
      virtual_joint_positions_[pair.first] += pair.second * dt;
    }
  }
}

std::vector<std::string>
OdeRobotSim::getContacts(const std::string &link_name) const {
  if (collision_manager_) {
    return collision_manager_->getContacts(link_name);
  }
  return {};
}

void OdeRobotSim::setLinkPose(const std::string &link_name,
                              const Eigen::Vector3d &pos,
                              const Eigen::Quaterniond &ori) {
  auto it = components_.find(link_name);
  if (it != components_.end() && it->second.body_id) {
    Eigen::Isometry3d link_frame = Eigen::Isometry3d::Identity();
    link_frame.translate(pos);
    link_frame.rotate(ori);

    // Apply COM offset
    auto offset_it = com_offsets_.find(link_name);
    if (offset_it != com_offsets_.end()) {
      Eigen::Isometry3d body_com_world = link_frame * offset_it->second;
      dBodySetPosition(it->second.body_id, body_com_world.translation().x(),
                       body_com_world.translation().y(),
                       body_com_world.translation().z());
      Eigen::Quaterniond q_comp(body_com_world.rotation());
      dQuaternion dq = {q_comp.w(), q_comp.x(), q_comp.y(), q_comp.z()};
      dBodySetQuaternion(it->second.body_id, dq);
    } else {
      dBodySetPosition(it->second.body_id, pos.x(), pos.y(), pos.z());
      dQuaternion dq = {ori.w(), ori.x(), ori.y(), ori.z()};
      dBodySetQuaternion(it->second.body_id, dq);
    }
  }
}

} // namespace simulation
