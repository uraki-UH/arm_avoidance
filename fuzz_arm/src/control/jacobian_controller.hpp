#pragma once

#include "control/trajectory_controller.hpp"
#include "kinematics/kinematic_chain.hpp"
#include <Eigen/Dense>
#include <deque>
#include <iostream>
#include <vector>

namespace control {

/**
 * Jacobian Pseudo-Inverse Controller.
 * Moves the robot towards the target 3D position defined by the end of the
 * path. Ignores intermediate path points (greedy local approach).
 */
class JacobianController : public ITrajectoryController {
public:
  JacobianController(kinematics::KinematicChain &chain,
                     double global_speed_scale = 1.0)
      : chain_(chain), speed_scale_(global_speed_scale), is_finished_(true) {}

  void setPath(const std::vector<Eigen::VectorXd> &path) override {
    target_queue_.clear();
    is_finished_ = true;

    if (path.empty()) {
      return;
    }

    // Add all points to queue
    for (const auto &p : path) {
      target_queue_.push_back(p);
    }

    // Set first target
    updateCurrentTarget();
    is_finished_ = false;
    override_active_ = false;
  }

  void appendPath(const std::vector<Eigen::VectorXd> &segment) override {
    if (segment.empty())
      return;

    // If finished, restart with this path
    if (is_finished_) {
      setPath(segment);
      return;
    }

    // Otherwise append to queue
    for (const auto &p : segment) {
      target_queue_.push_back(p);
    }
  }

  void setMaxVelocities(const std::vector<double> &max_vels) override {
    max_velocities_ = max_vels;
  }

  void setSpeedScale(double scale) override { speed_scale_ = scale; }

  void setOverrideCommand(const ControlCommand &cmd) override {
    if (!override_active_ || override_cmd_.canOverride(cmd)) {
      override_cmd_ = cmd;
      override_active_ = true;
    }
  }

  void setTargetJoints(const std::vector<std::string> &joint_names) override {
    // No-op for Jacobian controller as it uses chain directly
    // Or we can store them if needed.
  }

  // implementations
  size_t getCurrentIndex() const override {
    return 0;
  } // Not tracking index strictly

  size_t getRemainingPathSize() const override {
    return target_queue_.size() + (is_finished_ ? 0 : 1); // Queue + current
  }

  Eigen::VectorXd getPathEndpoint() const override {
    if (!target_queue_.empty())
      return target_queue_.back();
    if (!is_finished_)
      return target_q_; // Current active target
    return hold_q_;     // Finished state
  }

  void update(double dt, IRobotInterface &robot) override {
    if (override_active_) {
      auto names = robot.getJointNames();
      for (size_t i = 0;
           i < names.size() && i < (size_t)override_cmd_.joint_positions.size();
           ++i) {
        robot.setJointCommand(names[i], override_cmd_.joint_positions[i],
                              ControlMode::POSITION);
      }
      override_active_ = false;
      return;
    }

    if (is_finished_)
      return;

    // Get current state
    auto joint_names = robot.getJointNames(); // Assuming order matches chain
    Eigen::VectorXd q_curr(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
      q_curr[i] = robot.getJointState(joint_names[i]).position;
    }

    // FK for current
    std::vector<double> q_curr_vec(q_curr.data(),
                                   q_curr.data() + q_curr.size());
    chain_.forwardKinematicsAt(q_curr_vec);
    Eigen::Vector3d current_pos = chain_.getEEFPosition();

    // Position Error
    Eigen::Vector3d err = target_pos_ - current_pos;

    // Check completion for CURRENT target
    if (err.norm() < 0.02) { // 2cm tolerance for switching
      if (!target_queue_.empty()) {
        // Move to next target
        updateCurrentTarget();
        // Recalculate err for new target immediately
        err = target_pos_ - current_pos;
      } else {
        is_finished_ = true;
        hold_q_ = q_curr;
        return;
      }
    }

    // Speed Limit
    double max_speed = 0.5 * speed_scale_;
    if (err.norm() / dt > max_speed) {
      err = err.normalized() * max_speed * dt;
    }

    int eff_joint_idx = chain_.getNumJoints() - 1;
    Eigen::MatrixXd J = chain_.calculateJacobian(eff_joint_idx);
    Eigen::MatrixXd J_pos = J.block(0, 0, 3, J.cols());

    double lambda = 0.1;
    Eigen::MatrixXd J_pinv =
        J_pos.transpose() * (J_pos * J_pos.transpose() +
                             lambda * lambda * Eigen::MatrixXd::Identity(3, 3))
                                .inverse();

    Eigen::VectorXd dq = J_pinv * err;

    double max_joint_vel = 1.0;
    for (int i = 0; i < dq.size(); ++i) {
      if (std::abs(dq[i]) / dt > max_joint_vel) {
        dq *= (max_joint_vel * dt) / std::abs(dq[i]);
      }
    }

    Eigen::VectorXd q_next = q_curr + dq;

    for (size_t i = 0; i < joint_names.size(); ++i) {
      robot.setJointCommand(joint_names[i], q_next[i], ControlMode::POSITION);
    }
    hold_q_ = q_next;
  }

  bool isFinished() const override { return is_finished_; }

  void stop() override {
    is_finished_ = true;
    override_active_ = false;
    target_queue_.clear();
  }

  Eigen::VectorXd getLastCommand() const override { return hold_q_; }

private:
  void updateCurrentTarget() {
    if (target_queue_.empty())
      return;
    target_q_ = target_queue_.front();
    target_queue_.pop_front();

    std::vector<double> q_vec(target_q_.data(),
                              target_q_.data() + target_q_.size());
    chain_.forwardKinematicsAt(q_vec);
    target_pos_ = chain_.getEEFPosition();
  }

  kinematics::KinematicChain &chain_;
  double speed_scale_;
  bool is_finished_;

  std::deque<Eigen::VectorXd> target_queue_;
  Eigen::VectorXd target_q_;
  Eigen::Vector3d target_pos_;

  ControlCommand override_cmd_;
  bool override_active_ = false;
  std::vector<double> max_velocities_;
  Eigen::VectorXd hold_q_;
};

} // namespace control
