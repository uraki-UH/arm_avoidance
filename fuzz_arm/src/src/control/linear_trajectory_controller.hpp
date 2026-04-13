#pragma once

#include "control/trajectory_controller.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace control {

/**
 * Velocity-limited path follower that respects joint velocity limits.
 */
class LinearTrajectoryController : public ITrajectoryController {
public:
  LinearTrajectoryController(double global_speed_scale = 1.0)
      : speed_scale_(global_speed_scale), is_finished_(true), path_index_(0),
        t_(0.0) {}

  void setPath(const std::vector<Eigen::VectorXd> &path) override {
    path_ = path;
    path_index_ = 0;
    t_ = 0.0;
    is_finished_ = path_.empty();
    override_active_ = false;
  }

  void setMaxVelocities(const std::vector<double> &max_vels) {
    max_velocities_ = max_vels;
  }

  void setSpeedScale(double scale) override { speed_scale_ = scale; }

  void setOverrideCommand(const ControlCommand &cmd) override {
    if (!override_active_ || override_cmd_.canOverride(cmd)) {
      override_cmd_ = cmd;
      override_active_ = true;
    }
  }

  void setTargetJoints(const std::vector<std::string> &joint_names) {
    target_joints_ = joint_names;
  }

  void setGoalTolerance(double tol) { goal_tolerance_ = tol; }

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

    if (is_finished_ || path_.empty()) {
      // 停止中も最後に指令した姿勢を維持し続ける (ODEでの惰性を防ぐ)
      if (hold_q_.size() > 0) {
        for (size_t i = 0;
             i < target_joints_.size() && i < (size_t)hold_q_.size(); ++i) {
          robot.setJointCommand(target_joints_[i], hold_q_[i],
                                ControlMode::POSITION);
        }
      }
      return;
    }

    if (path_.size() == 1 || path_index_ == path_.size() - 1) {
      const Eigen::VectorXd &q_final = path_[path_index_];
      bool all_reached = true;
      for (size_t i = 0;
           i < target_joints_.size() && i < (size_t)q_final.size(); ++i) {
        const std::string &joint_name = target_joints_[i];
        robot.setJointCommand(joint_name, q_final[i], ControlMode::POSITION);

        double current = robot.getJointState(joint_name).position;
        double diff = q_final[i] - current;
        while (diff > M_PI)
          diff -= 2.0 * M_PI;
        while (diff < -M_PI)
          diff += 2.0 * M_PI;
        diff = std::abs(diff);

        if (diff > goal_tolerance_) {
          all_reached = false;
        }
      }
      if (all_reached) {
        is_finished_ = true;
      }
    } else {
      const Eigen::VectorXd &q1 = path_[path_index_];
      const Eigen::VectorXd &q2 = path_[path_index_ + 1];

      // Calculate the time required for this segment based on velocity limits
      double segment_time_needed = 0.0;
      Eigen::VectorXd delta = q2 - q1;

      for (int i = 0; i < delta.size(); ++i) {
        // Handle wraparound for revolute joints
        if (delta[i] > M_PI)
          delta[i] -= 2.0 * M_PI;
        else if (delta[i] < -M_PI)
          delta[i] += 2.0 * M_PI;

        double joint_max_v =
            (i < (int)max_velocities_.size()) ? max_velocities_[i] : 1.0;
        if (joint_max_v <= 0)
          joint_max_v = 1.0;

        double t_joint = std::abs(delta[i]) / joint_max_v;
        if (t_joint > segment_time_needed) {
          segment_time_needed = t_joint;
        }
      }

      // Apply speed scale
      segment_time_needed /= speed_scale_;

      if (segment_time_needed < 1e-6) {
        t_ = 1.0;
      } else {
        t_ += dt / segment_time_needed;
      }

      // Node transition
      while (t_ >= 1.0 && path_index_ < path_.size() - 1) {
        path_index_++;
        if (path_index_ < path_.size() - 1) {
          t_ -= 1.0;
        } else {
          t_ = 0.0;
          break;
        }
      }

      // Compute command
      Eigen::VectorXd q_cmd;
      if (path_index_ >= path_.size() - 1) {
        q_cmd = path_.back();
      } else {
        const Eigen::VectorXd &cur_q1 = path_[path_index_];
        const Eigen::VectorXd &cur_q2 = path_[path_index_ + 1];
        Eigen::VectorXd cur_delta = cur_q2 - cur_q1;
        for (int i = 0; i < cur_delta.size(); ++i) {
          if (cur_delta[i] > M_PI)
            cur_delta[i] -= 2.0 * M_PI;
          else if (cur_delta[i] < -M_PI)
            cur_delta[i] += 2.0 * M_PI;
        }
        q_cmd = cur_q1 + cur_delta * std::min(1.0, t_);
      }

      // Set commands to robot
      for (size_t i = 0; i < target_joints_.size() && i < (size_t)q_cmd.size();
           ++i) {
        robot.setJointCommand(target_joints_[i], q_cmd[i],
                              ControlMode::POSITION);
      }
      hold_q_ = q_cmd;
    }
  }

  bool isFinished() const override { return is_finished_; }
  size_t getCurrentIndex() const override { return path_index_; }
  void stop() override {
    is_finished_ = true;
    path_.clear();
    override_active_ = false;
    // stop しても hold_q_ は維持し、その場に留まらせる
  }

  Eigen::VectorXd getLastCommand() const override { return hold_q_; }

private:
  std::vector<Eigen::VectorXd> path_;
  std::vector<double> max_velocities_;
  double speed_scale_;
  bool is_finished_;
  size_t path_index_;
  double t_;

  ControlCommand override_cmd_;
  bool override_active_ = false;
  std::vector<std::string> target_joints_;
  Eigen::VectorXd hold_q_;        // 姿勢維持用のバッファ
  double goal_tolerance_ = 0.005; // Default strict tolerance (approx 0.28 deg)
};

} // namespace control