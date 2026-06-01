#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <gng_control_msgs/msg/joint_control_claim.hpp>

#include "common/resource_utils.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"

namespace {

static std::string joinStrings(const std::vector<std::string> &items,
                               const std::string &delimiter) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < items.size(); ++i) {
    if (i != 0) {
      oss << delimiter;
    }
    oss << items[i];
  }
  return oss.str();
}

} // namespace

namespace robot_sim::planning {

class RightArmTargetPlannerNode : public rclcpp::Node {
public:
  explicit RightArmTargetPlannerNode(const rclcpp::NodeOptions &options)
      : Node("right_arm_target_planner_node", options) {
  initial_joint_angle_rad_ = declare_parameter<double>(
    "initial_joint_angle_rad", -0.8);
  random_walk_span_rad_ =
    declare_parameter<double>("random_walk_span_rad", 0.4);
  random_walk_step_rad_ =
    declare_parameter<double>("random_walk_step_rad", 0.15);

    loadModelAndChain();

    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        state_topic_, rclcpp::QoS(10).reliable(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_state_ = *msg;
          have_state_ = true;
        });

    target_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        command_topic_, rclcpp::QoS(10).reliable());

    claim_pub_ = create_publisher<gng_control_msgs::msg::JointControlClaim>(
        claim_topic_, rclcpp::QoS(1).reliable().transient_local());

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz_)),
        [this]() { this->publishLocked(); });

    RCLCPP_INFO(
        get_logger(),
        "RightArmTargetPlanner random-walk mode. state=%s command=%s initial=%.3f rad walk_span=%.3f rad step=%.3f rad joint_speed=%.3f rad/s publish_hz=%.1f joints=%s",
        state_topic_.c_str(),
        command_topic_.c_str(),
        initial_joint_angle_rad_,
        random_walk_span_rad_,
        random_walk_step_rad_,
        joint_angular_speed_radps_,
        publish_hz_,
        joinStrings(chain_joint_names_, ", ").c_str());
  }

private:
  void loadModelAndChain() {
    const std::string resolved_urdf =
        robot_sim::common::resolvePath(robot_urdf_path_);

    if (resolved_urdf.empty()) {
      throw std::runtime_error(
          "Failed to resolve robot URDF path for right arm target planner.");
    }

    auto model = std::make_shared<::simulation::RobotModel>(
        ::simulation::loadRobotFromUrdf(resolved_urdf));

    auto chain = std::make_shared<::kinematics::KinematicChain>(
        ::simulation::createKinematicChainFromModel(
            *model, leaf_link_, Eigen::Vector3d::Zero(), root_link_));

    if (!chain) {
      throw std::runtime_error("Failed to build right arm kinematic chain.");
    }

    model_ = std::move(model);
    chain_ = std::move(chain);

    chain_joint_names_.clear();
    chain_joint_names_.reserve(
        static_cast<std::size_t>(chain_->getNumJoints()));

    for (int i = 0; i < chain_->getNumJoints(); ++i) {
      chain_joint_names_.push_back(chain_->getJointName(i));
    }

    if (chain_joint_names_.empty()) {
      throw std::runtime_error("Right arm chain has no joints.");
    }
  }

  Eigen::VectorXf currentJointVectorLocked() const {
    Eigen::VectorXf q(static_cast<int>(chain_joint_names_.size()));
    q.setZero();

    std::unordered_map<std::string, double> value_by_name;
    value_by_name.reserve(latest_state_.name.size());

    for (std::size_t i = 0; i < latest_state_.name.size(); ++i) {
      if (i < latest_state_.position.size()) {
        value_by_name[latest_state_.name[i]] = latest_state_.position[i];
      }
    }

    for (std::size_t i = 0; i < chain_joint_names_.size(); ++i) {
      const auto it = value_by_name.find(chain_joint_names_[i]);
      q[static_cast<int>(i)] =
          (it != value_by_name.end()) ? static_cast<float>(it->second) : 0.0f;
    }

    return q;
  }

  bool solveForwardPose(const std::vector<double> &q,
                        Eigen::Vector3d &out_pos,
                        Eigen::Quaterniond &out_ori) const {
    if (!chain_) {
      return false;
    }

    std::vector<Eigen::Vector3d,
                Eigen::aligned_allocator<Eigen::Vector3d>>
        positions;
    std::vector<Eigen::Quaterniond,
                Eigen::aligned_allocator<Eigen::Quaterniond>>
        orientations;

    chain_->forwardKinematicsAt(q, positions, orientations);

    if (positions.empty() || orientations.empty()) {
      return false;
    }

    out_pos = positions.back();
    out_ori = orientations.back();
    return true;
  }

  bool solvePositionIK(const Eigen::Vector3d &target_pos,
                       const std::vector<double> &seed,
                       std::vector<double> &out_solution) const {
    if (!chain_) {
      return false;
    }

    const int eef_index = chain_->getNumJoints() + 1;

    return chain_->inverseKinematicsAt(
        eef_index,
        target_pos,
        seed,
        ik_max_iterations_,
        ik_pos_tolerance_,
        out_solution);
  }

  std::vector<double> limitJointVelocity(
      const std::vector<double> &from_q,
      const std::vector<double> &to_q,
      const double dt) const {
    std::vector<double> limited = from_q;

    if (from_q.size() != to_q.size() || dt <= 0.0) {
      return limited;
    }

    const double max_delta = joint_angular_speed_radps_ * dt;

    for (std::size_t i = 0; i < from_q.size(); ++i) {
      const double diff = to_q[i] - from_q[i];

      if (diff > max_delta) {
        limited[i] = from_q[i] + max_delta;
      } else if (diff < -max_delta) {
        limited[i] = from_q[i] - max_delta;
      } else {
        limited[i] = to_q[i];
      }
    }

    return limited;
  }

  double computeDtSec() {
    const rclcpp::Time now_time = now();

    double dt = 1.0 / publish_hz_;

    if (have_last_update_time_) {
      dt = (now_time - last_update_time_).seconds();

      const double nominal_dt = 1.0 / publish_hz_;
      dt = std::clamp(dt, 0.25 * nominal_dt, 2.0 * nominal_dt);
    }

    last_update_time_ = now_time;
    have_last_update_time_ = true;

    return dt;
  }

  void publishJointCommand(const std::vector<double> &solution) {
    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    out.name = chain_joint_names_;
    out.position.resize(chain_joint_names_.size(), 0.0);
    out.velocity.resize(chain_joint_names_.size(), 0.0);
    out.effort.resize(chain_joint_names_.size(), 0.0);

    const std::size_t copy_dim =
        std::min(chain_joint_names_.size(), solution.size());

    for (std::size_t i = 0; i < copy_dim; ++i) {
      out.position[i] = solution[i];
    }

    target_pub_->publish(out);

    gng_control_msgs::msg::JointControlClaim claim;
    claim.command_topic = command_topic_;
    claim.joint_names = chain_joint_names_;
    claim.priority = claim_priority_;
    claim.mode = static_cast<uint8_t>(claim_mode_);
    claim.enabled = claim_enabled_;

    claim_pub_->publish(claim);
  }

  void publishLocked() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!chain_ || !have_state_) {
      return;
    }

    const Eigen::VectorXf feedback_q = currentJointVectorLocked();

    std::vector<double> feedback_q_vec(
        feedback_q.data(), feedback_q.data() + feedback_q.size());

    // 初回は実測関節角、2回目以降は前回出したコマンド角を基準にする。
    std::vector<double> seed_q =
        last_command_q_.empty() ? feedback_q_vec : last_command_q_;

    const double dt = computeDtSec();

    if (!have_initialized_pose_) {
      std::vector<double> initial_q(chain_joint_names_.size(),
                                    initial_joint_angle_rad_);
      last_command_q_ = initial_q;
      have_initialized_pose_ = true;
      publishJointCommand(last_command_q_);

      RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          3000,
          "Publishing initial joint posture at %.3f rad for all right-arm joints.",
          initial_joint_angle_rad_);
      return;
    }

    std::vector<double> random_target = last_command_q_;
    if (random_target.size() != chain_joint_names_.size()) {
      random_target.assign(chain_joint_names_.size(), initial_joint_angle_rad_);
    }

    std::uniform_real_distribution<double> step_dist(
        -random_walk_step_rad_, random_walk_step_rad_);
    const double lower_bound = initial_joint_angle_rad_ - random_walk_span_rad_;
    const double upper_bound = initial_joint_angle_rad_ + random_walk_span_rad_;

    for (double &joint_value : random_target) {
      joint_value += step_dist(random_engine_);
      joint_value = std::clamp(joint_value, lower_bound, upper_bound);
    }

    const std::vector<double> limited_solution =
        limitJointVelocity(last_command_q_, random_target, dt);

    last_command_q_ = limited_solution;
    publishJointCommand(last_command_q_);

    Eigen::Vector3d current_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond current_ori = Eigen::Quaterniond::Identity();
    Eigen::Vector3d next_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond next_ori = Eigen::Quaterniond::Identity();
    const bool have_current_pose = solveForwardPose(seed_q, current_pos, current_ori);
    const bool have_next_pose = solveForwardPose(last_command_q_, next_pos, next_ori);

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Random walk command. current=[%.3f %.3f %.3f] next=[%.3f %.3f %.3f] joint_speed=%.3f rad/s dt=%.4f",
        have_current_pose ? current_pos.x() : 0.0,
        have_current_pose ? current_pos.y() : 0.0,
        have_current_pose ? current_pos.z() : 0.0,
        have_next_pose ? next_pos.x() : 0.0,
        have_next_pose ? next_pos.y() : 0.0,
        have_next_pose ? next_pos.z() : 0.0,
        joint_angular_speed_radps_,
        dt);
  }

  // ---- Fixed test settings ----

  const std::string state_topic_ = "joint_states";
  const std::string command_topic_ = "right_arm_target_joint_states";
  const std::string claim_topic_ = "control_claims";

  const std::string robot_urdf_path_ =
      "package://topoarm_description/urdf/topo_dual_arm.urdf.xacro";
  const std::string root_link_ = "base_link";
  const std::string leaf_link_ = "right_end_effector_link";

  double initial_joint_angle_rad_ = -1.0;
  double random_walk_span_rad_ = 0.4;
  double random_walk_step_rad_ = 0.15;

  // 各関節の最大角速度 [rad/s]。
  const double joint_angular_speed_radps_ = 0.5;

  const int ik_max_iterations_ = 50;
  const double ik_pos_tolerance_ = 0.01;

  const double publish_hz_ = 50.0;

  const int claim_priority_ = 5;
  const int claim_mode_ =
      gng_control_msgs::msg::JointControlClaim::MODE_EXCLUSIVE;
  const bool claim_enabled_ = true;

  std::shared_ptr<::simulation::RobotModel> model_;
  std::shared_ptr<::kinematics::KinematicChain> chain_;
  std::vector<std::string> chain_joint_names_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_pub_;
  rclcpp::Publisher<gng_control_msgs::msg::JointControlClaim>::SharedPtr
      claim_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mt19937 random_engine_{std::random_device{}()};

  mutable std::mutex mutex_;
  sensor_msgs::msg::JointState latest_state_;

  bool have_state_ = false;
  bool have_initialized_pose_ = false;

  std::vector<double> last_command_q_;

  rclcpp::Time last_update_time_;
  bool have_last_update_time_ = false;
};

} // namespace robot_sim::planning

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::planning::RightArmTargetPlannerNode)

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(
      std::make_shared<robot_sim::planning::RightArmTargetPlannerNode>(
          rclcpp::NodeOptions()));

  rclcpp::shutdown();
  return 0;
}