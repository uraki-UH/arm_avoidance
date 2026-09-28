#include "core/control/joint_state_utils.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "core/control/motion_smoothing.hpp"
#include "core/kinematics/kinematics.hpp"

namespace robot_sim::planning
{

class TargetJointStateExecutorNode : public rclcpp::Node
{
public:
  explicit TargetJointStateExecutorNode(const rclcpp::NodeOptions & options)
  : Node("target_joint_state_executor_node", options)
  {
    target_topic_ = declare_parameter<std::string>("target_topic", "target_joint_states");
    state_topic_ = declare_parameter<std::string>("state_topic", "joint_states");
    command_topic_ = declare_parameter<std::string>("command_topic", "joint_commands");
    publish_hz_ = std::max(1.0, declare_parameter<double>("publish_hz", 50.0));
    max_joint_velocity_ = std::max(1e-6, declare_parameter<double>("max_joint_velocity", 0.6));
    position_tolerance_ = std::max(1e-6, declare_parameter<double>("position_tolerance", 0.01));
    use_wraparound_ = declare_parameter<bool>("use_wraparound", true);

    target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      target_topic_, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_target_ = *msg;
        have_target_ = true;
      });

    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      state_topic_, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_state_ = *msg;
        have_state_ = true;
      });

    command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      command_topic_, rclcpp::QoS(10).reliable());

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz_)),
      [this]() { this->update(); });

    RCLCPP_INFO(
      get_logger(),
      "TargetJointStateExecutor ready. target=%s state=%s command=%s hz=%.1f max_vel=%.3f wrap=%d",
      target_topic_.c_str(), state_topic_.c_str(), command_topic_.c_str(),
      publish_hz_, max_joint_velocity_, use_wraparound_ ? 1 : 0);
  }

private:

  void update()
  {
    sensor_msgs::msg::JointState target_msg;
    sensor_msgs::msg::JointState state_msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_state_) {
        return;
      }
      state_msg = latest_state_;
      if (have_target_) {
        target_msg = latest_target_;
      } else {
        target_msg = latest_state_;
      }
    }


    const std::vector<std::string> &command_names =
      !state_msg.name.empty() ? state_msg.name : target_msg.name;
    if (command_names.empty()) {
      return;
    }

    const auto positions = joint_state_utils::positions_in_order(state_msg, target_msg, command_names);
    const auto &current_q = positions.current;
    const auto &target_q = positions.target;

    Eigen::VectorXf diff = target_q - current_q;
    if (use_wraparound_) {
      kinematics::applyWraparound(diff);
    }

    Eigen::VectorXf command_q = target_q;

    const float max_abs_diff = diff.cwiseAbs().maxCoeff();
    if (max_abs_diff > static_cast<float>(position_tolerance_)) {
      const float scale = motion_smoothing::calculate_velocity_scale(
        diff, static_cast<float>(max_joint_velocity_), static_cast<float>(1.0 / publish_hz_));
      const float clamped_scale = std::max(0.0f, std::min(1.0f, scale));
      command_q = current_q + diff * clamped_scale;
    }

    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    out.name = command_names;
    out.position.resize(command_names.size());
    for (std::size_t i = 0; i < command_names.size(); ++i) {
      out.position[i] = static_cast<double>(command_q[static_cast<int>(i)]);
    }
    command_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Executor: target=%s state=%s cmd=%s max_diff=%.4f",
      target_topic_.c_str(), state_topic_.c_str(), command_topic_.c_str(), max_abs_diff);
  }

  std::string target_topic_;
  std::string state_topic_;
  std::string command_topic_;
  double publish_hz_ = 50.0;
  double max_joint_velocity_ = 0.6;
  double position_tolerance_ = 0.01;
  bool use_wraparound_ = true;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  sensor_msgs::msg::JointState latest_target_;
  sensor_msgs::msg::JointState latest_state_;
  bool have_target_ = false;
  bool have_state_ = false;
};

}  // namespace robot_sim::planning

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::planning::TargetJointStateExecutorNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_sim::planning::TargetJointStateExecutorNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
