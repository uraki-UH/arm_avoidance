#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/kinematics/kinematics.hpp"

namespace robot_sim::planning
{

class VirtualJointStateDriverNode : public rclcpp::Node
{
public:
  explicit VirtualJointStateDriverNode(const rclcpp::NodeOptions & options)
  : Node("virtual_joint_state_driver_node", options)
  {
    target_topic_ = declare_parameter<std::string>("target_topic", "target_joint_states_deprecated");
    state_topic_ = declare_parameter<std::string>("state_topic", "joint_states");
    output_topic_ = declare_parameter<std::string>("output_topic", "joint_states");
    publish_hz_ = std::max(1.0, declare_parameter<double>("publish_hz", 50.0));
    max_joint_velocity_ = std::max(1e-6, declare_parameter<double>("max_joint_velocity", 0.6));
    position_tolerance_ = std::max(1e-6, declare_parameter<double>("position_tolerance", 0.01));
    use_wraparound_ = declare_parameter<bool>("use_wraparound", true);
    ignore_state_after_first_target_ = declare_parameter<bool>("ignore_state_after_first_target", false);
    const auto initial_joint_names_csv =
        declare_parameter<std::string>("initial_joint_names_csv", "");
    const auto initial_joint_names = splitCsv(initial_joint_names_csv);
    if (!initial_joint_names.empty()) {
      latest_state_.name = initial_joint_names;
      latest_state_.position.assign(initial_joint_names.size(), 0.0);
      latest_state_.velocity.assign(initial_joint_names.size(), 0.0);
      latest_state_.effort.assign(initial_joint_names.size(), 0.0);
      latest_state_.header.stamp = now();
      have_state_ = true;
    }

    target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      target_topic_, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_target_ = *msg;
        have_target_ = true;
        ++target_msg_count_;
        if (target_msg_count_ <= 5) {
          RCLCPP_INFO(
            get_logger(),
            "Target received: topic=%s msg=%zu names=%zu positions=%zu first_joint=%s",
            target_topic_.c_str(),
            target_msg_count_,
            latest_target_.name.size(),
            latest_target_.position.size(),
            latest_target_.name.empty() ? "" : latest_target_.name.front().c_str());
        }
        if (!have_state_ && !latest_target_.name.empty()) {
          latest_state_.name = latest_target_.name;
          latest_state_.position.assign(latest_target_.name.size(), 0.0);
          latest_state_.velocity.assign(latest_target_.name.size(), 0.0);
          latest_state_.effort.assign(latest_target_.name.size(), 0.0);
          latest_state_.header.stamp = now();
          have_state_ = true;
        }
      });

    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      state_topic_, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (ignore_state_after_first_target_ && have_target_ && external_state_seeded_ &&
            latest_state_.name.size() > latest_target_.name.size()) {
          return;
        }
        latest_state_ = *msg;
        have_state_ = true;
        external_state_seeded_ = true;
      });

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      output_topic_, rclcpp::QoS(10).reliable().transient_local());

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz_)),
      [this]() { this->update(); });

    RCLCPP_INFO(
      get_logger(),
      "VirtualJointStateDriver ready. target=%s state_in=%s state_out=%s hz=%.1f max_vel=%.3f hold=%d wrap=%d ignore_state_after_target=%d",
      target_topic_.c_str(), state_topic_.c_str(), output_topic_.c_str(),
      publish_hz_, max_joint_velocity_ ? 1 : 0,
      use_wraparound_ ? 1 : 0, ignore_state_after_first_target_ ? 1 : 0);
  }

private:
  static std::vector<std::string> splitCsv(const std::string &csv)
  {
    std::vector<std::string> out;
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
      const auto begin = item.find_first_not_of(" \t\r\n");
      if (begin == std::string::npos) {
        continue;
      }
      const auto end = item.find_last_not_of(" \t\r\n");
      out.push_back(item.substr(begin, end - begin + 1));
    }
    return out;
  }

  static std::unordered_map<std::string, double> buildMap(
    const sensor_msgs::msg::JointState & msg)
  {
    std::unordered_map<std::string, double> out;
    out.reserve(msg.name.size());
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      if (i < msg.position.size()) {
        out[msg.name[i]] = msg.position[i];
      }
    }
    return out;
  }

  static std::vector<std::string> mergeNames(
    const std::vector<std::string> & a, const std::vector<std::string> & b)
  {
    std::vector<std::string> out = a;
    for (const auto & name : b) {
      if (std::find(out.begin(), out.end(), name) == out.end()) {
        out.push_back(name);
      }
    }
    return out;
  }

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
      }  else {
        target_msg = latest_state_;;
      }
    }

    std::vector<std::string> joint_names = mergeNames(state_msg.name, target_msg.name);
    if (joint_names.empty()) {
      return;
    }

    const auto state_map = buildMap(state_msg);
    const auto target_map = buildMap(target_msg);

    Eigen::VectorXf current_q(static_cast<int>(joint_names.size()));
    Eigen::VectorXf target_q(static_cast<int>(joint_names.size()));
    current_q.setZero();
    target_q.setZero();

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      const auto & name = joint_names[i];
      const auto state_it = state_map.find(name);
      const auto target_it = target_map.find(name);

      const double current = (state_it != state_map.end()) ? state_it->second : 0.0;
      const double target = (target_it != target_map.end()) ? target_it->second : current;

      current_q[static_cast<int>(i)] = static_cast<float>(current);
      target_q[static_cast<int>(i)] = static_cast<float>(target);
    }

    Eigen::VectorXf diff = target_q - current_q;
    if (use_wraparound_) {
      kinematics::applyWraparound(diff);
    }

    Eigen::VectorXf next_q = target_q;
    const float max_abs_diff = diff.cwiseAbs().maxCoeff();
    if (max_abs_diff > static_cast<float>(position_tolerance_)) {
      const float scale = kinematics::calculateVelocityScale(
        diff, static_cast<float>(max_joint_velocity_), static_cast<float>(1.0 / publish_hz_));
      const float clamped_scale = std::max(0.0f, std::min(1.0f, scale));
      next_q = current_q + diff * clamped_scale;
    }

    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    out.name = std::move(joint_names);
    out.position.resize(static_cast<std::size_t>(next_q.size()));
    out.velocity.resize(static_cast<std::size_t>(next_q.size()));
    out.effort.resize(static_cast<std::size_t>(next_q.size()));
    for (int i = 0; i < next_q.size(); ++i) {
      out.position[static_cast<std::size_t>(i)] = static_cast<double>(next_q[i]);
      out.velocity[static_cast<std::size_t>(i)] = static_cast<double>((next_q[i] - current_q[i]) * publish_hz_);
      out.effort[static_cast<std::size_t>(i)] = 0.0;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_state_ = out;
      have_state_ = true;
    }

    state_pub_->publish(out);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "VirtualDriver: target=%s state_in=%s state_out=%s max_diff=%.4f",
      target_topic_.c_str(), state_topic_.c_str(), output_topic_.c_str(), max_abs_diff);
  }

  std::string target_topic_;
  std::string state_topic_;
  std::string output_topic_;
  double publish_hz_ = 30.0;
  double max_joint_velocity_ = 0.6;
  double position_tolerance_ = 0.01;
  bool use_wraparound_ = true;
  bool ignore_state_after_first_target_ = false;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  sensor_msgs::msg::JointState latest_target_;
  sensor_msgs::msg::JointState latest_state_;
  bool have_target_ = false;
  bool have_state_ = false;
  bool external_state_seeded_ = false;
  std::size_t target_msg_count_ = 0;
};

}  // namespace robot_sim::planning

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::planning::VirtualJointStateDriverNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_sim::planning::VirtualJointStateDriverNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
