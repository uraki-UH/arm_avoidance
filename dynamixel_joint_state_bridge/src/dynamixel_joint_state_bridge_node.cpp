#include <algorithm>
#include <cstdint>
#include <functional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dynamixel_handler_msgs/msg/dynamixel_present.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
constexpr double kDegToRad = 0.017453292519943295769236907684886;

std::string joinNames(const std::vector<std::string>& names)
{
  std::ostringstream oss;
  for (size_t i = 0; i < names.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << names[i];
  }
  return oss.str();
}
}  // namespace

class DynamixelJointStateBridge : public rclcpp::Node
{
public:
  DynamixelJointStateBridge()
  : Node("dynamixel_joint_state_bridge")
  {
    declare_parameter<std::string>("input_topic", "/dynamixel/state/present");
    declare_parameter<std::string>("output_topic", "joint_states");
    declare_parameter<std::vector<int64_t>>("joint_ids", {});
    declare_parameter<std::vector<std::string>>("joint_names", {});

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    const auto joint_ids_raw = get_parameter("joint_ids").as_integer_array();
    joint_names_ = get_parameter("joint_names").as_string_array();

    joint_ids_.reserve(joint_ids_raw.size());
    for (const auto id : joint_ids_raw) {
      if (id < 0 || id > 65535) {
        RCLCPP_WARN(
          get_logger(),
          "joint_ids contains an out-of-range value: %ld. It will be ignored.",
          static_cast<long>(id));
        continue;
      }
      joint_ids_.push_back(static_cast<uint16_t>(id));
    }

    if (!joint_ids_.empty() && !joint_names_.empty() && joint_names_.size() != joint_ids_.size()) {
      RCLCPP_WARN(
        get_logger(),
        "joint_names size (%zu) does not match joint_ids size (%zu). "
        "Fallback names will be used for missing entries.",
        joint_names_.size(), joint_ids_.size());
    }

    publisher_namespaced_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, rclcpp::QoS(10));
    subscription_ = create_subscription<dynamixel_handler_msgs::msg::DynamixelPresent>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&DynamixelJointStateBridge::onPresent, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "bridging %s -> %s (%s)",
      input_topic_.c_str(),
      output_topic_.c_str(),
      joint_ids_.empty() ? "all incoming joints" : "configured joint order");
    if (!joint_names_.empty()) {
      RCLCPP_INFO(get_logger(), "joint_names: [%s]", joinNames(joint_names_).c_str());
    }
  }

private:
  void onPresent(const dynamixel_handler_msgs::msg::DynamixelPresent::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (msg->id_list.size() != msg->position_deg.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring /dynamixel/state/present because id_list size (%zu) != position_deg size (%zu).",
        msg->id_list.size(), msg->position_deg.size());
      return;
    }

    std::unordered_map<uint16_t, double> position_by_id;
    position_by_id.reserve(msg->id_list.size());
    for (size_t i = 0; i < msg->id_list.size(); ++i) {
      position_by_id[msg->id_list[i]] = msg->position_deg[i];
    }

    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    joint_names.reserve(joint_ids_.empty() ? msg->id_list.size() : joint_ids_.size());
    joint_positions.reserve(joint_names.capacity());

    if (joint_ids_.empty()) {
      for (size_t i = 0; i < msg->id_list.size(); ++i) {
        joint_names.push_back(resolveJointName(i, msg->id_list[i]));
        joint_positions.push_back(msg->position_deg[i] * kDegToRad);
      }
    } else {
      for (size_t i = 0; i < joint_ids_.size(); ++i) {
        const auto id = joint_ids_[i];
        const auto it = position_by_id.find(id);
        if (it == position_by_id.end()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Skipping publish because configured joint id %u was not found in the input message.",
            static_cast<unsigned>(id));
          return;
        }
        joint_names.push_back(resolveJointName(i, id));
        joint_positions.push_back(it->second * kDegToRad);
      }
    }

    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    out.name = std::move(joint_names);
    out.position = std::move(joint_positions);
    publisher_namespaced_->publish(out);
  }

  std::string resolveJointName(size_t index, uint16_t id) const
  {
    if (index < joint_names_.size() && !joint_names_[index].empty()) {
      return joint_names_[index];
    }
    return "dxl_" + std::to_string(id);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::vector<uint16_t> joint_ids_;
  std::vector<std::string> joint_names_;
  rclcpp::Subscription<dynamixel_handler_msgs::msg::DynamixelPresent>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_namespaced_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamixelJointStateBridge>());
  rclcpp::shutdown();
  return 0;
}
