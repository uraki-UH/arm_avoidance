#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "bridge/udp_comm.hpp"

#include <atomic>
#include <cmath>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

class TopoArmUdpBridge : public rclcpp::Node {
public:
  TopoArmUdpBridge() : Node("topoarm_udp_bridge") {
    // 1. Declare & Get Parameters in one line
    const int udp_port = declare_parameter<int>("udp_port", 12345);
    const std::string robot_ip = declare_parameter<std::string>("robot_ip", "192.168.4.1");
    const int robot_port = declare_parameter<int>("robot_port", 12346);
    base_frame_ = declare_parameter<std::string>("base_frame", "world");
    const std::string output_topic = declare_parameter<std::string>("output_topic", "joint_states");
    const std::string command_topic = declare_parameter<std::string>("command_topic", "joint_commands");

    // 2. ROS Interfaces
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic, rclcpp::QoS(1).reliable().transient_local());
    joint_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        command_topic, 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          if (msg->position.size() < 6) return;
          std::string packet;
          for (size_t i = 0; i < 6; ++i) packet += std::to_string(static_cast<int>(msg->position[i] * 1800.0 / M_PI)) + ",";
          packet += std::to_string(static_cast<int>((msg->position.size() >= 7) ? msg->position[6] : 0.0)) + ",";
          if (sender_) sender_->send(packet);
        });

    // 3. Hardware Setup
    receiver_ = std::make_unique<UdpReceiver>(udp_port);
    sender_ = std::make_unique<UdpSender>(robot_ip, robot_port);

    running_ = true;
    receive_thread_ = std::thread([this]() {
      while (running_ && rclcpp::ok()) {
        std::string packet = receiver_->receive();
        if (!packet.empty()) parseAndPublish(packet);
      }
    });

    // Initial / Default Pose
    default_pose_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (received_feedback_.load()) return;
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = now();
      msg.header.frame_id = base_frame_;
      msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_left_joint"};
      msg.position = {0.0, 0.2, -0.2, 0.0, 0.0, 0.0, 0.0};
      joint_state_pub_->publish(msg);
    });

    RCLCPP_INFO(get_logger(), "TopoArm UDP Bridge: %d -> %s:%d", udp_port, robot_ip.c_str(), robot_port);
  }

  ~TopoArmUdpBridge() {
    running_ = false;
    if (receive_thread_.joinable()) receive_thread_.detach();
  }

private:
  void parseAndPublish(const std::string &csv) {
    std::vector<double> vals;
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
      try { vals.push_back(std::stod(item)); } catch (...) {}
    }
    if (vals.size() < 6) return;

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_left_joint"};
    msg.position.assign(msg.name.size(), 0.0);

    for (size_t i = 0; i < 6; ++i) msg.position[i] = (vals[i] / 10.0) * M_PI / 180.0;
    if (vals.size() >= 7) msg.position[6] = std::max(-0.02, std::min(0.0, (vals[6] / 1800.0) * 0.02));

    joint_state_pub_->publish(msg);
    received_feedback_.store(true);
  }

  std::unique_ptr<UdpReceiver> receiver_;
  std::unique_ptr<UdpSender> sender_;
  std::string base_frame_;
  std::atomic<bool> running_{false};
  std::atomic<bool> received_feedback_{false};
  std::thread receive_thread_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
  rclcpp::TimerBase::SharedPtr default_pose_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopoArmUdpBridge>());
  rclcpp::shutdown();
  return 0;
}
