#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <atomic>
#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
double rad2deg10(double rad) { return rad * 180.0 / kPi * 10.0; }
double deg102rad(double deg10) { return (deg10 / 10.0) * kPi / 180.0; }
double clampDouble(double value, double lo, double hi) {
  return std::max(lo, std::min(hi, value));
}
}

class TopoArmUdpBridge : public rclcpp::Node {
public:
  TopoArmUdpBridge() : Node("topoarm_udp_bridge") {
    // Parameters
    udp_port_ = declare_parameter<int>("udp_port", 12345); // Port to listen
    robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.4.1"); // Target IP
    robot_port_ = declare_parameter<int>("robot_port", 12346); // Target Port
    base_frame_ = declare_parameter<std::string>("base_frame", "world");
    output_topic_ = declare_parameter<std::string>("output_topic", "/joint_states_real");
    command_topic_ = declare_parameter<std::string>("command_topic", "/joint_commands");

    // ROS Interfaces
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        output_topic_, rclcpp::QoS(1).reliable().transient_local());
    
    joint_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        command_topic_, 10, std::bind(&TopoArmUdpBridge::commandCallback, this, std::placeholders::_1));

    startSocket();

    // Initial pose
    publishDefaultPose();
    default_pose_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TopoArmUdpBridge::publishDefaultPose, this));

    RCLCPP_INFO(get_logger(), "TopoArm UDP Bridge started. Listening on %d, Sending to %s:%d", 
                udp_port_, robot_ip_.c_str(), robot_port_);
  }

  ~TopoArmUdpBridge() override { stopSocket(); }

private:
  void commandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < 6) return;

    // Convert to Topoarm CSV format: j1,j2,j3,j4,j5,j6,gripper,
    std::stringstream ss;
    for (size_t i = 0; i < 6; ++i) {
      ss << static_cast<int>(rad2deg10(msg->position[i])) << ",";
    }
    // Gripper (Handling if exists)
    double gripper_val = (msg->position.size() >= 7) ? msg->position[6] : 0.0;
    // Simple conversion for gripper if needed, otherwise just 0
    ss << static_cast<int>(gripper_val) << ",";

    sendUdp(ss.str());
  }

  void startSocket() {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create UDP socket");
      return;
    }

    sockaddr_in servaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(static_cast<uint16_t>(udp_port_));

    if (bind(sockfd_, reinterpret_cast<const sockaddr *>(&servaddr), sizeof(servaddr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to bind UDP port %d", udp_port_);
      close(sockfd_);
      sockfd_ = -1;
      return;
    }

    running_ = true;
    receive_thread_ = std::thread(&TopoArmUdpBridge::receiverLoop, this);
  }

  void stopSocket() {
    running_ = false;
    if (receive_thread_.joinable()) receive_thread_.join();
    if (sockfd_ >= 0) { close(sockfd_); sockfd_ = -1; }
  }

  void sendUdp(const std::string &data) {
    if (sockfd_ < 0) return;
    sockaddr_in destaddr{};
    destaddr.sin_family = AF_INET;
    destaddr.sin_port = htons(static_cast<uint16_t>(robot_port_));
    inet_pton(AF_INET, robot_ip_.c_str(), &destaddr.sin_addr);

    sendto(sockfd_, data.c_str(), data.size(), 0,
           reinterpret_cast<const sockaddr *>(&destaddr), sizeof(destaddr));
  }

  void receiverLoop() {
    char buffer[2048];
    while (running_ && rclcpp::ok()) {
      if (sockfd_ < 0) break;

      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(sockfd_, &readfds);
      timeval tv{0, 100000}; // 100ms timeout

      if (select(sockfd_ + 1, &readfds, nullptr, nullptr, &tv) <= 0) continue;

      sockaddr_in cliaddr{};
      socklen_t len = sizeof(cliaddr);
      int n = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, 0,
                       reinterpret_cast<sockaddr *>(&cliaddr), &len);
      if (n <= 0) continue;

      buffer[n] = '\0';
      parseAndPublish(std::string(buffer));
    }
  }

  void parseAndPublish(const std::string &csv) {
    std::vector<double> raw_vals;
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
      try { raw_vals.push_back(std::stod(item)); } catch (...) {}
    }

    if (raw_vals.size() < 6) return;

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_left_joint"};
    msg.position.resize(msg.name.size(), 0.0);

    for (size_t i = 0; i < 6; ++i) {
      msg.position[i] = deg102rad(raw_vals[i]);
    }
    if (raw_vals.size() >= 7) {
      msg.position[6] = clampDouble((raw_vals[6] / 1800.0) * 0.02, -0.02, 0.0);
    }

    joint_state_pub_->publish(msg);
    received_feedback_.store(true);
  }

  void publishDefaultPose() {
    if (received_feedback_.load()) return;
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_left_joint"};
    msg.position = {0.0, 0.2, -0.2, 0.0, 0.0, 0.0, 0.0};
    joint_state_pub_->publish(msg);
  }

  int udp_port_;
  std::string robot_ip_;
  int robot_port_;
  std::string base_frame_;
  std::string output_topic_;
  std::string command_topic_;

  int sockfd_ = -1;
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
