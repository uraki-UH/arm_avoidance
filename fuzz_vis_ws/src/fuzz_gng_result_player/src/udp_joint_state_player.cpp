#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <atomic>
#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
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

double clampDouble(double value, double lo, double hi) {
  return std::max(lo, std::min(hi, value));
}

} // namespace

class UdpJointStatePlayer : public rclcpp::Node {
public:
  UdpJointStatePlayer()
  : Node("udp_joint_state_player") {
    udp_port_ = declare_parameter<int>("udp_port", 12345);
    base_frame_ = declare_parameter<std::string>("base_frame", "world");
    publish_gripper_mirror_ = declare_parameter<bool>("publish_gripper_mirror", true);

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    startSocket();

    default_publish_timer_ = create_wall_timer(
        std::chrono::seconds(1), std::bind(&UdpJointStatePlayer::publishDefaultPose, this));
    publishDefaultPose();

    RCLCPP_INFO(get_logger(),
                "udp_joint_state_player started: port=%d base_frame=%s mirror_gripper=%s",
                udp_port_, base_frame_.c_str(),
                publish_gripper_mirror_ ? "true" : "false");
  }

  ~UdpJointStatePlayer() override { stopSocket(); }

private:
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
    thread_ = std::thread(&UdpJointStatePlayer::receiverLoop, this);
  }

  void stopSocket() {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (sockfd_ >= 0) {
      close(sockfd_);
      sockfd_ = -1;
    }
  }

  void receiverLoop() {
    char buffer[2048];
    while (running_) {
      if (sockfd_ < 0) {
        break;
      }

      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(sockfd_, &readfds);

      timeval tv{};
      tv.tv_sec = 0;
      tv.tv_usec = 100000;

      const int ret = select(sockfd_ + 1, &readfds, nullptr, nullptr, &tv);
      if (ret <= 0 || !FD_ISSET(sockfd_, &readfds)) {
        continue;
      }

      sockaddr_in cliaddr{};
      socklen_t len = sizeof(cliaddr);
      const int n = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, 0,
                             reinterpret_cast<sockaddr *>(&cliaddr), &len);
      if (n <= 0) {
        continue;
      }

      buffer[n] = '\0';
      publishJointState(std::string(buffer));
    }
  }

  void publishJointState(const std::string &csv) {
    std::vector<double> raw_vals;
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
      try {
        raw_vals.push_back(std::stod(item));
      } catch (...) {
        // Ignore malformed fields.
      }
    }

    if (raw_vals.size() < 7) {
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.name = {
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper_left_joint",
    };
    msg.position.resize(msg.name.size(), 0.0);

    for (int i = 0; i < 6; ++i) {
      msg.position[static_cast<std::size_t>(i)] = (raw_vals[static_cast<std::size_t>(i)] / 10.0) * kPi / 180.0;
    }

    const double g_raw = raw_vals[6];
    const double g_val = clampDouble((g_raw / 1800.0) * 0.02, -0.02, 0.0);
    msg.position[6] = g_val;

    joint_state_pub_->publish(msg);
    received_udp_.store(true);

    static int log_counter = 0;
    if (log_counter++ % 20 == 0) {
      RCLCPP_INFO(get_logger(),
                  "UDP joint state: [%.3f %.3f %.3f %.3f %.3f %.3f] gripper=%.4f",
                  msg.position[0], msg.position[1], msg.position[2],
                  msg.position[3], msg.position[4], msg.position[5],
                  msg.position[6]);
    }
  }

  void publishDefaultPose() {
    if (received_udp_.load()) {
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.name = {
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper_left_joint",
    };
    msg.position = {
        0.0,
        0.2,
        -0.2,
        0.0,
        0.0,
        0.0,
        0.0,
    };

    joint_state_pub_->publish(msg);
  }

  int udp_port_ = 12345;
  std::string base_frame_ = "world";
  bool publish_gripper_mirror_ = true;
  int sockfd_ = -1;
  std::atomic<bool> running_{false};
  std::atomic<bool> received_udp_{false};
  std::thread thread_;
  rclcpp::TimerBase::SharedPtr default_publish_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpJointStatePlayer>());
  rclcpp::shutdown();
  return 0;
}
