// ToPoArm専用のudpのブリッジ

#include "simulation/core/udp_command_bridge.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <cmath>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

namespace robot_sim {
namespace simulation {

UdpCommandBridge::UdpCommandBridge(int port) : port_(port) {
  latest_command_.joint_positions.resize(6, 0.0);
  latest_command_.gripper_position = 0.0;
}

UdpCommandBridge::~UdpCommandBridge() { stop(); }

void UdpCommandBridge::start() {
  if (running_)
    return;

  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    std::cerr << "[UdpBridge] Error: Failed to create socket." << std::endl;
    return;
  }

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(port_);

  if (bind(sockfd_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    std::cerr << "[UdpBridge] Error: Bind failed on port " << port_
              << std::endl;
    close(sockfd_);
    sockfd_ = -1;
    return;
  }

  running_ = true;
  thread_ = std::thread(&UdpCommandBridge::receiverLoop, this);
  std::cout << "[UdpBridge] Started listening on port " << port_ << std::endl;
}

void UdpCommandBridge::stop() {
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  if (sockfd_ >= 0) {
    close(sockfd_);
    sockfd_ = -1;
  }
}

bool UdpCommandBridge::getLatestCommand(JointCommand &out_command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_command_.is_valid) {
    out_command = latest_command_;
    bool was_new = has_new_data_;
    has_new_data_ = false;
    return was_new;
  }
  return false;
}

void UdpCommandBridge::receiverLoop() {
  char buffer[2048];
  while (running_) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sockfd_, &readfds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms timeout for non-blocking check

    int ret = select(sockfd_ + 1, &readfds, NULL, NULL, &tv);
    if (ret > 0 && FD_ISSET(sockfd_, &readfds)) {
      struct sockaddr_in cliaddr;
      socklen_t len = sizeof(cliaddr);
      int n = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, 0,
                       (struct sockaddr *)&cliaddr, &len);
      if (n > 0) {
        buffer[n] = '\0';
        parseCsv(std::string(buffer));
      }
    }
  }
}

void UdpCommandBridge::parseCsv(const std::string &csv) {
  std::vector<double> raw_vals;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    try {
      raw_vals.push_back(std::stod(item));
    } catch (...) {
      // Ignore malformed numbers
    }
  }

  // 必要要素数 (アーム6軸 + グリッパー1軸 = 7以上) をチェック
  if (raw_vals.size() >= 7) {
    std::lock_guard<std::mutex> lock(mutex_);

    // アーム 6軸 (degree * 10 -> radians)
    for (int i = 0; i < 6; ++i) {
      latest_command_.joint_positions[i] = (raw_vals[i] / 10.0) * M_PI / 180.0;
    }

    // グリッパー 1軸 -1800 ~ 1800 -> -0.02 ~ 0.02
    double g_raw = raw_vals[6];
    double g_val = (g_raw / 1800.0) * 0.02; // -0.02 ~ 0.02
    latest_command_.gripper_position = std::max(-0.02, std::min(0.0, g_val));

    latest_command_.timestamp = getCurrentTimeMs();
    latest_command_.is_valid = true;
    has_new_data_ = true;

    // Debug log (Throttle to avoid spam)
    static int log_counter = 0;
    if (log_counter++ % 20 == 0) {
      std::cout << "[UdpBridge] Rx: J[";
      for (int i = 0; i < 6; ++i) {
        std::cout << (raw_vals[i] / 10.0) << (i == 5 ? "" : ", ");
      }
      std::cout << "] G:" << (raw_vals[6] / 10.0) << std::endl;
    }
  }
}

} // namespace simulation
} // namespace robot_sim
