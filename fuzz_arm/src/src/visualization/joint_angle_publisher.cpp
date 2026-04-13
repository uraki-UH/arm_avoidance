#include "visualization/joint_angle_publisher.hpp"
#include <iostream>
#include <vector>

namespace robot_sim {
namespace visualization {

JointAnglePublisher::JointAnglePublisher(const std::string &host, int port)
    : sock_fd_(-1), initialized_(false) {

  // Create UDP socket
  sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) {
    std::cerr << "[JointAnglePublisher] Failed to create socket" << std::endl;
    return;
  }

  // Setup server address
  memset(&server_addr_, 0, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(port);

  if (inet_pton(AF_INET, host.c_str(), &server_addr_.sin_addr) <= 0) {
    std::cerr << "[JointAnglePublisher] Invalid address: " << host << std::endl;
    close(sock_fd_);
    sock_fd_ = -1;
    return;
  }

  initialized_ = true;
  std::cout << "[JointAnglePublisher] Initialized. Sending to " << host << ":"
            << port << std::endl;
}

JointAnglePublisher::~JointAnglePublisher() {
  if (sock_fd_ >= 0) {
    close(sock_fd_);
  }
}

void JointAnglePublisher::publish(double timestamp,
                                  const Eigen::VectorXd &joint_angles) {
  if (!initialized_ || sock_fd_ < 0) {
    return;
  }

  // Prepare data: timestamp (double) + joint_angles (floats)
  size_t num_joints = joint_angles.size();
  size_t buffer_size = sizeof(double) + num_joints * sizeof(float);
  std::vector<uint8_t> buffer(buffer_size);

  // Pack timestamp
  memcpy(buffer.data(), &timestamp, sizeof(double));

  // Pack joint angles as floats
  for (size_t i = 0; i < num_joints; ++i) {
    float angle = static_cast<float>(joint_angles[i]);
    memcpy(buffer.data() + sizeof(double) + i * sizeof(float), &angle,
           sizeof(float));
  }

  // Send via UDP
  ssize_t sent = sendto(sock_fd_, buffer.data(), buffer_size, 0,
                        (struct sockaddr *)&server_addr_, sizeof(server_addr_));

  if (sent < 0) {
    // Don't spam errors, just silently fail
    // std::cerr << "[JointAnglePublisher] Failed to send data" << std::endl;
  }
}

} // namespace visualization
} // namespace robot_sim
