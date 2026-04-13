#pragma once

#include <Eigen/Dense>
#include <arpa/inet.h>
#include <cstring>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace robot_sim {
namespace visualization {

/**
 * @brief Publishes robot joint angles via UDP for real-time visualization
 */
class JointAnglePublisher {
public:
  JointAnglePublisher(const std::string &host = "127.0.0.1", int port = 12346);
  ~JointAnglePublisher();

  /**
   * @brief Publish current joint angles
   * @param timestamp Current simulation time
   * @param joint_angles Vector of joint angles in radians
   */
  void publish(double timestamp, const Eigen::VectorXd &joint_angles);

  /**
   * @brief Check if publisher is ready
   */
  bool isReady() const { return sock_fd_ >= 0; }

private:
  int sock_fd_;
  struct sockaddr_in server_addr_;
  bool initialized_;
};

} // namespace visualization
} // namespace robot_sim
