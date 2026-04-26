#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "../robot_arm_bridge_node.cpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::bridge::RobotArmBridgeNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
