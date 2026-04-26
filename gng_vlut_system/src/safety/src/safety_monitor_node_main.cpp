#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "../safety_monitor_node.cpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::safety::SafetyMonitorNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
