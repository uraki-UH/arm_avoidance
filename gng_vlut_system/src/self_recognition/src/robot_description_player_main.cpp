#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "gng_vlut_system/self_recognition/robot_description_player.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::self_recognition::RobotDescriptionPlayer>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
