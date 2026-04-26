#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "../robot_description_player.cpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::self_recognition::RobotDescriptionPlayer>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
