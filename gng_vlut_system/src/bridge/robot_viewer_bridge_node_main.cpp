#include <rclcpp/rclcpp.hpp>
#include "gng_vlut_system/bridge/robot_viewer_bridge_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_sim::bridge::RobotViewerBridgeNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
