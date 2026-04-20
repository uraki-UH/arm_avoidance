#include "teleop_joy.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<TeleopJoyNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}