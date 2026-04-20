#include "fuzzbot_hardware/wheel_handler.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelHandler>());
    rclcpp::shutdown();
    return 0;
}