/* ********************************************
 *
 * fuzzbotアーム用ノード
 *
 * ********************************************/
#include "fuzzbot_hardware/pantilt.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fuzzbotHardware>());
    rclcpp::shutdown();
    return 0;
}