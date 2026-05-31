#include "dynamixel_handler.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>

using std::bind;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    /*Initialization*/
    auto node = std::make_shared<DynamixelHandler>();
    /*Mainloop*/
    auto timer_ = node.get()->create_wall_timer(
          1.0s / (node->loop_rate_)
        , bind(&DynamixelHandler::MainLoop, node.get())
        , node->cbg_serial_
    ); // 変数に保存する必要あり
    /*Interruption*/
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    /*Termination*/
    node.reset(); // rclcpp::shutdown() の前に呼ぶ必要あり
    rclcpp::shutdown();
    return 0;
}
