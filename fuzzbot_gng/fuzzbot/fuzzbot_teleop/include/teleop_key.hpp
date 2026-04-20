#pragma once

#include <cstdlib> 
#include <chrono>

#include "keyboard_reader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TeleopKeyNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    KeyboardReader keyboard_;

    std::string robot_name_;
    double max_linear_;
    double max_angular_;

    double linear_sign_;
    double angular_sign_;
    double speed_scale_;
    char   current_move_cmd_;

    void timer_callback();
    void displayMessage(const char move_cmd);
    void clearDisplay();
    void clearDisplay(const int line_num);

public:
    TeleopKeyNode();
    ~TeleopKeyNode();
};