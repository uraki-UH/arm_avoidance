#pragma once

#include <cstdlib> 
#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class TeleopJoyNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Joy current_joy_;
    bool is_data_updated_;

    std::mutex data_mtx_;
    
    std::string robot_name_;
    double max_linear_;
    double max_angular_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();
    void displayMessage(const double linear_x, const double angular_z);
    void clearDisplay();
    void clearDisplay(const int line_num);

public:
    TeleopJoyNode();
    ~TeleopJoyNode();
};