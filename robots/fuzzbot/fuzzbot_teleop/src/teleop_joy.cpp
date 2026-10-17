#include "teleop_joy.hpp"

TeleopJoyNode::TeleopJoyNode() : Node("teleop_joy")
{
    const char* fuzzbot_model = std::getenv("FUZZBOT_MODEL");

    if (fuzzbot_model == "fuzzbot_pro_normal" || fuzzbot_model == "fuzzbot_pro_tilt")
    {
        robot_name_  = "FuzzBot-Pro";
        max_linear_  = 0.48;
        max_angular_ = 3.28;
    }
    else
    {
        robot_name_  = "FuzzBot-Pro";
        max_linear_  = 0.48;
        max_angular_ = 3.28;
    }

    is_data_updated_ = false;

    clearDisplay();
    displayMessage(0.0, 0.0);

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    joy_sub_   = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&TeleopJoyNode::joy_callback, this, _1));
    timer_     = this->create_wall_timer(50ms, std::bind(&TeleopJoyNode::timer_callback, this));
}

TeleopJoyNode::~TeleopJoyNode()
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = 0.0;
    twist.angular.z = 0.0;
    twist_pub_->publish(twist);
    clearDisplay(7);
}

void TeleopJoyNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock{data_mtx_};
    current_joy_     = *msg;
    is_data_updated_ = true;
}

void TeleopJoyNode::timer_callback()
{
    std::lock_guard<std::mutex> lock{data_mtx_};
    geometry_msgs::msg::Twist twist;
    if (!is_data_updated_)
    {
        twist.linear.x  = 0.0;
        twist.angular.z = 0.0;
        twist_pub_->publish(twist);
    }
    else
    {
        twist.linear.x  = max_linear_  * current_joy_.axes[4];
        twist.angular.z = max_angular_ * current_joy_.axes[0];
        if (abs(twist.linear.x) < 0.01)  twist.linear.x  = 0.0;
        if (abs(twist.angular.z) < 0.01) twist.angular.z = 0.0;
        twist_pub_->publish(twist);
        is_data_updated_ = false;
    }
    clearDisplay(6);
    displayMessage(twist.linear.x, twist.angular.z);
}

void TeleopJoyNode::displayMessage(const double linear_x, const double angular_z)
{
    std::cout << "FuzzBot Teleop Joy!"            << std::endl;
    std::cout << std::endl;
    std::cout << "Linear : " << linear_x  << " m/s" << std::endl;
    std::cout << "Angular: " << angular_z << " rad/s" << std::endl;
    std::cout << std::endl;
    std::cout << "CTRL-C to quit" << std::endl;
}

void TeleopJoyNode::clearDisplay()
{
    std::cout << "\033[2J";
}

void TeleopJoyNode::clearDisplay(const int line_num)
{
    for (int i = 0; i < line_num; i++)
    {
        std::cout << "\033[2K";
        std::cout << "\033[1A";
    }
    std::cout << "\033[2K";
}