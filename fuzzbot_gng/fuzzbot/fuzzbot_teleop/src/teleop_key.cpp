#include "teleop_key.hpp"

TeleopKeyNode::TeleopKeyNode() : Node("teleop_key")
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

    linear_sign_  = 0.0;
    angular_sign_ = 0.0;
    speed_scale_ = 0.5;

    clearDisplay();
    displayMessage(' ');

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&TeleopKeyNode::timer_callback, this));
}

TeleopKeyNode::~TeleopKeyNode()
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = 0.0;
    twist.angular.z = 0.0;
    twist_pub_->publish(twist);
    clearDisplay(12);
}

void TeleopKeyNode::timer_callback()
{
    char key;
    try
    {
        key = keyboard_.readOne();
    }
    catch (const std::runtime_error &)
    {
        geometry_msgs::msg::Twist twist;
        twist.angular.z = 0.0;
        twist.linear.x  = 0.0;
        twist_pub_->publish(twist);
        return;
    }

    bool is_stop_cmd = false;

    switch (key)
    {
        case KEYCODE_W:
            linear_sign_      = 1.0;
            angular_sign_     = 0.0;
            current_move_cmd_ = key;
            break;
        case KEYCODE_A:
            linear_sign_      = 0.0;
            angular_sign_     = 1.0;
            current_move_cmd_ = key;
            break;
        case KEYCODE_S:
            is_stop_cmd = true;
            linear_sign_      = 0.0;
            angular_sign_     = 0.0;
            current_move_cmd_ = key;
            break;
        case KEYCODE_D:
            linear_sign_      =  0.0;
            angular_sign_     = -1.0;
            current_move_cmd_ = key;
            break;
        case KEYCODE_1:
            speed_scale_ = 0.25;
            break;
        case KEYCODE_2:
            speed_scale_ = 0.5;
            break;
        case KEYCODE_3:
            speed_scale_ = 0.75;
            break;
        case KEYCODE_4:
            speed_scale_ = 1.0;
            break;
        default:
            break;
    }

    if (is_stop_cmd)
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = 0.0;
        twist.angular.z = 0.0;
        twist_pub_->publish(twist);
    }
    else if (linear_sign_ != 0.0 || angular_sign_ != 0.0)
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = linear_sign_  * speed_scale_ * max_linear_;
        twist.angular.z = angular_sign_ * speed_scale_ * max_angular_;
        twist_pub_->publish(twist); 
    }
    clearDisplay(11);
    displayMessage(current_move_cmd_);
}

void TeleopKeyNode::displayMessage(const char move_cmd)
{
                                            std::cout << robot_name_ << " Teleop Key!"            << std::endl;
    if (move_cmd == KEYCODE_W)              std::cout << "\033[1m  W:        Move Forward \033[m" << std::endl;
    else                                    std::cout << "\033[2m  W:        Move Forward \033[m" << std::endl;
    if (move_cmd == KEYCODE_A)              std::cout << "\033[1m  A:        Turn Left \033[m"    << std::endl;
    else                                    std::cout << "\033[2m  A:        Turn Left \033[m"    << std::endl;
    if (move_cmd == KEYCODE_S)              std::cout << "\033[1m  S:        Stop \033[m"         << std::endl;
    else                                    std::cout << "\033[2m  S:        Stop \033[m"         << std::endl;
    if (move_cmd == KEYCODE_D)              std::cout << "\033[1m  D:        Turn Right \033[m"   << std::endl;
    else                                    std::cout << "\033[2m  D:        Turn Right \033[m"   << std::endl;
                                            std::cout <<        "=============================="   << std::endl;
    if (abs(speed_scale_ - 0.25) < 0.01)    std::cout << "\033[1m  1:        Max Speed * 0.25 \033[m" << std::endl;
    else                                    std::cout << "\033[2m  1:        Max Speed * 0.25 \033[m" << std::endl;
    if (abs(speed_scale_ - 0.5)  < 0.01)    std::cout << "\033[1m  2:        Max Speed * 0.5 \033[m"  << std::endl;
    else                                    std::cout << "\033[2m  2:        Max Speed * 0.5 \033[m"  << std::endl;
    if (abs(speed_scale_ - 0.75) < 0.01)    std::cout << "\033[1m  2:        Max Speed * 0.75 \033[m" << std::endl;
    else                                    std::cout << "\033[2m  2:        Max Speed * 0.75 \033[m" << std::endl;
    if (abs(speed_scale_ - 1.0)  < 0.01)    std::cout << "\033[1m  4:        Max Speed \033[m"        << std::endl;
    else                                    std::cout << "\033[2m  4:        Max Speed \033[m"        << std::endl;
                                            std::cout << "CTRL-C to quit"                 << std::endl;
}

void TeleopKeyNode::clearDisplay()
{
    std::cout << "\033[2J";
}

void TeleopKeyNode::clearDisplay(const int line_num)
{
    for (int i = 0; i < line_num; i++)
    {
        std::cout << "\033[2K";
        std::cout << "\033[1A";
    }
    std::cout << "\033[2K";
}