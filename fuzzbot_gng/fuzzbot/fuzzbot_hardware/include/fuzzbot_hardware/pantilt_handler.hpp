#include <functional>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_position.hpp"
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

// #define PANTILT

using namespace std::placeholders;
using namespace std::chrono_literals;

// Robot Parameter
#define WHEEL_RADIUS 0.060
#define WHEEL_BASE 0.060

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

class fuzzbotHardware : public rclcpp::Node {
    // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<dynamixel_handler_msgs::msg::DynamixelControlXPosition>::SharedPtr dxl_position_pub_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pantilt_sub_;
    // rclcpp::Subscription<dynamixel_handler_msgs::msg::DxlStates>::SharedPtr dxl_states_sub_;

    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle_;

    rclcpp::TimerBase::SharedPtr timer_;

    double pan_angle_ = M_PI;
    double tilt_angle_ = M_PI;
    double global_yaw_ = 0;

    sensor_msgs::msg::JointState jonit_states_;
    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::Imu imu_;
    dynamixel_handler_msgs::msg::DynamixelControlXPosition dxl_position_;
    std::shared_ptr<control_msgs::action::FollowJointTrajectory::Feedback> arm_feedback_;


   public:
    fuzzbotHardware() : Node("fuzzbot_hardware") {
        // joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        dxl_position_pub_ = this->create_publisher<dynamixel_handler_msgs::msg::DynamixelControlXPosition>("dynamixel/command/x/position_control", 10);

        pantilt_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("pantilt_controller/commands", 10, std::bind(&fuzzbotHardware ::pantilt_cb, this, _1));
        // dxl_states_sub_ = this->create_subscription<dynamixel_handler_msgs::msg::DxlStates>("dynamixel/states", 10, std::bind(&fuzzbotHardware ::dxl_states_cb, this, _1));
        
        timer_ = this->create_wall_timer(10ms, std::bind(&fuzzbotHardware::timer_cb, this));

        dxl_position_.id_list.resize(2);
        for (int i = 0; i < 2;++i)
            dxl_position_.id_list[i] = i+2;
        dxl_position_.position_deg.resize(2);
    }

    ~fuzzbotHardware(){
    }

   private:
    void timer_cb() {
        dxl_position_pub_->publish(dxl_position_);
    }
    void pantilt_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if(msg->data.size() == 2){
            pan_angle_ = msg->data[0] + M_PI;
            tilt_angle_ = msg->data[1] + M_PI;
        }
    }
    // void dxl_states_cb(const dynamixel_handler_msgs::msg::DxlStates::SharedPtr msg) {
    //     jonit_states_.header.stamp = this->get_clock()->now();
    //     for (int i = 2; i < msg->present.id_list.size(); ++i){
    //         jonit_states_.position[i] = (double)msg->present.position_deg[i] * M_PI / 180.;
    //         jonit_states_.velocity[i] = (double)msg->present.velocity_deg_s[i] * M_PI / 180.;
    //         jonit_states_.effort[i]   = (double)msg->present.current_ma[i];
    //     }
    //     joint_pub_->publish(jonit_states_);
    // }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fuzzbotHardware>());
    rclcpp::shutdown();
    return 0;
}