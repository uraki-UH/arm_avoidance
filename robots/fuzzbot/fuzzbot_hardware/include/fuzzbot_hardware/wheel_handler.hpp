#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_velocity.hpp"
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class WheelHandler : public rclcpp::Node {
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<dynamixel_handler_msgs::msg::DynamixelControlXVelocity>::SharedPtr dxl_velocity_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<dynamixel_handler_msgs::msg::DxlStates>::SharedPtr dxl_states_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double wheel_radius_;
    double tread_width_; 

    double left_wheel_velocity = 0;
    double right_wheel_velocity = 0;
    int prev_pos_cnt[2];

    double robot_x_   = 0;
    double robot_y_   = 0;
    double robot_yaw_ = 0;

    sensor_msgs::msg::JointState joint_states_;
    nav_msgs::msg::Odometry odom_;
    dynamixel_handler_msgs::msg::DynamixelControlXVelocity dxl_velocity_;


   public:
    WheelHandler() : Node("wheel_handler") {

        this->declare_parameter<double>("wheels.radius", 0.120);
        this->declare_parameter<double>("wheels.tread_width", 0.287);

        this->declare_parameter<std::string>("odometry.topic", "odom");
        this->declare_parameter<std::string>("odometry.frame_id", "odom");
        this->declare_parameter<std::string>("odometry.child_frame_id", "base_footprint");

        wheel_radius_ = this->get_parameter("wheels.radius").as_double();
        tread_width_  = this->get_parameter("wheels.tread_width").as_double();

        std::string odom_topic = this->get_parameter("odometry.topic").as_string();
        std::string odom_frame_id = this->get_parameter("odometry.frame_id").as_string();
        std::string odom_child_frame_id = this->get_parameter("odometry.child_frame_id").as_string();

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        dxl_velocity_pub_ = this->create_publisher<dynamixel_handler_msgs::msg::DynamixelControlXVelocity>("dynamixel/command/x/velocity_control", 10);

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&WheelHandler ::twist_cb, this, _1));
        dxl_states_sub_ = this->create_subscription<dynamixel_handler_msgs::msg::DxlStates>("dynamixel/states", 10, std::bind(&WheelHandler ::dxl_states_cb, this, _1));
        
        timer_ = this->create_wall_timer(10ms, std::bind(&WheelHandler::timer_cb, this));

        joint_states_.name.resize(2);
        joint_states_.name[0] = "wheel_left_joint";
        joint_states_.name[1] = "wheel_right_joint";
        joint_states_.position.resize(2);
        joint_states_.velocity.resize(2);
        joint_states_.effort.resize(2);

        odom_.header.stamp = this->get_clock()->now();
        odom_.header.frame_id = odom_frame_id;
        odom_.child_frame_id  = odom_child_frame_id;
        odom_.pose.pose.position.x = 0;
        odom_.pose.pose.position.y = 0;
        odom_.pose.pose.position.z = 0;
        odom_.pose.pose.orientation.x = 0;
        odom_.pose.pose.orientation.y = 0;
        odom_.pose.pose.orientation.z = 0;
        odom_.pose.pose.orientation.w = 1;

        dxl_velocity_.id_list.resize(2);
        dxl_velocity_.id_list[0] = 1;
        dxl_velocity_.id_list[1] = 2;
        dxl_velocity_.velocity_deg_s.resize(2);
    }

    ~WheelHandler(){
    }

   private:
    void timer_cb() {
        dxl_velocity_pub_->publish(dxl_velocity_);
    }
    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
        dxl_velocity_.velocity_deg_s[0] =  ((msg->linear.x - msg->angular.z * tread_width_ * 0.5) / wheel_radius_) * 180. / M_PI;
        dxl_velocity_.velocity_deg_s[1] = -((msg->linear.x + msg->angular.z * tread_width_ * 0.5) / wheel_radius_) * 180. / M_PI;
    }
    void dxl_states_cb(const dynamixel_handler_msgs::msg::DxlStates::SharedPtr msg) {
        const  rclcpp::Time current_time = this->get_clock()->now();
        static rclcpp::Time last_time    = current_time;
        const  rclcpp::Duration duration = current_time - last_time;
        if (msg->status.id_list.size() != 2)
            return;
        joint_states_.header.stamp = current_time;
        for (uint32_t i = 0; i < msg->present.id_list.size(); ++i){
            joint_states_.position[i] = (double)msg->present.position_deg[i] * M_PI / 180.;
            joint_states_.velocity[i] = (double)msg->present.velocity_deg_s[i] * M_PI / 180.;
            joint_states_.effort[i]   = (double)msg->present.current_ma[i];
        }
        joint_pub_->publish(joint_states_);

        updateOdometry(current_time, duration);
        odom_pub_->publish(odom_);

        last_time = current_time;
    }

    void updateOdometry(const rclcpp::Time &current_time, const rclcpp::Duration &duration)
    {
        double linear_vel  =  (joint_states_.velocity[0] - joint_states_.velocity[1]) * wheel_radius_ * 0.5;
        double angular_vel = (-joint_states_.velocity[1] - joint_states_.velocity[0]) * wheel_radius_ / tread_width_;

        double step_time = duration.seconds();
        robot_yaw_ += angular_vel * step_time;
        robot_x_   += cos(robot_yaw_) * linear_vel * step_time;
        robot_y_   += sin(robot_yaw_) * linear_vel * step_time;

        odom_.header.stamp = current_time;
        odom_.pose.pose.position.x = robot_x_;
        odom_.pose.pose.position.y = robot_y_;
        odom_.pose.pose.orientation.w = cos(robot_yaw_ * 0.5);
    }
};