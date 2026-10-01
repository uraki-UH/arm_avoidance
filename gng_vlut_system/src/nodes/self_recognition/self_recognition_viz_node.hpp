#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <memory>
#include <chrono>
#include <mutex>
#include <vector>

#include "robot_model/robot_model.hpp"
#include "kinematics/kinematic_chain.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

namespace tf2_ros {
class Buffer;
class TransformListener;
}

namespace robot_sim {
namespace recognition {
class SelfRecognitionManager;
}

namespace self_recognition {

class SelfRecognitionVizNode : public rclcpp::Node {
public:
    explicit SelfRecognitionVizNode(const rclcpp::NodeOptions & options);

private:
    void updateAndPublish();

    std::unique_ptr<robot_sim::recognition::SelfRecognitionManager> recognition_manager_;
    std::shared_ptr<simulation::RobotModel> model_;
    std::shared_ptr<kinematics::KinematicChain> chain_;
    
    std::string root_link_;
    std::string mask_topic_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<double> current_joints_;
    // 実測更新が停止した自己形状の再配信抑止
    double max_joint_state_age_sec_{0.0};
    std::chrono::steady_clock::time_point joint_received_at_;
    int64_t last_joint_stamp_ns_{-1};
    std::mutex mutex_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr latched_joint_sub_;
    rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr mask_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr graph_timer_;
};

} // namespace self_recognition
} // namespace robot_sim
