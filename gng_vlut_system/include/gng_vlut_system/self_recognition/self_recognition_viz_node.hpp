#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <memory>
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

/**
 * @brief ロボットの現在姿勢から占有ボクセルIDのリスト（マスク）を計算し、配信するノード。
 * 可視化（マーカー）は行わず、データの配信のみに特化する。
 */
class SelfRecognitionVizNode : public rclcpp::Node {
public:
    explicit SelfRecognitionVizNode(const rclcpp::NodeOptions & options);

private:
    void updateAndPublish();

    std::unique_ptr<robot_sim::recognition::SelfRecognitionManager> recognition_manager_;
    std::shared_ptr<simulation::RobotModel> model_;
    std::shared_ptr<kinematics::KinematicChain> chain_;
    
    std::string root_link_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<double> current_joints_;
    std::mutex mutex_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr mask_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace self_recognition
} // namespace robot_sim
