#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "description/kinematic_adapter.hpp"

namespace robot_sim {
namespace bridge {

class RobotViewerBridgeNode : public rclcpp::Node {
public:
    explicit RobotViewerBridgeNode(const rclcpp::NodeOptions & options);

private:
    bool loadRobotDescription(std::string& out_text, const std::string& source_path) const;
    void buildJointIndexMap();
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    std::string buildRobotJsonLocked(
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& positions,
        const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& orientations) const;
    void publishCurrentState();
    void publishCurrentStateLocked();

    kinematics::KinematicChain chain_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::mutex state_mutex_;
    std::vector<std::string> active_joint_names_;
    std::unordered_map<std::string, size_t> joint_name_to_active_index_;
    std::vector<double> current_joint_values_;
    builtin_interfaces::msg::Time last_joint_state_stamp_;
    bool has_joint_state_ = false;
    std::string urdf_content_;
    std::string joint_state_topic_;
    std::string stream_topic_;
    std::string frame_id_;
    double publish_hz_;
};

} // namespace bridge
} // namespace robot_sim
