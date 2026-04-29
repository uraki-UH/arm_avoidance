#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Geometry>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
        const std::string& type,
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& positions,
        const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& orientations,
        bool include_urdf) const;
    void publishCurrentState();
    void publishCurrentStateLocked();

    kinematics::KinematicChain chain_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::mutex state_mutex_;
    std::vector<std::string> active_joint_names_;
    std::unordered_map<std::string, size_t> joint_name_to_active_index_;
    std::vector<double> current_joint_values_;
    builtin_interfaces::msg::Time last_joint_state_stamp_;
    bool has_joint_state_ = false;
    std::string urdf_content_;
    std::string robot_name_;
    std::string joint_state_topic_;
    std::string stream_topic_;
    std::string frame_id_;
    double publish_hz_;
    bool first_publish_ = true;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    mutable Eigen::Vector3d last_base_pos_{0, 0, 0};
    mutable Eigen::Quaterniond last_base_quat_{1, 0, 0, 0};

    rclcpp::Time start_time_;
    int tick_count_ = 0;
};

} // namespace bridge
} // namespace robot_sim
