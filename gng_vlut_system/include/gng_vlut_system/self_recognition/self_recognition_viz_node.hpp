#pragma once

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include "robot_model/robot_model.hpp"
#include "kinematics/kinematic_chain.hpp"

namespace robot_sim {
namespace recognition {
class SelfRecognitionManager;
}

namespace self_recognition {

class SelfRecognitionVizNode : public rclcpp::Node {
public:
    explicit SelfRecognitionVizNode(const rclcpp::NodeOptions & options);

private:
    void publishViz();
    geometry_msgs::msg::Point makePoint(double x, double y, double z) const;
    visualization_msgs::msg::Marker makeCubeListMarker(
        const std::string& ns,
        int id,
        const std::string& frame_id,
        const geometry_msgs::msg::Pose& pose,
        const std::vector<geometry_msgs::msg::Point>& points) const;
    visualization_msgs::msg::Marker makeLineListMarker(
        const std::string& ns,
        int id,
        const std::string& frame_id,
        const geometry_msgs::msg::Pose& pose,
        const std::vector<geometry_msgs::msg::Point>& points) const;
    std::vector<geometry_msgs::msg::Point> buildAabbLines(
        const Eigen::Vector3d& min_pt,
        const Eigen::Vector3d& max_pt) const;
    std::vector<geometry_msgs::msg::Point> buildVoxelCenters(
        const std::unordered_set<long>& vids) const;

    std::unique_ptr<robot_sim::recognition::SelfRecognitionManager> recognition_manager_;
    std::shared_ptr<simulation::RobotModel> model_;
    std::shared_ptr<kinematics::KinematicChain> chain_;
    std::map<std::string, std::pair<std::string, Eigen::Isometry3d>> fixed_link_info_;
    float voxel_size_f_ = 0.1f;
    std::string marker_frame_id_ = "world";
    bool publish_self_mask_ = true;
    bool publish_link_voxels_ = true;
    bool publish_link_aabb_ = true;
    bool display_world_coordinates_ = false;
    std::vector<double> current_joints_;
    std::mutex mutex_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace self_recognition
} // namespace robot_sim
