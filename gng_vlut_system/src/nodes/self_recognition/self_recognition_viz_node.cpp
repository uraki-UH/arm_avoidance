#include "gng_vlut_system/self_recognition/self_recognition_viz_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Core>

#include <geometry_msgs/msg/point.hpp>

#include "common/resource_utils.hpp"
#include "safety_engine/recognition/self_recognition_manager.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"

namespace robot_sim {
namespace self_recognition {

SelfRecognitionVizNode::SelfRecognitionVizNode(const rclcpp::NodeOptions & options)
: Node("self_recognition_viz_node", options) {
    std::string pkg_share = ament_index_cpp::get_package_share_directory("gng_vlut_system");
    std::string default_urdf = pkg_share + "/urdf/topoarm_robot_model/urdf/topoarm.urdf.xacro";

    declare_parameter("robot_urdf_path", default_urdf);
    declare_parameter("frame_id", "base_link");
    declare_parameter("voxel_size", 0.02);
    declare_parameter("update_hz", 10.0);

    std::string urdf_rel = get_parameter("robot_urdf_path").as_string();
    std::string urdf_path = robot_sim::common::resolvePath(urdf_rel);
    frame_id_ = get_parameter("frame_id").as_string();
    double voxel_size_param = get_parameter("voxel_size").as_double();
    double hz = get_parameter("update_hz").as_double();

    auto model = std::make_shared<simulation::RobotModel>(simulation::loadRobotFromUrdf(urdf_path));

    auto chain = std::make_shared<kinematics::KinematicChain>(
        simulation::createKinematicChainFromModel(*model)
    );

    recognition_manager_ = std::make_unique<robot_sim::recognition::SelfRecognitionManager>();
    recognition_manager_->initialize(*model, chain, voxel_size_param);
    voxel_size_f_ = static_cast<float>(voxel_size_param);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            current_joints_ = msg->position;
        });

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/self_mask_viz", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
        [this]() { this->publishViz(); });

    RCLCPP_INFO(get_logger(), "Self Recognition Viz Node started (Component).");
}

void SelfRecognitionVizNode::publishViz() {
    std::vector<double> joints;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_joints_.empty()) return;
        joints = current_joints_;
    }

    auto vids = recognition_manager_->getSelfVoxelMask(joints);

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "self_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = static_cast<double>(voxel_size_f_);
    marker.scale.y = static_cast<double>(voxel_size_f_);
    marker.scale.z = static_cast<double>(voxel_size_f_);
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 0.5;

    for (long vid : vids) {
        Eigen::Vector3i idx = GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);
        geometry_msgs::msg::Point p;
        Eigen::Vector3f pf = (idx.cast<float>() * voxel_size_f_) + Eigen::Vector3f::Constant(voxel_size_f_ * 0.5f);
        p.x = static_cast<double>(pf.x());
        p.y = static_cast<double>(pf.y());
        p.z = static_cast<double>(pf.z());
        marker.points.push_back(p);
    }

    markers.markers.push_back(marker);
    marker_pub_->publish(markers);
}

} // namespace self_recognition
} // namespace robot_sim

#include <rclcpp/rclcpp.hpp>
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_sim::self_recognition::SelfRecognitionVizNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::self_recognition::SelfRecognitionVizNode)
