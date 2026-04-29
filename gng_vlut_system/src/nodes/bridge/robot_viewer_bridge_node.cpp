#include "gng_vlut_system/bridge/robot_viewer_bridge_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <tf2_eigen/tf2_eigen.hpp>

#include "common/resource_utils.hpp"
#include "robot_model/urdf_loader.hpp"

using json = nlohmann::json;

namespace robot_sim::bridge {

RobotViewerBridgeNode::RobotViewerBridgeNode(const rclcpp::NodeOptions & options)
: Node("robot_viewer_bridge_node", options) {
    const std::string pkg_share = ament_index_cpp::get_package_share_directory("gng_vlut_system");
    const std::string default_urdf = pkg_share + "/urdf/topoarm_robot_model/urdf/topoarm.urdf.xacro";

    robot_name_ = declare_parameter<std::string>("robot_name", "topoarm");
    const std::string robot_description_file = declare_parameter<std::string>("robot_description_file", default_urdf);
    const std::string resource_root_dir = declare_parameter<std::string>("resource_root_dir", "");
    const std::string mesh_root_dir = declare_parameter<std::string>("mesh_root_dir", "");
    const std::string end_effector_name = declare_parameter<std::string>("end_effector_name", "");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "joint_states");
    stream_topic_ = declare_parameter<std::string>("stream_topic", "/viewer/internal/stream/robot");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    publish_hz_ = std::max(1.0, declare_parameter<double>("publish_hz", 30.0));

    const std::string resolved_urdf_path = robot_sim::common::resolvePath(robot_description_file);
    if (!loadRobotDescription(urdf_content_, resolved_urdf_path)) {
        throw std::runtime_error("Failed to load robot description: " + resolved_urdf_path);
    }

    auto robot_model = simulation::loadRobotFromUrdf(resolved_urdf_path, resource_root_dir, mesh_root_dir);
    chain_ = simulation::createKinematicChainFromModel(robot_model, end_effector_name);

    buildJointIndexMap();
    current_joint_values_.assign(active_joint_names_.size(), 0.0);

    description_pub_ = create_publisher<std_msgs::msg::String>(stream_topic_ + "/description", rclcpp::QoS(1).reliable().transient_local());
    pose_pub_ = create_publisher<std_msgs::msg::String>(stream_topic_ + "/pose", rclcpp::QoS(1).best_effort());

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_, 10, std::bind(&RobotViewerBridgeNode::jointStateCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz_)),
        std::bind(&RobotViewerBridgeNode::publishCurrentState, this));

    // Initially publish description
    publishCurrentState();

    RCLCPP_INFO(get_logger(), "Robot Viewer Bridge initialized: %s", stream_topic_.c_str());
}

bool RobotViewerBridgeNode::loadRobotDescription(std::string& out_text, const std::string& source_path) const {
    if (source_path.empty()) return false;
    
    if (source_path.rfind(".xacro") != std::string::npos) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(("xacro " + source_path).c_str(), "r"), pclose);
        if (!pipe) return false;
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        out_text = result;
    } else {
        std::ifstream ifs(source_path);
        if (!ifs) return false;
        out_text = std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    }
    return !out_text.empty();
}

void RobotViewerBridgeNode::buildJointIndexMap() {
    active_joint_names_.clear();
    joint_name_to_active_index_.clear();
    for (int i = 0; i < chain_.getNumJoints(); ++i) {
        if (chain_.getJointDOF(i) <= 0) continue;
        const std::string name = chain_.getJointName(i);
        joint_name_to_active_index_[name] = active_joint_names_.size();
        active_joint_names_.push_back(name);
    }
}

void RobotViewerBridgeNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
        auto it = joint_name_to_active_index_.find(msg->name[i]);
        if (it != joint_name_to_active_index_.end() && i < msg->position.size()) {
            current_joint_values_[it->second] = msg->position[i];
        }
    }
    last_joint_state_stamp_ = msg->header.stamp;
    has_joint_state_ = true;
}

std::string RobotViewerBridgeNode::buildRobotJsonLocked(
    const std::string& type,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& positions,
    const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& orientations,
    bool include_urdf) const {
    
    json robot;
    robot["timestamp"] = has_joint_state_ 
        ? (static_cast<double>(last_joint_state_stamp_.sec) + last_joint_state_stamp_.nanosec * 1e-9)
        : this->now().seconds();
    robot["frameId"] = frame_id_;

    if (frame_id_ != "world") {
        try {
            auto tf = tf_buffer_->lookupTransform("world", frame_id_, tf2::TimePointZero);
            last_base_pos_ = {tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z};
            last_base_quat_ = {tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z};
        } catch (...) {}
    }

    robot["basePosition"] = {last_base_pos_.x(), last_base_pos_.y(), last_base_pos_.z()};
    robot["baseOrientation"] = {last_base_quat_.x(), last_base_quat_.y(), last_base_quat_.z(), last_base_quat_.w()};
    
    if (include_urdf) robot["urdf"] = urdf_content_;
    
    robot["jointNames"] = active_joint_names_;
    robot["jointValues"] = current_joint_values_;

    auto& pos_arr = robot["positions"] = json::array();
    for (const auto& v : positions) pos_arr.push_back({v.x(), v.y(), v.z()});

    auto& quat_arr = robot["orientations"] = json::array();
    for (const auto& q : orientations) quat_arr.push_back({q.x(), q.y(), q.z(), q.w()});

    json root;
    root["type"] = type;
    root["tag"] = robot_name_;
    root["robot"] = robot;

    return root.dump();
}

void RobotViewerBridgeNode::publishCurrentState() {
    if (pose_pub_->get_subscription_count() == 0 && description_pub_->get_subscription_count() == 0) return;
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
    chain_.forwardKinematicsAt(current_joint_values_, positions, orientations);

    if (description_pub_->get_subscription_count() > 0 && first_publish_) {
        std_msgs::msg::String msg;
        msg.data = buildRobotJsonLocked("stream.robot.description", positions, orientations, true);
        description_pub_->publish(msg);
        first_publish_ = false;
    }

    if (pose_pub_->get_subscription_count() > 0) {
        std_msgs::msg::String msg;
        msg.data = buildRobotJsonLocked("stream.robot.pose", positions, orientations, false);
        pose_pub_->publish(msg);
    }
}


} // namespace robot_sim::bridge

#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_sim::bridge::RobotViewerBridgeNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::RobotViewerBridgeNode)
