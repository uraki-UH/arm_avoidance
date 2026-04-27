#include "gng_vlut_system/bridge/robot_viewer_bridge_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <sstream>
#include <utility>

#include "common/resource_utils.hpp"
#include "description/urdf_loader.hpp"

namespace robot_sim {
namespace bridge {

namespace {
std::string escapeJson(const std::string& text) {
    std::string out;
    out.reserve(text.size() + 8);
    for (const char c : text) {
        switch (c) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            out += c;
            break;
        }
    }
    return out;
}
}

RobotViewerBridgeNode::RobotViewerBridgeNode(const rclcpp::NodeOptions & options)
: Node("robot_viewer_bridge_node", options) {
    const std::string pkg_share = ament_index_cpp::get_package_share_directory("gng_vlut_system");
    const std::string default_urdf = pkg_share + "/urdf/topoarm_description/urdf/topoarm.urdf.xacro";

    declare_parameter<std::string>("robot_name", "topoarm");
    declare_parameter<std::string>("robot_description_file", default_urdf);
    declare_parameter<std::string>("resource_root_dir", "");
    declare_parameter<std::string>("mesh_root_dir", "");
    declare_parameter<std::string>("end_effector_name", "");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>("stream_topic", "/viewer/internal/stream/robot");
    declare_parameter<std::string>("frame_id", "world");
    declare_parameter<double>("publish_hz", 30.0);

    robot_name_ = get_parameter("robot_name").as_string();

    const std::string robot_description_file = get_parameter("robot_description_file").as_string();
    const std::string resolved_urdf_path = robot_sim::common::resolvePath(robot_description_file);
    if (!loadRobotDescription(urdf_content_, resolved_urdf_path)) {
        throw std::runtime_error("Failed to load robot description: " + resolved_urdf_path);
    }

    const std::string resource_root_dir = get_parameter("resource_root_dir").as_string();
    const std::string mesh_root_dir = get_parameter("mesh_root_dir").as_string();
    const std::string end_effector_name = get_parameter("end_effector_name").as_string();

    auto robot_model = simulation::loadRobotFromUrdf(resolved_urdf_path, resource_root_dir, mesh_root_dir);
    chain_ = simulation::createKinematicChainFromModel(robot_model, end_effector_name);

    buildJointIndexMap();
    current_joint_values_.assign(active_joint_names_.size(), 0.0);

    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    stream_topic_ = get_parameter("stream_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    publish_hz_ = std::max(1.0, get_parameter("publish_hz").as_double());

    robot_pub_ = create_publisher<std_msgs::msg::String>(
        stream_topic_, rclcpp::QoS(1).reliable().transient_local());

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_, rclcpp::QoS(10),
        std::bind(&RobotViewerBridgeNode::jointStateCallback, this, std::placeholders::_1));

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz_)),
        std::bind(&RobotViewerBridgeNode::publishCurrentState, this));

    RCLCPP_INFO(get_logger(), "Robot Viewer Bridge initialized: %s", stream_topic_.c_str());
}

bool RobotViewerBridgeNode::loadRobotDescription(std::string& out_text, const std::string& source_path) const {
    std::ifstream source_file(source_path);
    if (!source_file) {
        return false;
    }

    if (source_path.rfind(".xacro") != std::string::npos) {
        const std::string cmd = "xacro " + source_path;
        std::array<char, 4096> buffer{};
        std::string result;

        if (FILE* pipe = popen(cmd.c_str(), "r")) {
            while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
                result += buffer.data();
            }

            const int status = pclose(pipe);
            if (status != 0) {
                return false;
            }
        } else {
            return false;
        }

        out_text = std::move(result);
    } else {
        out_text = std::string((std::istreambuf_iterator<char>(source_file)), std::istreambuf_iterator<char>());
    }

    return !out_text.empty();
}

void RobotViewerBridgeNode::buildJointIndexMap() {
    active_joint_names_.clear();
    joint_name_to_active_index_.clear();
    const int num_joints = chain_.getNumJoints();
    for (int i = 0; i < num_joints; ++i) {
        if (chain_.getJointDOF(i) <= 0) continue;
        const std::string joint_name = chain_.getJointName(i);
        joint_name_to_active_index_[joint_name] = active_joint_names_.size();
        active_joint_names_.push_back(joint_name);
    }
}

void RobotViewerBridgeNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (msg->name.empty() || msg->position.empty()) return;

    std::unordered_map<std::string, size_t> incoming_index;
    for (size_t i = 0; i < msg->name.size(); ++i) incoming_index[msg->name[i]] = i;

    for (const auto& [name, active_idx] : joint_name_to_active_index_) {
        auto it = incoming_index.find(name);
        if (it != incoming_index.end() && it->second < msg->position.size()) {
            current_joint_values_[active_idx] = msg->position[it->second];
        }
    }
    last_joint_state_stamp_ = msg->header.stamp;
    has_joint_state_ = true;
    // Removed immediate publish to eliminate jitter and double-publishing.
    // The publish_timer_ will handle steady 30Hz (or configured rate) streaming.
}

std::string RobotViewerBridgeNode::buildRobotJsonLocked(
    const std::string& type,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& positions,
    const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& orientations,
    bool include_urdf) const {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(6);

    const double ts = has_joint_state_
        ? static_cast<double>(last_joint_state_stamp_.sec) + static_cast<double>(last_joint_state_stamp_.nanosec) * 1e-9
        : now().seconds();

    oss << '{'
        << "\"type\":\"" << type << "\","
        << "\"tag\":\"" << escapeJson(robot_name_) << "\","
        << "\"robot\":{"
        << "\"timestamp\":" << ts << ','
        << "\"frameId\":\"" << escapeJson(frame_id_) << "\",";

    if (include_urdf) {
        oss << "\"urdf\":\"" << escapeJson(urdf_content_) << "\",";
    }

    oss << "\"jointNames\":[";

    for (size_t i = 0; i < active_joint_names_.size(); ++i) {
        if (i > 0) oss << ',';
        oss << '"' << escapeJson(active_joint_names_[i]) << '"';
    }
    oss << "],\"jointValues\":[";
    for (size_t i = 0; i < current_joint_values_.size(); ++i) {
        if (i > 0) oss << ',';
        oss << current_joint_values_[i];
    }
    oss << "],\"positions\":[";
    for (size_t i = 0; i < positions.size(); ++i) {
        if (i > 0) oss << ',';
        oss << '[' << positions[i].x() << ',' << positions[i].y() << ',' << positions[i].z() << ']';
    }
    oss << "],\"orientations\":[";
    for (size_t i = 0; i < orientations.size(); ++i) {
        if (i > 0) oss << ',';
        oss << '[' << orientations[i].x() << ',' << orientations[i].y() << ',' << orientations[i].z() << ',' << orientations[i].w() << ']';
    }
    oss << "]}}";
    return oss.str();
}

void RobotViewerBridgeNode::publishCurrentState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    publishCurrentStateLocked();
}

void RobotViewerBridgeNode::publishCurrentStateLocked() {
    if (!robot_pub_) return;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
    chain_.forwardKinematicsAt(current_joint_values_, positions, orientations);

    std_msgs::msg::String msg;
    if (first_publish_) {
        // Send full description once (or when needed)
        msg.data = buildRobotJsonLocked("stream.robot.description", positions, orientations, true);
        robot_pub_->publish(msg);
        first_publish_ = false;
    }

    // Send lightweight pose update
    msg.data = buildRobotJsonLocked("stream.robot.pose", positions, orientations, false);
    robot_pub_->publish(msg);
}

} // namespace bridge
} // namespace robot_sim

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::RobotViewerBridgeNode)
