#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/resource_utils.hpp"
#include "description/kinematic_adapter.hpp"
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

} // namespace

class RobotArmBridgeNode : public rclcpp::Node {
public:
    explicit RobotArmBridgeNode(const rclcpp::NodeOptions & options)
    : Node("robot_arm_bridge_node", options) {
        const std::string pkg_share = ament_index_cpp::get_package_share_directory("gng_vlut_system");
        const std::string default_urdf = pkg_share + "/urdf/topoarm_description/urdf/topoarm.urdf.xacro";

        declare_parameter<std::string>("robot_description_file", default_urdf);
        declare_parameter<std::string>("resource_root_dir", "");
        declare_parameter<std::string>("mesh_root_dir", "");
        declare_parameter<std::string>("end_effector_name", "");
        declare_parameter<std::string>("joint_state_topic", "/joint_states");
        declare_parameter<std::string>("stream_topic", "/viewer/internal/stream/robot_arm");
        declare_parameter<std::string>("frame_id", "world");
        declare_parameter<double>("publish_hz", 20.0);

        const std::string robot_description_file = get_parameter("robot_description_file").as_string();
        const std::string resolved_urdf_path = robot_sim::common::resolvePath(robot_description_file);
        
        // Read URDF content for the viewer
        std::ifstream urdf_file(resolved_urdf_path);
        if (urdf_file) {
            urdf_content_ = std::string((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
        } else {
            RCLCPP_WARN(get_logger(), "Could not open URDF file for reading: %s", resolved_urdf_path.c_str());
        }

        const std::string resource_root_dir = get_parameter("resource_root_dir").as_string();
        const std::string mesh_root_dir = get_parameter("mesh_root_dir").as_string();
        const std::string end_effector_name = get_parameter("end_effector_name").as_string();

        auto robot_model = simulation::loadRobotFromUrdf(
            resolved_urdf_path, resource_root_dir, mesh_root_dir);
        chain_ = simulation::createKinematicChainFromModel(robot_model, end_effector_name);

        buildJointIndexMap();
        current_joint_values_.assign(active_joint_names_.size(), 0.0);

        joint_state_topic_ = get_parameter("joint_state_topic").as_string();
        stream_topic_ = get_parameter("stream_topic").as_string();
        frame_id_ = get_parameter("frame_id").as_string();
        publish_hz_ = std::max(1.0, get_parameter("publish_hz").as_double());

        robot_arm_pub_ = create_publisher<std_msgs::msg::String>(
            stream_topic_, rclcpp::QoS(1).reliable().transient_local());

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic_, rclcpp::QoS(10),
            std::bind(&RobotArmBridgeNode::jointStateCallback, this, std::placeholders::_1));

        publish_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz_)),
            std::bind(&RobotArmBridgeNode::publishCurrentState, this));

        publishCurrentState();

        RCLCPP_INFO(get_logger(),
                    "robot_arm_bridge_node initialized (Component): joint_state_topic=%s stream_topic=%s joints=%zu",
                    joint_state_topic_.c_str(), stream_topic_.c_str(), active_joint_names_.size());
    }

private:
    void buildJointIndexMap() {
        active_joint_names_.clear();
        joint_name_to_active_index_.clear();

        const int num_joints = chain_.getNumJoints();
        for (int i = 0; i < num_joints; ++i) {
            if (chain_.getJointDOF(i) <= 0) {
                continue;
            }
            const std::string joint_name = chain_.getJointName(i);
            joint_name_to_active_index_[joint_name] = static_cast<size_t>(active_joint_names_.size());
            active_joint_names_.push_back(joint_name);
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (msg->name.empty() || msg->position.empty()) {
            return;
        }

        std::unordered_map<std::string, size_t> incoming_index;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            incoming_index.emplace(msg->name[i], i);
        }

        for (const auto& [joint_name, active_index] : joint_name_to_active_index_) {
            const auto it = incoming_index.find(joint_name);
            if (it == incoming_index.end()) {
                continue;
            }
            const size_t msg_index = it->second;
            if (msg_index < msg->position.size() && active_index < current_joint_values_.size()) {
                current_joint_values_[active_index] = msg->position[msg_index];
            }
        }

        last_joint_state_stamp_ = msg->header.stamp;
        has_joint_state_ = true;
        publishCurrentStateLocked();
    }

    std::string buildRobotArmJsonLocked(
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& positions,
        const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& orientations) const {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss << std::setprecision(6);

        const double timestamp_sec = has_joint_state_
            ? static_cast<double>(last_joint_state_stamp_.sec) + static_cast<double>(last_joint_state_stamp_.nanosec) * 1e-9
            : now().seconds();

        oss << '{'
            << "\"type\":\"stream.robot_arm\","
            << "\"robotArm\":{"
            << "\"timestamp\":" << timestamp_sec << ','
            << "\"frameId\":\"" << escapeJson(frame_id_) << "\","
            << "\"urdf\":\"" << escapeJson(urdf_content_) << "\","
            << "\"jointNames\":[";

        for (size_t i = 0; i < active_joint_names_.size(); ++i) {
            if (i > 0) {
                oss << ',';
            }
            oss << '"' << escapeJson(active_joint_names_[i]) << '"';
        }

        oss << "],\"jointValues\":[";
        for (size_t i = 0; i < current_joint_values_.size(); ++i) {
            if (i > 0) {
                oss << ',';
            }
            oss << current_joint_values_[i];
        }
        oss << "],\"positions\":[";
        for (size_t i = 0; i < positions.size(); ++i) {
            if (i > 0) {
                oss << ',';
            }
            oss << '[' << positions[i].x() << ',' << positions[i].y() << ',' << positions[i].z() << ']';
        }
        oss << "],\"orientations\":[";
        for (size_t i = 0; i < orientations.size(); ++i) {
            if (i > 0) {
                oss << ',';
            }
            oss << '[' << orientations[i].x() << ',' << orientations[i].y() << ','
                << orientations[i].z() << ',' << orientations[i].w() << ']';
        }
        oss << "]}}";

        return oss.str();
    }

    void publishCurrentState() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        publishCurrentStateLocked();
    }

    void publishCurrentStateLocked() {
        if (!robot_arm_pub_) {
            return;
        }

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
        chain_.forwardKinematicsAt(current_joint_values_, positions, orientations);

        std_msgs::msg::String msg;
        msg.data = buildRobotArmJsonLocked(positions, orientations);
        robot_arm_pub_->publish(msg);
    }

private:
    kinematics::KinematicChain chain_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_arm_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::mutex state_mutex_;
    std::vector<std::string> active_joint_names_;
    std::unordered_map<std::string, size_t> joint_name_to_active_index_;
    std::vector<double> current_joint_values_;
    builtin_interfaces::msg::Time last_joint_state_stamp_;
    bool has_joint_state_ = false;
    std::string urdf_content_;

    std::string joint_state_topic_ = "/joint_states";
    std::string stream_topic_ = "/viewer/internal/stream/robot_arm";
    std::string frame_id_ = "world";
    double publish_hz_ = 20.0;
};

} // namespace bridge
} // namespace robot_sim

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::RobotArmBridgeNode)
