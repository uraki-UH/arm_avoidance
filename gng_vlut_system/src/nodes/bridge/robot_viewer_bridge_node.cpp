#include "bridge/robot_viewer_bridge_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <map>
#include <stdexcept>
#include <tf2_eigen/tf2_eigen.hpp>

#include "common/resource_utils.hpp"
#include "core/common/manipulability_serialization.hpp"
#include "core/metrics/manipulability.hpp"
#include "robot_model/urdf_loader.hpp"
#include "core/common/constants.hpp"

using json = nlohmann::json;

namespace robot_sim::bridge {

namespace {

std::string detectLocalMeshPackageName(const std::string &source_path) {
    std::filesystem::path current(source_path);
    if (current.empty()) {
        return "";
    }

    current = std::filesystem::absolute(current).parent_path();
    while (!current.empty()) {
        if (std::filesystem::exists(current / "meshes")) {
            return current.filename().string();
        }
        const auto parent = current.parent_path();
        if (parent == current) {
            break;
        }
        current = parent;
    }
    return "";
}

std::string rewriteRelativeMeshUris(const std::string &urdf_text,
                                    const std::string &package_name) {
    if (urdf_text.empty() || package_name.empty()) {
        return urdf_text;
    }

    std::string rewritten = urdf_text;
    const std::string prefix = "package://" + package_name + "/";

    const std::string double_quote_key = "filename=\"meshes/";
    std::size_t pos = 0;
    while ((pos = rewritten.find(double_quote_key, pos)) != std::string::npos) {
        rewritten.replace(pos, double_quote_key.size(), "filename=\"" + prefix);
        pos += prefix.size();
    }

    const std::string single_quote_key = "filename='meshes/";
    pos = 0;
    while ((pos = rewritten.find(single_quote_key, pos)) != std::string::npos) {
        rewritten.replace(pos, single_quote_key.size(), "filename='" + prefix);
        pos += prefix.size();
    }

    return rewritten;
}

} // namespace

RobotViewerBridgeNode::RobotViewerBridgeNode(const rclcpp::NodeOptions & options)
: Node("robot_viewer_bridge_node", options) {
    robot_name_ = declare_parameter<std::string>("robot_name", "topoarm");
    const std::string urdf_path = declare_parameter<std::string>("urdf_path", "");
    const std::string resource_root_dir = declare_parameter<std::string>("resource_root_dir", "");
    const std::string mesh_root_dir = declare_parameter<std::string>("mesh_root_dir", "");
    const std::string end_effector_name = declare_parameter<std::string>("end_effector_name", "");
    const std::string eef_link_names = declare_parameter<std::string>("robot.eef_link_names", "");
    arm_leaf_link_names_ = declare_parameter<std::string>("robot.arm_leaf_link_names", "");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "joint_states");
    stream_topic_ = declare_parameter<std::string>("stream_topic", "/viewer/internal/stream/robot");
    frame_id_ = declare_parameter<std::string>("frame_id", ::robot_sim::common::Constants::DEFAULT_WORLD_FRAME);
    publish_hz_ = std::max(1.0, declare_parameter<double>("publish_hz", ::robot_sim::common::Constants::DEFAULT_VIEWER_HZ));

    // Resolve frame_id with namespace if it's not a global frame
    std::string ns = get_namespace();
    if (ns != "/" && !ns.empty()) {
        if (ns[0] == '/') ns = ns.substr(1);
        // Only prefix if it's a relative frame and not "world"
        if (!frame_id_.empty() && frame_id_ != "world" && frame_id_[0] != '/') {
            frame_id_ = ns + "/" + frame_id_;
        }
    }

    if (urdf_path.empty()) {
        throw std::runtime_error(
            "No robot description path was provided. "
            "Set urdf_path in the params file or pass urdf_path explicitly.");
    }

    const std::string resolved_urdf_path = robot_sim::common::resolvePath(urdf_path);
    if (!loadRobotDescription(urdf_content_, resolved_urdf_path)) {
        throw std::runtime_error("Failed to load robot description: " + resolved_urdf_path);
    }

    robot_model_ = simulation::loadRobotFromUrdf(resolved_urdf_path, resource_root_dir, mesh_root_dir);
    auto eef_names = splitCommaSeparated(eef_link_names);
    auto arm_leaf_names = splitCommaSeparated(arm_leaf_link_names_);
    if (!eef_names.empty()) {
        arm_leaf_names = eef_names;
    } else if (arm_leaf_names.empty()) {
        arm_leaf_names = inferLeafLinkNames();
    }
    if (arm_leaf_names.size() > 1) {
        chain_ = simulation::createMultiArmKinematicChainFromModels(robot_model_, arm_leaf_names);
    } else if (arm_leaf_names.size() == 1) {
        chain_ = std::make_unique<kinematics::KinematicChain>(
            simulation::createKinematicChainFromModel(robot_model_, arm_leaf_names.front()));
    } else {
        chain_ = std::make_unique<kinematics::KinematicChain>(
            simulation::createKinematicChainFromModel(robot_model_, end_effector_name));
    }

    buildJointIndexMap();
    buildChainJointIndexMap();
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

    RCLCPP_INFO(
        get_logger(),
        "Robot Viewer Bridge initialized: robot=%s joint_state_topic=%s stream_topic=%s joints=%zu dof=%d",
        robot_name_.c_str(),
        joint_state_topic_.c_str(),
        stream_topic_.c_str(),
        active_joint_names_.size(),
        chain_ ? chain_->getTotalDOF() : 0);
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
    out_text = rewriteRelativeMeshUris(out_text, detectLocalMeshPackageName(source_path));
    return !out_text.empty();
}

std::vector<std::string> RobotViewerBridgeNode::splitCommaSeparated(const std::string &text) {
    std::vector<std::string> items;
    std::stringstream ss(text);
    std::string token;
    while (std::getline(ss, token, ',')) {
        auto begin = token.find_first_not_of(" \t");
        auto end = token.find_last_not_of(" \t");
        if (begin == std::string::npos) {
            continue;
        }
        items.push_back(token.substr(begin, end - begin + 1));
    }
    return items;
}

std::vector<std::string> RobotViewerBridgeNode::inferLeafLinkNames() const {
    std::vector<std::string> leaf_names;
    std::unordered_map<std::string, bool> is_parent;
    for (const auto &[joint_name, joint_props] : robot_model_.getJoints()) {
        (void)joint_name;
        is_parent[joint_props.parent_link] = true;
    }

    const std::string root_name = robot_model_.getRootLinkName();
    for (const auto &[link_name, link_props] : robot_model_.getLinks()) {
        (void)link_props;
        if (link_name == root_name) {
            continue;
        }
        if (is_parent.find(link_name) == is_parent.end()) {
            leaf_names.push_back(link_name);
        }
    }
    return leaf_names;
}

void RobotViewerBridgeNode::buildJointIndexMap() {
    active_joint_names_.clear();
    joint_name_to_active_index_.clear();
    for (const auto &[joint_name, joint_props] : robot_model_.getJoints()) {
        if (joint_props.type == kinematics::JointType::Fixed) {
            continue;
        }
        joint_name_to_active_index_[joint_name] = active_joint_names_.size();
        active_joint_names_.push_back(joint_name);
    }
}

void RobotViewerBridgeNode::buildChainJointIndexMap() {
    chain_joint_names_.clear();
    chain_joint_name_to_active_index_.clear();
    if (!chain_) {
        return;
    }

    const int total_dof = chain_->getTotalDOF();
    chain_joint_names_.reserve(static_cast<std::size_t>(std::max(0, total_dof)));
    for (int i = 0; i < total_dof; ++i) {
        const std::string joint_name = chain_->getJointName(i);
        if (joint_name.empty()) {
            continue;
        }
        chain_joint_name_to_active_index_[joint_name] = chain_joint_names_.size();
        chain_joint_names_.push_back(joint_name);
    }
}

void RobotViewerBridgeNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::size_t matched = 0;
    std::size_t ignored = 0;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        auto it = joint_name_to_active_index_.find(msg->name[i]);
        if (it != joint_name_to_active_index_.end() && i < msg->position.size()) {
            current_joint_values_[it->second] = msg->position[i];
            ++matched;
        } else {
            ++ignored;
        }
    }
    last_joint_state_stamp_ = msg->header.stamp;
    has_joint_state_ = true;
    ++joint_state_msg_count_;

    if (joint_state_msg_count_ <= 5 || matched == 0) {
        RCLCPP_INFO(
            get_logger(),
            "joint_state received: topic=%s msg=%zu matched=%zu ignored=%zu first_joint=%s",
            joint_state_topic_.c_str(),
            joint_state_msg_count_,
            matched,
            ignored,
            active_joint_names_.empty() ? "" : active_joint_names_.front().c_str());
    }
}

std::string RobotViewerBridgeNode::buildRobotJsonLocked(
    const std::string& type,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& positions,
    const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& orientations,
    const std::vector<double>& chain_joint_values,
    bool include_urdf) {
    
    json robot;
    robot["timestamp"] = has_joint_state_ 
        ? (static_cast<double>(last_joint_state_stamp_.sec) + last_joint_state_stamp_.nanosec * ::robot_sim::common::Constants::NANO_TO_SEC)
        : this->now().seconds();
    robot["frameId"] = frame_id_;

    if (frame_id_ != ::robot_sim::common::Constants::DEFAULT_WORLD_FRAME) {
        try {
            auto tf = tf_buffer_->lookupTransform(::robot_sim::common::Constants::DEFAULT_WORLD_FRAME, frame_id_, tf2::TimePointZero);
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

    robot["linkNames"] = json::array();
    robot["linkManipulabilities"] = json::array();
    Eigen::Isometry3d base_tf = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond base_q = Eigen::Quaterniond::Identity();
    if (!positions.empty() && !orientations.empty()) {
        base_tf.translation() = positions.front();
        base_q = orientations.front().normalized();
        base_tf.linear() = base_q.toRotationMatrix();
    }
    if (chain_) {
        const int link_count = chain_->getNumJoints();
        chain_->updateKinematics(chain_joint_values);
        for (int i = 0; i < link_count; ++i) {
            const std::string link_name = chain_->getLinkName(i);
            if (link_name.empty()) continue;
            robot["linkNames"].push_back(link_name);
            Eigen::Isometry3d link_tf = Eigen::Isometry3d::Identity();
            if (!chain_->getLinkTransform(link_name, link_tf)) continue;
            const Eigen::MatrixXd J = chain_->calculateJacobianAt(i + 1, chain_joint_values);
            const Eigen::MatrixXd Jv = J.topRows(3);
            auto link_manip = Manipulability::calculateManipulabilityEllipsoid(Jv);
            robot_sim::common::appendLinkManipulabilityJson(
                robot["linkManipulabilities"], link_name, base_tf, link_tf, link_manip);
        }
    }

    Manipulability::ManipulabilityEllipsoid manip;
    if (chain_ && chain_->getTotalDOF() > 0) {
        const Eigen::MatrixXd J =
            chain_->calculateJacobianAt(chain_->getNumJoints() + 1, chain_joint_values);
        const Eigen::MatrixXd Jv = J.topRows(3);
        manip = Manipulability::calculateManipulabilityEllipsoid(Jv);
    }
    robot["manipValid"] = manip.valid;
    robot["manipValue"] = manip.manipulability;
    robot["manipConditionNumber"] = manip.condition_number;
    robot["isGoal"] = false;
    if (manip.valid) {
        const Eigen::Vector3d world_center = positions.empty() ? Eigen::Vector3d::Zero() : positions.back();
        const Eigen::Vector3d center = base_q.conjugate() * (world_center - base_tf.translation());
        Eigen::Quaterniond q = base_q.conjugate() * Eigen::Quaterniond(manip.principal_directions);
        q.normalize();
        robot["manipCenter"] = {center.x(), center.y(), center.z()};
        robot["manipScale"] = {manip.singular_values.x(), manip.singular_values.y(), manip.singular_values.z()};
        robot["manipOrientation"] = {q.x(), q.y(), q.z(), q.w()};
    } else {
        robot["manipCenter"] = {0.0, 0.0, 0.0};
        robot["manipScale"] = {0.0, 0.0, 0.0};
        robot["manipOrientation"] = {0.0, 0.0, 0.0, 1.0};
    }

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
    std::vector<double> chain_joint_values;
    if (chain_ && chain_->getTotalDOF() > 0) {
        chain_joint_values.assign(chain_->getTotalDOF(), 0.0);
        for (const auto &[joint_name, chain_index] : chain_joint_name_to_active_index_) {
            auto it = joint_name_to_active_index_.find(joint_name);
            if (it != joint_name_to_active_index_.end() &&
                it->second < current_joint_values_.size() &&
                chain_index < chain_joint_values.size()) {
                chain_joint_values[chain_index] = current_joint_values_[it->second];
            }
        }
        chain_->forwardKinematicsAt(chain_joint_values, positions, orientations);
    }

    if (description_pub_->get_subscription_count() > 0 && first_publish_) {
        std_msgs::msg::String msg;
        msg.data = buildRobotJsonLocked("stream.robot.description", positions, orientations, chain_joint_values, true);
        description_pub_->publish(msg);
        first_publish_ = false;
    }

    if (pose_pub_->get_subscription_count() > 0) {
        std_msgs::msg::String msg;
        msg.data = buildRobotJsonLocked("stream.robot.pose", positions, orientations, chain_joint_values, false);
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
