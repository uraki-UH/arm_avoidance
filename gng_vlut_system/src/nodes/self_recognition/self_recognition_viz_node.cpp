#include "gng_vlut_system/self_recognition/self_recognition_viz_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common/resource_utils.hpp"
#include "common/voxel_utils.hpp"
#include "safety_engine/recognition/self_recognition_manager.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"

namespace {
geometry_msgs::msg::Point makePoint(double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

geometry_msgs::msg::Pose poseFromIsometry(const Eigen::Isometry3d& tf) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = tf.translation().x();
    pose.position.y = tf.translation().y();
    pose.position.z = tf.translation().z();
    const Eigen::Quaterniond q(tf.rotation());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}
}  // namespace

namespace robot_sim {
namespace self_recognition {

SelfRecognitionVizNode::SelfRecognitionVizNode(const rclcpp::NodeOptions & options)
: Node("self_recognition_viz_node", options) {
    const std::string pkg_share = ament_index_cpp::get_package_share_directory("gng_vlut_system");
    const std::string default_urdf = pkg_share + "/urdf/topoarm_robot_model/urdf/topoarm.urdf.xacro";

    declare_parameter("robot_urdf_path", default_urdf);
    declare_parameter("joint_topic", "/joint_states");
    declare_parameter("voxel_size", 0.02);
    declare_parameter("update_hz", 10.0);
    declare_parameter("marker_frame_id", "world");
    declare_parameter("publish_self_mask", true);
    declare_parameter("publish_link_voxels", true);
    declare_parameter("publish_link_aabb", true);
    declare_parameter("display_mode", "link_local");

    const std::string urdf_rel = get_parameter("robot_urdf_path").as_string();
    const std::string urdf_path = robot_sim::common::resolvePath(urdf_rel);
    const std::string joint_topic = get_parameter("joint_topic").as_string();
    const double voxel_size_param = get_parameter("voxel_size").as_double();
    const double hz = get_parameter("update_hz").as_double();
    marker_frame_id_ = get_parameter("marker_frame_id").as_string();
    publish_self_mask_ = get_parameter("publish_self_mask").as_bool();
    publish_link_voxels_ = get_parameter("publish_link_voxels").as_bool();
    publish_link_aabb_ = get_parameter("publish_link_aabb").as_bool();
    display_world_coordinates_ = get_parameter("display_mode").as_string() == "world";

    model_ = std::make_shared<simulation::RobotModel>(simulation::loadRobotFromUrdf(urdf_path));
    chain_ = std::make_shared<kinematics::KinematicChain>(
        simulation::createKinematicChainFromModel(*model_));
    fixed_link_info_ = model_->getFixedLinkInfo();

    recognition_manager_ = std::make_unique<robot_sim::recognition::SelfRecognitionManager>();
    recognition_manager_->initialize(*model_, chain_, voxel_size_param);
    voxel_size_f_ = static_cast<float>(voxel_size_param);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, 10,
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            current_joints_ = msg->position;
        });

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/self_recognition_voxel_viz", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(std::max(50, static_cast<int>(1000.0 / std::max(hz, 0.1)))),
        [this]() { this->publishViz(); });

    RCLCPP_INFO(get_logger(),
                "SelfRecognitionVizNode started: urdf=%s joint_topic=%s marker_frame=%s voxel=%.4f",
                urdf_path.c_str(), joint_topic.c_str(), marker_frame_id_.c_str(), voxel_size_param);
}

geometry_msgs::msg::Point SelfRecognitionVizNode::makePoint(double x, double y, double z) const {
    return ::makePoint(x, y, z);
}

std::vector<geometry_msgs::msg::Point> transformPoints(
    const std::vector<geometry_msgs::msg::Point>& points,
    const Eigen::Isometry3d& tf) {
    std::vector<geometry_msgs::msg::Point> transformed;
    transformed.reserve(points.size());
    for (const auto& point : points) {
        const Eigen::Vector3d p(point.x, point.y, point.z);
        const Eigen::Vector3d tp = tf * p;
        transformed.push_back(makePoint(tp.x(), tp.y(), tp.z()));
    }
    return transformed;
}

std::vector<geometry_msgs::msg::Point> SelfRecognitionVizNode::buildVoxelCenters(
    const std::unordered_set<long>& vids) const {
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(vids.size());
    for (long vid : vids) {
        const Eigen::Vector3i idx = GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);
        const Eigen::Vector3f center =
            ::common::geometry::VoxelUtils::voxelToWorld(idx, voxel_size_f_);
        points.push_back(makePoint(center.x(), center.y(), center.z()));
    }
    return points;
}

std::vector<geometry_msgs::msg::Point> SelfRecognitionVizNode::buildAabbLines(
    const Eigen::Vector3d& min_pt,
    const Eigen::Vector3d& max_pt) const {
    const geometry_msgs::msg::Point p000 = makePoint(min_pt.x(), min_pt.y(), min_pt.z());
    const geometry_msgs::msg::Point p001 = makePoint(min_pt.x(), min_pt.y(), max_pt.z());
    const geometry_msgs::msg::Point p010 = makePoint(min_pt.x(), max_pt.y(), min_pt.z());
    const geometry_msgs::msg::Point p011 = makePoint(min_pt.x(), max_pt.y(), max_pt.z());
    const geometry_msgs::msg::Point p100 = makePoint(max_pt.x(), min_pt.y(), min_pt.z());
    const geometry_msgs::msg::Point p101 = makePoint(max_pt.x(), min_pt.y(), max_pt.z());
    const geometry_msgs::msg::Point p110 = makePoint(max_pt.x(), max_pt.y(), min_pt.z());
    const geometry_msgs::msg::Point p111 = makePoint(max_pt.x(), max_pt.y(), max_pt.z());

    return {
        p000, p001, p000, p010, p000, p100,
        p111, p101, p111, p110, p111, p011,
        p001, p011, p001, p101,
        p010, p011, p010, p110,
        p100, p101, p100, p110,
    };
}

visualization_msgs::msg::Marker SelfRecognitionVizNode::makeCubeListMarker(
    const std::string& ns,
    int id,
    const std::string& frame_id,
    const geometry_msgs::msg::Pose& pose,
    const std::vector<geometry_msgs::msg::Point>& points) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = voxel_size_f_;
    marker.scale.y = voxel_size_f_;
    marker.scale.z = voxel_size_f_;
    marker.points = points;
    return marker;
}

visualization_msgs::msg::Marker SelfRecognitionVizNode::makeLineListMarker(
    const std::string& ns,
    int id,
    const std::string& frame_id,
    const geometry_msgs::msg::Pose& pose,
    const std::vector<geometry_msgs::msg::Point>& points) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = std::max(0.001f, voxel_size_f_ * 0.1f);
    marker.points = points;
    return marker;
}

void SelfRecognitionVizNode::publishViz() {
    std::vector<double> joints;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_joints_.empty()) {
            return;
        }
        joints = current_joints_;
    }

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> j_pos;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> j_ori;
    chain_->forwardKinematicsAt(joints, j_pos, j_ori);

    std::map<std::string, Eigen::Isometry3d> link_tfs;
    chain_->buildAllLinkTransforms(j_pos, j_ori, fixed_link_info_, link_tfs);

    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    if (publish_self_mask_) {
        recognition_manager_->updateRobotState(joints);
    const auto vids = recognition_manager_->getSelfVoxelMask();
        std::unordered_set<long> voxel_set(vids.begin(), vids.end());
        const auto points = buildVoxelCenters(voxel_set);
        geometry_msgs::msg::Pose identity_pose;
        identity_pose.orientation.w = 1.0;
        markers.markers.push_back(makeCubeListMarker(
            "self_mask", marker_id++, marker_frame_id_, identity_pose, points));
    }

    const auto& caches = recognition_manager_->getLinkVoxelDataList();
    for (const auto& cache : caches) {
        const auto tf_it = link_tfs.find(cache.name);
        if (tf_it == link_tfs.end()) {
            continue;
        }

        const geometry_msgs::msg::Pose pose = poseFromIsometry(tf_it->second);

        if (publish_link_voxels_) {
            geometry_msgs::msg::Pose marker_pose = pose;
            std::unordered_set<long> local_vids_set;
            for (const auto& p : cache.local_voxel_centers) {
                Eigen::Vector3i idx = ::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), voxel_size_f_);
                local_vids_set.insert(::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx));
            }
            auto points = buildVoxelCenters(local_vids_set);
            if (display_world_coordinates_) {
                points = transformPoints(points, tf_it->second);
                marker_pose.position.x = 0.0;
                marker_pose.position.y = 0.0;
                marker_pose.position.z = 0.0;
                marker_pose.orientation.x = 0.0;
                marker_pose.orientation.y = 0.0;
                marker_pose.orientation.z = 0.0;
                marker_pose.orientation.w = 1.0;
            }
            markers.markers.push_back(makeCubeListMarker(
                "link_voxels/" + cache.name,
                marker_id++,
                marker_frame_id_,
                marker_pose,
                points));
        }

        if (publish_link_aabb_) {
            geometry_msgs::msg::Pose marker_pose = pose;
            auto points = buildAabbLines(cache.local_min, cache.local_max);
            if (display_world_coordinates_) {
                points = transformPoints(points, tf_it->second);
                marker_pose.position.x = 0.0;
                marker_pose.position.y = 0.0;
                marker_pose.position.z = 0.0;
                marker_pose.orientation.x = 0.0;
                marker_pose.orientation.y = 0.0;
                marker_pose.orientation.z = 0.0;
                marker_pose.orientation.w = 1.0;
            }
            markers.markers.push_back(makeLineListMarker(
                "link_aabb/" + cache.name,
                marker_id++,
                marker_frame_id_,
                marker_pose,
                points));
        }
    }

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
