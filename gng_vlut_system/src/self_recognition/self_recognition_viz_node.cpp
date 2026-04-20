#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Relocated loaders
#include "description/urdf_loader.hpp"
#include "description/robot_model.hpp"
#include "description/kinematic_adapter.hpp"
#include "core_safety/recognition/self_recognition_manager.hpp"
#include "common/resource_utils.hpp"
#include "common/config_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <memory>
#include <vector>
#include <mutex>
#include <chrono>

/**
 * @brief SelfRecognitionVizNode
 * ロボットの自己認識領域（メッシュベースのボクセル・マスク）をリアルタイムで可視化するノード。
 * 継承関係のエラーを避けるため、委譲（Composition）パターンを採用。
 */
class SelfRecognitionVizNode {
public:
    SelfRecognitionVizNode() {
        // ノード本体の生成
        node_ = ::rclcpp::Node::make_shared("self_recognition_viz_node");

        // Compute default URDF path using package share
        std::string pkg_share = ament_index_cpp::get_package_share_directory("gng_safety");
        std::string default_urdf = pkg_share + "/temp_robot.urdf";

        node_->declare_parameter("robot_urdf_path", default_urdf);
        node_->declare_parameter("voxel_size", 0.02);
        node_->declare_parameter("update_hz", 10.0);

        std::string urdf_rel = node_->get_parameter("robot_urdf_path").as_string();
        std::string urdf_path = robot_sim::common::resolvePath(urdf_rel);
        double voxel_size_param = node_->get_parameter("voxel_size").as_double();
        double hz = node_->get_parameter("update_hz").as_double();

        // 1. Robot Model Load (description_lib)
        auto model = std::make_shared<simulation::RobotModel>(simulation::loadRobotFromUrdf(urdf_path));
        
        // 2. Kinematic Chain Setup
        auto chain = std::make_shared<kinematics::KinematicChain>(
            simulation::createKinematicChainFromModel(*model)
        );

        // 3. Recognition Manager Setup
        recognition_manager_ = std::make_unique<robot_sim::recognition::SelfRecognitionManager>();
        recognition_manager_->initialize(*model, chain, voxel_size_param);
        voxel_size_f_ = (float)voxel_size_param;

        // 4. ROS Interfaces
        joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, 
            [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                current_joints_ = msg->position;
            });

        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/self_mask_viz", 10);

        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
            [this]() { this->publishViz(); });

        RCLCPP_INFO(node_->get_logger(), "Self Recognition Viz Node started (Modern Struct).");
    }

    std::shared_ptr<::rclcpp::Node> get_node() { return node_; }

private:
    void publishViz() {
        std::vector<double> joints;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (current_joints_.empty()) return;
            joints = current_joints_;
        }

        // 高精度判定を実行
        auto vids = recognition_manager_->getSelfVoxelMask(joints, true);
        
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = node_->now();
        marker.ns = "self_voxels";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = (double)voxel_size_f_;
        marker.scale.y = (double)voxel_size_f_;
        marker.scale.z = (double)voxel_size_f_;
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 0.5;

        for (long vid : vids) {
            Eigen::Vector3i idx = GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);
            geometry_msgs::msg::Point p;
            Eigen::Vector3f pf = (idx.cast<float>() * voxel_size_f_) + Eigen::Vector3f::Constant(voxel_size_f_ * 0.5f);
            p.x = (double)pf.x();
            p.y = (double)pf.y();
            p.z = (double)pf.z();
            marker.points.push_back(p);
        }

        markers.markers.push_back(marker);
        marker_pub_->publish(markers);
    }

    std::shared_ptr<::rclcpp::Node> node_;
    std::unique_ptr<robot_sim::recognition::SelfRecognitionManager> recognition_manager_;
    float voxel_size_f_;
    std::vector<double> current_joints_;
    std::mutex mutex_;

    ::rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    ::rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    ::rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    ::rclcpp::init(argc, argv);
    auto viz_wrapper = std::make_shared<SelfRecognitionVizNode>();
    ::rclcpp::spin(viz_wrapper->get_node());
    ::rclcpp::shutdown();
    return 0;
}
