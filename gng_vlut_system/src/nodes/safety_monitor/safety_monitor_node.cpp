#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

// safety_engine headers
#include "safety_engine/vlut/voxel_processor.hpp"
#include "safety_engine/vlut/safety_vlut_mapper.hpp"
#include "core/safety_engine/runtime/safety_system_loader.hpp"
#include "safety_engine/gng/GrowingNeuralGas.hpp"
#include "safety_engine/recognition/self_recognition_manager.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include "common/resource_utils.hpp"
#include "core/common/constants.hpp"
#include "core/safety_engine/runtime/safety_monitor_helpers.hpp"

#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cctype>
#include <filesystem>
#include <mutex>
#include <vector>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/msg/topological_node.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <unordered_map>
#include <unordered_set>

namespace robot_sim {
namespace safety {

class SafetyMonitorNode : public rclcpp::Node {
public:
    explicit SafetyMonitorNode(const rclcpp::NodeOptions & options)
    : Node("safety_monitor_node", options) {
        // Parameters
        this->declare_parameter("gng_model_path", "");
        this->declare_parameter("vlut_path", "");
        this->declare_parameter("voxel_size", ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
        this->declare_parameter("dilation_radius", 0.05);
        this->declare_parameter("data_directory", "gng_results");
        this->declare_parameter("experiment_id", "standard_train");
        this->declare_parameter("gng_model_filename", "gng.bin");
        this->declare_parameter("vlut_filename", "vlut.bin");
        this->declare_parameter("urdf_path", "");
        this->declare_parameter("base_frame", ::robot_sim::common::Constants::DEFAULT_BASE_FRAME);
        this->declare_parameter("publish_hz", ::robot_sim::common::Constants::DEFAULT_UPDATE_HZ);
        this->declare_parameter("safety_margin", ::robot_sim::common::Constants::DEFAULT_SAFETY_MARGIN);
        this->declare_parameter("tag", "dynamic");
        this->declare_parameter("mode", "dynamic");

        std::string gng_path = resolveResultPath(
            this->get_parameter("gng_model_path").as_string(),
            /*is_vlut=*/false);
        std::string vlut_path = resolveResultPath(
            this->get_parameter("vlut_path").as_string(),
            /*is_vlut=*/true);
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        safety_margin_ = this->get_parameter("safety_margin").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        tag_ = this->get_parameter("tag").as_string();
        mode_ = this->get_parameter("mode").as_string();
        double hz = this->get_parameter("publish_hz").as_double();
        
        // Setup TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Load Safety System
        context_ = robot_sim::analysis::SafetySystemLoader::load(gng_path, vlut_path, 7);
        if (!context_ || !context_->gng || !context_->mapper) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load GNG/VLUT data from %s", gng_path.c_str());
            throw std::runtime_error("Safety system load error");
        }

        // Initialize Processor
        processor_ = std::make_unique<robot_sim::analysis::VoxelProcessor>(voxel_size_);

        // ROS Interfaces
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", rclcpp::SensorDataQoS(), std::bind(&SafetyMonitorNode::pointCloudCallback, this, std::placeholders::_1));

        occupied_voxel_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "occupied_voxels", 10, std::bind(&SafetyMonitorNode::occupiedVoxelCallback, this, std::placeholders::_1));
        
        danger_voxel_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "danger_voxels", 10, std::bind(&SafetyMonitorNode::dangerVoxelCallback, this, std::placeholders::_1));
        
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&SafetyMonitorNode::jointStateCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("gng_viz", 10);
        
        topological_map_pub_ = this->create_publisher<ais_gng_msgs::msg::TopologicalMap>(
            "topological_map", rclcpp::QoS(1).reliable().transient_local());

        viz_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / hz)), 
            std::bind(&SafetyMonitorNode::publishTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Safety Monitor (Integrated) initialized with %zu nodes. Publishing to /topological_map at %.1f Hz", 
                    context_->gng->getNodes().size(), hz);
    }

private:
    std::string resolveResultPath(const std::string& path, bool is_vlut) const {
        if (!path.empty()) {
            if (std::filesystem::path(path).is_absolute()) {
                return path;
            }
        }

        const std::string data_dir = get_parameter("data_directory").as_string();
        const std::string exp_id = get_parameter("experiment_id").as_string();
        std::string filename = path;

        if (filename.empty()) {
            if (is_vlut) {
                filename = get_parameter("vlut_filename").as_string();
            } else {
                filename = get_parameter("gng_model_filename").as_string();
                if (filename.empty()) {
                    filename = exp_id + ".bin";
                }
            }
        }
        
        // Resolve path relative to package root
        std::string data_dir_abs = robot_sim::common::resolveDataPath(data_dir);
        return (std::filesystem::path(data_dir_abs) / exp_id / filename).string();
    }

    void updateSafety(const std::vector<long>& occ_vids, const std::vector<long>& dan_vids) {
        std::lock_guard<std::mutex> lock(update_mutex_);
        if (!context_) return;

        const auto t0 = std::chrono::steady_clock::now();
        context_->update(occ_vids, dan_vids);
        const auto t1 = std::chrono::steady_clock::now();
        const double vlut_update_ms =
            std::chrono::duration<double, std::milli>(t1 - t0).count();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "VLUT update: %.3f ms | occ=%zu dan=%zu",
            vlut_update_ms, occ_vids.size(), dan_vids.size());
        
        auto& gng = *context_->gng;
        const auto& col_counts = context_->mapper->getCollisionCounts();
        const auto& dgr_counts = context_->mapper->getDangerCounts();

        for (size_t i = 0; i < gng.getNodes().size(); ++i) {
            auto& node = gng.getNodes()[i];
            if (node.id == -1) continue;
            auto& status = node.status;
            status.collision_count = (i < col_counts.size()) ? col_counts[i] : 0;
            status.danger_count = (i < dgr_counts.size()) ? dgr_counts[i] : 0;
            
            status.is_colliding = (status.collision_count > 0);
            status.is_danger = (status.danger_count > 0 && !status.is_colliding);
        }
        graph_dirty_ = true;
    }

    void occupiedVoxelCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
        std::vector<long> occ_vids;
        occ_vids.reserve(msg->data.size());
        for (auto val : msg->data) occ_vids.push_back((long)val);
        
        updateSafety(occ_vids, latest_dan_vids_);
        latest_occ_vids_ = occ_vids;
    }

    void dangerVoxelCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
        std::vector<long> dan_vids;
        dan_vids.reserve(msg->data.size());
        for (auto val : msg->data) dan_vids.push_back((long)val);
        
        updateSafety(latest_occ_vids_, dan_vids);
        latest_dan_vids_ = dan_vids;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Received point cloud with empty frame_id!");
            return;
        }

        // 1. 世界座標（base_frame）への変換
        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not transform from %s to %s: %s", 
                                msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
            return;
        }

        Eigen::Isometry3d sensor_to_base = tf2::transformToEigen(tf_msg);
        std::vector<Eigen::Vector3d> all_points;
        all_points.reserve(msg->width * msg->height);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            all_points.push_back(sensor_to_base * Eigen::Vector3d(*iter_x, *iter_y, *iter_z));
        }

        // 3. ボクセル化と安全判定の更新
        std::vector<Eigen::Vector3f> float_points;
        float_points.reserve(all_points.size());
        for (const auto& p : all_points) float_points.push_back(p.cast<float>());

        auto occupied_vids = processor_->voxelize(float_points);
        auto danger_vids = processor_->dilate(occupied_vids, (float)safety_margin_);

        updateSafety(occupied_vids, danger_vids);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr) {
        // No longer needed for self-recognition in monitor node
    }

    void publishTimerCallback() {
        std::lock_guard<std::mutex> lock(update_mutex_);
        if (!graph_dirty_) return;
        
        publishGraphLocked();
        publishMarkersLocked();
        graph_dirty_ = false;
    }

    void publishGraphLocked() {
        topological_map_pub_->publish(
            robot_sim::safety::monitor_helpers::buildGraphMessage(
                *this, context_, base_frame_));
    }

    void publishMarkersLocked() {
        marker_pub_->publish(
            robot_sim::safety::monitor_helpers::buildMarkerArray(
                *this, context_, base_frame_));
    }

    // Members
    std::shared_ptr<robot_sim::analysis::SafetySystemContext> context_;
    std::unique_ptr<robot_sim::analysis::VoxelProcessor> processor_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string base_frame_;
    std::string tag_ = "dynamic";
    std::string mode_ = "dynamic";
    bool graph_dirty_ = true;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr occupied_voxel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr danger_voxel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr topological_map_pub_;
    rclcpp::TimerBase::SharedPtr viz_timer_;

    std::mutex update_mutex_;
    std::vector<long> latest_occ_vids_;
    std::vector<long> latest_dan_vids_;

    double voxel_size_;
    double safety_margin_;
};

} // namespace safety
} // namespace robot_sim

#include <rclcpp/rclcpp.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::safety::SafetyMonitorNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::safety::SafetyMonitorNode)
