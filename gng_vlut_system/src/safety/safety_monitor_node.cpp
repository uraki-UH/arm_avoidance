#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

// core_safety headers
#include "core_safety/analysis/voxel_processor.hpp"
#include "core_safety/analysis/safety_vlut_mapper.hpp"
#include "core_safety/persistence/safety_system_loader.hpp"
#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "common/resource_utils.hpp"

#include <Eigen/Geometry>

#include <filesystem>
#include <mutex>
#include <vector>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace robot_sim {
namespace safety {

class SafetyMonitorNode : public rclcpp::Node {
public:
    explicit SafetyMonitorNode(const rclcpp::NodeOptions & options)
    : Node("safety_monitor_node", options) {
        // Parameters
        this->declare_parameter("gng_model_path", "");
        this->declare_parameter("vlut_path", "");
        this->declare_parameter("voxel_size", 0.02);
        this->declare_parameter("dilation_radius", 1);
        this->declare_parameter("data_directory", "gng_results");
        this->declare_parameter("experiment_id", "standard_train");
        this->declare_parameter("gng_model_filename", "gng.bin");
        this->declare_parameter("vlut_filename", "vlut.bin");
        this->declare_parameter("robot_urdf_path", "");
        this->declare_parameter("base_frame", "base_link");

        std::string gng_path = resolveResultPath(
            this->get_parameter("gng_model_path").as_string(),
            /*is_vlut=*/false);
        std::string vlut_path = resolveResultPath(
            this->get_parameter("vlut_path").as_string(),
            /*is_vlut=*/true);
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        dilation_ = this->get_parameter("dilation_radius").as_int();
        base_frame_ = this->get_parameter("base_frame").as_string();
        
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
            "/points", rclcpp::SensorDataQoS(), std::bind(&SafetyMonitorNode::pointCloudCallback, this, std::placeholders::_1));

        // New Direct Voxel Inputs
        occupied_voxel_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "/occupied_voxels", 10, std::bind(&SafetyMonitorNode::occupiedVoxelCallback, this, std::placeholders::_1));
        
        danger_voxel_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "/danger_voxels", 10, std::bind(&SafetyMonitorNode::dangerVoxelCallback, this, std::placeholders::_1));
        
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&SafetyMonitorNode::jointStateCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/gng_viz", 10);

        viz_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SafetyMonitorNode::publishVisualization, this));

        RCLCPP_INFO(this->get_logger(), "Safety Monitor initialized (Component) with %zu nodes. Listening to /points and /occupied_voxels.", 
                    context_->gng->getNodes().size());
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

        // Update Mapper & GNG Node Status using VLUT
        context_->update(occ_vids, dan_vids);
        
        auto& gng = *context_->gng;
        const auto& col_counts = context_->mapper->getCollisionCounts();
        const auto& dgr_counts = context_->mapper->getDangerCounts();

        for (int i = 0; i < (int)gng.getNodes().size(); ++i) {
            if (gng.getNodes()[i].id == -1) continue;
            auto& status = gng.getNodes()[i].status;
            status.collision_count = col_counts[i];
            status.danger_count = dgr_counts[i];
            
            status.is_colliding = (col_counts[i] > 0);
            status.is_danger = (dgr_counts[i] > 0);
        }
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

        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            // Lookup transform from sensor frame to base frame
            tf_msg = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not transform from %s to %s: %s", 
                                msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
            return;
        }

        Eigen::Isometry3d sensor_to_base = tf2::transformToEigen(tf_msg);
        std::vector<Eigen::Vector3f> transformed_points;
        transformed_points.reserve(msg->width * msg->height);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            Eigen::Vector3d p_sensor(*iter_x, *iter_y, *iter_z);
            Eigen::Vector3d p_robot = sensor_to_base * p_sensor;
            transformed_points.push_back(p_robot.cast<float>());
        }

        // Process to Voxels
        auto occupied_vids = processor_->voxelize(transformed_points);
        auto danger_vids = processor_->dilate(occupied_vids, (float)dilation_ * (float)voxel_size_);

        updateSafety(occupied_vids, danger_vids);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(update_mutex_);
        latest_joints_ = *msg;
    }

    void publishVisualization() {
        std::lock_guard<std::mutex> lock(update_mutex_);
        if (!context_ || !context_->gng) return;

        visualization_msgs::msg::MarkerArray markers;
        const auto& nodes = context_->gng->getNodes();
        
        int id = 0;
        for (const auto& node : nodes) {
            if (node.id == -1) continue;

            visualization_msgs::msg::Marker m;
            m.header.frame_id = base_frame_; 
            m.header.stamp = this->now();
            m.ns = "gng_safety";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.scale.x = m.scale.y = m.scale.z = 0.01;
            
            m.pose.position.x = node.weight_coord.x();
            m.pose.position.y = node.weight_coord.y();
            m.pose.position.z = node.weight_coord.z();

            if (node.status.is_colliding) {
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.15;
            } else if (node.status.is_danger) {
                m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.15;
            } else {
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.1;
            }
            markers.markers.push_back(m);
        }
        marker_pub_->publish(markers);
    }

    // Members
    std::shared_ptr<robot_sim::analysis::SafetySystemContext> context_;
    std::unique_ptr<robot_sim::analysis::VoxelProcessor> processor_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string base_frame_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr occupied_voxel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr danger_voxel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr viz_timer_;

    std::mutex update_mutex_;
    sensor_msgs::msg::JointState latest_joints_;
    std::vector<long> latest_occ_vids_;
    std::vector<long> latest_dan_vids_;

    double voxel_size_;
    int dilation_;
};

} // namespace safety
} // namespace robot_sim

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::safety::SafetyMonitorNode)
