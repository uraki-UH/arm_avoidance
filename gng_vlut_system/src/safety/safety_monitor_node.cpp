#include <rclcpp/rclcpp.hpp>
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

class SafetyMonitorNode : public rclcpp::Node {
public:
    SafetyMonitorNode() : Node("safety_monitor_node") {
        // Parameters
        this->declare_parameter("gng_model_path", "topoarm_full_v2_phase2.bin");
        this->declare_parameter("vlut_path", "gng_spatial_correlation.bin");
        this->declare_parameter("voxel_size", 0.02);
        this->declare_parameter("dilation_radius", 1);
        
        // LiDAR to World Transform Params (default: identity)
        this->declare_parameter("lidar_pos", std::vector<double>{0.0, 0.0, 1.0}); // x,y,z
        this->declare_parameter("lidar_rot", std::vector<double>{0.0, 0.0, 0.0}); // r,p,y (deg)
        
        // Robot to World Transform Params (default: identity)
        this->declare_parameter("robot_pos", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("robot_rot", std::vector<double>{0.0, 0.0, 0.0});

        std::string gng_path = resolveResultPath(this->get_parameter("gng_model_path").as_string());
        std::string vlut_path = resolveResultPath(this->get_parameter("vlut_path").as_string());
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        dilation_ = this->get_parameter("dilation_radius").as_int();
        
        // Setup Transforms
        auto setup_transform = [](const std::vector<double>& pos, const std::vector<double>& rot) {
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.translation() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
            T.linear() = (Eigen::AngleAxisd(rot[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(rot[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(rot[0] * M_PI / 180.0, Eigen::Vector3d::UnitX())).toRotationMatrix();
            return T;
        };

        lidar_to_world_ = setup_transform(
            this->get_parameter("lidar_pos").as_double_array(),
            this->get_parameter("lidar_rot").as_double_array());
        
        Eigen::Isometry3d robot_to_world = setup_transform(
            this->get_parameter("robot_pos").as_double_array(),
            this->get_parameter("robot_rot").as_double_array());
        
        world_to_robot_ = robot_to_world.inverse();

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

        RCLCPP_INFO(this->get_logger(), "Safety Monitor initialized with %zu nodes. Listening to /points and /occupied_voxels.", 
                    context_->gng->getNodes().size());
    }

private:
    std::string resolveResultPath(const std::string& path) const {
        if (path.empty()) {
            return path;
        }

        if (std::filesystem::path(path).is_absolute()) {
            return path;
        }

        if (path.rfind("gng_results/", 0) == 0) {
            return robot_sim::common::resolvePath(path);
        }

        return robot_sim::common::resolvePath(std::string("gng_results/") + path);
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
        
        // For direct voxel input, if danger voxels are not provided, we could dilate here,
        // but typically "final voxel info" should already include dilation if needed.
        // For now, we update only occupied state if danger list is empty.
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
        std::vector<Eigen::Vector3f> transformed_points;
        transformed_points.reserve(msg->width * msg->height);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            Eigen::Vector3d p_env(*iter_x, *iter_y, *iter_z);
            Eigen::Vector3d p_robot = world_to_robot_ * (lidar_to_world_ * p_env);
            transformed_points.push_back(p_robot.cast<float>());
        }

        // Process to Voxels (Internal conversion fallback)
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
            m.header.frame_id = "world"; 
            m.header.stamp = this->now();
            m.ns = "gng_safety";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.scale.x = m.scale.y = m.scale.z = 0.01;
            
            Eigen::Vector3d p_world = world_to_robot_.inverse() * node.weight_coord.cast<double>();
            m.pose.position.x = p_world.x();
            m.pose.position.y = p_world.y();
            m.pose.position.z = p_world.z();

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
    
    Eigen::Isometry3d lidar_to_world_;
    Eigen::Isometry3d world_to_robot_;

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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
