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

namespace {
constexpr float kEps = 1e-6f;

uint8_t viewerLabelFromStatus(const GNG::Status &status) {
    if (status.is_colliding) return 2; // red
    if (status.is_danger) return 3;    // yellow
    return 1;                         // green
}

geometry_msgs::msg::Point32 toPoint32(const Eigen::Vector3f &v) {
    geometry_msgs::msg::Point32 p;
    p.x = v.x(); p.y = v.y(); p.z = v.z();
    return p;
}
}

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
        this->declare_parameter("publish_hz", 20.0);

        std::string gng_path = resolveResultPath(
            this->get_parameter("gng_model_path").as_string(),
            /*is_vlut=*/false);
        std::string vlut_path = resolveResultPath(
            this->get_parameter("vlut_path").as_string(),
            /*is_vlut=*/true);
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        safety_margin_ = this->get_parameter("safety_margin").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
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
            "/points", rclcpp::SensorDataQoS(), std::bind(&SafetyMonitorNode::pointCloudCallback, this, std::placeholders::_1));

        occupied_voxel_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "/occupied_voxels", 10, std::bind(&SafetyMonitorNode::occupiedVoxelCallback, this, std::placeholders::_1));
        
        danger_voxel_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "/danger_voxels", 10, std::bind(&SafetyMonitorNode::dangerVoxelCallback, this, std::placeholders::_1));
        
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&SafetyMonitorNode::jointStateCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/gng_viz", 10);
        
        topological_map_pub_ = this->create_publisher<ais_gng_msgs::msg::TopologicalMap>(
            "/topological_map", rclcpp::QoS(1).reliable().transient_local());

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

        context_->update(occ_vids, dan_vids);
        
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

        geometry_msgs::msg::TransformStamped tf_msg;
        try {
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

        auto occupied_vids = processor_->voxelize(transformed_points);
        auto danger_vids = processor_->dilate(occupied_vids, (float)safety_margin_);

        updateSafety(occupied_vids, danger_vids);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(update_mutex_);
        latest_joints_ = *msg;
    }

    void publishTimerCallback() {
        std::lock_guard<std::mutex> lock(update_mutex_);
        if (!graph_dirty_) return;
        
        publishGraphLocked();
        publishMarkersLocked();
        graph_dirty_ = false;
    }

    void publishGraphLocked() {
        if (!context_ || !context_->gng) return;

        ais_gng_msgs::msg::TopologicalMap msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = base_frame_;

        auto& gng = *context_->gng;
        std::unordered_map<int, uint16_t> id_to_index;
        msg.nodes.reserve(gng.getNodes().size());

        // 1. Build Nodes
        for (size_t i = 0; i < gng.getNodes().size(); ++i) {
            const auto& node = gng.getNodes()[i];
            if (node.id == -1) continue;

            ais_gng_msgs::msg::TopologicalNode out;
            out.id = static_cast<uint16_t>(node.id);
            out.pos = toPoint32(node.weight_coord);
            
            const Eigen::Vector3f normal = (node.status.ee_direction.norm() > kEps)
                                               ? node.status.ee_direction.normalized()
                                               : Eigen::Vector3f::UnitZ();
            out.normal = toPoint32(normal);
            out.label = viewerLabelFromStatus(node.status);
            
            id_to_index[node.id] = static_cast<uint16_t>(msg.nodes.size());
            msg.nodes.push_back(std::move(out));
        }

        // 2. Build Edges (Coord edges for standard offline map)
        std::unordered_set<uint64_t> seen_edges;
        for (size_t i = 0; i < gng.getNodes().size(); ++i) {
            const auto& node = gng.getNodes()[i];
            if (node.id == -1) continue;

            const auto& neighbors = gng.getNeighborsCoord(static_cast<int>(i));
            for (int neighbor_id : neighbors) {
                if (neighbor_id < 0 || id_to_index.find(neighbor_id) == id_to_index.end()) continue;

                int lo = std::min((int)node.id, neighbor_id);
                int hi = std::max((int)node.id, neighbor_id);
                uint64_t key = (static_cast<uint64_t>(lo) << 32) | static_cast<uint32_t>(hi);
                if (!seen_edges.insert(key).second) continue;

                msg.edges.push_back(id_to_index[node.id]);
                msg.edges.push_back(id_to_index[neighbor_id]);
            }
        }

        topological_map_pub_->publish(msg);
    }

    void publishMarkersLocked() {
        if (!context_ || !context_->gng) return;
        visualization_msgs::msg::MarkerArray markers;
        int id = 0;
        for (const auto& node : context_->gng->getNodes()) {
            if (node.id == -1) continue;
            visualization_msgs::msg::Marker m;
            m.header.frame_id = base_frame_; m.header.stamp = this->now();
            m.ns = "gng_safety"; m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.scale.x = m.scale.y = m.scale.z = 0.01;
            m.pose.position.x = node.weight_coord.x();
            m.pose.position.y = node.weight_coord.y();
            m.pose.position.z = node.weight_coord.z();
            m.pose.orientation.w = 1.0; // Essential for valid quaternion
            
            uint8_t label = viewerLabelFromStatus(node.status);
            if (label == 2) { m.color.r = 1.0; m.color.a = 0.5; } // Red
            else if (label == 3) { m.color.r = 1.0; m.color.g = 1.0; m.color.a = 0.5; } // Yellow
            else { m.color.g = 1.0; m.color.a = 0.2; } // Green
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
    bool graph_dirty_ = true;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr occupied_voxel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr danger_voxel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr topological_map_pub_;
    rclcpp::TimerBase::SharedPtr viz_timer_;

    std::mutex update_mutex_;
    sensor_msgs::msg::JointState latest_joints_;
    std::vector<long> latest_occ_vids_;
    std::vector<long> latest_dan_vids_;

    double voxel_size_;
    double safety_margin_;
};

} // namespace safety
} // namespace robot_sim

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::safety::SafetyMonitorNode)
