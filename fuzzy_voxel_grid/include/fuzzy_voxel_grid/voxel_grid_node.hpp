#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <point_cloud_store.hpp>

#include "ais_gng_msgs/msg/topological_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace fuzzy_voxel_grid
{

enum class UpdateMode : uint8_t
{
    LIVE = 0,
    FROZEN = 1
};

enum class VoxelLabel : uint8_t
{
    Normal = 0,
    AddCandidate = 1,
    DeleteCandidate = 2,
    SkipCandidate = 3
};

struct ColorRGBA
{
    double r;
    double g;
    double b;
    double a;
};

using VoxelKey = voxel_idx::world_bucket_key;
using VoxelKeyHash = voxel_idx::world_bucket_key_hash;

// freezeフィルタに必要なノード座標のみの保持。
struct ManagedVoxel
{
    std::vector<geometry_msgs::msg::Point32> topological_nodes;
    VoxelLabel display_label{VoxelLabel::Normal};
};

struct VoxelView
{
    VoxelKey key;
    uint32_t point_count{0};
    uint32_t node_count{0};
    VoxelLabel display_label{VoxelLabel::Normal};
};

class VoxelGridNode : public rclcpp::Node
{
public:
    explicit VoxelGridNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    struct Parameters
    {
        std::string input_pointcloud_topic;
        std::string input_topological_map_topic;
        std::string marker_array_topic;
        std::string voxel_centers_topic;
        std::string marker_namespace_prefix;

        bool publish_voxel_centers;
        bool print_processing_time;

        double update_rate_hz;

        double voxel_size_x;
        double voxel_size_y;
        double voxel_size_z;

        double grid_origin_x;
        double grid_origin_y;
        double grid_origin_z;

        double range_min_x;
        double range_max_x;
        double range_min_y;
        double range_max_y;
        double range_min_z;
        double range_max_z;

        bool use_exclusion_box;

        double exclude_min_x;
        double exclude_max_x;
        double exclude_min_y;
        double exclude_max_y;
        double exclude_min_z;
        double exclude_max_z;

        ColorRGBA normal_color;
        ColorRGBA add_candidate_color;
        ColorRGBA delete_candidate_color;
        ColorRGBA skip_candidate_color;

        std::string frozen_topological_map_topic;
        std::string filtered_new_points_topic;
            
        int filter_target_label;
        double filter_distance_threshold;
    };

    void declareParameters();
    void loadParameters();
    ColorRGBA loadColor(const std::string & prefix);

    bool hasXYZFields(const sensor_msgs::msg::PointCloud2 & msg) const;
    bool isInRange(float x, float y, float z) const noexcept;
    bool isInExcludedBox(float x, float y, float z) const noexcept;
    VoxelKey pointToVoxelKey(float x, float y, float z) const;
    geometry_msgs::msg::Point voxelCenter(const VoxelKey & key) const;

    void update_point_counts(const sensor_msgs::msg::PointCloud2 &msg);

    VoxelLabel assignIntegratedLabel(uint32_t point_count, uint32_t node_count) const noexcept;
    visualization_msgs::msg::MarkerArray make_marker_cache(
        const std_msgs::msg::Header &header, const std::vector<VoxelView> &voxels, bool is_frozen) const;

    ColorRGBA colorForLabel(VoxelLabel label) const;
    std::string labelName(VoxelLabel label) const;
    int32_t markerIdForLabel(VoxelLabel label) const;
    std::string namespaceForLabel(VoxelLabel label) const;

    visualization_msgs::msg::Marker makeCubeListMarker(
        const std_msgs::msg::Header & header,
        VoxelLabel label,
        const std::vector<geometry_msgs::msg::Point> & points) const;

    void rebuildVoxelsFromLatestMessages();
    void rebuildViewsFromManagedVoxels();
    void printDebugVoxelSummary() const;
    void printProcessingTime(std::int64_t elapsed_us) const;

    void publishCombinedMarkerArray();
    void publishVoxelCenters(
        const std_msgs::msg::Header & header,
        const std::vector<VoxelView> & voxels);

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void topologicalMapCallback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg);
    void timerCallback();

    void handleFreeze(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleResume(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void publishFrozenMarkerArray();
    void publishFrozenTopologicalMap();

    bool shouldRemovePointInFrozenMode(float x, float y, float z) const;
    sensor_msgs::msg::PointCloud2 filterIncomingPointCloud(
        const sensor_msgs::msg::PointCloud2 & msg,
        const Eigen::Isometry3d &source_to_grid = Eigen::Isometry3d::Identity()) const;
    void update_shared_points();

    Parameters params_{};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr topological_map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_centers_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr freeze_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;

    rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr frozen_topological_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_new_points_pub_;

    std::mutex data_mutex_;

    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_msg_;
    std::shared_ptr<voxel_idx::point_frame_channel> shared_points_;
    uint64_t shared_revision_{0};
    std_msgs::msg::Header point_header_;
    bool has_point_header_{false};
    bool has_pending_update_{true};
    double max_tmap_age_sec_{1.0};
    ais_gng_msgs::msg::TopologicalMap::SharedPtr latest_topological_map_msg_;

    voxel_idx::point_cell_spec point_spec_;
    std::shared_ptr<voxel_idx::point_cell_query> point_query_;
    std::shared_ptr<voxel_idx::point_cell_counts> standalone_points_;
    std::shared_ptr<const voxel_idx::point_cell_counts> point_cells_;
    ais_gng_msgs::msg::TopologicalMap::SharedPtr active_topological_map_msg_;
    visualization_msgs::msg::MarkerArray live_markers_, frozen_markers_;
    std::vector<VoxelView> combined_voxels_;

    std_msgs::msg::Header latest_output_header_;
    bool have_output_header_{false};

    UpdateMode mode_{UpdateMode::LIVE};

    std::unordered_map<VoxelKey, ManagedVoxel, VoxelKeyHash> frozen_voxels_;
    std::vector<VoxelView> frozen_combined_voxels_;

    ais_gng_msgs::msg::TopologicalMap::SharedPtr frozen_topological_map_msg_;

    std_msgs::msg::Header frozen_output_header_;
    bool have_frozen_output_header_{false};
};

}  // namespace fuzzy_voxel_grid
