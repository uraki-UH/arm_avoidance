#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

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

struct VoxelKey
{
    int32_t ix;
    int32_t iy;
    int32_t iz;

    bool operator==(const VoxelKey & other) const noexcept
    {
        return ix == other.ix && iy == other.iy && iz == other.iz;
    }
};

struct VoxelKeyHash
{
    std::size_t operator()(const VoxelKey & key) const noexcept;
};

struct PointSample
{
    float x;
    float y;
    float z;
};

struct TopologicalNodeRecord
{
    uint32_t node_id;
    uint8_t label;
    uint32_t age;
    float x;
    float y;
    float z;
};

struct ManagedVoxel
{
    std::vector<PointSample> pointcloud_points;
    std::vector<TopologicalNodeRecord> topological_nodes;

    std::array<uint32_t, 256> topological_label_counts{};
    uint8_t dominant_topological_label{0};

    double average_topological_age{0.0};

    double topological_match_rate{0.0};
    uint32_t topological_matched_count{0};
    uint32_t previous_topological_node_count{0};
    uint32_t current_topological_node_count{0};

    double add_degree{0.0};
    double del_degree{0.0};
    double skip_degree{0.0};

    VoxelLabel display_label{VoxelLabel::Normal};
};

struct PointCloudVoxelSnapshot
{
    std::vector<PointSample> points;
};

struct TopologicalVoxelSnapshot
{
    std::vector<TopologicalNodeRecord> nodes;
    std::array<uint32_t, 256> label_counts{};
};

struct PreviousTopologicalVoxelState
{
    std::vector<uint32_t> node_ids;
};

struct PointCloudVoxelDiff
{
    bool existed_before{false};
    bool exists_now{false};
    int32_t point_count_diff{0};
};

struct TopologicalVoxelDiff
{
    bool existed_before{false};
    bool exists_now{false};
    int32_t node_count_diff{0};
    std::array<int32_t, 256> label_count_diff{};
};

struct VoxelView
{
    VoxelKey key;
    uint32_t point_count;
    uint32_t node_count;
    VoxelLabel display_label;
    uint8_t dominant_topological_label;

    double average_topological_age;
    double add_degree{0.0};
    double del_degree{0.0};
    double skip_degree{0.0};
};

class VoxelGridNode : public rclcpp::Node
{
public:
    VoxelGridNode();

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
    int32_t computeVoxelIndex(float value, double origin, double voxel_size) const noexcept;
    double computeAverageTopologicalAge(const std::vector<TopologicalNodeRecord> & nodes) const noexcept;
    VoxelKey pointToVoxelKey(float x, float y, float z) const noexcept;
    geometry_msgs::msg::Point voxelCenter(const VoxelKey & key) const;

    std::unordered_map<VoxelKey, PointCloudVoxelSnapshot, VoxelKeyHash>
    buildPointCloudVoxelSnapshot(const sensor_msgs::msg::PointCloud2 & msg) const;
    std::unordered_map<VoxelKey, TopologicalVoxelSnapshot, VoxelKeyHash>
    buildTopologicalVoxelSnapshot(const ais_gng_msgs::msg::TopologicalMap & msg) const;
    std::unordered_map<VoxelKey, PointCloudVoxelDiff, VoxelKeyHash>
    diffPointCloudVoxels(
    const std::unordered_map<VoxelKey, PointCloudVoxelSnapshot, VoxelKeyHash> & prev_map,
    const std::unordered_map<VoxelKey, PointCloudVoxelSnapshot, VoxelKeyHash> & curr_map) const;
    std::unordered_map<VoxelKey, TopologicalVoxelDiff, VoxelKeyHash>
    diffTopologicalVoxels(
    const std::unordered_map<VoxelKey, TopologicalVoxelSnapshot, VoxelKeyHash> & prev_map,
    const std::unordered_map<VoxelKey, TopologicalVoxelSnapshot, VoxelKeyHash> & curr_map) const;

    uint32_t countMatchedNodeIds(
    const std::vector<TopologicalNodeRecord> & current_nodes,
    const std::vector<uint32_t> & previous_node_ids) const;
    double computeTopologicalMatchRate(
    uint32_t matched_count,
    uint32_t current_count) const noexcept;

    uint8_t dominantTopologicalLabel(const std::array<uint32_t, 256> & counts) const noexcept;

    VoxelLabel assignIntegratedLabel(
        const VoxelKey & key,
        uint32_t point_count,
        uint32_t node_count,
        fuzzy_voxel_grid::ManagedVoxel &voxel) const noexcept;

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
    void updatePreviousTopologicalVoxelStates();
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
    const sensor_msgs::msg::PointCloud2 & msg) const;

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
    ais_gng_msgs::msg::TopologicalMap::SharedPtr latest_topological_map_msg_;

    std::unordered_map<VoxelKey, ManagedVoxel, VoxelKeyHash> voxels_;
    std::vector<VoxelView> combined_voxels_;

    std_msgs::msg::Header latest_output_header_;
    bool have_output_header_{false};

    std::unordered_map<VoxelKey, PointCloudVoxelSnapshot, VoxelKeyHash> prev_pointcloud_voxels_;
    std::unordered_map<VoxelKey, TopologicalVoxelSnapshot, VoxelKeyHash> prev_topological_voxels_;
    std::unordered_map<VoxelKey, PreviousTopologicalVoxelState, VoxelKeyHash> prev_topological_voxel_states_;

    std::unordered_map<VoxelKey, PointCloudVoxelDiff, VoxelKeyHash> last_pointcloud_diffs_;
    std::unordered_map<VoxelKey, TopologicalVoxelDiff, VoxelKeyHash> last_topological_diffs_;

    UpdateMode mode_{UpdateMode::LIVE};

    std::unordered_map<VoxelKey, ManagedVoxel, VoxelKeyHash> frozen_voxels_;
    std::vector<VoxelView> frozen_combined_voxels_;

    ais_gng_msgs::msg::TopologicalMap::SharedPtr frozen_topological_map_msg_;

    std_msgs::msg::Header frozen_output_header_;
    bool have_frozen_output_header_{false};
};

}  // namespace fuzzy_voxel_grid