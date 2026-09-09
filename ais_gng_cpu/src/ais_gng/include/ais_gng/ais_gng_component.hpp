#pragma once

#include <ais_gng/plugin/downsampling.hpp>
#include <ais_gng/plugin/visualize_filter.hpp>
#include <ais_gng/plugin/cluster_classification.hpp>
#include <ais_gng/point_selection.hpp>

#include <fuzzrobo/libgng/api.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_map>
#include <array>
#include <chrono>
#include <string>
#include <vector>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>

#include "ais_gng_msgs/msg/topological_map.hpp"
#include "ais_gng_msgs/msg/topological_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"

#if defined(AIS_GNG_BACKEND_CPU)
#include "ais_gng/node_support.hpp"
#include "ais_gng/observation_pixels.hpp"
#include "ais_gng/boundary_evidence.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ais_gng/topological_plane/nonplane_component_extractor.hpp"
#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include "ais_gng_msgs/msg/plane_cluster_array.hpp"
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace fuzzrobo {

class AiSGNGComponent : public rclcpp::Node {
    using PC2 = sensor_msgs::msg::PointCloud2;

    rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr topological_map_pub_;
    rclcpp::Publisher<PC2>::SharedPtr transformed_pcl_pub_;
#if defined(AIS_GNG_BACKEND_CPU)
    uint32_t max_boundary_neighbors_{4};
    bool enable_boundary_candidates_{false};
    bool enable_boundary_evidence_{true};
    boundary_evidence::classifier boundary_classifier_;
    std::vector<double> boundary_lidar_angles_deg_;
    void classify_boundary_evidence(ais_gng_msgs::msg::TopologicalMap &map, const float *points, uint32_t point_num);
    bool direct_plane_cluster_enabled_{false};
    std::unique_ptr<topological_plane::incremental::Clusterizer> direct_plane_clusterizer_;
    rclcpp::Publisher<ais_gng_msgs::msg::PlaneClusterArray>::SharedPtr direct_plane_cluster_pub_;
    bool direct_nonplane_component_enabled_{false};
    topological_plane::nonplane::extractor_options direct_nonplane_component_options_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr direct_nonplane_component_pub_;
    node_support::options node_support_options_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr node_support_pub_;
    void publish_node_support(const TopologicalMap &map, const std_msgs::msg::Header &header);
    bool enable_observation_support_{false};
    bool has_observation_origin_{false};
    bool has_observation_cloud_transform_{false};
    std::vector<double> observation_origin_;
    std::string observation_origin_frame_;
    std::string observation_sensor_frame_;
    std::string observation_camera_info_topic_;
    bool enable_observation_organized_{false};
    bool has_observation_rotation_{false};
    std::vector<double> observation_camera_rotation_;
    std::array<double, 9> observation_rotation_{};
    std::deque<sensor_msgs::msg::CameraInfo::ConstSharedPtr> observation_camera_infos_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr observation_camera_info_sub_;
    observation_pixels::angle_table observation_angle_table_;
    void prepare_observation_pixels(const PC2::ConstSharedPtr &msg, const std::vector<uint32_t> *selected_ids,
        uint32_t point_num, gng_observation_input &input);
    std::unique_ptr<tf2_ros::Buffer> observation_transform_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> observation_transform_listener_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr observation_support_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr observation_lookup_pub_;
    void prepare_observation_origin(const PC2::ConstSharedPtr &msg, const LiDAR_Config &config,
        bool has_single_sensor, gng_observation_input &input);
    void publish_observation_support(const TopologicalMap &map, const std_msgs::msg::Header &header);
#endif

    rclcpp::Subscription<PC2>::SharedPtr pcl_sub_;
    std::vector<std::shared_ptr<message_filters::Subscriber<PC2>>> pcl_subs_;
    rclcpp::Subscription<PC2>::SharedPtr semseg_sub_;
    OnSetParametersCallbackHandle::SharedPtr param_handle_;

    std::shared_ptr<void> sync_keeper_;

    std::string base_frame_id_;
    bool local_coordinates_{false};
    uint32_t input_point_cloud_num_{20000};
    PointSamplingMode input_sampling_mode_{PointSamplingMode::Head};
    PC2::SharedPtr sampled_cloud_buffer_{std::make_shared<PC2>()};
    std::vector<uint32_t> sampled_point_indices_;
    uint32_t sampled_source_point_count_{};
    uint32_t sampled_max_point_count_{};
    PointSamplingMode sampled_mode_{PointSamplingMode::Head};
    bool sampled_indices_valid_{false};
    std::vector<uint8_t> semantic_label_buffer_;
    std::vector<uint32_t> source_point_index_buffer_;

    std::vector<std::string> input_topic_names_;
    uint32_t semantic_handle_label_value_{};
    double semantic_handle_ratio_threshold_{};
    std::size_t semantic_handle_history_size_{};
    std::vector<std::deque<uint8_t>> semantic_label_history_;
    bool node_covariance_enabled_{};
    uint16_t node_covariance_winner_rank_max_{1};
    int64_t performance_log_interval_ms_{0};
    std::chrono::steady_clock::time_point last_process_start_{};
    bool has_last_process_start_{false};

    // Add Plugin
    Downsampling downsampling_;
    VisualizeFilter filter_;
    ClusterClassification cluster_classification_;

    bool initialized_ = false;
   public:
    AiSGNGComponent(const rclcpp::NodeOptions & options);
    ~AiSGNGComponent();

   private:
    rcl_interfaces::msg::SetParametersResult param_cb(const std::vector<rclcpp::Parameter> &params);
    void process_clouds(const std::vector<PC2::ConstSharedPtr>& msg);
    void semseg_cb(const PC2::SharedPtr msg);
    void updateSemanticLabelHistory(ais_gng_msgs::msg::TopologicalMap &map_msg);
    std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> makeTopologicalMapMsg(
        const TopologicalMap &map,
        const std_msgs::msg::Header &msg,
        const std::vector<uint8_t> *semantic_labels = nullptr,
        const std::vector<uint32_t> *source_point_indices = nullptr);
    LiDAR_Config getBase2LidarFrame(const PC2::ConstSharedPtr msg);
    std::unique_ptr<PC2> mixPointCloud2Msg(const std_msgs::msg::Header &header,
        const PC2::SharedPtr &msg,
        const float *transformed_pcl,
        const uint32_t transformed_pcl_num);
    std::unique_ptr<PC2> makePointCloud2Msg(
        const std_msgs::msg::Header &header,
        const float *transformed_pcl,
        const uint32_t transformed_pcl_num);
    std::unique_ptr<PC2> makePointCloud2MsgFromClustedNode(
        const std_msgs::msg::Header &header,
        const ais_gng_msgs::msg::TopologicalMap &map);
};
}  // namespace fuzzrobo
