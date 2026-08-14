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

struct SequentialNodeStats {
    double count = 0.0;
    std::array<double, 3> mean{0.0, 0.0, 0.0};
    std::array<double, 9> m2{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

class AiSGNGComponent : public rclcpp::Node {
    using PC2 = sensor_msgs::msg::PointCloud2;

    rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr topological_map_pub_;
    rclcpp::Publisher<PC2>::SharedPtr transformed_pcl_pub_;

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
    double node_eta_s1_{};
    double node_eta_s2_{};
    double node_cov_decay_k_{};
    bool node_covariance_enabled_{};
    std::unordered_map<uint16_t, SequentialNodeStats> winner_point_stats_;
    int64_t performance_log_interval_ms_{5000};
    std::chrono::steady_clock::time_point last_process_start_{};
    bool has_last_process_start_{false};

    // Add Plugin
    Downsampling downsampling_;
    VisualizeFilter filter_;
    ClusterClassification cluster_classification_;

    bool initialized_ = false;
    std::unordered_map<uint16_t, ais_gng_msgs::msg::TopologicalNode> last_published_nodes_;

   public:
    AiSGNGComponent(const rclcpp::NodeOptions & options);
    ~AiSGNGComponent();

   private:
    rcl_interfaces::msg::SetParametersResult param_cb(const std::vector<rclcpp::Parameter> &params);
    void process_clouds(const std::vector<PC2::ConstSharedPtr>& msg);
    void semseg_cb(const PC2::SharedPtr msg);
    void publishTopologicalMapUpdate(const ais_gng_msgs::msg::TopologicalMap &map_msg);
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
