#pragma once

#include <ais_gng/plugin/downsampling.hpp>
#include <ais_gng/plugin/visualize_filter.hpp>
#include <ais_gng/plugin/cluster_classification.hpp>

#include <fuzzrobo/libgng/api.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <iomanip>
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

    std::vector<std::string> input_topic_names_;

    // Add Plugin
    Downsampling downsampling_;
    VisualizeFilter filter_;
    ClusterClassification cluster_classification_;

    bool initialized_ = false;
    bool has_topological_map_snapshot_ = false;
    std::unordered_map<uint16_t, ais_gng_msgs::msg::TopologicalNode> last_published_nodes_;

   public:
    AiSGNGComponent(const rclcpp::NodeOptions & options);
    ~AiSGNGComponent();

   private:
    rcl_interfaces::msg::SetParametersResult param_cb(const std::vector<rclcpp::Parameter> &params);
    void process_clouds(const std::vector<PC2::ConstSharedPtr>& msg);
    void semseg_cb(const PC2::SharedPtr msg);
    void publishTopologicalMapUpdate(const ais_gng_msgs::msg::TopologicalMap &map_msg);
    std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> makeTopologicalMapMsg(
        const TopologicalMap &map,
        const std_msgs::msg::Header &msg,
        const float *transformed_pcl = nullptr,
        const uint32_t transformed_pcl_num = 0);
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