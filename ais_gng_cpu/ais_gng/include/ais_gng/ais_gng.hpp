#pragma once

#include <ais_gng/downsampling.hpp>
#include <ais_gng/visualize_filter.hpp>
#include <ais_gng/cluster_classification.hpp>

#include <fuzzrobo/libgng/api.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <vector>

#include "ais_gng_msgs/msg/topological_map.hpp"
#include "ais_gng_msgs/msg/topological_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace fuzzrobo {

class AiSGNG : public rclcpp::Node {
    rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr topological_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pcl_pub_, human_pcl_pub_, transformed_pcl_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr filter_pub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;

    OnSetParametersCallbackHandle::SharedPtr param_handle_;

    std::string base_frame_id_;

    Downsampling downsampling_;
    VisualizeFilter filter_;
    ClusterClassification cluster_classification_;

    bool initialized_ = false;

   public:
    AiSGNG(const rclcpp::NodeOptions & options);
    ~AiSGNG();

   private:
    rcl_interfaces::msg::SetParametersResult param_cb(const std::vector<rclcpp::Parameter> &params);
    void pcl_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> makeTopologicalMapMsg(
        const TopologicalMap &map,
        const std_msgs::msg::Header &msg);
    LiDAR_Config getBase2LidarFrame(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    std::unique_ptr<sensor_msgs::msg::PointCloud2> makePointCloud2Msg(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
        const float *transformed_pcl,
        const uint32_t transformed_pcl_num,
        const std_msgs::msg::Header &header);
};
}  // namespace fuzzrobo