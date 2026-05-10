#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <voxel_msgs/msg/voxel.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <unordered_set>
#include <mutex>
#include <string>
#include "safety_engine/indexing/index_voxel_grid.hpp"

namespace robot_sim::self_recognition {

class SelfRecognitionFilterNode : public rclcpp::Node {
public:
    explicit SelfRecognitionFilterNode(const rclcpp::NodeOptions & options);

private:
    void pcl_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    // 軽量フィルタに必要なメンバのみ
    rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr mask_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

    std::unordered_set<long> current_mask_vids_;
    std::mutex mask_mutex_;
    ::GNG::Analysis::IndexVoxelGrid grid_{0.0};
};

} // namespace robot_sim::self_recognition
