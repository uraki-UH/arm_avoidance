#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "topo_fuzzy_viewer/protocol/protocol.h"

namespace utils {

/**
 * @brief Unified point cloud data structure for internal representation
 */
struct PointCloudData {
    std::vector<float> positions;      // x, y, z interleaved
    std::vector<uint8_t> colors;       // r, g, b interleaved
    std::vector<float> intensities;    // intensity per point
    size_t pointCount = 0;
    uint8_t dataMask = 0;              // pcd_protocol::MASK_* flags
    
    bool hasColors() const { return (dataMask & pcd_protocol::MASK_RGB) != 0; }
    bool hasIntensity() const { return (dataMask & pcd_protocol::MASK_INTENSITY) != 0; }
    
    void clear() {
        positions.clear();
        colors.clear();
        intensities.clear();
        pointCount = 0;
        dataMask = 0;
    }
};

// =============================================================================
// ROS2 PointCloud2 Conversion
// =============================================================================

/**
 * @brief Convert ROS2 PointCloud2 message to internal format
 * 
 * Automatically detects RGB and intensity fields and extracts them.
 * NaN points are filtered out.
 */
PointCloudData convertFromRosMsg(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

/**
 * @brief Convert internal format to ROS2 PointCloud2 message
 */
sensor_msgs::msg::PointCloud2 convertToRosMsg(
    const PointCloudData& data, 
    const std::string& frame_id,
    const rclcpp::Time& stamp);

/**
 * @brief Convert positions/colors arrays to ROS2 PointCloud2 message
 * 
 * Convenience overload for when PointCloudData struct is not available.
 */
sensor_msgs::msg::PointCloud2 convertToRosMsg(
    const std::vector<float>& positions,
    const std::vector<uint8_t>& colors,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

// =============================================================================
// PCL Cloud Conversion
// =============================================================================

/**
 * @brief Convert PCL PointXYZRGB cloud to internal format
 */
PointCloudData convertFromPclCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

/**
 * @brief Convert PCL PointXYZI cloud to internal format
 */
PointCloudData convertFromPclCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

/**
 * @brief Convert PCL PointXYZ cloud to internal format
 */
PointCloudData convertFromPclCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

// =============================================================================
// Protocol Conversion
// =============================================================================

/**
 * @brief Convert internal format to pcd_protocol::PointCloudMessage
 */
pcd_protocol::PointCloudMessage convertToProtocolMessage(const PointCloudData& data);

} // namespace utils
