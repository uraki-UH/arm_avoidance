#include "topo_fuzzy_viewer/common/pcl_converter.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

namespace utils {

// =============================================================================
// ROS2 PointCloud2 Conversion
// =============================================================================

PointCloudData convertFromRosMsg(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    PointCloudData result;
    
    // Detect available fields
    bool hasRGB = false;
    bool hasIntensity = false;
    for (const auto& field : msg->fields) {
        if (field.name == "rgb" || field.name == "rgba") {
            hasRGB = true;
        }
        if (field.name == "intensity") {
            hasIntensity = true;
        }
    }
    
    uint32_t expectedPointCount = msg->width * msg->height;
    result.positions.reserve(expectedPointCount * 3);
    if (hasRGB) result.colors.reserve(expectedPointCount * 3);
    if (hasIntensity) result.intensities.reserve(expectedPointCount);
    
    if (hasRGB && hasIntensity) {
        // XYZRGB + Intensity
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);
        for (const auto& point : cloud->points) {
            if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
                result.positions.push_back(point.x);
                result.positions.push_back(point.y);
                result.positions.push_back(point.z);
                result.colors.push_back(point.r);
                result.colors.push_back(point.g);
                result.colors.push_back(point.b);
            }
        }
        // Read intensity separately
        pcl::PointCloud<pcl::PointXYZI>::Ptr icloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *icloud);
        for (const auto& point : icloud->points) {
            if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
                result.intensities.push_back(point.intensity);
            }
        }
    } else if (hasIntensity) {
        // XYZI only
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        for (const auto& point : cloud->points) {
            if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
                result.positions.push_back(point.x);
                result.positions.push_back(point.y);
                result.positions.push_back(point.z);
                result.intensities.push_back(point.intensity);
            }
        }
    } else if (hasRGB) {
        // XYZRGB only
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);
        for (const auto& point : cloud->points) {
            if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
                result.positions.push_back(point.x);
                result.positions.push_back(point.y);
                result.positions.push_back(point.z);
                result.colors.push_back(point.r);
                result.colors.push_back(point.g);
                result.colors.push_back(point.b);
            }
        }
    } else {
        // XYZ only
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        for (const auto& point : cloud->points) {
            if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
                result.positions.push_back(point.x);
                result.positions.push_back(point.y);
                result.positions.push_back(point.z);
            }
        }
    }
    
    result.pointCount = result.positions.size() / 3;
    result.dataMask = 0;
    if (hasRGB) result.dataMask |= pcd_protocol::MASK_RGB;
    if (hasIntensity) result.dataMask |= pcd_protocol::MASK_INTENSITY;
    
    return result;
}

sensor_msgs::msg::PointCloud2 convertToRosMsg(
    const PointCloudData& data, 
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    
    return convertToRosMsg(data.positions, data.colors, frame_id, stamp);
}

sensor_msgs::msg::PointCloud2 convertToRosMsg(
    const std::vector<float>& positions,
    const std::vector<uint8_t>& colors,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    size_t num_points = positions.size() / 3;
    pcl_cloud.width = num_points;
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = false;
    pcl_cloud.points.resize(num_points);
    
    bool hasColors = !colors.empty() && colors.size() == positions.size();
    
    for (size_t i = 0; i < num_points; ++i) {
        pcl_cloud.points[i].x = positions[i * 3];
        pcl_cloud.points[i].y = positions[i * 3 + 1];
        pcl_cloud.points[i].z = positions[i * 3 + 2];
        
        if (hasColors) {
            pcl_cloud.points[i].r = colors[i * 3];
            pcl_cloud.points[i].g = colors[i * 3 + 1];
            pcl_cloud.points[i].b = colors[i * 3 + 2];
        } else {
            pcl_cloud.points[i].r = 255;
            pcl_cloud.points[i].g = 255;
            pcl_cloud.points[i].b = 255;
        }
    }
    
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = stamp;
    
    return cloud_msg;
}

// =============================================================================
// PCL Cloud Conversion
// =============================================================================

PointCloudData convertFromPclCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    PointCloudData result;
    if (!cloud || cloud->empty()) return result;
    
    result.positions.reserve(cloud->size() * 3);
    result.colors.reserve(cloud->size() * 3);
    
    for (const auto& point : cloud->points) {
        if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
            result.positions.push_back(point.x);
            result.positions.push_back(point.y);
            result.positions.push_back(point.z);
            result.colors.push_back(point.r);
            result.colors.push_back(point.g);
            result.colors.push_back(point.b);
        }
    }
    
    result.pointCount = result.positions.size() / 3;
    result.dataMask = pcd_protocol::MASK_RGB;
    return result;
}

PointCloudData convertFromPclCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    PointCloudData result;
    if (!cloud || cloud->empty()) return result;
    
    result.positions.reserve(cloud->size() * 3);
    result.intensities.reserve(cloud->size());
    
    for (const auto& point : cloud->points) {
        if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
            result.positions.push_back(point.x);
            result.positions.push_back(point.y);
            result.positions.push_back(point.z);
            result.intensities.push_back(point.intensity);
        }
    }
    
    result.pointCount = result.positions.size() / 3;
    result.dataMask = pcd_protocol::MASK_INTENSITY;
    return result;
}

PointCloudData convertFromPclCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    PointCloudData result;
    if (!cloud || cloud->empty()) return result;
    
    result.positions.reserve(cloud->size() * 3);
    
    for (const auto& point : cloud->points) {
        if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
            result.positions.push_back(point.x);
            result.positions.push_back(point.y);
            result.positions.push_back(point.z);
        }
    }
    
    result.pointCount = result.positions.size() / 3;
    result.dataMask = 0;
    return result;
}

// =============================================================================
// Protocol Conversion
// =============================================================================

pcd_protocol::PointCloudMessage convertToProtocolMessage(const PointCloudData& data) {
    pcd_protocol::PointCloudMessage msg(data.pointCount, data.dataMask);
    msg.setPositions(data.positions);
    if (data.hasColors()) msg.setColors(data.colors);
    if (data.hasIntensity()) msg.setIntensities(data.intensities);
    return msg;
}

} // namespace utils
