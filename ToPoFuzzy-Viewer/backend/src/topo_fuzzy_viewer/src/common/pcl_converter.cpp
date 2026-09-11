#include "topo_fuzzy_viewer/common/pcl_converter.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_sampling/stratified.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>

namespace utils {

namespace {

inline bool isFinitePoint(float x, float y, float z) {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

} // namespace

// =============================================================================
// ROS2 PointCloud2 Conversion
// =============================================================================

PointCloudData convertFromRosMsg(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    size_t max_points) {
    PointCloudData result;
    
    // 上限超過時だけのランダム抽出。有効点が枠内なら入力順の維持。
    static std::atomic<uint32_t> sampling_frame{0};
    std::vector<uint32_t> selected_indices;
    try {
        selected_indices = pointcloud_sampling::select_stratified(*msg, 0, 0);
        if (max_points != 0 && selected_indices.size() > max_points) {
            std::mt19937 random(sampling_frame.fetch_add(1, std::memory_order_relaxed));
            std::shuffle(selected_indices.begin(), selected_indices.end(), random);
            selected_indices.resize(max_points);
        }
    } catch (const std::invalid_argument &) {
        return result;
    }
    const auto for_each_selected = [&](auto &&callback) {
        for (const uint32_t idx : selected_indices) callback(idx);
    };

    // 利用可能な属性フィールドの検出。
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
    
    const size_t expectedPointCount = selected_indices.size();
    result.positions.reserve(expectedPointCount * 3);
    if (hasRGB) result.colors.reserve(expectedPointCount * 3);
    if (hasIntensity) result.intensities.reserve(expectedPointCount);
    
    if (hasRGB && hasIntensity) {
        // XYZRGB + Intensity
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);
        for_each_selected([&](size_t index) {
            const auto& point = cloud->points[index];
            if (isFinitePoint(point.x, point.y, point.z)) {
                result.positions.push_back(point.x);
                result.positions.push_back(point.y);
                result.positions.push_back(point.z);
                result.colors.push_back(point.r);
                result.colors.push_back(point.g);
                result.colors.push_back(point.b);
            }
        });
        // Read intensity separately
        pcl::PointCloud<pcl::PointXYZI>::Ptr icloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *icloud);
        for_each_selected([&](size_t index) {
            const auto& point = icloud->points[index];
            if (isFinitePoint(point.x, point.y, point.z)) {
                result.intensities.push_back(point.intensity);
            }
        });
    } else if (hasIntensity) {
        // XYZI only
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        for_each_selected([&](size_t index) {
            const auto& point = cloud->points[index];
            if (isFinitePoint(point.x, point.y, point.z)) {
                result.positions.push_back(point.x);
                result.positions.push_back(point.y);
                result.positions.push_back(point.z);
                result.intensities.push_back(point.intensity);
            }
        });
    } else if (hasRGB) {
        // XYZRGB only
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);
        for_each_selected([&](size_t index) {
            const auto& point = cloud->points[index];
            if (isFinitePoint(point.x, point.y, point.z)) {
                result.positions.push_back(point.x);
                result.positions.push_back(point.y);
                result.positions.push_back(point.z);
                result.colors.push_back(point.r);
                result.colors.push_back(point.g);
                result.colors.push_back(point.b);
            }
        });
    } else {
        // XYZ only
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        for_each_selected([&](size_t index) {
            const auto& point = cloud->points[index];
            if (isFinitePoint(point.x, point.y, point.z)) {
                result.positions.push_back(point.x);
                result.positions.push_back(point.y);
                result.positions.push_back(point.z);
            }
        });
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
        if (isFinitePoint(point.x, point.y, point.z)) {
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
        if (isFinitePoint(point.x, point.y, point.z)) {
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
        if (isFinitePoint(point.x, point.y, point.z)) {
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
