#pragma once

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <vector>

namespace fuzzrobo::handle_label
{

std::vector<uint8_t> extractSemanticLabels(
    const sensor_msgs::msg::PointCloud2 &msg,
    uint32_t handle_label_value);

void applySemanticLabelsToMap(
    ais_gng_msgs::msg::TopologicalMap &map_msg,
    const float *transformed_pcl,
    uint32_t transformed_pcl_num,
    const std::vector<uint8_t> &semantic_labels,
    double handle_ratio_threshold);

}  // namespace fuzzrobo::handle_label
