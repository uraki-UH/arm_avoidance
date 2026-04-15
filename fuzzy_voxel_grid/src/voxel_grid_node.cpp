#include "fuzzy_voxel_grid/voxel_grid_node.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>

#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace fuzzy_voxel_grid
{

std::size_t VoxelKeyHash::operator()(const VoxelKey & key) const noexcept
{
    std::size_t seed = 0;

    auto combine = [&seed](int32_t value) {
        const std::size_t h = std::hash<int32_t>{}(value);
        seed ^= h + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
    };

    combine(key.ix);
    combine(key.iy);
    combine(key.iz);
    return seed;
}

VoxelGridNode::VoxelGridNode()
: Node("voxel_grid_node")
{
    declareParameters();
    loadParameters();

    voxels_.max_load_factor(0.7f);

    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        params_.marker_array_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    voxel_centers_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        params_.voxel_centers_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.input_pointcloud_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&VoxelGridNode::pointCloudCallback, this, std::placeholders::_1));

    topological_map_sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
        params_.input_topological_map_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&VoxelGridNode::topologicalMapCallback, this, std::placeholders::_1));

    frozen_topological_map_pub_ =
        this->create_publisher<ais_gng_msgs::msg::TopologicalMap>(
            params_.frozen_topological_map_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    filtered_new_points_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            params_.filtered_new_points_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    freeze_service_ = this->create_service<std_srvs::srv::Trigger>(
        "freeze_voxel_state",
        std::bind(&VoxelGridNode::handleFreeze, this, std::placeholders::_1, std::placeholders::_2));

    resume_service_ = this->create_service<std_srvs::srv::Trigger>(
        "resume_voxel_update",
        std::bind(&VoxelGridNode::handleResume, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / params_.update_rate_hz);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&VoxelGridNode::timerCallback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "fuzzy_voxel_grid started. pointcloud=%s topological_map=%s marker_array=%s update_rate=%.3f Hz",
        params_.input_pointcloud_topic.c_str(),
        params_.input_topological_map_topic.c_str(),
        params_.marker_array_topic.c_str(),
        params_.update_rate_hz);
}

void VoxelGridNode::declareParameters()
{
    this->declare_parameter<std::string>("input_pointcloud_topic", "/points_raw");
    this->declare_parameter<std::string>("input_topological_map_topic", "/topological_map");
    this->declare_parameter<std::string>("marker_array_topic", "/voxel_markers");
    this->declare_parameter<std::string>("voxel_centers_topic", "/voxel_centers");
    this->declare_parameter<std::string>("marker_namespace_prefix", "voxel_labels");

    this->declare_parameter<bool>("publish_voxel_centers", true);
    this->declare_parameter<bool>("print_processing_time", true);

    this->declare_parameter<double>("update_rate_hz", 10.0);

    this->declare_parameter<double>("voxel_size_x", 0.20);
    this->declare_parameter<double>("voxel_size_y", 0.20);
    this->declare_parameter<double>("voxel_size_z", 0.20);

    this->declare_parameter<double>("grid_origin_x", 0.0);
    this->declare_parameter<double>("grid_origin_y", 0.0);
    this->declare_parameter<double>("grid_origin_z", 0.0);

    this->declare_parameter<double>("range_min_x", -20.0);
    this->declare_parameter<double>("range_max_x", 20.0);
    this->declare_parameter<double>("range_min_y", -20.0);
    this->declare_parameter<double>("range_max_y", 20.0);
    this->declare_parameter<double>("range_min_z", -2.0);
    this->declare_parameter<double>("range_max_z", 5.0);

    this->declare_parameter<bool>("use_exclusion_box", false);

    this->declare_parameter<double>("exclude_min_x", -1.0);
    this->declare_parameter<double>("exclude_max_x", 1.0);
    this->declare_parameter<double>("exclude_min_y", -1.0);
    this->declare_parameter<double>("exclude_max_y", 1.0);
    this->declare_parameter<double>("exclude_min_z", -1.0);
    this->declare_parameter<double>("exclude_max_z", 1.0);

    this->declare_parameter<double>("normal_color.r", 0.20);
    this->declare_parameter<double>("normal_color.g", 0.80);
    this->declare_parameter<double>("normal_color.b", 1.00);
    this->declare_parameter<double>("normal_color.a", 0.15);

    this->declare_parameter<double>("add_candidate_color.r", 0.10);
    this->declare_parameter<double>("add_candidate_color.g", 1.00);
    this->declare_parameter<double>("add_candidate_color.b", 0.10);
    this->declare_parameter<double>("add_candidate_color.a", 0.25);

    this->declare_parameter<double>("delete_candidate_color.r", 1.00);
    this->declare_parameter<double>("delete_candidate_color.g", 0.15);
    this->declare_parameter<double>("delete_candidate_color.b", 0.15);
    this->declare_parameter<double>("delete_candidate_color.a", 0.25);

    this->declare_parameter<double>("skip_candidate_color.r", 1.00);
    this->declare_parameter<double>("skip_candidate_color.g", 0.90);
    this->declare_parameter<double>("skip_candidate_color.b", 0.10);
    this->declare_parameter<double>("skip_candidate_color.a", 0.18);

    this->declare_parameter<std::string>("frozen_topological_map_topic", "/frozen_topological_map");
    this->declare_parameter<std::string>("filtered_new_points_topic", "/filtered_new_points");

    this->declare_parameter<int>("filter_target_label", 2);
    this->declare_parameter<double>("filter_distance_threshold", 0.30);
}

void VoxelGridNode::loadParameters()
{
    params_.input_pointcloud_topic = this->get_parameter("input_pointcloud_topic").as_string();
    params_.input_topological_map_topic = this->get_parameter("input_topological_map_topic").as_string();
    params_.marker_array_topic = this->get_parameter("marker_array_topic").as_string();
    params_.voxel_centers_topic = this->get_parameter("voxel_centers_topic").as_string();
    params_.marker_namespace_prefix = this->get_parameter("marker_namespace_prefix").as_string();

    params_.publish_voxel_centers = this->get_parameter("publish_voxel_centers").as_bool();
    params_.print_processing_time = this->get_parameter("print_processing_time").as_bool();

    params_.update_rate_hz = this->get_parameter("update_rate_hz").as_double();

    params_.voxel_size_x = this->get_parameter("voxel_size_x").as_double();
    params_.voxel_size_y = this->get_parameter("voxel_size_y").as_double();
    params_.voxel_size_z = this->get_parameter("voxel_size_z").as_double();

    params_.grid_origin_x = this->get_parameter("grid_origin_x").as_double();
    params_.grid_origin_y = this->get_parameter("grid_origin_y").as_double();
    params_.grid_origin_z = this->get_parameter("grid_origin_z").as_double();

    params_.range_min_x = this->get_parameter("range_min_x").as_double();
    params_.range_max_x = this->get_parameter("range_max_x").as_double();
    params_.range_min_y = this->get_parameter("range_min_y").as_double();
    params_.range_max_y = this->get_parameter("range_max_y").as_double();
    params_.range_min_z = this->get_parameter("range_min_z").as_double();
    params_.range_max_z = this->get_parameter("range_max_z").as_double();

    params_.use_exclusion_box = this->get_parameter("use_exclusion_box").as_bool();

    params_.exclude_min_x = this->get_parameter("exclude_min_x").as_double();
    params_.exclude_max_x = this->get_parameter("exclude_max_x").as_double();
    params_.exclude_min_y = this->get_parameter("exclude_min_y").as_double();
    params_.exclude_max_y = this->get_parameter("exclude_max_y").as_double();
    params_.exclude_min_z = this->get_parameter("exclude_min_z").as_double();
    params_.exclude_max_z = this->get_parameter("exclude_max_z").as_double();

    params_.normal_color = loadColor("normal_color");
    params_.add_candidate_color = loadColor("add_candidate_color");
    params_.delete_candidate_color = loadColor("delete_candidate_color");
    params_.skip_candidate_color = loadColor("skip_candidate_color");

    if (params_.update_rate_hz <= 0.0) {
        throw std::runtime_error("update_rate_hz must be > 0");
    }

    if (params_.voxel_size_x <= 0.0 || params_.voxel_size_y <= 0.0 || params_.voxel_size_z <= 0.0) {
        throw std::runtime_error("voxel_size_x/y/z must be > 0");
    }

    if (params_.range_min_x >= params_.range_max_x ||
        params_.range_min_y >= params_.range_max_y ||
        params_.range_min_z >= params_.range_max_z)
    {
        throw std::runtime_error("range_min must be smaller than range_max");
    }

    params_.frozen_topological_map_topic =
        this->get_parameter("frozen_topological_map_topic").as_string();
    params_.filtered_new_points_topic =
        this->get_parameter("filtered_new_points_topic").as_string();

    params_.filter_target_label =
        this->get_parameter("filter_target_label").as_int();
    params_.filter_distance_threshold =
        this->get_parameter("filter_distance_threshold").as_double();

    if (params_.filter_distance_threshold < 0.0) {
        throw std::runtime_error("filter_distance_threshold must be >= 0");
    }
}

ColorRGBA VoxelGridNode::loadColor(const std::string & prefix)
{
    ColorRGBA color{};
    color.r = this->get_parameter(prefix + ".r").as_double();
    color.g = this->get_parameter(prefix + ".g").as_double();
    color.b = this->get_parameter(prefix + ".b").as_double();
    color.a = this->get_parameter(prefix + ".a").as_double();
    return color;
}

bool VoxelGridNode::hasXYZFields(const sensor_msgs::msg::PointCloud2 & msg) const
{
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;

    for (const auto & field : msg.fields) {
        if (field.name == "x") {
            has_x = true;
        } else if (field.name == "y") {
            has_y = true;
        } else if (field.name == "z") {
            has_z = true;
        }
    }

    return has_x && has_y && has_z;
}

bool VoxelGridNode::isInRange(float x, float y, float z) const noexcept
{
    return
        static_cast<double>(x) >= params_.range_min_x &&
        static_cast<double>(x) <= params_.range_max_x &&
        static_cast<double>(y) >= params_.range_min_y &&
        static_cast<double>(y) <= params_.range_max_y &&
        static_cast<double>(z) >= params_.range_min_z &&
        static_cast<double>(z) <= params_.range_max_z;
}

bool VoxelGridNode::isInExcludedBox(float x, float y, float z) const noexcept
{
    if (!params_.use_exclusion_box) {
        return false;
    }

    return
        static_cast<double>(x) >= params_.exclude_min_x &&
        static_cast<double>(x) <= params_.exclude_max_x &&
        static_cast<double>(y) >= params_.exclude_min_y &&
        static_cast<double>(y) <= params_.exclude_max_y &&
        static_cast<double>(z) >= params_.exclude_min_z &&
        static_cast<double>(z) <= params_.exclude_max_z;
}

int32_t VoxelGridNode::computeVoxelIndex(float value, double origin, double voxel_size) const noexcept
{
    return static_cast<int32_t>(std::floor((static_cast<double>(value) - origin) / voxel_size));
}

double VoxelGridNode::computeAverageTopologicalAge(
    const std::vector<TopologicalNodeRecord> & nodes) const noexcept
{
    if (nodes.empty()) {
        return 0.0;
    }

    uint64_t sum = 0;
    for (const auto & node : nodes) {
        sum += static_cast<uint64_t>(node.age);
    }

    return static_cast<double>(sum) / static_cast<double>(nodes.size());
}

VoxelKey VoxelGridNode::pointToVoxelKey(float x, float y, float z) const noexcept
{
    return VoxelKey{
        computeVoxelIndex(x, params_.grid_origin_x, params_.voxel_size_x),
        computeVoxelIndex(y, params_.grid_origin_y, params_.voxel_size_y),
        computeVoxelIndex(z, params_.grid_origin_z, params_.voxel_size_z)
    };
}

geometry_msgs::msg::Point VoxelGridNode::voxelCenter(const VoxelKey & key) const
{
    geometry_msgs::msg::Point p;
    p.x = params_.grid_origin_x + (static_cast<double>(key.ix) + 0.5) * params_.voxel_size_x;
    p.y = params_.grid_origin_y + (static_cast<double>(key.iy) + 0.5) * params_.voxel_size_y;
    p.z = params_.grid_origin_z + (static_cast<double>(key.iz) + 0.5) * params_.voxel_size_z;
    return p;
}

std::unordered_map<VoxelKey, PointCloudVoxelSnapshot, VoxelKeyHash>
VoxelGridNode::buildPointCloudVoxelSnapshot(const sensor_msgs::msg::PointCloud2 & msg) const
{
    std::unordered_map<VoxelKey, PointCloudVoxelSnapshot, VoxelKeyHash> current;

    if (!hasXYZFields(msg)) {
        return current;
    }

    const std::size_t point_count =
        static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);

    current.reserve(point_count);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    for (std::size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        if (!isInRange(x, y, z)) {
            continue;
        }

        const VoxelKey key = pointToVoxelKey(x, y, z);
        current[key].points.push_back(PointSample{x, y, z});
    }

    return current;
}

std::unordered_map<VoxelKey, TopologicalVoxelSnapshot, VoxelKeyHash>
VoxelGridNode::buildTopologicalVoxelSnapshot(const ais_gng_msgs::msg::TopologicalMap & msg) const
{
    std::unordered_map<VoxelKey, TopologicalVoxelSnapshot, VoxelKeyHash> current;
    current.reserve(msg.nodes.size());

    for (std::size_t i = 0; i < msg.nodes.size(); ++i) {
        const auto & node = msg.nodes[i];

        const float x = node.pos.x;
        const float y = node.pos.y;
        const float z = node.pos.z;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        if (!isInRange(x, y, z)) {
            continue;
        }

        const VoxelKey key = pointToVoxelKey(x, y, z);
        auto & voxel = current[key];

        voxel.nodes.push_back(TopologicalNodeRecord{
            static_cast<uint32_t>(i),
            node.label,
            node.age,
            x,
            y,
            z
        });
        voxel.label_counts[static_cast<std::size_t>(node.label)] += 1U;
    }

    return current;
}

std::unordered_map<VoxelKey, PointCloudVoxelDiff, VoxelKeyHash>
VoxelGridNode::diffPointCloudVoxels(
    const std::unordered_map<VoxelKey, PointCloudVoxelSnapshot, VoxelKeyHash> & prev_map,
    const std::unordered_map<VoxelKey, PointCloudVoxelSnapshot, VoxelKeyHash> & curr_map) const
{
    std::unordered_map<VoxelKey, PointCloudVoxelDiff, VoxelKeyHash> diffs;

    for (const auto & entry : prev_map) {
        const auto & key = entry.first;
        const auto prev_count = static_cast<int32_t>(entry.second.points.size());

        auto curr_it = curr_map.find(key);
        if (curr_it == curr_map.end()) {
            diffs[key] = PointCloudVoxelDiff{
                true,
                false,
                -prev_count
            };
        }
    }

    for (const auto & entry : curr_map) {
        const auto & key = entry.first;
        const auto curr_count = static_cast<int32_t>(entry.second.points.size());

        auto prev_it = prev_map.find(key);
        if (prev_it == prev_map.end()) {
            diffs[key] = PointCloudVoxelDiff{
                false,
                true,
                curr_count
            };
        } else {
            const auto prev_count = static_cast<int32_t>(prev_it->second.points.size());
            const int32_t diff = curr_count - prev_count;
            if (diff != 0) {
                diffs[key] = PointCloudVoxelDiff{
                    true,
                    true,
                    diff
                };
            }
        }
    }

    return diffs;
}

std::unordered_map<VoxelKey, TopologicalVoxelDiff, VoxelKeyHash>
VoxelGridNode::diffTopologicalVoxels(
    const std::unordered_map<VoxelKey, TopologicalVoxelSnapshot, VoxelKeyHash> & prev_map,
    const std::unordered_map<VoxelKey, TopologicalVoxelSnapshot, VoxelKeyHash> & curr_map) const
{
    std::unordered_map<VoxelKey, TopologicalVoxelDiff, VoxelKeyHash> diffs;

    for (const auto & entry : curr_map) {
        const auto & key = entry.first;
        const auto curr_count = static_cast<int32_t>(entry.second.nodes.size());

        auto prev_it = prev_map.find(key);
        if (prev_it == prev_map.end()) {
            TopologicalVoxelDiff diff;
            diff.existed_before = false;
            diff.exists_now = true;
            diff.node_count_diff = curr_count;

            for (std::size_t i = 0; i < diff.label_count_diff.size(); ++i) {
                diff.label_count_diff[i] = static_cast<int32_t>(entry.second.label_counts[i]);
            }
            diffs[key] = diff;
        } else {
            TopologicalVoxelDiff diff;
            diff.existed_before = true;
            diff.exists_now = true;
            diff.node_count_diff =
                static_cast<int32_t>(entry.second.nodes.size()) -
                static_cast<int32_t>(prev_it->second.nodes.size());

            bool changed = (diff.node_count_diff != 0);

            for (std::size_t i = 0; i < diff.label_count_diff.size(); ++i) {
                diff.label_count_diff[i] =
                    static_cast<int32_t>(entry.second.label_counts[i]) -
                    static_cast<int32_t>(prev_it->second.label_counts[i]);
                if (diff.label_count_diff[i] != 0) {
                    changed = true;
                }
            }

            if (changed) {
                diffs[key] = diff;
            }
        }
    }

    for (const auto & entry : prev_map) {
        const auto & key = entry.first;
        if (curr_map.find(key) == curr_map.end()) {
            TopologicalVoxelDiff diff;
            diff.existed_before = true;
            diff.exists_now = false;
            diff.node_count_diff = -static_cast<int32_t>(entry.second.nodes.size());

            for (std::size_t i = 0; i < diff.label_count_diff.size(); ++i) {
                diff.label_count_diff[i] = -static_cast<int32_t>(entry.second.label_counts[i]);
            }
            diffs[key] = diff;
        }
    }

    return diffs;
}

uint32_t VoxelGridNode::countMatchedNodeIds(
    const std::vector<TopologicalNodeRecord> & current_nodes,
    const std::vector<uint32_t> & previous_node_ids) const
{
    if (current_nodes.empty() || previous_node_ids.empty()) {
        return 0U;
    }

    std::unordered_set<uint32_t> prev_ids(previous_node_ids.begin(), previous_node_ids.end());

    uint32_t matched_count = 0;
    for (const auto & node : current_nodes) {
        if (prev_ids.find(node.node_id) != prev_ids.end()) {
            ++matched_count;
        }
    }

    return matched_count;
}

double VoxelGridNode::computeTopologicalMatchRate(
    uint32_t matched_count,
    uint32_t current_count) const noexcept
{
    if (current_count == 0U) {
        return 0.0;
    }

    return static_cast<double>(matched_count) / static_cast<double>(current_count);
}

uint8_t VoxelGridNode::dominantTopologicalLabel(const std::array<uint32_t, 256> & counts) const noexcept
{
    uint32_t best_count = 0;
    uint8_t best_label = 0;

    for (std::size_t i = 0; i < counts.size(); ++i) {
        if (counts[i] > best_count) {
            best_count = counts[i];
            best_label = static_cast<uint8_t>(i);
        }
    }

    return best_label;
}

VoxelLabel VoxelGridNode::assignIntegratedLabel(
    const VoxelKey & key,
    uint32_t point_count,
    uint32_t node_count,
    fuzzy_voxel_grid::ManagedVoxel &voxel) const noexcept
{
    // const uint32_t sx = static_cast<uint32_t>(key.ix * 73856093);
    // const uint32_t sy = static_cast<uint32_t>(key.iy * 19349663);
    // const uint32_t sz = static_cast<uint32_t>(key.iz * 83492791);

    // const uint32_t mix = sx ^ sy ^ sz ^
    //                      (point_count * 2654435761u) ^
    //                      (node_count * 2246822519u) ^
    //                      (static_cast<uint32_t>(dominant_topological_label) * 3266489917u);

    // switch (mix & 0x3u) {
    //     case 0u:
    //         return VoxelLabel::Normal;
    //     case 1u:
    //         return VoxelLabel::AddCandidate;
    //     case 2u:
    //         return VoxelLabel::DeleteCandidate;
    //     default:
    //         return VoxelLabel::SkipCandidate;
    // }

    // (void)key;
    // (void)dominant_topological_label;

    // if (point_count == 0 && node_count == 0) {
    //     return VoxelLabel::SkipCandidate;
    // }

    // if (point_count > node_count) {
    //     return VoxelLabel::Normal;
    // }

    if (0 < node_count || 0 < point_count) {
        return VoxelLabel::SkipCandidate;
    }

    // return VoxelLabel::DeleteCandidate;

    if (0.5 < voxel.topological_match_rate)
    {
        return VoxelLabel::SkipCandidate;
    }

    if (voxel.dominant_topological_label == 1 || voxel.dominant_topological_label == 2)
    {
        return VoxelLabel::SkipCandidate;
    }
    // else if (node_count > point_count)
    // {
    //     return VoxelLabel::AddCandidate;
    // }
    // else
    // {
    //     return VoxelLabel::Normal;
    // }

    if (5 < voxel.average_topological_age)
    {
        return VoxelLabel::SkipCandidate;
    }

    return VoxelLabel::AddCandidate;
}

ColorRGBA VoxelGridNode::colorForLabel(VoxelLabel label) const
{
    switch (label) {
        case VoxelLabel::Normal:
            return params_.normal_color;
        case VoxelLabel::AddCandidate:
            return params_.add_candidate_color;
        case VoxelLabel::DeleteCandidate:
            return params_.delete_candidate_color;
        case VoxelLabel::SkipCandidate:
        default:
            return params_.skip_candidate_color;
    }
}

std::string VoxelGridNode::labelName(VoxelLabel label) const
{
    switch (label) {
        case VoxelLabel::Normal:
            return "normal";
        case VoxelLabel::AddCandidate:
            return "add_candidate";
        case VoxelLabel::DeleteCandidate:
            return "delete_candidate";
        case VoxelLabel::SkipCandidate:
        default:
            return "skip_candidate";
    }
}

int32_t VoxelGridNode::markerIdForLabel(VoxelLabel label) const
{
    switch (label) {
        case VoxelLabel::Normal:
            return 0;
        case VoxelLabel::AddCandidate:
            return 1;
        case VoxelLabel::DeleteCandidate:
            return 2;
        case VoxelLabel::SkipCandidate:
        default:
            return 3;
    }
}

std::string VoxelGridNode::namespaceForLabel(VoxelLabel label) const
{
    return params_.marker_namespace_prefix + "/" + labelName(label);
}

visualization_msgs::msg::Marker VoxelGridNode::makeCubeListMarker(
    const std_msgs::msg::Header & header,
    VoxelLabel label,
    const std::vector<geometry_msgs::msg::Point> & points) const
{
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = namespaceForLabel(label);
    marker.id = markerIdForLabel(label);
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = params_.voxel_size_x;
    marker.scale.y = params_.voxel_size_y;
    marker.scale.z = params_.voxel_size_z;

    const auto color = colorForLabel(label);
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = color.a;

    marker.points = points;
    return marker;
}

void VoxelGridNode::rebuildVoxelsFromLatestMessages()
{
    voxels_.clear();

    if (latest_pointcloud_msg_) {
        const auto & msg = *latest_pointcloud_msg_;

        if (!hasXYZFields(msg)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Latest PointCloud2 does not have x/y/z fields.");
        } else {
            const std::size_t point_count =
                static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);

            std::size_t finite_count = 0;
            std::size_t in_range_count = 0;
            std::size_t accepted_count = 0;

            voxels_.reserve(std::max(voxels_.size(), point_count));

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

            for (std::size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
                const float x = *iter_x;
                const float y = *iter_y;
                const float z = *iter_z;

                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                    continue;
                }
                ++finite_count;

                if (!isInRange(x, y, z)) {
                    continue;
                }

                if (isInExcludedBox(x, y, z)) {
                    continue;
                }
                ++in_range_count;

                const VoxelKey key = pointToVoxelKey(x, y, z);
                voxels_[key].pointcloud_points.push_back(PointSample{x, y, z});
                ++accepted_count;
            }

            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     2000,
            //     "PointCloud stats: total=%zu finite=%zu in_range=%zu accepted=%zu voxel_count=%zu frame_id=%s",
            //     point_count,
            //     finite_count,
            //     in_range_count,
            //     accepted_count,
            //     voxels_.size(),
            //     msg.header.frame_id.c_str());
        }
    } else {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "No latest pointcloud message yet.");
    }

    if (latest_topological_map_msg_) {
        const auto & msg = *latest_topological_map_msg_;

        voxels_.reserve(std::max(voxels_.size(), msg.nodes.size()));

        std::size_t accepted_nodes = 0;

        for (std::size_t i = 0; i < msg.nodes.size(); ++i) {
            const auto & node = msg.nodes[i];

            const float x = node.pos.x;
            const float y = node.pos.y;
            const float z = node.pos.z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            if (!isInRange(x, y, z)) {
                continue;
            }

            if (isInExcludedBox(x, y, z)) {
                continue;
            }

            const VoxelKey key = pointToVoxelKey(x, y, z);
            auto & voxel = voxels_[key];

            voxel.topological_nodes.push_back(TopologicalNodeRecord{
                static_cast<uint32_t>(i),
                node.label,
                node.age,
                x,
                y,
                z
            });
            voxel.topological_label_counts[static_cast<std::size_t>(node.label)] += 1U;
        }

        // RCLCPP_INFO_THROTTLE(
        //     this->get_logger(),
        //     *this->get_clock(),
        //     2000,
        //     "TopologicalMap stats: nodes=%zu accepted=%zu",
        //     msg.nodes.size(),
        //     accepted_nodes);
    } else {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "No latest topological_map message yet.");
    }

    if (latest_pointcloud_msg_) {
        latest_output_header_ = latest_pointcloud_msg_->header;
        have_output_header_ = true;
    } else if (latest_topological_map_msg_) {
        latest_output_header_ = latest_topological_map_msg_->header;
        have_output_header_ = true;
    } else {
        have_output_header_ = false;
    }
}

void VoxelGridNode::rebuildViewsFromManagedVoxels()
{
    combined_voxels_.clear();
    combined_voxels_.reserve(voxels_.size());

    for (auto & entry : voxels_) {
        const auto & key = entry.first;
        auto & voxel = entry.second;

        const uint32_t point_count = static_cast<uint32_t>(voxel.pointcloud_points.size());
        const uint32_t node_count = static_cast<uint32_t>(voxel.topological_nodes.size());

        if (node_count > 0U) {
            voxel.dominant_topological_label = dominantTopologicalLabel(voxel.topological_label_counts);
        } else {
            voxel.dominant_topological_label = 0;
        }

        voxel.average_topological_age = computeAverageTopologicalAge(voxel.topological_nodes);

        // TODO そのうち追加
        // voxel.add_degree = computeAddDegree(
        //     point_count,
        //     node_count,
        //     voxel.average_topological_age,
        //     voxel.topological_label_counts);

        // voxel.del_degree = computeDelDegree(
        //     point_count,
        //     node_count,
        //     voxel.average_topological_age,
        //     voxel.topological_label_counts);

        // voxel.skip_degree = computeSkipDegree(
        //     point_count,
        //     node_count,
        //     voxel.average_topological_age,
        //     voxel.topological_label_counts);

        uint32_t matched_count = 0;
        uint32_t previous_count = 0;

        auto prev_it = prev_topological_voxel_states_.find(key);
        if (prev_it != prev_topological_voxel_states_.end()) {
            previous_count = static_cast<uint32_t>(prev_it->second.node_ids.size());
            matched_count = countMatchedNodeIds(voxel.topological_nodes, prev_it->second.node_ids);
        }

        voxel.topological_matched_count = matched_count;
        voxel.previous_topological_node_count = previous_count;
        voxel.current_topological_node_count = node_count;
        voxel.topological_match_rate = computeTopologicalMatchRate(matched_count, node_count);

        voxel.display_label = assignIntegratedLabel(
            key,
            point_count,
            node_count,
            voxel);

        combined_voxels_.push_back(VoxelView{
            key,
            point_count,
            node_count,
            voxel.display_label,
            voxel.dominant_topological_label,
            voxel.add_degree,
            voxel.del_degree,
            voxel.skip_degree
        });
    }
}

void VoxelGridNode::updatePreviousTopologicalVoxelStates()
{
    prev_topological_voxel_states_.clear();
    prev_topological_voxel_states_.reserve(voxels_.size());

    for (const auto & entry : voxels_) {
        const auto & key = entry.first;
        const auto & voxel = entry.second;

        auto & state = prev_topological_voxel_states_[key];
        state.node_ids.reserve(voxel.topological_nodes.size());

        for (const auto & node : voxel.topological_nodes) {
            state.node_ids.push_back(node.node_id);
        }
    }
}

void VoxelGridNode::printDebugVoxelSummary() const
{
    bool found_pointcloud = false;
    bool found_topological = false;

    VoxelKey max_pointcloud_key{0, 0, 0};
    VoxelKey max_topological_key{0, 0, 0};

    std::size_t max_pointcloud_count = 0;
    std::size_t max_topological_count = 0;

    for (const auto & entry : voxels_) {
        const auto & key = entry.first;
        const auto & voxel = entry.second;

        if (voxel.pointcloud_points.size() > max_pointcloud_count) {
            max_pointcloud_count = voxel.pointcloud_points.size();
            max_pointcloud_key = key;
            found_pointcloud = true;
        }

        if (voxel.topological_nodes.size() > max_topological_count) {
            max_topological_count = voxel.topological_nodes.size();
            max_topological_key = key;
            found_topological = true;
        }
    }

    if (found_pointcloud) {
        RCLCPP_INFO(
            this->get_logger(),
            "Max pointcloud voxel: id=(%d, %d, %d), point_count=%zu",
            max_pointcloud_key.ix,
            max_pointcloud_key.iy,
            max_pointcloud_key.iz,
            max_pointcloud_count);
    } else {
        RCLCPP_INFO(this->get_logger(), "Max pointcloud voxel: none");
    }

    if (found_topological) {
        RCLCPP_INFO(
            this->get_logger(),
            "Max topological voxel: id=(%d, %d, %d), node_count=%zu",
            max_topological_key.ix,
            max_topological_key.iy,
            max_topological_key.iz,
            max_topological_count);
    } else {
        RCLCPP_INFO(this->get_logger(), "Max topological voxel: none");
    }
}

void VoxelGridNode::printProcessingTime(std::int64_t elapsed_us) const
{
    if (!params_.print_processing_time) {
        return;
    }

    const double elapsed_ms = static_cast<double>(elapsed_us) / 1000.0;

    RCLCPP_INFO(
        this->get_logger(),
        "[timer] processing_time = %.3f ms",
        elapsed_ms);
}

void VoxelGridNode::publishCombinedMarkerArray()
{
    if (!have_output_header_) {
        return;
    }

    const auto & header = latest_output_header_;

    std::vector<geometry_msgs::msg::Point> normal_points;
    std::vector<geometry_msgs::msg::Point> add_points;
    std::vector<geometry_msgs::msg::Point> delete_points;
    std::vector<geometry_msgs::msg::Point> skip_points;

    normal_points.reserve(combined_voxels_.size());
    add_points.reserve(combined_voxels_.size());
    delete_points.reserve(combined_voxels_.size());
    skip_points.reserve(combined_voxels_.size());

    for (const auto & voxel : combined_voxels_) {
        const auto center = voxelCenter(voxel.key);

        switch (voxel.display_label) {
            case VoxelLabel::Normal:
                normal_points.push_back(center);
                break;
            case VoxelLabel::AddCandidate:
                add_points.push_back(center);
                break;
            case VoxelLabel::DeleteCandidate:
                delete_points.push_back(center);
                break;
            case VoxelLabel::SkipCandidate:
            default:
                skip_points.push_back(center);
                break;
        }
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.reserve(4);

    array.markers.push_back(makeCubeListMarker(header, VoxelLabel::Normal, normal_points));
    array.markers.push_back(makeCubeListMarker(header, VoxelLabel::AddCandidate, add_points));
    array.markers.push_back(makeCubeListMarker(header, VoxelLabel::DeleteCandidate, delete_points));
    array.markers.push_back(makeCubeListMarker(header, VoxelLabel::SkipCandidate, skip_points));

    marker_array_pub_->publish(array);
}

void VoxelGridNode::publishVoxelCenters(
    const std_msgs::msg::Header & header,
    const std::vector<VoxelView> & voxels)
{
    if (!params_.publish_voxel_centers) {
        return;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = header;

    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2Fields(
        6,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "display_label", 1, sensor_msgs::msg::PointField::UINT8,
        "point_count", 1, sensor_msgs::msg::PointField::UINT32,
        "node_count", 1, sensor_msgs::msg::PointField::UINT32);
    modifier.resize(voxels.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(out, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_display_label(out, "display_label");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_point_count(out, "point_count");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_node_count(out, "node_count");

    for (const auto & voxel : voxels) {
        const auto center = voxelCenter(voxel.key);

        *iter_x = static_cast<float>(center.x);
        *iter_y = static_cast<float>(center.y);
        *iter_z = static_cast<float>(center.z);
        *iter_display_label = static_cast<uint8_t>(voxel.display_label);
        *iter_point_count = voxel.point_count;
        *iter_node_count = voxel.node_count;

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_display_label;
        ++iter_point_count;
        ++iter_node_count;
    }

    out.is_dense = true;
    voxel_centers_pub_->publish(out);
}

void VoxelGridNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    latest_pointcloud_msg_ = msg;

    if (mode_ == UpdateMode::FROZEN) {
        auto filtered = filterIncomingPointCloud(*msg);
        filtered_new_points_pub_->publish(filtered);
    }
    else
    {
        auto current_voxels = buildPointCloudVoxelSnapshot(*msg);
        last_pointcloud_diffs_ = diffPointCloudVoxels(prev_pointcloud_voxels_, current_voxels);
        prev_pointcloud_voxels_ = std::move(current_voxels);
    }

    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "PointCloud diff: changed_voxels=%zu",
    //     last_pointcloud_diffs_.size());
}

void VoxelGridNode::topologicalMapCallback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    latest_topological_map_msg_ = msg;

    auto current_voxels = buildTopologicalVoxelSnapshot(*msg);
    last_topological_diffs_ = diffTopologicalVoxels(prev_topological_voxels_, current_voxels);
    prev_topological_voxels_ = std::move(current_voxels);

    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "TopologicalMap diff: changed_voxels=%zu",
    //     last_topological_diffs_.size());
}

void VoxelGridNode::timerCallback()
{
    const auto start_time = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(data_mutex_);

    if (mode_ == UpdateMode::LIVE) {
        rebuildVoxelsFromLatestMessages();
        rebuildViewsFromManagedVoxels();
        publishCombinedMarkerArray();

        if (have_output_header_) {
            publishVoxelCenters(latest_output_header_, combined_voxels_);
        }
    } else {
        publishFrozenMarkerArray();

        if (have_frozen_output_header_) {
            publishVoxelCenters(frozen_output_header_, frozen_combined_voxels_);
        }

        publishFrozenTopologicalMap();
    }

    printDebugVoxelSummary();

    const auto elapsed_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start_time).count();
    printProcessingTime(elapsed_us);
}

void VoxelGridNode::handleFreeze(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (mode_ == UpdateMode::FROZEN) {
        response->success = true;
        response->message = "Already frozen.";
        return;
    }

    frozen_voxels_ = voxels_;
    frozen_combined_voxels_ = combined_voxels_;
    frozen_topological_map_msg_ = latest_topological_map_msg_;
    frozen_output_header_ = latest_output_header_;
    have_frozen_output_header_ = have_output_header_;
    mode_ = UpdateMode::FROZEN;

    response->success = true;
    response->message = "Voxel state frozen.";
}

void VoxelGridNode::handleResume(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    mode_ = UpdateMode::LIVE;
    response->success = true;
    response->message = "Voxel update resumed.";
}

void VoxelGridNode::publishFrozenMarkerArray()
{
    if (!have_frozen_output_header_) {
        return;
    }

    const auto & header = frozen_output_header_;

    std::vector<geometry_msgs::msg::Point> normal_points;
    std::vector<geometry_msgs::msg::Point> add_points;
    std::vector<geometry_msgs::msg::Point> remove_points;
    std::vector<geometry_msgs::msg::Point> omit_points;

    normal_points.reserve(frozen_combined_voxels_.size());
    add_points.reserve(frozen_combined_voxels_.size());
    remove_points.reserve(frozen_combined_voxels_.size());
    omit_points.reserve(frozen_combined_voxels_.size());

    for (const auto & voxel : frozen_combined_voxels_) {
        const auto center = voxelCenter(voxel.key);

        switch (voxel.display_label) {
            case VoxelLabel::Normal:
                normal_points.push_back(center);
                break;
            case VoxelLabel::AddCandidate:
                add_points.push_back(center);
                break;
            case VoxelLabel::SkipCandidate:
                remove_points.push_back(center);
                break;
            case VoxelLabel::DeleteCandidate:
            default:
                omit_points.push_back(center);
                break;
        }
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.reserve(4);

    array.markers.push_back(makeCubeListMarker(header, VoxelLabel::Normal, normal_points));
    array.markers.push_back(makeCubeListMarker(header, VoxelLabel::AddCandidate, add_points));
    array.markers.push_back(makeCubeListMarker(header, VoxelLabel::SkipCandidate, remove_points));
    array.markers.push_back(makeCubeListMarker(header, VoxelLabel::DeleteCandidate, omit_points));

    marker_array_pub_->publish(array);
}

void VoxelGridNode::publishFrozenTopologicalMap()
{
    if (!frozen_topological_map_msg_) {
        return;
    }
    frozen_topological_map_pub_->publish(*frozen_topological_map_msg_);
}

// bool VoxelGridNode::shouldRemovePointInFrozenMode(float x, float y, float z) const
// {
//     if (mode_ != UpdateMode::FROZEN) {
//         return false;
//     }

//     const VoxelKey key = pointToVoxelKey(x, y, z);

//     auto it = frozen_voxels_.find(key);
//     if (it == frozen_voxels_.end()) {
//         return false;
//     }

//     const auto & voxel = it->second;
//     if (static_cast<int>(voxel.display_label) != params_.filter_target_label) {
//         return false;
//     }

//     const double th2 =
//         params_.filter_distance_threshold * params_.filter_distance_threshold;

//     for (const auto & node : voxel.topological_nodes) {
//         const double dx = static_cast<double>(x) - static_cast<double>(node.x);
//         const double dy = static_cast<double>(y) - static_cast<double>(node.y);
//         const double dz = static_cast<double>(z) - static_cast<double>(node.z);
//         const double d2 = dx * dx + dy * dy + dz * dz;

//         if (d2 <= th2) {
//             return true;
//         }
//     }

//     return false;
// }

bool VoxelGridNode::shouldRemovePointInFrozenMode(float x, float y, float z) const
{
    if (mode_ != UpdateMode::FROZEN) {
        return false;
    }

    const VoxelKey center_key = pointToVoxelKey(x, y, z);

    // 少なくとも隣接ボクセルまでは見る。
    // 閾値がボクセルサイズより大きい場合は、その分だけ探索半径を広げる。
    // const int search_rx = std::max(
    //     1,
    //     static_cast<int>(std::ceil(params_.filter_distance_threshold / params_.voxel_size_x)));
    // const int search_ry = std::max(
    //     1,
    //     static_cast<int>(std::ceil(params_.filter_distance_threshold / params_.voxel_size_y)));
    // const int search_rz = std::max(
    //     1,
    //     static_cast<int>(std::ceil(params_.filter_distance_threshold / params_.voxel_size_z)));

    const int search_rx = 1;
    const int search_ry = 1;
    const int search_rz = 1;

    const double th2 =
        params_.filter_distance_threshold * params_.filter_distance_threshold;

    for (int dz = -search_rz; dz <= search_rz; ++dz) {
        for (int dy = -search_ry; dy <= search_ry; ++dy) {
            for (int dx = -search_rx; dx <= search_rx; ++dx) {
                const VoxelKey neighbor_key{
                    center_key.ix + dx,
                    center_key.iy + dy,
                    center_key.iz + dz
                };

                auto it = frozen_voxels_.find(neighbor_key);
                if (it == frozen_voxels_.end()) {
                    continue;
                }

                const auto & voxel = it->second;

                // 指定ラベルのボクセルだけ対象
                if (static_cast<int>(voxel.display_label) != params_.filter_target_label) {
                    continue;
                }

                for (const auto & node : voxel.topological_nodes) {
                    const double ddx = static_cast<double>(x) - static_cast<double>(node.x);
                    const double ddy = static_cast<double>(y) - static_cast<double>(node.y);
                    const double ddz = static_cast<double>(z) - static_cast<double>(node.z);
                    const double d2 = ddx * ddx + ddy * ddy + ddz * ddz;

                    if (d2 <= th2) {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

sensor_msgs::msg::PointCloud2 VoxelGridNode::filterIncomingPointCloud(
    const sensor_msgs::msg::PointCloud2 & msg) const
{
    sensor_msgs::msg::PointCloud2 out = msg;
    out.data.clear();
    out.data.reserve(msg.data.size());
    out.width = 0;
    out.height = 1;

    if (!hasXYZFields(msg)) {
        return out;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    const std::size_t point_count =
        static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);

    for (std::size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;

        bool remove = false;
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && isInRange(x, y, z)) {
            remove = shouldRemovePointInFrozenMode(x, y, z);
        }

        if (!remove) {
            const std::size_t offset = i * msg.point_step;
            out.data.insert(
                out.data.end(),
                msg.data.begin() + static_cast<std::ptrdiff_t>(offset),
                msg.data.begin() + static_cast<std::ptrdiff_t>(offset + msg.point_step));
            ++out.width;
        }
    }

    out.row_step = out.width * out.point_step;
    return out;
}

}  // namespace fuzzy_voxel_grid