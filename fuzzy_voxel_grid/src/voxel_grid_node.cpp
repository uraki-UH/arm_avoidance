#include "fuzzy_voxel_grid/voxel_grid_node.hpp"
#include <fuzzrobo/libgng/voxel_framework.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>

#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace fuzzy_voxel_grid
{

VoxelGridNode::VoxelGridNode(const rclcpp::NodeOptions &options)
: Node("voxel_grid_node", options)
{
    declareParameters();
    loadParameters();

    point_spec_.size = {params_.voxel_size_x, params_.voxel_size_y, params_.voxel_size_z};
    point_spec_.origin = {params_.grid_origin_x, params_.grid_origin_y, params_.grid_origin_z};
    point_spec_.min_corner = {params_.range_min_x, params_.range_min_y, params_.range_min_z};
    point_spec_.max_corner = {params_.range_max_x, params_.range_max_y, params_.range_max_z};
    point_spec_.exclude_min = {params_.exclude_min_x, params_.exclude_min_y, params_.exclude_min_z};
    point_spec_.exclude_max = {params_.exclude_max_x, params_.exclude_max_y, params_.exclude_max_z};
    point_spec_.enable_exclusion = params_.use_exclusion_box;
    const auto max_dense_voxel_num = declare_parameter<int64_t>("max_dense_voxel_num", 8000000);
    if (max_dense_voxel_num < 0) {throw std::invalid_argument("max_dense_voxel_numの負値");}
    point_spec_.max_dense_voxel_num = static_cast<std::size_t>(max_dense_voxel_num);

    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        params_.marker_array_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    voxel_centers_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        params_.voxel_centers_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    const auto shared_store = declare_parameter<std::string>("shared_point_store", "");
    max_tmap_age_sec_ = declare_parameter<double>("max_tmap_age_sec", 1.0);
    if (!std::isfinite(max_tmap_age_sec_) || max_tmap_age_sec_ < 0) {
        throw std::invalid_argument("max_tmap_age_secには有限・非負値が必要");
    }
    if (shared_store.empty()) {
        standalone_points_ = std::make_shared<voxel_idx::point_cell_counts>(point_spec_);
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            params_.input_pointcloud_topic, rclcpp::SensorDataQoS(),
            std::bind(&VoxelGridNode::pointCloudCallback, this, std::placeholders::_1));
    } else {
        shared_points_ = voxel_idx::shared_point_frames(shared_store);
        point_query_ = shared_points_->cell_query(point_spec_);
        RCLCPP_INFO(get_logger(), "共有点群を参照: store=%s、点群の独立購読なし", shared_store.c_str());
    }

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

VoxelKey VoxelGridNode::pointToVoxelKey(float x, float y, float z) const
{
    return point_spec_.key(Eigen::Vector3d(x, y, z));
}

geometry_msgs::msg::Point VoxelGridNode::voxelCenter(const VoxelKey & key) const
{
    geometry_msgs::msg::Point p;
    p.x = params_.grid_origin_x + (static_cast<double>(key.x) + 0.5) * params_.voxel_size_x;
    p.y = params_.grid_origin_y + (static_cast<double>(key.y) + 0.5) * params_.voxel_size_y;
    p.z = params_.grid_origin_z + (static_cast<double>(key.z) + 0.5) * params_.voxel_size_z;
    return p;
}

void VoxelGridNode::update_point_counts(const sensor_msgs::msg::PointCloud2 &msg)
{
    point_cells_.reset();
    standalone_points_->begin_frame();
    if (hasXYZFields(msg)) {
        sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
        const std::size_t num_points = static_cast<std::size_t>(msg.width) * msg.height;
        for (std::size_t idx = 0; idx < num_points; ++idx, ++x, ++y, ++z) {
            standalone_points_->add_point(Eigen::Vector3f(*x, *y, *z));
        }
    }
    point_cells_ = standalone_points_;
}

VoxelLabel VoxelGridNode::assignIntegratedLabel(uint32_t point_count, uint32_t node_count) const noexcept
{
    // 現行判定に必要な件数だけの属性。評価拡張の差込口は既存pipeline。
    struct input_view {uint32_t point_count, node_count;};
    struct label_policy {
        VoxelLabel baseline(const input_view &input) const {
            return input.node_count || input.point_count ? VoxelLabel::SkipCandidate : VoxelLabel::AddCandidate;
        }
        const input_view &collect(const input_view &input) const {return input;}
        VoxelLabel evaluate(const input_view &input) const {return baseline(input);}
    };
    label_policy policy;
    fuzzrobo::voxel_framework::pipeline<fuzzrobo::voxel_framework::configured_features<>, label_policy> pipeline;
    return pipeline.evaluate(input_view{point_count, node_count}, policy);
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
    combined_voxels_.clear();
    if (point_cells_) {
        combined_voxels_.reserve(point_cells_->cells().size());
        for (const auto &cell : point_cells_->cells()) {
            combined_voxels_.push_back({cell.key, cell.num_points, 0U, VoxelLabel::Normal});
        }
    }
    bool has_compatible_map = bool(latest_topological_map_msg_);
    if (has_compatible_map && shared_points_) {
        const auto &header = latest_topological_map_msg_->header;
        const double age_sec = (rclcpp::Time(point_header_.stamp) - rclcpp::Time(header.stamp)).seconds();
        has_compatible_map = has_point_header_ && header.frame_id == point_header_.frame_id &&
            age_sec >= 0 && age_sec <= max_tmap_age_sec_;
    }
    active_topological_map_msg_ = has_compatible_map ? latest_topological_map_msg_ : nullptr;
    if (active_topological_map_msg_) {
        // 点群由来セルは共有索引を参照。Tmapだけのセルに限った補助表。
        std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> node_only_cells;
        for (const auto &node : active_topological_map_msg_->nodes) {
            const Eigen::Vector3f point(node.pos.x, node.pos.y, node.pos.z);
            if (!point_spec_.contains(point)) {continue;}
            const auto key = pointToVoxelKey(point.x(), point.y(), point.z());
            auto idx = point_cells_ ? point_cells_->find(key) : voxel_idx::point_cell_counts::no_cell;
            if (idx == voxel_idx::point_cell_counts::no_cell) {
                const auto inserted = node_only_cells.emplace(key, combined_voxels_.size());
                idx = inserted.first->second;
                if (inserted.second) {combined_voxels_.push_back({key, 0U, 0U, VoxelLabel::Normal});}
            }
            ++combined_voxels_[idx].node_count;
        }
    } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Tmap未到着、または共有点群とのframe・時刻条件の不一致");
    }
    if (has_point_header_) {
        latest_output_header_ = point_header_;
        have_output_header_ = true;
    } else if (latest_topological_map_msg_) {
        latest_output_header_ = latest_topological_map_msg_->header;
        have_output_header_ = true;
    } else {have_output_header_ = false;}
}

void VoxelGridNode::rebuildViewsFromManagedVoxels()
{
    for (auto &voxel : combined_voxels_) {
        voxel.display_label = assignIntegratedLabel(voxel.point_count, voxel.node_count);
    }
    if (have_output_header_) {
        live_markers_ = make_marker_cache(latest_output_header_, combined_voxels_, false);
    }
}

visualization_msgs::msg::MarkerArray VoxelGridNode::make_marker_cache(
    const std_msgs::msg::Header &header, const std::vector<VoxelView> &voxels, bool is_frozen) const
{
    visualization_msgs::msg::MarkerArray result;
    const std::array<VoxelLabel, 4> labels{VoxelLabel::Normal, VoxelLabel::AddCandidate,
        is_frozen ? VoxelLabel::SkipCandidate : VoxelLabel::DeleteCandidate,
        is_frozen ? VoxelLabel::DeleteCandidate : VoxelLabel::SkipCandidate};
    for (const auto label : labels) {result.markers.push_back(makeCubeListMarker(header, label, {}));}
    // 中間4配列と全セル4倍の予約領域なし。送信メッセージへ直接格納。
    for (const auto &voxel : voxels) {
        auto idx = static_cast<std::size_t>(voxel.display_label);
        if (is_frozen && idx >= 2) {idx = 5-idx;}
        result.markers[idx].points.push_back(voxelCenter(voxel.key));
    }
    return result;
}

void VoxelGridNode::printDebugVoxelSummary() const
{
    // 通常ログではデバッグ集計の全セル走査なし。
    if (!rcutils_logging_logger_is_enabled_for(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG)) {return;}
    uint32_t max_points = 0, max_nodes = 0;
    for (const auto &voxel : combined_voxels_) {
        max_points = std::max(max_points, voxel.point_count);
        max_nodes = std::max(max_nodes, voxel.node_count);
    }
    RCLCPP_DEBUG(get_logger(), "cells=%zu max_points=%u max_nodes=%u",
        combined_voxels_.size(), max_points, max_nodes);
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
    if (have_output_header_) {marker_array_pub_->publish(live_markers_);}
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

void VoxelGridNode::update_shared_points()
{
    const auto frame = shared_points_->latest();
    if (!frame) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "共有点群の待機中。同一プロセスのwriterとshared_point_store名の確認が必要");
        return;
    }
    if (frame->revision == shared_revision_) {return;}
    shared_revision_ = frame->revision;
    if (mode_ == UpdateMode::FROZEN) {
        if (frame->source_type == "sensor_msgs/msg/PointCloud2" && frame->source_owner) {
            const auto source = std::static_pointer_cast<const sensor_msgs::msg::PointCloud2>(frame->source_owner);
            // 判定座標だけworldへ変換。出力の元座標系・intensity等の属性は保持。
            filtered_new_points_pub_->publish(filterIncomingPointCloud(*source, frame->source_to_world));
        }
        return;
    }
    // 旧集計の保持は不要。ほかの読者がいない場合の同一バッファ再利用。
    point_cells_.reset();
    point_cells_ = point_query_->read(frame);
    point_header_.frame_id = frame->frame_id;
    point_header_.stamp = rclcpp::Time(frame->stamp_ns);
    has_point_header_ = true;
    has_pending_update_ = true;
    if (latest_topological_map_msg_ && latest_topological_map_msg_->header.frame_id != frame->frame_id) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "共有点群とTmapのframe不一致。Tmapの集計を除外: points=%s Tmap=%s",
            frame->frame_id.c_str(), latest_topological_map_msg_->header.frame_id.c_str());
    }
}

void VoxelGridNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    latest_pointcloud_msg_ = msg;
    point_header_ = msg->header;
    has_point_header_ = true;
    has_pending_update_ = true;

    if (mode_ == UpdateMode::FROZEN) {
        auto filtered = filterIncomingPointCloud(*msg);
        filtered_new_points_pub_->publish(filtered);
    }
    else
    {
        update_point_counts(*msg);
    }
}

void VoxelGridNode::topologicalMapCallback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    latest_topological_map_msg_ = msg;
    has_pending_update_ = true;

}

void VoxelGridNode::timerCallback()
{
    const auto start_time = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(data_mutex_);

    if (shared_points_) {update_shared_points();}
    if (mode_ == UpdateMode::LIVE) {
        if (has_pending_update_) {
            rebuildVoxelsFromLatestMessages();
            rebuildViewsFromManagedVoxels();
            has_pending_update_ = false;
        }
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

    frozen_voxels_.clear();
    if (active_topological_map_msg_) {
        for (const auto &node : active_topological_map_msg_->nodes) {
            if (point_spec_.contains(Eigen::Vector3f(node.pos.x, node.pos.y, node.pos.z))) {
                frozen_voxels_[pointToVoxelKey(node.pos.x, node.pos.y, node.pos.z)].topological_nodes.push_back(node.pos);
            }
        }
    }
    for (const auto &voxel : combined_voxels_) {
        const auto found = frozen_voxels_.find(voxel.key);
        if (found != frozen_voxels_.end()) {found->second.display_label = voxel.display_label;}
    }
    frozen_combined_voxels_ = combined_voxels_;
    frozen_topological_map_msg_ = latest_topological_map_msg_;
    frozen_output_header_ = latest_output_header_;
    have_frozen_output_header_ = have_output_header_;
    if (have_frozen_output_header_) {
        frozen_markers_ = make_marker_cache(frozen_output_header_, frozen_combined_voxels_, true);
    }
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
    shared_revision_ = 0;
    has_pending_update_ = true;
    if (!shared_points_ && latest_pointcloud_msg_) {
        update_point_counts(*latest_pointcloud_msg_);
    }
    response->success = true;
    response->message = "Voxel update resumed.";
}

void VoxelGridNode::publishFrozenMarkerArray()
{
    if (have_frozen_output_header_) {marker_array_pub_->publish(frozen_markers_);}
}

void VoxelGridNode::publishFrozenTopologicalMap()
{
    if (!frozen_topological_map_msg_) {
        return;
    }
    frozen_topological_map_pub_->publish(*frozen_topological_map_msg_);
}


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
                    center_key.x + dx,
                    center_key.y + dy,
                    center_key.z + dz
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
    const sensor_msgs::msg::PointCloud2 & msg, const Eigen::Isometry3d &source_to_grid) const
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
        const Eigen::Vector3d point = source_to_grid * Eigen::Vector3d(*iter_x, *iter_y, *iter_z);
        const float x = point.x(), y = point.y(), z = point.z();
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

RCLCPP_COMPONENTS_REGISTER_NODE(fuzzy_voxel_grid::VoxelGridNode)
