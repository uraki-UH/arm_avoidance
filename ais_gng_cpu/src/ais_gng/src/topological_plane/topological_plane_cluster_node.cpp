#include <ais_gng/topological_plane/topological_plane_cluster.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

geometry_msgs::msg::Point markerPoint(const geometry_msgs::msg::Point32 &point)
{
  geometry_msgs::msg::Point result;
  result.x = point.x;
  result.y = point.y;
  result.z = point.z;
  return result;
}

geometry_msgs::msg::Point offsetPoint(
  const geometry_msgs::msg::Point32 &point,
  const geometry_msgs::msg::Vector3 &direction,
  double distance)
{
  geometry_msgs::msg::Point result = markerPoint(point);
  result.x += direction.x * distance;
  result.y += direction.y * distance;
  result.z += direction.z * distance;
  return result;
}

geometry_msgs::msg::Point planarCorner(
  const ais_gng_msgs::msg::PlanarCluster &cluster, double tangent_u_distance,
  double tangent_v_distance)
{
  geometry_msgs::msg::Point result = markerPoint(cluster.centroid);
  result.x += cluster.tangent_u.x * tangent_u_distance + cluster.tangent_v.x * tangent_v_distance;
  result.y += cluster.tangent_u.y * tangent_u_distance + cluster.tangent_v.y * tangent_v_distance;
  result.z += cluster.tangent_u.z * tangent_u_distance + cluster.tangent_v.z * tangent_v_distance;
  return result;
}

visualization_msgs::msg::Marker baseMarker(
  const std_msgs::msg::Header &header, const std::string &name_space, std::int32_t id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = name_space;
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

std::size_t countOverlappingMembers(
  const ais_gng_msgs::msg::PlanarClusterArray &clusters, std::size_t node_count)
{
  std::vector<std::uint8_t> membership_count(node_count, 0U);
  std::size_t overlapping_node_count = 0U;
  for (const auto &cluster : clusters.clusters) {
    for (const std::uint32_t node_index : cluster.node_indices) {
      if (node_index >= membership_count.size()) {
        continue;
      }
      if (membership_count[node_index] == 1U) {
        ++overlapping_node_count;
      }
      if (membership_count[node_index] < std::numeric_limits<std::uint8_t>::max()) {
        ++membership_count[node_index];
      }
    }
  }
  return overlapping_node_count;
}

visualization_msgs::msg::MarkerArray makeObbMarkers(
  const ais_gng_msgs::msg::PlanarClusterArray &clusters, bool publish_text)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header = clusters.header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(std::move(clear));

  for (const auto &cluster : clusters.clusters) {
    const auto marker_id = static_cast<std::int32_t>(cluster.id);
    const bool terrain_cluster = cluster.source_label ==
      ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN;
    // 法線は向きを示す記号であり、面積を表すものではない。固定長にして、
    // メンバー数の変化でビューア上のクラスタが拡大・縮小して見えないようにする。
    constexpr double kNormalLength = 0.06;
    const double line_width = std::clamp(
      0.12 * static_cast<double>(cluster.local_spacing), 0.002, 0.012);

    if (cluster.extent_u > 0.0F && cluster.extent_v > 0.0F) {
      // 面積や占有面を推定するための枠ではない。クラスタIDごとの広がりを
      // エッジ群より見分けやすくする、平面上の有向境界枠として描画する。
      const double half_u = 0.5 * static_cast<double>(cluster.extent_u);
      const double half_v = 0.5 * static_cast<double>(cluster.extent_v);
      const auto first = planarCorner(cluster, -half_u, -half_v);
      const auto second = planarCorner(cluster, half_u, -half_v);
      const auto third = planarCorner(cluster, half_u, half_v);
      const auto fourth = planarCorner(cluster, -half_u, half_v);
      auto bounds = baseMarker(clusters.header, "planar_patch_obb", marker_id);
      bounds.type = visualization_msgs::msg::Marker::LINE_LIST;
      bounds.scale.x = std::clamp(1.6 * line_width, 0.003, 0.016);
      bounds.color.r = 1.0F;
      bounds.color.g = 0.18F;
      bounds.color.b = terrain_cluster ? 0.72F : 0.35F;
      bounds.color.a = 0.95F;
      bounds.points = {first, second, second, third, third, fourth, fourth, first};
      markers.markers.push_back(std::move(bounds));
    }

    auto normal = baseMarker(clusters.header, "planar_patch_normal", marker_id);
    normal.type = visualization_msgs::msg::Marker::ARROW;
    normal.scale.x = 0.005;
    normal.scale.y = 0.012;
    normal.scale.z = 0.018;
    normal.color.r = 1.0F;
    normal.color.g = 0.72F;
    normal.color.b = 0.05F;
    normal.color.a = 1.0F;
    normal.points.push_back(markerPoint(cluster.centroid));
    normal.points.push_back(offsetPoint(cluster.centroid, cluster.normal, kNormalLength));
    markers.markers.push_back(std::move(normal));

    if (publish_text) {
      auto text = baseMarker(clusters.header, "planar_patch_label", marker_id);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position = offsetPoint(cluster.centroid, cluster.normal, kNormalLength * 1.12);
      text.scale.z = 0.03;
      text.color.r = 1.0F;
      text.color.g = 1.0F;
      text.color.b = 1.0F;
      text.color.a = 0.95F;
      std::ostringstream label;
      label << (terrain_cluster ? "T" : "W") << cluster.id << "  N=" <<
        cluster.node_indices.size();
      text.text = label.str();
      markers.markers.push_back(std::move(text));
    }
  }
  return markers;
}

visualization_msgs::msg::MarkerArray makeNodeMarkers(
  const ais_gng_msgs::msg::PlanarClusterArray &clusters,
  const std::vector<ais_gng_msgs::msg::TopologicalNode> &nodes)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header = clusters.header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(std::move(clear));

  for (const auto &cluster : clusters.clusters) {
    const auto marker_id = static_cast<std::int32_t>(cluster.id);
    const bool terrain_cluster = cluster.source_label ==
      ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN;
    const double point_size = std::clamp(
      0.35 * static_cast<double>(cluster.local_spacing), 0.004, 0.018);
    const double line_width = std::clamp(
      0.12 * static_cast<double>(cluster.local_spacing), 0.002, 0.012);

    auto member_nodes = baseMarker(clusters.header, "planar_patch_nodes", marker_id);
    member_nodes.type = visualization_msgs::msg::Marker::POINTS;
    member_nodes.scale.x = point_size;
    member_nodes.scale.y = point_size;
    member_nodes.color.r = terrain_cluster ? 0.18F : 0.0F;
    member_nodes.color.g = terrain_cluster ? 0.95F : 0.68F;
    member_nodes.color.b = terrain_cluster ? 0.30F : 1.0F;
    member_nodes.color.a = 0.95F;
    member_nodes.points.reserve(cluster.node_indices.size());
    for (const std::uint32_t node_index : cluster.node_indices) {
      if (node_index < nodes.size()) {
        member_nodes.points.push_back(markerPoint(nodes[node_index].pos));
      }
    }
    markers.markers.push_back(std::move(member_nodes));

    if (cluster.support_edges.size() >= 2U) {
      auto edges = baseMarker(clusters.header, "planar_patch_support_edges", marker_id);
      edges.type = visualization_msgs::msg::Marker::LINE_LIST;
      edges.scale.x = line_width;
      edges.color.r = terrain_cluster ? 0.18F : 0.0F;
      edges.color.g = terrain_cluster ? 0.95F : 0.68F;
      edges.color.b = terrain_cluster ? 0.30F : 1.0F;
      edges.color.a = 0.65F;
      edges.points.reserve(cluster.support_edges.size());
      for (const auto &point : cluster.support_edges) {
        edges.points.push_back(markerPoint(point));
      }
      markers.markers.push_back(std::move(edges));
    }
  }
  return markers;
}

}  // namespace

namespace fuzzrobo::topological_plane
{

class TopologicalPlaneClusterNode : public rclcpp::Node
{
public:
  explicit TopologicalPlaneClusterNode(const rclcpp::NodeOptions &options)
  : Node("topological_plane_cluster_node", options), plane_options_(readOptions()),
    extractor_(plane_options_), tracker_(plane_options_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/topological_map");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/topological_planar_clusters");
    obb_marker_topic_ = output_topic_ + "/markers/obb";
    node_marker_topic_ = output_topic_ + "/markers/nodes";
    publish_text_ = declare_parameter<bool>("publish_text", false);

    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    cluster_publisher_ = create_publisher<ais_gng_msgs::msg::PlanarClusterArray>(
      output_topic_, output_qos);
    obb_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      obb_marker_topic_, output_qos);
    node_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      node_marker_topic_, output_qos);
    input_subscription_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&TopologicalPlaneClusterNode::onMap, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "topological_plane_cluster: %s -> %s (obb: %s, nodes: %s)",
      input_topic_.c_str(), output_topic_.c_str(), obb_marker_topic_.c_str(),
      node_marker_topic_.c_str());
  }

private:
  struct NodeSnapshot
  {
    bool valid = false;
    std::uint32_t frame = 0U;
    std::uint32_t winner_point_count = 0U;
    std::uint8_t label = 0U;
    geometry_msgs::msg::Point32 position;
    geometry_msgs::msg::Point32 normal;
  };

  struct MapChangeSummary
  {
    std::size_t changed_node_count = 0U;
    std::size_t winner_updated_node_count = 0U;
    std::size_t label_changed_node_count = 0U;
    bool topology_changed = false;
  };

  PlaneClusterOptions readOptions()
  {
    PlaneClusterOptions options;
    options.min_cluster_planarity = declare_parameter<double>(
      "min_cluster_planarity", options.min_cluster_planarity);
    options.max_normalized_cluster_residual = declare_parameter<double>(
      "max_normalized_cluster_residual", options.max_normalized_cluster_residual);
    const int min_cluster_nodes = declare_parameter<int>(
      "min_cluster_nodes", static_cast<int>(options.min_cluster_nodes));
    options.min_cluster_nodes = static_cast<std::size_t>(std::max(3, min_cluster_nodes));
    options.min_normal_alignment_cos = declare_parameter<double>(
      "min_normal_alignment_cos", options.min_normal_alignment_cos);
    options.max_growth_dist_ratio = declare_parameter<double>(
      "max_growth_dist_ratio", options.max_growth_dist_ratio);
    options.max_seed_plane_dist_ratio = declare_parameter<double>(
      "max_seed_plane_dist_ratio", options.max_seed_plane_dist_ratio);
    options.min_cluster_edge_per_node = declare_parameter<double>(
      "min_cluster_edge_per_node", options.min_cluster_edge_per_node);
    return options;
  }

  void onMap(const ais_gng_msgs::msg::TopologicalMap::SharedPtr map)
  {
    const auto started = std::chrono::steady_clock::now();
    const MapChangeSummary changes = observeMapChanges(*map);
    const auto observed = std::chrono::steady_clock::now();
    const bool rebuild = !has_cached_clusters_ || changes.winner_updated_node_count > 0U;

    PlaneClusterStatistics statistics = last_statistics_;
    ais_gng_msgs::msg::PlanarClusterArray tracked_clusters = last_tracked_clusters_;
    std::size_t overlapping_member_count = last_overlapping_member_count_;
    auto extracted = observed;
    auto tracked = observed;
    if (rebuild) {
      auto result = extractor_.extract(*map);
      extracted = std::chrono::steady_clock::now();
      tracked_clusters = tracker_.update(result.clusters, *map);
      statistics = result.statistics;
    } else {
      // 抽出を省略するフレームでも、既存クラスタの各ノードは現在の追跡平面からの
      // 逸脱で個別に更新する。空の入力は新規クラスタを誕生させない保守更新を表す。
      ais_gng_msgs::msg::PlanarClusterArray maintenance;
      maintenance.header = map->header;
      maintenance.frame_number = map->frame_number;
      tracked_clusters = tracker_.update(maintenance, *map);
    }
    tracked = std::chrono::steady_clock::now();
    overlapping_member_count = countOverlappingMembers(tracked_clusters, map->nodes.size());
    last_tracked_clusters_ = tracked_clusters;
    last_statistics_ = statistics;
    last_overlapping_member_count_ = overlapping_member_count;
    has_cached_clusters_ = true;
    cluster_publisher_->publish(tracked_clusters);
    obb_marker_publisher_->publish(makeObbMarkers(tracked_clusters, publish_text_));
    node_marker_publisher_->publish(makeNodeMarkers(tracked_clusters, map->nodes));
    const auto completed = std::chrono::steady_clock::now();
    const double observation_ms = std::chrono::duration<double, std::milli>(observed - started).count();
    const double extraction_ms = std::chrono::duration<double, std::milli>(extracted - observed).count();
    const double tracking_ms = std::chrono::duration<double, std::milli>(tracked - extracted).count();
    const double publishing_ms = std::chrono::duration<double, std::milli>(completed - tracked).count();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(completed - started).count();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "planar clusters=%zu frame_patches=%zu nodes=%zu normal=%zu triangle_seeds=%zu "
      "small=%zu line=%zu rejected=%zu members=%zu(default=%zu terrain=%zu wall=%zu "
      "unknown=%zu human=%zu car=%zu other=%zu) overlap_nodes=%zu changed=%zu winners=%zu "
      "labels=%zu topology=%s rebuild=%s "
      "observe=%.2f extract=%.2f track=%.2f publish=%.2f total=%.2f ms",
      tracked_clusters.clusters.size(), statistics.cluster_count,
      statistics.valid_node_count,
      statistics.locally_planar_node_count, statistics.seed_component_count,
      statistics.insufficient_seed_component_count,
      statistics.line_like_component_count,
      statistics.geometrically_rejected_component_count,
      statistics.clustered_node_count,
      statistics.clustered_default_node_count,
      statistics.clustered_terrain_node_count,
      statistics.clustered_wall_node_count,
      statistics.clustered_unknown_node_count,
      statistics.clustered_human_node_count,
      statistics.clustered_car_node_count,
      statistics.clustered_other_node_count,
      overlapping_member_count,
      changes.changed_node_count,
      changes.winner_updated_node_count,
      changes.label_changed_node_count,
      changes.topology_changed ? "changed" : "same",
      rebuild ? "yes" : "no",
      observation_ms,
      extraction_ms,
      tracking_ms,
      publishing_ms,
      elapsed_ms);
  }

  MapChangeSummary observeMapChanges(const ais_gng_msgs::msg::TopologicalMap &map)
  {
    MapChangeSummary summary;
    for (std::size_t node_index = 0U; node_index < map.nodes.size(); ++node_index) {
      const auto &node = map.nodes[node_index];
      if (previous_nodes_.size() <= node.id) {
        previous_nodes_.resize(static_cast<std::size_t>(node.id) + 1U);
      }
      NodeSnapshot &previous = previous_nodes_[node.id];
      const bool unchanged = previous.valid && previous.frame == node.frame &&
        previous.label == node.label &&
        previous.position.x == node.pos.x && previous.position.y == node.pos.y &&
        previous.position.z == node.pos.z &&
        previous.normal.x == node.normal.x && previous.normal.y == node.normal.y &&
        previous.normal.z == node.normal.z;
      summary.changed_node_count += !unchanged;
      summary.winner_updated_node_count += previous.valid &&
        previous.winner_point_count != node.winner_point_count;
      const bool label_changed_here = previous.valid && previous.label != node.label;
      summary.label_changed_node_count += label_changed_here;
      previous.valid = true;
      previous.frame = node.frame;
      previous.winner_point_count = node.winner_point_count;
      previous.label = node.label;
      previous.position = node.pos;
      previous.normal = node.normal;
    }
    summary.topology_changed = !has_previous_topology_ || previous_edges_ != map.edges;
    previous_edges_ = map.edges;
    has_previous_topology_ = true;

    return summary;
  }

  PlaneClusterOptions plane_options_;
  TopologicalPlaneClusterExtractor extractor_;
  PersistentPlaneClusterTracker tracker_;
  std::string input_topic_;
  std::string output_topic_;
  std::string obb_marker_topic_;
  std::string node_marker_topic_;
  bool publish_text_ = false;
  bool has_cached_clusters_ = false;
  ais_gng_msgs::msg::PlanarClusterArray last_tracked_clusters_;
  PlaneClusterStatistics last_statistics_;
  std::size_t last_overlapping_member_count_ = 0U;
  std::vector<NodeSnapshot> previous_nodes_;
  std::vector<std::uint16_t> previous_edges_;
  bool has_previous_topology_ = false;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr input_subscription_;
  rclcpp::Publisher<ais_gng_msgs::msg::PlanarClusterArray>::SharedPtr cluster_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obb_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr node_marker_publisher_;
};

}  // namespace fuzzrobo::topological_plane

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fuzzrobo::topological_plane::TopologicalPlaneClusterNode>(
      rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
