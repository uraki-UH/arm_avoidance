#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <utility>

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

// 平面内の (u, v) オフセットにある点を、クラスタの接平面基底から求める。
geometry_msgs::msg::Point planarCorner(
  const ais_gng_msgs::msg::PlanarCluster &cluster, const double u, const double v)
{
  geometry_msgs::msg::Point result = markerPoint(cluster.centroid);
  result.x += cluster.tangent_u.x * u + cluster.tangent_v.x * v;
  result.y += cluster.tangent_u.y * u + cluster.tangent_v.y * v;
  result.z += cluster.tangent_u.z * u + cluster.tangent_v.z * v;
  return result;
}

// 色番号から決定的に色を作る。番号が変わらない限り、色もフレーム間で変わらない。
std_msgs::msg::ColorRGBA clusterColor(const std::uint32_t color_index)
{
  // 黄金比で色相を回すと、連続した番号でも色が離れる。
  const double hue = std::fmod(static_cast<double>(color_index) * 0.618033988749895, 1.0) * 6.0;
  const int sector = static_cast<int>(hue);
  const double fraction = hue - static_cast<double>(sector);
  const double high = 0.95;
  const double low = 0.25;
  const double rising = low + (high - low) * fraction;
  const double falling = high - (high - low) * fraction;

  std_msgs::msg::ColorRGBA color;
  color.a = 1.0F;
  switch (sector) {
    case 0: color.r = high; color.g = rising; color.b = low; break;
    case 1: color.r = falling; color.g = high; color.b = low; break;
    case 2: color.r = low; color.g = high; color.b = rising; break;
    case 3: color.r = low; color.g = falling; color.b = high; break;
    case 4: color.r = rising; color.g = low; color.b = high; break;
    default: color.r = high; color.g = low; color.b = falling; break;
  }
  return color;
}

visualization_msgs::msg::Marker baseMarker(
  const std_msgs::msg::Header &header, const std::string &name_space, const std::int32_t id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = name_space;
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

// 前フレームに出したIDを渡し、消えたものだけ DELETE する。
//
// 毎フレーム DELETEALL を送ると、削除と再追加の間で表示が一瞬抜けて明滅する。
// 生き残っているクラスタのマーカーは上書き更新に任せる。
visualization_msgs::msg::MarkerArray makeObbMarkers(
  const ais_gng_msgs::msg::PlanarClusterArray &clusters, const bool publish_text,
  std::set<std::uint32_t> &published_ids,
  const std::unordered_map<std::uint32_t, std::uint32_t> &color_of)
{
  visualization_msgs::msg::MarkerArray markers;
  std::set<std::uint32_t> current_ids;
  for (const auto &cluster : clusters.clusters) {
    current_ids.insert(cluster.id);
  }
  for (const std::uint32_t id : published_ids) {
    if (current_ids.count(id) != 0U) {
      continue;
    }
    for (const char *name_space : {"incremental_plane_obb",
        "incremental_plane_normal", "incremental_plane_label"})
    {
      visualization_msgs::msg::Marker remove;
      remove.ns = name_space;
      remove.id = static_cast<std::int32_t>(id);
      remove.action = visualization_msgs::msg::Marker::DELETE;
      markers.markers.push_back(std::move(remove));
    }
  }
  published_ids = std::move(current_ids);

  for (const auto &cluster : clusters.clusters) {
    const auto marker_id = static_cast<std::int32_t>(cluster.id);
    const auto color_it = color_of.find(cluster.id);
    const auto color = clusterColor(
      color_it != color_of.end() ? color_it->second : cluster.id);

    // 平面上の有向境界枠。エッジ群より見分けやすく、クラスタの広がりが一目で分かる。
    // extent は重心まわりに対称と見なして描くため、実際の分布が偏っている場合は近似になる。
    if (cluster.extent_u > 0.0F || cluster.extent_v > 0.0F) {
      const double half_u = 0.5 * static_cast<double>(cluster.extent_u);
      const double half_v = 0.5 * static_cast<double>(cluster.extent_v);
      auto bounds = baseMarker(clusters.header, "incremental_plane_obb", marker_id);
      bounds.type = visualization_msgs::msg::Marker::LINE_LIST;
      bounds.scale.x = std::clamp(
        2.5 * 0.15 * static_cast<double>(cluster.local_spacing), 0.004, 0.016);
      bounds.color = color;
      bounds.color.a = 1.0F;
      const auto c0 = planarCorner(cluster, -half_u, -half_v);
      const auto c1 = planarCorner(cluster, half_u, -half_v);
      const auto c2 = planarCorner(cluster, half_u, half_v);
      const auto c3 = planarCorner(cluster, -half_u, half_v);
      bounds.points = {c0, c1, c1, c2, c2, c3, c3, c0};
      markers.markers.push_back(std::move(bounds));
    }

    auto normal = baseMarker(clusters.header, "incremental_plane_normal", marker_id);
    normal.type = visualization_msgs::msg::Marker::ARROW;
    const double length = std::max(0.05, 2.0 * static_cast<double>(cluster.local_spacing));
    normal.scale.x = 0.25 * length;
    normal.scale.y = 0.5 * normal.scale.x;
    normal.scale.z = 0.0;
    normal.color = color;
    normal.points.push_back(markerPoint(cluster.centroid));
    geometry_msgs::msg::Point tip = markerPoint(cluster.centroid);
    tip.x += cluster.normal.x * length;
    tip.y += cluster.normal.y * length;
    tip.z += cluster.normal.z * length;
    normal.points.push_back(tip);
    markers.markers.push_back(std::move(normal));

    if (publish_text) {
      auto text = baseMarker(clusters.header, "incremental_plane_label", marker_id);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position = markerPoint(cluster.centroid);
      text.scale.z = std::max(0.03, 1.5 * static_cast<double>(cluster.local_spacing));
      text.color = color;
      text.text = "id=" + std::to_string(cluster.id) +
        " n=" + std::to_string(cluster.node_indices.size());
      markers.markers.push_back(std::move(text));
    }
  }
  return markers;
}

// 前フレームに出したIDを渡し、消えた所属ノードとクラスタ内エッジだけ DELETE する。
visualization_msgs::msg::MarkerArray makeNodeMarkers(
  const ais_gng_msgs::msg::PlanarClusterArray &clusters,
  const std::vector<ais_gng_msgs::msg::TopologicalNode> &nodes,
  std::set<std::uint32_t> &published_ids,
  const std::unordered_map<std::uint32_t, std::uint32_t> &color_of)
{
  visualization_msgs::msg::MarkerArray markers;
  std::set<std::uint32_t> current_ids;
  for (const auto &cluster : clusters.clusters) {
    current_ids.insert(cluster.id);
  }
  for (const std::uint32_t id : published_ids) {
    if (current_ids.count(id) != 0U) {
      continue;
    }
    for (const char *name_space : {"incremental_plane_nodes", "incremental_plane_edges"}) {
      visualization_msgs::msg::Marker remove;
      remove.ns = name_space;
      remove.id = static_cast<std::int32_t>(id);
      remove.action = visualization_msgs::msg::Marker::DELETE;
      markers.markers.push_back(std::move(remove));
    }
  }
  published_ids = std::move(current_ids);

  for (const auto &cluster : clusters.clusters) {
    const auto marker_id = static_cast<std::int32_t>(cluster.id);
    const auto color_it = color_of.find(cluster.id);
    const auto color = clusterColor(
      color_it != color_of.end() ? color_it->second : cluster.id);
    const double point_size = std::clamp(
      0.35 * static_cast<double>(cluster.local_spacing), 0.004, 0.018);

    auto member_nodes = baseMarker(clusters.header, "incremental_plane_nodes", marker_id);
    member_nodes.type = visualization_msgs::msg::Marker::POINTS;
    member_nodes.scale.x = point_size;
    member_nodes.scale.y = point_size;
    member_nodes.color = color;
    member_nodes.points.reserve(cluster.node_indices.size());
    for (const std::uint32_t node_index : cluster.node_indices) {
      if (node_index < nodes.size()) {
        member_nodes.points.push_back(markerPoint(nodes[node_index].pos));
      }
    }
    markers.markers.push_back(std::move(member_nodes));

    if (!cluster.support_edges.empty()) {
      auto edges = baseMarker(clusters.header, "incremental_plane_edges", marker_id);
      edges.type = visualization_msgs::msg::Marker::LINE_LIST;
      edges.scale.x = std::max(0.004, 0.15 * static_cast<double>(cluster.local_spacing));
      edges.color = color;
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

}  // 無名名前空間

namespace fuzzrobo::topological_plane::incremental
{

// 増分方式の平面クラスタ生成をROSノードとして動かす。
// 既存の topological_plane_cluster_node とは別トピックへ出すため、同時に動かして
// 同じ入力で結果を比べられる。
class PlaneClusterIncrementalNode : public rclcpp::Node
{
public:
  PlaneClusterIncrementalNode()
  : rclcpp::Node("plane_cluster_incremental_node"),
    clusterizer_(readOptions())
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/topological_map");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/topological_planar_clusters_incremental");
    obb_marker_topic_ = output_topic_ + "/markers/obb";
    node_marker_topic_ = output_topic_ + "/markers/nodes";
    publish_text_ = declare_parameter<bool>("publish_text", false);

    const auto output_qos = rclcpp::QoS(1).transient_local();
    cluster_publisher_ =
      create_publisher<ais_gng_msgs::msg::PlanarClusterArray>(output_topic_, output_qos);
    obb_marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(obb_marker_topic_, output_qos);
    node_marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(node_marker_topic_, output_qos);
    subscription_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      input_topic_, rclcpp::QoS(1),
      [this](const ais_gng_msgs::msg::TopologicalMap::ConstSharedPtr &map) {onMap(*map);});

    RCLCPP_INFO(
      get_logger(), "plane_cluster_incremental: %s -> %s (obb: %s, nodes: %s)",
      input_topic_.c_str(), output_topic_.c_str(), obb_marker_topic_.c_str(),
      node_marker_topic_.c_str());
  }

private:
  ClusterOptions readOptions()
  {
    ClusterOptions options;
    options.min_cluster_nodes = static_cast<std::size_t>(
      std::max<std::int64_t>(3, declare_parameter<int>("min_cluster_nodes", 7)));
    options.growth_residual_ratio =
      declare_parameter<double>("growth_residual_ratio", 0.35);
    options.retention_residual_ratio =
      declare_parameter<double>("retention_residual_ratio", 0.70);
    options.normal_alignment_cos =
      declare_parameter<double>("normal_alignment_cos", 0.50);
    options.min_cluster_planarity =
      declare_parameter<double>("min_cluster_planarity", 0.45);
    options.max_normalized_cluster_residual =
      declare_parameter<double>("max_normalized_cluster_residual", 0.35);
    options.min_growth_planarity =
      declare_parameter<double>("min_growth_planarity", 0.25);
    options.absorb_neighbor_requirement = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("absorb_neighbor_requirement", 2)));
    options.migration_neighbor_requirement = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("migration_neighbor_requirement", 2)));
    options.birth_neighbor_requirement = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("birth_neighbor_requirement", 1)));
    options.migration_improvement_margin =
      declare_parameter<double>("migration_improvement_margin", 0.30);
    options.coplanar_multi_edge_overrides_distance =
      declare_parameter<bool>("coplanar_multi_edge_overrides_distance", true);
    options.maintenance_iterations = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("maintenance_iterations", 2)));
    options.merge_edge_requirement = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("merge_edge_requirement", 2)));
    options.prefer_larger_cluster_on_migration =
      declare_parameter<bool>("prefer_larger_cluster_on_migration", false);
    options.migration_size_bias_tolerance =
      declare_parameter<double>("migration_size_bias_tolerance", 0.10);
    options.migration_size_bias_ratio =
      declare_parameter<double>("migration_size_bias_ratio", 2.0);
    options.donor_protection_buffer = static_cast<std::size_t>(
      std::max<std::int64_t>(0, declare_parameter<int>("donor_protection_buffer", 3)));
    options.merge_normal_alignment_cos =
      declare_parameter<double>("merge_normal_alignment_cos", 0.80);
    options.merge_residual_growth_ratio =
      declare_parameter<double>("merge_residual_growth_ratio", 1.3);
    options.merge_residual_growth_floor =
      declare_parameter<double>("merge_residual_growth_floor", 0.15);
    options.birth_confirm_frames = static_cast<std::size_t>(
      std::max<std::int64_t>(0, declare_parameter<int>("birth_confirm_frames", 3)));
    options.split_confirm_frames = static_cast<std::size_t>(
      std::max<std::int64_t>(0, declare_parameter<int>("split_confirm_frames", 3)));
    options.weak_frame_allowance = static_cast<std::size_t>(
      std::max<std::int64_t>(0, declare_parameter<int>("weak_frame_allowance", 5)));
    return options;
  }

  // 隣り合うクラスタに同じ色を割り当てないようにする。
  //
  // 毎フレーム貪欲彩色をやり直すと色が飛び回るので、前フレームの色を優先して保ち、
  // 隣接と衝突したときだけ空いている番号へ移す。処理量はノード数とエッジ数に比例する。
  void assignColors(
    const ais_gng_msgs::msg::PlanarClusterArray &clusters,
    const ais_gng_msgs::msg::TopologicalMap &map)
  {
    const std::size_t node_count = map.nodes.size();
    std::vector<int> owner(node_count, -1);
    std::vector<std::uint32_t> ids;
    ids.reserve(clusters.clusters.size());
    for (std::size_t index = 0U; index < clusters.clusters.size(); ++index) {
      ids.push_back(clusters.clusters[index].id);
      for (const std::uint32_t member : clusters.clusters[index].node_indices) {
        if (member < node_count) {
          owner[member] = static_cast<int>(index);
        }
      }
    }

    std::vector<std::unordered_set<std::size_t>> adjacency(clusters.clusters.size());
    for (std::size_t edge = 0U; edge + 1U < map.edges.size(); edge += 2U) {
      const std::size_t first = map.edges[edge];
      const std::size_t second = map.edges[edge + 1U];
      if (first >= node_count || second >= node_count) {
        continue;
      }
      const int first_owner = owner[first];
      const int second_owner = owner[second];
      if (first_owner < 0 || second_owner < 0 || first_owner == second_owner) {
        continue;
      }
      adjacency[static_cast<std::size_t>(first_owner)].insert(
        static_cast<std::size_t>(second_owner));
      adjacency[static_cast<std::size_t>(second_owner)].insert(
        static_cast<std::size_t>(first_owner));
    }

    // 古いクラスタから決めることで、既存の色がなるべく動かないようにする。
    std::vector<std::size_t> order(clusters.clusters.size());
    for (std::size_t index = 0U; index < order.size(); ++index) { order[index] = index; }
    std::sort(order.begin(), order.end(), [&ids](std::size_t a, std::size_t b) {
        return ids[a] < ids[b];
      });

    std::unordered_map<std::uint32_t, std::uint32_t> next_color;
    next_color.reserve(clusters.clusters.size() * 2U + 1U);
    std::vector<int> assigned(clusters.clusters.size(), -1);
    for (const std::size_t index : order) {
      std::unordered_set<std::uint32_t> taken;
      for (const std::size_t neighbour : adjacency[index]) {
        if (assigned[neighbour] >= 0) {
          taken.insert(static_cast<std::uint32_t>(assigned[neighbour]));
        }
      }
      std::uint32_t chosen = 0U;
      const auto previous = cluster_color_.find(ids[index]);
      if (previous != cluster_color_.end() && taken.count(previous->second) == 0U) {
        chosen = previous->second;
      } else {
        while (taken.count(chosen) != 0U) { ++chosen; }
      }
      assigned[index] = static_cast<int>(chosen);
      next_color[ids[index]] = chosen;
    }
    cluster_color_.swap(next_color);
  }

  void onMap(const ais_gng_msgs::msg::TopologicalMap &map)
  {
    const auto started = std::chrono::steady_clock::now();
    ClusterResult result = clusterizer_.update(map);
    const auto updated = std::chrono::steady_clock::now();

    assignColors(result.clusters, map);
    cluster_publisher_->publish(result.clusters);
    obb_marker_publisher_->publish(
      makeObbMarkers(result.clusters, publish_text_, published_obb_marker_ids_, cluster_color_));
    node_marker_publisher_->publish(
      makeNodeMarkers(result.clusters, map.nodes, published_node_marker_ids_, cluster_color_));
    const auto completed = std::chrono::steady_clock::now();

    const double update_ms =
      std::chrono::duration<double, std::milli>(updated - started).count();
    const double publish_ms =
      std::chrono::duration<double, std::milli>(completed - updated).count();
    const std::size_t changes = result.statistics.released_node_count +
      result.statistics.migrated_node_count + result.statistics.absorbed_node_count;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "clusters=%zu nodes=%zu/%zu clustered=%zu | changes=%zu "
      "(release=%zu migrate=%zu absorb=%zu) born=%zu chain=%zu split=%zu merged=%zu removed=%zu "
      "passes=%zu | members(default=%zu terrain=%zu wall=%zu unknown=%zu human=%zu car=%zu "
      "other=%zu) | update=%.2f publish=%.2f ms",
      result.statistics.cluster_count,
      result.statistics.usable_node_count,
      result.statistics.valid_node_count,
      result.statistics.clustered_node_count,
      changes,
      result.statistics.released_node_count,
      result.statistics.migrated_node_count,
      result.statistics.absorbed_node_count,
      result.statistics.born_cluster_count,
      result.statistics.chain_rejected_count,
      result.statistics.split_cluster_count,
      result.statistics.merged_cluster_count,
      result.statistics.removed_cluster_count,
      result.statistics.maintenance_iterations_used,
      result.statistics.clustered_default_node_count,
      result.statistics.clustered_terrain_node_count,
      result.statistics.clustered_wall_node_count,
      result.statistics.clustered_unknown_node_count,
      result.statistics.clustered_human_node_count,
      result.statistics.clustered_car_node_count,
      result.statistics.clustered_other_node_count,
      update_ms, publish_ms);
  }

  std::set<std::uint32_t> published_obb_marker_ids_;
  std::set<std::uint32_t> published_node_marker_ids_;
  std::unordered_map<std::uint32_t, std::uint32_t> cluster_color_;
  std::string input_topic_;
  std::string output_topic_;
  std::string obb_marker_topic_;
  std::string node_marker_topic_;
  bool publish_text_ = false;

  Clusterizer clusterizer_;
  rclcpp::Publisher<ais_gng_msgs::msg::PlanarClusterArray>::SharedPtr cluster_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obb_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr node_marker_publisher_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr subscription_;
};

}  // fuzzrobo::topological_plane::incremental 名前空間

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<fuzzrobo::topological_plane::incremental::PlaneClusterIncrementalNode>());
  rclcpp::shutdown();
  return 0;
}
