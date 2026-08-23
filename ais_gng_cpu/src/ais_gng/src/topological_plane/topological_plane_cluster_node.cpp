#include <ais_gng/topological_plane/topological_plane_cluster.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
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

visualization_msgs::msg::MarkerArray makeMarkers(
  const ais_gng_msgs::msg::PlanarClusterArray &clusters, bool publish_text)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header = clusters.header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(std::move(clear));

  for (const auto &cluster : clusters.clusters) {
    const auto marker_id = static_cast<std::int32_t>(cluster.id);
    const double normal_length = std::max(
      0.04, 0.35 * std::max(static_cast<double>(cluster.extent_u),
      static_cast<double>(cluster.extent_v)));
    const double line_width = std::clamp(
      0.12 * static_cast<double>(cluster.local_spacing), 0.002, 0.012);

    if (cluster.boundary.size() >= 3U) {
      auto fill = baseMarker(clusters.header, "planar_patch_fill", marker_id);
      fill.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      fill.scale.x = 1.0;
      fill.scale.y = 1.0;
      fill.scale.z = 1.0;
      fill.color.r = 0.05F;
      fill.color.g = 0.82F;
      fill.color.b = 0.95F;
      fill.color.a = 0.22F;
      const auto &first = cluster.boundary.front();
      for (std::size_t vertex = 1; vertex + 1U < cluster.boundary.size(); ++vertex) {
        fill.points.push_back(markerPoint(first));
        fill.points.push_back(markerPoint(cluster.boundary[vertex]));
        fill.points.push_back(markerPoint(cluster.boundary[vertex + 1U]));
      }
      markers.markers.push_back(std::move(fill));

      auto outline = baseMarker(clusters.header, "planar_patch_outline", marker_id);
      outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
      outline.scale.x = line_width;
      outline.color.r = 0.0F;
      outline.color.g = 0.95F;
      outline.color.b = 1.0F;
      outline.color.a = 0.95F;
      outline.points.reserve(cluster.boundary.size() + 1U);
      for (const auto &vertex : cluster.boundary) {
        outline.points.push_back(markerPoint(vertex));
      }
      outline.points.push_back(markerPoint(first));
      markers.markers.push_back(std::move(outline));
    }

    auto normal = baseMarker(clusters.header, "planar_patch_normal", marker_id);
    normal.type = visualization_msgs::msg::Marker::ARROW;
    normal.scale.x = std::max(0.004, normal_length * 0.08);
    normal.scale.y = std::max(0.010, normal_length * 0.18);
    normal.scale.z = std::max(0.012, normal_length * 0.24);
    normal.color.r = 1.0F;
    normal.color.g = 0.72F;
    normal.color.b = 0.05F;
    normal.color.a = 1.0F;
    normal.points.push_back(markerPoint(cluster.centroid));
    normal.points.push_back(offsetPoint(cluster.centroid, cluster.normal, normal_length));
    markers.markers.push_back(std::move(normal));

    if (publish_text) {
      auto text = baseMarker(clusters.header, "planar_patch_label", marker_id);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position = offsetPoint(cluster.centroid, cluster.normal, normal_length * 1.12);
      text.scale.z = std::clamp(normal_length * 0.38, 0.025, 0.08);
      text.color.r = 1.0F;
      text.color.g = 1.0F;
      text.color.b = 1.0F;
      text.color.a = 0.95F;
      std::ostringstream label;
      label << "P" << cluster.id << "  " << std::fixed << std::setprecision(2) <<
        cluster.area << "m2";
      text.text = label.str();
      markers.markers.push_back(std::move(text));
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
  : Node("topological_plane_cluster_node", options), extractor_(readOptions())
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/topological_map");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/topological_planar_clusters");
    marker_topic_ = declare_parameter<std::string>("marker_topic", output_topic_ + "/markers");
    publish_text_ = declare_parameter<bool>("publish_text", false);

    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    cluster_publisher_ = create_publisher<ais_gng_msgs::msg::PlanarClusterArray>(
      output_topic_, output_qos);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, output_qos);
    input_subscription_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&TopologicalPlaneClusterNode::onMap, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "topological_plane_cluster: %s -> %s (markers: %s)",
      input_topic_.c_str(), output_topic_.c_str(), marker_topic_.c_str());
  }

private:
  PlaneClusterOptions readOptions()
  {
    PlaneClusterOptions options;
    options.minimum_node_planarity = declare_parameter<double>("minimum_node_planarity", 0.25);
    options.minimum_cluster_planarity = declare_parameter<double>("minimum_cluster_planarity", 0.45);
    options.normal_alignment_cosine = declare_parameter<double>("normal_alignment_cosine", 0.90);
    options.maximum_normalized_edge_residual = declare_parameter<double>(
      "maximum_normalized_edge_residual", 0.35);
    options.maximum_normalized_cluster_residual = declare_parameter<double>(
      "maximum_normalized_cluster_residual", 0.35);
    const int minimum_cluster_nodes = declare_parameter<int>("minimum_cluster_nodes", 4);
    options.minimum_cluster_nodes = static_cast<std::size_t>(std::max(3, minimum_cluster_nodes));
    return options;
  }

  void onMap(const ais_gng_msgs::msg::TopologicalMap::SharedPtr map)
  {
    const auto started = std::chrono::steady_clock::now();
    auto result = extractor_.extract(*map);
    cluster_publisher_->publish(result.clusters);
    marker_publisher_->publish(makeMarkers(result.clusters, publish_text_));
    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "planar clusters=%zu nodes=%zu locally_planar=%zu clustered=%zu (%.2f ms)",
      result.statistics.cluster_count, result.statistics.valid_node_count,
      result.statistics.locally_planar_node_count, result.statistics.clustered_node_count,
      elapsed_ms);
  }

  TopologicalPlaneClusterExtractor extractor_;
  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  bool publish_text_ = false;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr input_subscription_;
  rclcpp::Publisher<ais_gng_msgs::msg::PlanarClusterArray>::SharedPtr cluster_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
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
