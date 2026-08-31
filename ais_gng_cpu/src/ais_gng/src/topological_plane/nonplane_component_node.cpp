#include "ais_gng/topological_plane/nonplane_component_extractor.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace
{

using ais_gng_msgs::msg::PlaneClusterArray;
using ais_gng_msgs::msg::TopologicalMap;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

std_msgs::msg::ColorRGBA component_color(const std::size_t component_index)
{
  constexpr std::array<std::array<float, 3U>, 8U> colors{{
    {{0.93F, 0.33F, 0.31F}}, {{0.20F, 0.75F, 0.38F}},
    {{0.24F, 0.56F, 0.95F}}, {{0.88F, 0.56F, 0.18F}},
    {{0.69F, 0.36F, 0.90F}}, {{0.10F, 0.73F, 0.76F}},
    {{0.94F, 0.44F, 0.67F}}, {{0.64F, 0.78F, 0.24F}},
  }};
  const auto &rgb = colors[component_index % colors.size()];
  std_msgs::msg::ColorRGBA color;
  color.r = rgb[0];
  color.g = rgb[1];
  color.b = rgb[2];
  color.a = 1.0F;
  return color;
}

geometry_msgs::msg::Point marker_point(const geometry_msgs::msg::Point32 &point)
{
  geometry_msgs::msg::Point result;
  result.x = point.x;
  result.y = point.y;
  result.z = point.z;
  return result;
}

class nonplane_component_node : public rclcpp::Node
{
public:
  nonplane_component_node()
  : Node("nonplane_component_node")
  {
    const bool enable_extractor = declare_parameter<bool>(
      "enable_nonplane_component_extractor", false);
    if (!enable_extractor) {
      RCLCPP_INFO(
        get_logger(),
        "Nonplane component extraction disabled by enable_nonplane_component_extractor.");
      return;
    }

    input_topic_ = declare_parameter<std::string>("input_topic", "/topological_map");
    plane_clusters_topic_ = declare_parameter<std::string>(
      "plane_clusters_topic", "/plane_clusters");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/nonplane_components");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", output_topic_ + "/markers");
    extractor_options_.min_component_nodes = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<std::int64_t>("min_component_nodes", 2)));
    marker_scale_m_ = declare_parameter<double>("marker_scale_m", 0.012);
    anchor_line_width_m_ = declare_parameter<double>("anchor_line_width_m", 0.004);

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    output_publisher_ = create_publisher<TopologicalMap>(output_topic_, qos);
    marker_publisher_ = create_publisher<MarkerArray>(marker_topic_, qos);
    map_subscription_ = create_subscription<TopologicalMap>(
      input_topic_, qos,
      [this](const TopologicalMap::ConstSharedPtr &map) {
        latest_map_ = map;
        publish_if_matched();
      });
    plane_clusters_subscription_ = create_subscription<PlaneClusterArray>(
      plane_clusters_topic_, qos,
      [this](const PlaneClusterArray::ConstSharedPtr &plane_clusters) {
        latest_plane_clusters_ = plane_clusters;
        publish_if_matched();
      });

    RCLCPP_INFO(
      get_logger(),
      "Nonplane component extraction enabled: map=%s clusters=%s output=%s markers=%s min_nodes=%zu",
      input_topic_.c_str(), plane_clusters_topic_.c_str(), output_topic_.c_str(),
      marker_topic_.c_str(), extractor_options_.min_component_nodes);
  }

private:
  MarkerArray make_markers(
    const TopologicalMap &source_map,
    const fuzzrobo::topological_plane::nonplane::extraction_result &result) const
  {
    MarkerArray markers;
    Marker clear;
    clear.header = result.map.header;
    clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (std::size_t component_index = 0U; component_index < result.map.clusters.size(); ++component_index) {
      const auto &component = result.map.clusters[component_index];
      Marker marker;
      marker.header = result.map.header;
      marker.ns = "nonplane_components";
      marker.id = static_cast<std::int32_t>(component_index);
      marker.type = Marker::SPHERE_LIST;
      marker.action = Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker_scale_m_;
      marker.scale.y = marker_scale_m_;
      marker.scale.z = marker_scale_m_;
      marker.color = component_color(component_index);
      marker.points.reserve(component.nodes.size());
      for (const std::uint16_t node_index : component.nodes) {
        if (node_index < result.map.nodes.size()) {
          marker.points.push_back(marker_point(result.map.nodes[node_index].pos));
        }
      }
      markers.markers.push_back(std::move(marker));
    }

    Marker edges;
    edges.header = result.map.header;
    edges.ns = "nonplane_component_edges";
    edges.id = 0;
    edges.type = Marker::LINE_LIST;
    edges.action = Marker::ADD;
    edges.pose.orientation.w = 1.0;
    edges.scale.x = anchor_line_width_m_;
    edges.color.r = 0.92F;
    edges.color.g = 0.92F;
    edges.color.b = 0.92F;
    edges.color.a = 0.90F;
    edges.points.reserve(result.map.edges.size());
    for (std::size_t edge_index = 0U; edge_index + 1U < result.map.edges.size(); edge_index += 2U) {
      const std::size_t first = result.map.edges[edge_index];
      const std::size_t second = result.map.edges[edge_index + 1U];
      if (first >= result.map.nodes.size() || second >= result.map.nodes.size()) {
        continue;
      }
      edges.points.push_back(marker_point(result.map.nodes[first].pos));
      edges.points.push_back(marker_point(result.map.nodes[second].pos));
    }
    markers.markers.push_back(std::move(edges));

    Marker anchors;
    anchors.header = result.map.header;
    anchors.ns = "nonplane_component_plane_anchors";
    anchors.id = 0;
    anchors.type = Marker::LINE_LIST;
    anchors.action = Marker::ADD;
    anchors.pose.orientation.w = 1.0;
    anchors.scale.x = anchor_line_width_m_;
    anchors.color.r = 1.0F;
    anchors.color.g = 0.78F;
    anchors.color.b = 0.05F;
    anchors.color.a = 1.0F;
    anchors.points.reserve(result.plane_anchor_edges.size() * 2U);
    for (const auto &anchor : result.plane_anchor_edges) {
      if (anchor.source_node_index >= source_map.nodes.size() ||
        anchor.plane_node_index >= source_map.nodes.size())
      {
        continue;
      }
      anchors.points.push_back(marker_point(source_map.nodes[anchor.source_node_index].pos));
      anchors.points.push_back(marker_point(source_map.nodes[anchor.plane_node_index].pos));
    }
    markers.markers.push_back(std::move(anchors));
    return markers;
  }

  void publish_if_matched()
  {
    if (!latest_map_ || !latest_plane_clusters_ ||
      latest_map_->frame_number != latest_plane_clusters_->frame_number ||
      last_published_frame_number_ == latest_map_->frame_number)
    {
      return;
    }
    const auto result = fuzzrobo::topological_plane::nonplane::extract_components(
      *latest_map_, *latest_plane_clusters_, extractor_options_);
    marker_publisher_->publish(make_markers(*latest_map_, result));
    output_publisher_->publish(result.map);
    last_published_frame_number_ = latest_map_->frame_number;
  }

  std::string input_topic_;
  std::string plane_clusters_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  fuzzrobo::topological_plane::nonplane::extractor_options extractor_options_;
  double marker_scale_m_ = 0.012;
  double anchor_line_width_m_ = 0.004;
  std::uint32_t last_published_frame_number_ = std::numeric_limits<std::uint32_t>::max();
  rclcpp::Publisher<TopologicalMap>::SharedPtr output_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<TopologicalMap>::SharedPtr map_subscription_;
  rclcpp::Subscription<PlaneClusterArray>::SharedPtr plane_clusters_subscription_;
  TopologicalMap::ConstSharedPtr latest_map_;
  PlaneClusterArray::ConstSharedPtr latest_plane_clusters_;
};

}  // 無名名前空間

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nonplane_component_node>());
  rclcpp::shutdown();
  return 0;
}
