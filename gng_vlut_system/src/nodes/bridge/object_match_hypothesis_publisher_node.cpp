#include <ais_gng_msgs/msg/topological_cluster.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/msg/topological_node.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace robot_sim::bridge
{
namespace
{

using json = nlohmann::json;

bool isTopicToken(const std::string &value)
{
  if (value.empty() || !std::isalpha(static_cast<unsigned char>(value.front()))) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) || character == '_';
  });
}

struct GridKey
{
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const GridKey &other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridKeyHash
{
  std::size_t operator()(const GridKey &key) const
  {
    const std::size_t x_hash = std::hash<std::int64_t>{}(key.x);
    const std::size_t y_hash = std::hash<std::int64_t>{}(key.y);
    const std::size_t z_hash = std::hash<std::int64_t>{}(key.z);
    return x_hash ^ (y_hash << 1U) ^ (z_hash << 7U);
  }
};

struct Aabb
{
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  float max_z = std::numeric_limits<float>::lowest();

  void append(const ais_gng_msgs::msg::TopologicalNode &node)
  {
    min_x = std::min(min_x, node.pos.x);
    min_y = std::min(min_y, node.pos.y);
    min_z = std::min(min_z, node.pos.z);
    max_x = std::max(max_x, node.pos.x);
    max_y = std::max(max_y, node.pos.y);
    max_z = std::max(max_z, node.pos.z);
  }
};

struct CellAggregate
{
  std::vector<std::size_t> source_indices;
  double weight_sum = 0.0;
  double position_x_sum = 0.0;
  double position_y_sum = 0.0;
  double position_z_sum = 0.0;
  double normal_x_sum = 0.0;
  double normal_y_sum = 0.0;
  double normal_z_sum = 0.0;
  double rho_sum = 0.0;
  std::uint64_t winner_point_count = 0;
  std::unordered_map<std::uint8_t, double> label_weights;
};

double nodeWeight(const ais_gng_msgs::msg::TopologicalNode &node)
{
  return node.winner_point_count > 0 ? static_cast<double>(node.winner_point_count) : 1.0;
}

std::uint8_t dominantLabel(const CellAggregate &cell)
{
  std::uint8_t label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
  double largest_weight = -1.0;
  for (const auto &[candidate, weight] : cell.label_weights) {
    if (weight > largest_weight || (weight == largest_weight && candidate < label)) {
      label = candidate;
      largest_weight = weight;
    }
  }
  return label;
}

geometry_msgs::msg::Point makePoint(float x, float y, float z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

}

class ObjectMatchHypothesisPublisherNode : public rclcpp::Node
{
public:
  explicit ObjectMatchHypothesisPublisherNode(const rclcpp::NodeOptions &options)
  : Node("object_match_hypothesis_publisher_node", options)
  {
    template_id_ = declare_parameter<std::string>("template_id", "");
    hypothesis_id_ = declare_parameter<std::string>("hypothesis_id", "candidate_0");
    environment_topic_ = declare_parameter<std::string>(
      "environment_topological_map_topic", "/topological_map");
    template_topic_ = declare_parameter<std::string>("template_topological_map_topic", "");
    environment_cluster_id_ = declare_parameter<int>("environment_cluster_id", -1);
    grid_cell_size_ = declare_parameter<double>("grid_cell_size", 0.03);
    score_ = declare_parameter<double>("score", 0.0);
    yaw_deg_ = declare_parameter<double>("yaw_deg", 0.0);
    frame_id_override_ = declare_parameter<std::string>("frame_id", "");
    const double publish_hz = std::max(0.1, declare_parameter<double>("publish_hz", 1.0));
    aabb_line_width_ = std::max(0.0001, declare_parameter<double>("aabb_line_width", 0.004));

    if (!isTopicToken(template_id_) || !isTopicToken(hypothesis_id_)) {
      throw std::runtime_error("template_idとhypothesis_idは英字開始の英数字または_だけを指定してください。");
    }
    if (environment_topic_.empty() || template_topic_.empty()) {
      throw std::runtime_error("環境側とテンプレート側のTopologicalMap topic指定が必要です。");
    }
    if (!std::isfinite(grid_cell_size_) || grid_cell_size_ <= 0.0) {
      throw std::runtime_error("grid_cell_sizeは正の有限値で指定してください。");
    }
    if (!std::isfinite(score_) || !std::isfinite(yaw_deg_)) {
      throw std::runtime_error("scoreとyaw_degは有限値で指定してください。");
    }

    output_prefix_ = "/" + template_id_ + "/hypotheses/" + hypothesis_id_;
    const auto output_qos = rclcpp::QoS(1).reliable().transient_local();
    graph_publisher_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
      output_prefix_ + "/topological_map", output_qos);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      output_prefix_ + "/markers", output_qos);
    metadata_publisher_ = create_publisher<std_msgs::msg::String>(
      output_prefix_ + "/metadata", output_qos);

    const auto input_qos = rclcpp::QoS(1).reliable().transient_local();
    environment_subscription_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      environment_topic_, input_qos,
      [this](ais_gng_msgs::msg::TopologicalMap::SharedPtr message) {
        environment_map_ = std::move(message);
        publishHypothesis();
      });
    template_subscription_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      template_topic_, input_qos,
      [this](ais_gng_msgs::msg::TopologicalMap::SharedPtr message) {
        template_map_ = std::move(message);
        publishHypothesis();
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ObjectMatchHypothesisPublisherNode::publishHypothesis, this));

    RCLCPP_INFO(
      get_logger(),
      "物体照合仮説配信待機: template=%s hypothesis=%s environment=%s template_map=%s",
      template_id_.c_str(), hypothesis_id_.c_str(), environment_topic_.c_str(), template_topic_.c_str());
  }

private:
  std::vector<std::size_t> selectEnvironmentNodes(
    const ais_gng_msgs::msg::TopologicalMap &environment) const
  {
    std::vector<std::size_t> selected;
    if (environment_cluster_id_ < 0) {
      selected.resize(environment.nodes.size());
      for (std::size_t index = 0; index < selected.size(); ++index) {
        selected[index] = index;
      }
      return selected;
    }

    std::unordered_map<std::uint16_t, std::size_t> id_to_index;
    id_to_index.reserve(environment.nodes.size());
    for (std::size_t index = 0; index < environment.nodes.size(); ++index) {
      id_to_index.emplace(environment.nodes[index].id, index);
    }
    for (const auto &cluster : environment.clusters) {
      if (cluster.id != static_cast<std::uint32_t>(environment_cluster_id_)) {
        continue;
      }
      std::unordered_set<std::size_t> unique_indices;
      for (const std::uint16_t node_id : cluster.nodes) {
        const auto found = id_to_index.find(node_id);
        if (found != id_to_index.end()) {
          unique_indices.insert(found->second);
        }
      }
      selected.assign(unique_indices.begin(), unique_indices.end());
      std::sort(selected.begin(), selected.end());
      return selected;
    }
    return selected;
  }

  GridKey gridKey(const ais_gng_msgs::msg::TopologicalNode &node) const
  {
    return {
      static_cast<std::int64_t>(std::floor(static_cast<double>(node.pos.x) / grid_cell_size_)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(node.pos.y) / grid_cell_size_)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(node.pos.z) / grid_cell_size_))};
  }

  ais_gng_msgs::msg::TopologicalMap buildGraph(
    const ais_gng_msgs::msg::TopologicalMap &environment,
    const std::vector<std::size_t> &selected,
    const Aabb &aabb)
  {
    ais_gng_msgs::msg::TopologicalMap output;
    output.header = environment.header;
    if (!frame_id_override_.empty()) {
      output.header.frame_id = frame_id_override_;
    }
    output.frame_number = environment.frame_number;

    std::unordered_map<GridKey, std::uint16_t, GridKeyHash> grid_to_node_id;
    std::vector<CellAggregate> cells;
    std::vector<std::optional<std::uint16_t>> source_to_output(environment.nodes.size());
    cells.reserve(selected.size());

    for (const std::size_t source_index : selected) {
      const auto &source = environment.nodes[source_index];
      const GridKey key = gridKey(source);
      auto found = grid_to_node_id.find(key);
      if (found == grid_to_node_id.end()) {
        if (cells.size() >= std::numeric_limits<std::uint16_t>::max()) {
          throw std::runtime_error("簡略化グラフのnode数がTopologicalMapのuint16上限を超えました。");
        }
        const auto node_id = static_cast<std::uint16_t>(cells.size());
        found = grid_to_node_id.emplace(key, node_id).first;
        cells.emplace_back();
      }

      const std::uint16_t output_id = found->second;
      source_to_output[source_index] = output_id;
      CellAggregate &cell = cells[output_id];
      const double weight = nodeWeight(source);
      cell.source_indices.push_back(source_index);
      cell.weight_sum += weight;
      cell.position_x_sum += weight * source.pos.x;
      cell.position_y_sum += weight * source.pos.y;
      cell.position_z_sum += weight * source.pos.z;
      cell.normal_x_sum += weight * source.normal.x;
      cell.normal_y_sum += weight * source.normal.y;
      cell.normal_z_sum += weight * source.normal.z;
      cell.rho_sum += weight * source.rho;
      cell.winner_point_count += source.winner_point_count;
      cell.label_weights[source.label] += weight;
    }

    output.nodes.reserve(cells.size());
    for (std::size_t output_index = 0; output_index < cells.size(); ++output_index) {
      const CellAggregate &cell = cells[output_index];
      const double inverse_weight = 1.0 / cell.weight_sum;
      ais_gng_msgs::msg::TopologicalNode node;
      node.id = static_cast<std::uint16_t>(output_index);
      node.pos.x = static_cast<float>(cell.position_x_sum * inverse_weight);
      node.pos.y = static_cast<float>(cell.position_y_sum * inverse_weight);
      node.pos.z = static_cast<float>(cell.position_z_sum * inverse_weight);
      node.normal.x = static_cast<float>(cell.normal_x_sum * inverse_weight);
      node.normal.y = static_cast<float>(cell.normal_y_sum * inverse_weight);
      node.normal.z = static_cast<float>(cell.normal_z_sum * inverse_weight);
      const float normal_length = std::sqrt(
        node.normal.x * node.normal.x + node.normal.y * node.normal.y + node.normal.z * node.normal.z);
      if (normal_length > 1e-6F) {
        node.normal.x /= normal_length;
        node.normal.y /= normal_length;
        node.normal.z /= normal_length;
      } else {
        node.normal.z = 1.0F;
      }
      node.rho = static_cast<float>(cell.rho_sum * inverse_weight);
      node.label = dominantLabel(cell);
      node.winner_point_count = static_cast<std::uint32_t>(std::min<std::uint64_t>(
          cell.winner_point_count, std::numeric_limits<std::uint32_t>::max()));

      std::array<double, 9> covariance_sum{};
      for (const std::size_t source_index : cell.source_indices) {
        const auto &source = environment.nodes[source_index];
        const double weight = nodeWeight(source);
        const std::array<double, 3> delta{
          static_cast<double>(source.pos.x - node.pos.x),
          static_cast<double>(source.pos.y - node.pos.y),
          static_cast<double>(source.pos.z - node.pos.z)};
        for (std::size_t row = 0; row < 3; ++row) {
          for (std::size_t column = 0; column < 3; ++column) {
            const std::size_t covariance_index = row * 3 + column;
            covariance_sum[covariance_index] += weight * (
              source.winner_point_covariance[covariance_index] + delta[row] * delta[column]);
          }
        }
      }
      for (std::size_t covariance_index = 0;
        covariance_index < node.winner_point_covariance.size(); ++covariance_index)
      {
        node.winner_point_covariance[covariance_index] =
          static_cast<float>(covariance_sum[covariance_index] * inverse_weight);
      }
      output.nodes.push_back(std::move(node));
    }

    appendEdges(environment, source_to_output, output);
    appendCluster(output, aabb);
    return output;
  }

  static void appendEdges(
    const ais_gng_msgs::msg::TopologicalMap &environment,
    const std::vector<std::optional<std::uint16_t>> &source_to_output,
    ais_gng_msgs::msg::TopologicalMap &output)
  {
    std::unordered_map<std::uint16_t, std::size_t> id_to_index;
    id_to_index.reserve(environment.nodes.size());
    for (std::size_t index = 0; index < environment.nodes.size(); ++index) {
      id_to_index.emplace(environment.nodes[index].id, index);
    }
    std::unordered_set<std::uint32_t> seen;
    for (std::size_t index = 0; index + 1 < environment.edges.size(); index += 2) {
      const auto first_found = id_to_index.find(environment.edges[index]);
      const auto second_found = id_to_index.find(environment.edges[index + 1]);
      if (first_found == id_to_index.end() || second_found == id_to_index.end()) {
        continue;
      }
      const auto first = source_to_output[first_found->second];
      const auto second = source_to_output[second_found->second];
      if (!first || !second || *first == *second) {
        continue;
      }
      const std::uint16_t lower = std::min(*first, *second);
      const std::uint16_t upper = std::max(*first, *second);
      const std::uint32_t edge_key =
        (static_cast<std::uint32_t>(lower) << 16U) | static_cast<std::uint32_t>(upper);
      if (seen.insert(edge_key).second) {
        output.edges.push_back(lower);
        output.edges.push_back(upper);
      }
    }
  }

  void appendCluster(ais_gng_msgs::msg::TopologicalMap &output, const Aabb &aabb) const
  {
    if (output.nodes.empty()) {
      return;
    }
    ais_gng_msgs::msg::TopologicalCluster cluster;
    cluster.id = 0;
    cluster.label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
    cluster.pos.x = (aabb.min_x + aabb.max_x) * 0.5F;
    cluster.pos.y = (aabb.min_y + aabb.max_y) * 0.5F;
    cluster.pos.z = (aabb.min_z + aabb.max_z) * 0.5F;
    cluster.scale.x = aabb.max_x - aabb.min_x;
    cluster.scale.y = aabb.max_y - aabb.min_y;
    cluster.scale.z = aabb.max_z - aabb.min_z;
    cluster.quat.w = 1.0F;
    cluster.match = static_cast<float>(score_);
    cluster.nodes.reserve(output.nodes.size());
    for (const auto &node : output.nodes) {
      cluster.nodes.push_back(node.id);
    }
    output.clusters.push_back(std::move(cluster));
  }

  visualization_msgs::msg::MarkerArray buildMarkers(
    const ais_gng_msgs::msg::TopologicalMap &graph, const Aabb &aabb) const
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = graph.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(std::move(clear));

    visualization_msgs::msg::Marker box;
    box.header = graph.header;
    box.ns = "object_match_hypothesis_aabb";
    box.id = 0;
    box.type = visualization_msgs::msg::Marker::LINE_LIST;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.scale.x = aabb_line_width_;
    box.color.r = 0.25F;
    box.color.g = 0.85F;
    box.color.b = 1.0F;
    box.color.a = 0.95F;
    const std::array<geometry_msgs::msg::Point, 8> corners{
      makePoint(aabb.min_x, aabb.min_y, aabb.min_z), makePoint(aabb.max_x, aabb.min_y, aabb.min_z),
      makePoint(aabb.max_x, aabb.max_y, aabb.min_z), makePoint(aabb.min_x, aabb.max_y, aabb.min_z),
      makePoint(aabb.min_x, aabb.min_y, aabb.max_z), makePoint(aabb.max_x, aabb.min_y, aabb.max_z),
      makePoint(aabb.max_x, aabb.max_y, aabb.max_z), makePoint(aabb.min_x, aabb.max_y, aabb.max_z)};
    constexpr std::array<std::array<std::size_t, 2>, 12> edge_indices{{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}}, {{4, 5}}, {{5, 6}},
      {{6, 7}}, {{7, 4}}, {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}}}};
    for (const auto &edge : edge_indices) {
      box.points.push_back(corners[edge[0]]);
      box.points.push_back(corners[edge[1]]);
    }
    markers.markers.push_back(std::move(box));

    visualization_msgs::msg::Marker text;
    text.header = graph.header;
    text.ns = "object_match_hypothesis_label";
    text.id = 0;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position = makePoint(
      (aabb.min_x + aabb.max_x) * 0.5F,
      (aabb.min_y + aabb.max_y) * 0.5F,
      aabb.max_z + 0.03F);
    text.pose.orientation.w = 1.0F;
    text.scale.z = 0.04F;
    text.color.r = 0.8F;
    text.color.g = 0.95F;
    text.color.b = 1.0F;
    text.color.a = 1.0F;
    text.text = template_id_ + "/" + hypothesis_id_ + " score=" + std::to_string(score_);
    markers.markers.push_back(std::move(text));
    return markers;
  }

  void publishHypothesis()
  {
    if (!environment_map_ || !template_map_) {
      return;
    }
    const auto selected = selectEnvironmentNodes(*environment_map_);
    if (selected.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "仮説対象の環境nodeがありません: environment_cluster_id=%d", environment_cluster_id_);
      return;
    }

    Aabb aabb;
    for (const std::size_t source_index : selected) {
      aabb.append(environment_map_->nodes[source_index]);
    }
    ais_gng_msgs::msg::TopologicalMap graph = buildGraph(*environment_map_, selected, aabb);
    graph.header.stamp = now();
    graph_publisher_->publish(graph);
    marker_publisher_->publish(buildMarkers(graph, aabb));

    std_msgs::msg::String metadata;
    metadata.data = json{
      {"schema_version", 1},
      {"kind", "object_graph_match_hypothesis"},
      {"state", "placeholder"},
      {"template_id", template_id_},
      {"hypothesis_id", hypothesis_id_},
      {"score", score_},
      {"yaw_deg", yaw_deg_},
      {"environment_topic", environment_topic_},
      {"template_topic", template_topic_},
      {"environment_cluster_id", environment_cluster_id_},
      {"template_node_count", template_map_->nodes.size()},
      {"environment_node_count", environment_map_->nodes.size()},
      {"selected_environment_node_count", selected.size()},
      {"simplified_node_count", graph.nodes.size()},
      {"simplified_edge_count", graph.edges.size() / 2},
      {"grid_cell_size", grid_cell_size_},
      {"aabb", {
        {"min", {aabb.min_x, aabb.min_y, aabb.min_z}},
        {"max", {aabb.max_x, aabb.max_y, aabb.max_z}}}}
    }.dump();
    metadata_publisher_->publish(metadata);

    if (!has_published_) {
      RCLCPP_INFO(
        get_logger(),
        "物体照合仮説配信開始: prefix=%s selected=%zu simplified_nodes=%zu simplified_edges=%zu",
        output_prefix_.c_str(), selected.size(), graph.nodes.size(), graph.edges.size() / 2);
      has_published_ = true;
    }
  }

  std::string template_id_;
  std::string hypothesis_id_;
  std::string environment_topic_;
  std::string template_topic_;
  std::string frame_id_override_;
  std::string output_prefix_;
  int environment_cluster_id_ = -1;
  double grid_cell_size_ = 0.03;
  double score_ = 0.0;
  double yaw_deg_ = 0.0;
  double aabb_line_width_ = 0.004;
  bool has_published_ = false;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr environment_map_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr template_map_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr environment_subscription_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr template_subscription_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr graph_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_sim::bridge::ObjectMatchHypothesisPublisherNode>(
      rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("object_match_hypothesis_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
