#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
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

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinShapePairScore = 0.55;

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct TemplateNode
{
  std::uint16_t id = 0;
  Vec3 position;
  Vec3 normal;
  double rho = 0.0;
  std::size_t degree = 0;
};

struct TemplatePlaneCluster
{
  std::uint32_t id = 0U;
  Vec3 normal;
  double min_extent = 0.0;
  double max_extent = 0.0;
  std::vector<std::size_t> node_indices;
};

struct TemplateGraph
{
  std::vector<TemplateNode> nodes;
  std::vector<std::pair<std::size_t, std::size_t>> edges;
  std::vector<std::vector<std::size_t>> neighbors;
  std::unordered_set<std::uint32_t> edge_keys;
  std::vector<TemplatePlaneCluster> plane_clusters;
  double canonical_yaw_deg = 0.0;
};

struct PairScore
{
  std::size_t environment_index = 0;
  std::size_t template_index = 0;
  double score = 0.0;
};

struct ScaleEvaluation
{
  bool is_observed = false;
  std::size_t edge_num = 0;
  double ratio = 1.0;
  double score = 1.0;
};

struct PlaneFilterResult
{
  std::unordered_set<std::size_t> ignored_node_indices;
  std::vector<std::uint32_t> ignored_cluster_ids;
};

struct PlaneCorrespondence
{
  std::size_t template_index = 0U;
  std::size_t environment_index = 0U;
  double score = 0.0;
};

struct PlaneEvaluation
{
  bool is_observed = false;
  std::vector<PlaneCorrespondence> correspondences;
  std::vector<int> template_node_to_environment_plane;
  std::unordered_set<std::size_t> matched_environment_node_indices;
  double score = 0.0;
  double matched_ratio = 0.0;
  double support_score = 0.0;
};

bool isFinite(double value)
{
  return std::isfinite(value);
}

double degreeToRad(double value)
{
  return value * kPi / 180.0;
}

double clampUnit(double value)
{
  return std::max(-1.0, std::min(1.0, value));
}

std::array<double, 2> sortedExtents(double first, double second)
{
  return {std::min(first, second), std::max(first, second)};
}

Vec3 normalize(const Vec3 &value)
{
  const double length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  if (length <= 1e-9) {
    return {0.0, 0.0, 1.0};
  }
  return {value.x / length, value.y / length, value.z / length};
}

double dot(const Vec3 &first, const Vec3 &second)
{
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

Vec3 rotateRpy(const Vec3 &value, double roll_deg, double pitch_deg, double yaw_deg)
{
  const double roll = degreeToRad(roll_deg);
  const double pitch = degreeToRad(pitch_deg);
  const double yaw = degreeToRad(yaw_deg);
  const double roll_x = value.x;
  const double roll_y = std::cos(roll) * value.y - std::sin(roll) * value.z;
  const double roll_z = std::sin(roll) * value.y + std::cos(roll) * value.z;
  const double pitch_x = std::cos(pitch) * roll_x + std::sin(pitch) * roll_z;
  const double pitch_y = roll_y;
  const double pitch_z = -std::sin(pitch) * roll_x + std::cos(pitch) * roll_z;
  return normalize({
    std::cos(yaw) * pitch_x - std::sin(yaw) * pitch_y,
    std::sin(yaw) * pitch_x + std::cos(yaw) * pitch_y,
    pitch_z});
}

double descendingMembership(double value, double full_match_max, double partial_match_max)
{
  if (value <= full_match_max) {
    return 1.0;
  }
  if (value >= partial_match_max || partial_match_max <= full_match_max) {
    return 0.0;
  }
  return (partial_match_max - value) / (partial_match_max - full_match_max);
}

double intervalMembership(
  double value, double min_allow, double min_full_match,
  double max_full_match, double max_allow)
{
  if (value >= min_full_match && value <= max_full_match) {
    return 1.0;
  }
  if (value <= min_allow || value >= max_allow || min_allow >= min_full_match ||
    max_full_match >= max_allow)
  {
    return 0.0;
  }
  if (value < min_full_match) {
    return (value - min_allow) / (min_full_match - min_allow);
  }
  return (max_allow - value) / (max_allow - max_full_match);
}

std::uint32_t edgeKey(std::size_t first, std::size_t second)
{
  const auto lower = static_cast<std::uint16_t>(std::min(first, second));
  const auto upper = static_cast<std::uint16_t>(std::max(first, second));
  return (static_cast<std::uint32_t>(lower) << 16U) | static_cast<std::uint32_t>(upper);
}

json readDataset(const std::string &dataset_path)
{
  if (dataset_path.size() >= 3U && dataset_path.substr(dataset_path.size() - 3U) == ".gz") {
    gzFile file = gzopen(dataset_path.c_str(), "rb");
    if (file == nullptr) {
      throw std::runtime_error("テンプレートgzipを開けません: " + dataset_path);
    }
    std::string text;
    std::array<char, 64U * 1024U> buffer{};
    int read_byte_num = 0;
    while ((read_byte_num = gzread(file, buffer.data(), buffer.size())) > 0) {
      text.append(buffer.data(), static_cast<std::size_t>(read_byte_num));
    }
    const bool has_read_error = read_byte_num < 0;
    const int close_result = gzclose(file);
    if (has_read_error || close_result != Z_OK) {
      throw std::runtime_error("テンプレートgzipの読込失敗: " + dataset_path);
    }
    return json::parse(text);
  }
  std::ifstream stream(dataset_path);
  if (!stream) {
    throw std::runtime_error("テンプレートJSONを開けません: " + dataset_path);
  }
  return json::parse(stream);
}

const json &selectTemplate(const json &root)
{
  if (root.value("kind", "") == "object_template") {
    return root;
  }
  if (root.value("kind", "") == "object_surface_dataset" &&
    root.contains("gng_template") && root.at("gng_template").is_object())
  {
    return root.at("gng_template");
  }
  throw std::runtime_error("object_templateまたはGNG同梱済みobject_surface_datasetが必要です。");
}

TemplateGraph loadTemplateGraph(const std::string &dataset_path)
{
  const json root = readDataset(dataset_path);
  const json &template_root = selectTemplate(root);
  if (!template_root.contains("gng") || !template_root.at("gng").is_object()) {
    throw std::runtime_error("テンプレートにgngがありません。");
  }
  const json &graph_json = template_root.at("gng");
  if (!graph_json.contains("nodes") || !graph_json.at("nodes").is_array()) {
    throw std::runtime_error("テンプレートにGNG node配列がありません。");
  }
  const json &nodes_json = graph_json.at("nodes");
  if (nodes_json.empty() || nodes_json.size() > std::numeric_limits<std::uint16_t>::max()) {
    throw std::runtime_error("テンプレートのGNG node数が不正です。");
  }

  TemplateGraph graph;
  graph.canonical_yaw_deg = template_root.value("canonical_yaw_deg", 0.0);
  graph.nodes.reserve(nodes_json.size());
  for (std::size_t index = 0; index < nodes_json.size(); ++index) {
    const json &node = nodes_json.at(index);
    if (!node.is_object()) {
      throw std::runtime_error("テンプレートのGNG node形式が不正です。");
    }
    const Vec3 normal = normalize({
      node.value("nx", 0.0), node.value("ny", 0.0), node.value("nz", 1.0)});
    graph.nodes.push_back({
      static_cast<std::uint16_t>(index),
      {node.value("x", 0.0), node.value("y", 0.0), node.value("z", 0.0)},
      normal,
      node.value("rho", 0.0),
      0U});
  }
  graph.neighbors.resize(graph.nodes.size());

  if (graph_json.contains("edges") && graph_json.at("edges").is_array()) {
    for (const json &edge : graph_json.at("edges")) {
      if (!edge.is_array() || edge.size() < 2U || !edge.at(0).is_number_unsigned() ||
        !edge.at(1).is_number_unsigned())
      {
        continue;
      }
      const std::size_t first = edge.at(0).get<std::size_t>();
      const std::size_t second = edge.at(1).get<std::size_t>();
      if (first >= graph.nodes.size() || second >= graph.nodes.size() || first == second ||
        !graph.edge_keys.insert(edgeKey(first, second)).second)
      {
        continue;
      }
      graph.edges.emplace_back(first, second);
      graph.neighbors[first].push_back(second);
      graph.neighbors[second].push_back(first);
      ++graph.nodes[first].degree;
      ++graph.nodes[second].degree;
    }
  }
  if (graph_json.contains("plane_clusters") && graph_json.at("plane_clusters").is_array()) {
    const json &plane_clusters_json = graph_json.at("plane_clusters");
    graph.plane_clusters.reserve(plane_clusters_json.size());
    for (std::size_t index = 0; index < plane_clusters_json.size(); ++index) {
      const json &plane_cluster = plane_clusters_json.at(index);
      if (!plane_cluster.is_object() || !plane_cluster.contains("normal") ||
        !plane_cluster.at("normal").is_array() || plane_cluster.at("normal").size() < 3U ||
        !plane_cluster.contains("extent") || !plane_cluster.at("extent").is_array() ||
        plane_cluster.at("extent").size() < 2U)
      {
        continue;
      }
      const json &normal = plane_cluster.at("normal");
      const json &extent = plane_cluster.at("extent");
      if (!normal.at(0).is_number() || !normal.at(1).is_number() || !normal.at(2).is_number() ||
        !extent.at(0).is_number() || !extent.at(1).is_number())
      {
        continue;
      }
      const auto extents = sortedExtents(extent.at(0).get<double>(), extent.at(1).get<double>());
      if (!isFinite(extents[0]) || !isFinite(extents[1]) || extents[0] <= 0.0) {
        continue;
      }
      std::vector<std::size_t> node_indices;
      if (plane_cluster.contains("idx") && plane_cluster.at("idx").is_array()) {
        node_indices.reserve(plane_cluster.at("idx").size());
        for (const json &node_index : plane_cluster.at("idx")) {
          if (node_index.is_number_unsigned() && node_index.get<std::size_t>() < graph.nodes.size()) {
            node_indices.push_back(node_index.get<std::size_t>());
          }
        }
      }
      graph.plane_clusters.push_back({
        plane_cluster.value("id", static_cast<std::uint32_t>(index)),
        normalize({normal.at(0).get<double>(), normal.at(1).get<double>(), normal.at(2).get<double>()}),
        extents[0], extents[1], std::move(node_indices)});
    }
  }
  return graph;
}

}

class ObjectTemplateMatcherNode : public rclcpp::Node
{
public:
  explicit ObjectTemplateMatcherNode(const rclcpp::NodeOptions &options)
  : Node("object_template_matcher_node", options)
  {
    template_id_ = declare_parameter<std::string>("template_id", "");
    template_dataset_path_ = declare_parameter<std::string>("template_dataset_path", "");
    environment_topic_ = declare_parameter<std::string>(
      "environment_topological_map_topic", "/topological_map");
    plane_clusters_topic_ = declare_parameter<std::string>("plane_clusters_topic", "/plane_clusters");
    candidate_topic_ = declare_parameter<std::string>("candidate_topic", "");
    shape_tolerance_ = declare_parameter<double>("shape_tolerance", 0.35);
    scale_tolerance_ = declare_parameter<double>("scale_tolerance", 0.30);
    contradiction_limit_ = declare_parameter<double>("contradiction_limit", 0.20);
    yaw_step_deg_ = declare_parameter<double>("yaw_step_deg", 10.0);
    enable_roll_pitch_search_ = declare_parameter<bool>("enable_roll_pitch_search", true);
    roll_tolerance_deg_ = declare_parameter<double>("roll_tolerance_deg", 8.0);
    pitch_tolerance_deg_ = declare_parameter<double>("pitch_tolerance_deg", 8.0);
    roll_pitch_step_deg_ = declare_parameter<double>("roll_pitch_step_deg", 4.0);
    max_orientation_hypothesis_num_ = declare_parameter<int>("max_orientation_hypothesis_num", 1000);
    max_normal_angle_full_deg_ = declare_parameter<double>("max_normal_angle_full_deg", 12.0);
    max_normal_angle_partial_deg_ = declare_parameter<double>("max_normal_angle_partial_deg", 35.0);
    enable_rho_evaluation_ = declare_parameter<bool>("enable_rho_evaluation", true);
    max_rho_dev_full_ratio_ = declare_parameter<double>("max_rho_dev_full_ratio", 0.15);
    max_rho_dev_partial_ratio_ = declare_parameter<double>("max_rho_dev_partial_ratio", 0.45);
    enable_scale_evaluation_ = declare_parameter<bool>("enable_scale_evaluation", false);
    min_scale_allow_ratio_ = declare_parameter<double>("min_scale_allow_ratio", 0.70);
    min_scale_full_match_ratio_ = declare_parameter<double>("min_scale_full_match_ratio", 0.95);
    max_scale_full_match_ratio_ = declare_parameter<double>("max_scale_full_match_ratio", 1.05);
    max_scale_allow_ratio_ = declare_parameter<double>("max_scale_allow_ratio", 1.30);
    scale_weight_ = declare_parameter<double>("scale_weight", 0.20);
    min_scale_edge_num_ = declare_parameter<int>("min_scale_edge_num", 3);
    max_degree_dev_full_ = declare_parameter<double>("max_degree_dev_full", 0.0);
    max_degree_dev_partial_ = declare_parameter<double>("max_degree_dev_partial", 2.0);
    normal_weight_ = declare_parameter<double>("normal_weight", 0.55);
    rho_weight_ = declare_parameter<double>("rho_weight", 0.20);
    degree_weight_ = declare_parameter<double>("degree_weight", 0.25);
    edge_weight_ = declare_parameter<double>("edge_weight", 0.35);
    min_node_score_ = declare_parameter<double>("min_node_score", 0.55);
    enable_contradiction_evaluation_ = declare_parameter<bool>("enable_contradiction_evaluation", true);
    contradiction_weight_ = declare_parameter<double>("contradiction_weight", 0.40);
    max_contradiction_point_ratio_ = declare_parameter<double>("max_contradiction_point_ratio", 0.20);
    enable_oversized_plane_filter_ = declare_parameter<bool>("enable_oversized_plane_filter", true);
    max_plane_normal_angle_deg_ = declare_parameter<double>("max_plane_normal_angle_deg", 35.0);
    max_plane_extent_overflow_ratio_ = declare_parameter<double>(
      "max_plane_extent_overflow_ratio", 1.30);
    max_plane_cluster_frame_lag_ = declare_parameter<int>("max_plane_cluster_frame_lag", 5);
    enable_plane_cluster_evaluation_ = declare_parameter<bool>("enable_plane_cluster_evaluation", true);
    min_plane_extent_allow_ratio_ = declare_parameter<double>("min_plane_extent_allow_ratio", 0.35);
    min_plane_extent_full_match_ratio_ = declare_parameter<double>(
      "min_plane_extent_full_match_ratio", 0.70);
    max_plane_extent_full_match_ratio_ = declare_parameter<double>(
      "max_plane_extent_full_match_ratio", 1.10);
    plane_weight_ = declare_parameter<double>("plane_weight", 0.65);
    plane_support_score_scale_ = declare_parameter<double>("plane_support_score_scale", 1.80);
    min_plane_support_score_ = declare_parameter<double>("min_plane_support_score", 0.60);

    if (template_id_.empty() || template_dataset_path_.empty() || environment_topic_.empty()) {
      throw std::runtime_error("template_id、template_dataset_path、environment_topological_map_topicの指定が必要です。");
    }
    if (candidate_topic_.empty()) {
      candidate_topic_ = "/" + template_id_ + "/object_template_match_candidates";
    }
    validateParameters();
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&ObjectTemplateMatcherNode::onSetParameters, this, std::placeholders::_1));
    template_graph_ = loadTemplateGraph(template_dataset_path_);
    candidate_publisher_ = create_publisher<std_msgs::msg::String>(
      candidate_topic_, rclcpp::QoS(1).reliable().transient_local());
    environment_subscription_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      environment_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&ObjectTemplateMatcherNode::onEnvironmentMap, this, std::placeholders::_1));
    plane_clusters_subscription_ = create_subscription<ais_gng_msgs::msg::PlaneClusterArray>(
      plane_clusters_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&ObjectTemplateMatcherNode::onPlaneClusters, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "物体テンプレート照合開始: template=%s nodes=%zu edges=%zu planes=%zu input=%s",
      template_id_.c_str(), template_graph_.nodes.size(), template_graph_.edges.size(),
      template_graph_.plane_clusters.size(), environment_topic_.c_str());
  }

private:
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    std::vector<std::function<void()>> rollbacks;
    const auto stage = [&rollbacks](auto &field, const auto value) {
        using Field = std::decay_t<decltype(field)>;
        Field *target = &field;
        const Field previous = field;
        const Field next = static_cast<Field>(value);
        if (previous != next) {
          rollbacks.emplace_back([target, previous]() {*target = previous;});
          field = next;
        }
      };

    try {
      for (const auto &parameter : parameters) {
        const std::string &name = parameter.get_name();
        if (name == "shape_tolerance") stage(shape_tolerance_, parameter.as_double());
        else if (name == "scale_tolerance") stage(scale_tolerance_, parameter.as_double());
        else if (name == "contradiction_limit") stage(contradiction_limit_, parameter.as_double());
        else if (name == "yaw_step_deg") stage(yaw_step_deg_, parameter.as_double());
        else if (name == "enable_roll_pitch_search") stage(enable_roll_pitch_search_, parameter.as_bool());
        else if (name == "roll_tolerance_deg") stage(roll_tolerance_deg_, parameter.as_double());
        else if (name == "pitch_tolerance_deg") stage(pitch_tolerance_deg_, parameter.as_double());
        else if (name == "roll_pitch_step_deg") stage(roll_pitch_step_deg_, parameter.as_double());
        else if (name == "max_orientation_hypothesis_num") {
          stage(max_orientation_hypothesis_num_, parameter.as_int());
        } else if (name == "max_normal_angle_full_deg") {
          stage(max_normal_angle_full_deg_, parameter.as_double());
        } else if (name == "max_normal_angle_partial_deg") {
          stage(max_normal_angle_partial_deg_, parameter.as_double());
        } else if (name == "enable_rho_evaluation") stage(enable_rho_evaluation_, parameter.as_bool());
        else if (name == "max_rho_dev_full_ratio") stage(max_rho_dev_full_ratio_, parameter.as_double());
        else if (name == "max_rho_dev_partial_ratio") {
          stage(max_rho_dev_partial_ratio_, parameter.as_double());
        } else if (name == "enable_scale_evaluation") stage(enable_scale_evaluation_, parameter.as_bool());
        else if (name == "min_scale_allow_ratio") stage(min_scale_allow_ratio_, parameter.as_double());
        else if (name == "min_scale_full_match_ratio") {
          stage(min_scale_full_match_ratio_, parameter.as_double());
        } else if (name == "max_scale_full_match_ratio") {
          stage(max_scale_full_match_ratio_, parameter.as_double());
        } else if (name == "max_scale_allow_ratio") stage(max_scale_allow_ratio_, parameter.as_double());
        else if (name == "scale_weight") stage(scale_weight_, parameter.as_double());
        else if (name == "min_scale_edge_num") stage(min_scale_edge_num_, parameter.as_int());
        else if (name == "max_degree_dev_full") stage(max_degree_dev_full_, parameter.as_double());
        else if (name == "max_degree_dev_partial") stage(max_degree_dev_partial_, parameter.as_double());
        else if (name == "normal_weight") stage(normal_weight_, parameter.as_double());
        else if (name == "rho_weight") stage(rho_weight_, parameter.as_double());
        else if (name == "degree_weight") stage(degree_weight_, parameter.as_double());
        else if (name == "edge_weight") stage(edge_weight_, parameter.as_double());
        else if (name == "min_node_score") stage(min_node_score_, parameter.as_double());
        else if (name == "enable_contradiction_evaluation") {
          stage(enable_contradiction_evaluation_, parameter.as_bool());
        } else if (name == "contradiction_weight") stage(contradiction_weight_, parameter.as_double());
        else if (name == "max_contradiction_point_ratio") {
          stage(max_contradiction_point_ratio_, parameter.as_double());
        } else if (name == "enable_oversized_plane_filter") {
          stage(enable_oversized_plane_filter_, parameter.as_bool());
        } else if (name == "max_plane_normal_angle_deg") {
          stage(max_plane_normal_angle_deg_, parameter.as_double());
        } else if (name == "max_plane_extent_overflow_ratio") {
          stage(max_plane_extent_overflow_ratio_, parameter.as_double());
        } else if (name == "max_plane_cluster_frame_lag") {
          stage(max_plane_cluster_frame_lag_, parameter.as_int());
        } else if (name == "enable_plane_cluster_evaluation") {
          stage(enable_plane_cluster_evaluation_, parameter.as_bool());
        } else if (name == "min_plane_extent_allow_ratio") {
          stage(min_plane_extent_allow_ratio_, parameter.as_double());
        } else if (name == "min_plane_extent_full_match_ratio") {
          stage(min_plane_extent_full_match_ratio_, parameter.as_double());
        } else if (name == "max_plane_extent_full_match_ratio") {
          stage(max_plane_extent_full_match_ratio_, parameter.as_double());
        } else if (name == "plane_weight") stage(plane_weight_, parameter.as_double());
        else if (name == "plane_support_score_scale") {
          stage(plane_support_score_scale_, parameter.as_double());
        } else if (name == "min_plane_support_score") {
          stage(min_plane_support_score_, parameter.as_double());
        }
      }
      validateParameters();
      result.successful = true;
      result.reason = "success";
      return result;
    } catch (const std::exception &error) {
      for (auto iterator = rollbacks.rbegin(); iterator != rollbacks.rend(); ++iterator) {
        (*iterator)();
      }
      result.reason = error.what();
      return result;
    }
  }

  void validateParameters() const
  {
    const std::array<double, 24> values = {
      shape_tolerance_, scale_tolerance_, contradiction_limit_, yaw_step_deg_,
      roll_tolerance_deg_, pitch_tolerance_deg_, roll_pitch_step_deg_,
      min_node_score_, edge_weight_,
      contradiction_weight_, max_contradiction_point_ratio_, min_scale_allow_ratio_,
      min_scale_full_match_ratio_, max_scale_full_match_ratio_, max_scale_allow_ratio_, scale_weight_,
      max_plane_normal_angle_deg_, max_plane_extent_overflow_ratio_, min_plane_extent_allow_ratio_,
      min_plane_extent_full_match_ratio_, max_plane_extent_full_match_ratio_, plane_weight_,
      plane_support_score_scale_, min_plane_support_score_};
    if (std::any_of(values.begin(), values.end(), [](double value) {return !isFinite(value);}) ||
      shape_tolerance_ < 0.0 || shape_tolerance_ > 1.0 ||
      scale_tolerance_ < 0.05 || scale_tolerance_ >= 1.0 ||
      contradiction_limit_ < 0.0 || contradiction_limit_ > 1.0 ||
      yaw_step_deg_ <= 0.0 || yaw_step_deg_ > 90.0 ||
      roll_pitch_step_deg_ <= 0.0 || roll_pitch_step_deg_ > 90.0 ||
      roll_tolerance_deg_ < 0.0 || roll_tolerance_deg_ > 90.0 ||
      pitch_tolerance_deg_ < 0.0 || pitch_tolerance_deg_ > 90.0 ||
      max_orientation_hypothesis_num_ <= 0 || min_node_score_ < 0.0 || min_node_score_ > 1.0 ||
      edge_weight_ < 0.0 || edge_weight_ > 1.0 || normal_weight_ < 0.0 || rho_weight_ < 0.0 ||
      degree_weight_ < 0.0 || contradiction_weight_ < 0.0 || contradiction_weight_ > 1.0 ||
      max_contradiction_point_ratio_ < 0.0 || max_contradiction_point_ratio_ > 1.0 ||
      min_scale_allow_ratio_ <= 0.0 || min_scale_allow_ratio_ >= min_scale_full_match_ratio_ ||
      min_scale_full_match_ratio_ > max_scale_full_match_ratio_ ||
      max_scale_full_match_ratio_ >= max_scale_allow_ratio_ || scale_weight_ < 0.0 ||
      scale_weight_ > 1.0 || min_scale_edge_num_ <= 0 || max_plane_normal_angle_deg_ < 0.0 ||
      max_plane_normal_angle_deg_ > 90.0 || max_plane_extent_overflow_ratio_ < 1.0 ||
      max_plane_cluster_frame_lag_ < 0 || min_plane_extent_allow_ratio_ <= 0.0 ||
      min_plane_extent_allow_ratio_ >= min_plane_extent_full_match_ratio_ ||
      min_plane_extent_full_match_ratio_ > max_plane_extent_full_match_ratio_ ||
      max_plane_extent_full_match_ratio_ >= max_plane_extent_overflow_ratio_ || plane_weight_ < 0.0 ||
      plane_weight_ > 1.0 || plane_support_score_scale_ <= 0.0 ||
      min_plane_support_score_ < 0.0 || min_plane_support_score_ > 1.0 ||
      (enable_oversized_plane_filter_ && plane_clusters_topic_.empty()))
    {
      throw std::runtime_error("物体テンプレート照合パラメータが不正です。");
    }
    if (max_normal_angle_full_deg_ < 0.0 ||
      max_normal_angle_full_deg_ >= max_normal_angle_partial_deg_ ||
      max_rho_dev_full_ratio_ < 0.0 ||
      max_rho_dev_full_ratio_ >= max_rho_dev_partial_ratio_ ||
      max_degree_dev_full_ < 0.0 || max_degree_dev_full_ >= max_degree_dev_partial_)
    {
      throw std::runtime_error("ファジー評価範囲が不正です。");
    }
  }

  void onPlaneClusters(const ais_gng_msgs::msg::PlaneClusterArray::SharedPtr plane_clusters)
  {
    latest_plane_clusters_ = plane_clusters;
  }

  static std::vector<double> sampleTolerance(double tolerance_deg, double step_deg, bool enable)
  {
    if (!enable || tolerance_deg <= 1e-9) {
      return {0.0};
    }
    std::vector<double> samples{0.0};
    for (double value = step_deg; value < tolerance_deg - 1e-9; value += step_deg) {
      samples.push_back(-value);
      samples.push_back(value);
    }
    samples.push_back(-tolerance_deg);
    samples.push_back(tolerance_deg);
    std::sort(samples.begin(), samples.end());
    return samples;
  }

  static std::vector<double> sampleFullYaw(double step_deg)
  {
    std::vector<double> samples;
    for (double value = 0.0; value < 360.0 - 1e-9; value += step_deg) {
      samples.push_back(value);
    }
    return samples;
  }

  double nodeScore(
    const TemplateNode &template_node,
    const ais_gng_msgs::msg::TopologicalNode &environment_node,
    double roll_deg,
    double pitch_deg,
    double yaw_deg) const
  {
    const Vec3 environment_normal = normalize({
      environment_node.normal.x, environment_node.normal.y, environment_node.normal.z});
    const Vec3 rotated_normal = rotateRpy(
      template_node.normal, roll_deg, pitch_deg, yaw_deg + template_graph_.canonical_yaw_deg);
    const double normal_angle_deg = std::acos(clampUnit(std::abs(dot(rotated_normal, environment_normal)))) *
      180.0 / kPi;
    const double normal_reject_deg = 10.0 + 70.0 * shape_tolerance_;
    const double normal_score = descendingMembership(
      normal_angle_deg, normal_reject_deg / 3.0, normal_reject_deg);
    const double environment_rho = static_cast<double>(environment_node.rho);
    const double rho_scale = std::max({std::abs(template_node.rho), std::abs(environment_rho), 1e-4});
    const double rho_reject_ratio = 0.10 + shape_tolerance_;
    const double rho_score = descendingMembership(
      std::abs(template_node.rho - environment_rho) / rho_scale,
      rho_reject_ratio / 3.0, rho_reject_ratio);
    return 0.75 * normal_score + 0.25 * rho_score;
  }

  static std::uint64_t pointSupport(const ais_gng_msgs::msg::TopologicalNode &node)
  {
    return node.winner_point_count > 0U ? node.winner_point_count : 1U;
  }

  void onEnvironmentMap(const ais_gng_msgs::msg::TopologicalMap::SharedPtr environment)
  {
    if (environment->nodes.empty()) {
      return;
    }
    const auto id_to_index = buildIdToIndex(*environment);
    const auto environment_edges = buildEnvironmentEdges(*environment, id_to_index);
    const auto yaw_samples = sampleFullYaw(yaw_step_deg_);
    const auto roll_samples = sampleTolerance(
      roll_tolerance_deg_, roll_pitch_step_deg_, roll_tolerance_deg_ > 0.0);
    const auto pitch_samples = sampleTolerance(
      pitch_tolerance_deg_, roll_pitch_step_deg_, pitch_tolerance_deg_ > 0.0);
    if (yaw_samples.size() * roll_samples.size() * pitch_samples.size() >
      static_cast<std::size_t>(max_orientation_hypothesis_num_))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "姿勢仮説数が上限を超えました: %zu", yaw_samples.size() * roll_samples.size() * pitch_samples.size());
      return;
    }

    std::vector<json> candidates;
    candidates.reserve(yaw_samples.size() * roll_samples.size() * pitch_samples.size());
    for (const double roll_deg : roll_samples) {
      for (const double pitch_deg : pitch_samples) {
        for (const double yaw_deg : yaw_samples) {
          candidates.push_back(evaluateOrientation(
            *environment, environment_edges,
            roll_deg, pitch_deg, yaw_deg));
        }
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const json &first, const json &second) {
      return first.at("score").get<double>() > second.at("score").get<double>();
    });
    json best_candidate = {
      {"template_id", template_id_},
      {"state", "no_hypothesis"},
      {"score", 0.0},
      {"is_falsified", true},
      {"rejected_hypothesis_num", candidates.size()},
      {"rejection_reason", "contradiction_or_scale_out_of_range"}};
    std::size_t rejected_hypothesis_num = 0U;
    bool has_selected_candidate = false;
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (candidates[index].value("is_falsified", false)) {
        ++rejected_hypothesis_num;
        continue;
      }
      best_candidate = std::move(candidates[index]);
      best_candidate["candidate_rank"] = index + 1U;
      best_candidate["rejected_hypothesis_num"] = rejected_hypothesis_num;
      has_selected_candidate = true;
      break;
    }
    if (!has_selected_candidate && !candidates.empty()) {
      const json &best_rejected_candidate = candidates.front();
      best_candidate["base_score"] = best_rejected_candidate.value("base_score", 0.0);
      best_candidate["contradiction_node_num"] =
        best_rejected_candidate.value("contradiction_node_num", 0U);
      best_candidate["contradiction_edge_num"] =
        best_rejected_candidate.value("contradiction_edge_num", 0U);
      best_candidate["contradiction_point_num"] =
        best_rejected_candidate.value("contradiction_point_num", 0U);
      best_candidate["contradiction_point_ratio"] =
        best_rejected_candidate.value("contradiction_point_ratio", 1.0);
      best_candidate["ignored_plane_node_num"] =
        best_rejected_candidate.value("ignored_plane_node_num", 0U);
      best_candidate["ignored_plane_cluster_ids"] =
        best_rejected_candidate.value("ignored_plane_cluster_ids", json::array());
    }
    std_msgs::msg::String message;
    message.data = best_candidate.dump();
    candidate_publisher_->publish(message);
  }

  static std::unordered_map<std::uint16_t, std::size_t> buildIdToIndex(
    const ais_gng_msgs::msg::TopologicalMap &map)
  {
    std::unordered_map<std::uint16_t, std::size_t> result;
    result.reserve(map.nodes.size());
    for (std::size_t index = 0; index < map.nodes.size(); ++index) {
      result.emplace(map.nodes[index].id, index);
    }
    return result;
  }

  static std::vector<std::pair<std::size_t, std::size_t>> buildEnvironmentEdges(
    const ais_gng_msgs::msg::TopologicalMap &map,
    const std::unordered_map<std::uint16_t, std::size_t> &id_to_index)
  {
    std::unordered_set<std::uint32_t> seen;
    std::vector<std::pair<std::size_t, std::size_t>> result;
    for (std::size_t index = 0; index + 1U < map.edges.size(); index += 2U) {
      const auto first = id_to_index.find(map.edges[index]);
      const auto second = id_to_index.find(map.edges[index + 1U]);
      if (first == id_to_index.end() || second == id_to_index.end() || first->second == second->second ||
        !seen.insert(edgeKey(first->second, second->second)).second)
      {
        continue;
      }
      result.emplace_back(first->second, second->second);
    }
    return result;
  }

  static std::vector<std::vector<std::size_t>> buildNeighbors(
    std::size_t node_num,
    const std::vector<std::pair<std::size_t, std::size_t>> &edges)
  {
    std::vector<std::vector<std::size_t>> neighbors(node_num);
    for (const auto &[first, second] : edges) {
      neighbors[first].push_back(second);
      neighbors[second].push_back(first);
    }
    return neighbors;
  }

  static std::vector<std::pair<std::size_t, std::size_t>> filterEnvironmentEdges(
    const std::vector<std::pair<std::size_t, std::size_t>> &environment_edges,
    const std::unordered_set<std::size_t> &ignored_node_indices)
  {
    std::vector<std::pair<std::size_t, std::size_t>> result;
    result.reserve(environment_edges.size());
    for (const auto &[first, second] : environment_edges) {
      if (ignored_node_indices.count(first) > 0U || ignored_node_indices.count(second) > 0U) {
        continue;
      }
      result.emplace_back(first, second);
    }
    return result;
  }

  PlaneFilterResult collectOversizedPlaneFilter(
    const ais_gng_msgs::msg::TopologicalMap &environment,
    double roll_deg,
    double pitch_deg,
    double yaw_deg) const
  {
    PlaneFilterResult result;
    if (!enable_oversized_plane_filter_ || template_graph_.plane_clusters.empty() ||
      latest_plane_clusters_ == nullptr)
    {
      return result;
    }
    const std::uint32_t environment_frame = environment.frame_number;
    const std::uint32_t plane_frame = latest_plane_clusters_->frame_number;
    const std::uint32_t frame_lag = environment_frame >= plane_frame ?
      environment_frame - plane_frame : plane_frame - environment_frame;
    if (frame_lag > static_cast<std::uint32_t>(max_plane_cluster_frame_lag_)) {
      return result;
    }
    for (const auto &environment_plane : latest_plane_clusters_->clusters) {
      const auto environment_extents = sortedExtents(
        static_cast<double>(environment_plane.extent_u), static_cast<double>(environment_plane.extent_v));
      if (!isFinite(environment_extents[0]) || !isFinite(environment_extents[1]) ||
        environment_extents[0] <= 0.0)
      {
        continue;
      }
      const Vec3 environment_normal = normalize({
        environment_plane.normal.x, environment_plane.normal.y, environment_plane.normal.z});
      bool has_normal_relation = false;
      bool has_extent_relation = false;
      for (const auto &template_plane : template_graph_.plane_clusters) {
        const Vec3 rotated_normal = rotateRpy(
          template_plane.normal, roll_deg, pitch_deg, yaw_deg + template_graph_.canonical_yaw_deg);
        const double normal_angle_deg = std::acos(
          clampUnit(std::abs(dot(rotated_normal, environment_normal)))) * 180.0 / kPi;
        if (normal_angle_deg > max_plane_normal_angle_deg_) {
          continue;
        }
        has_normal_relation = true;
        if (environment_extents[0] <= template_plane.min_extent * max_plane_extent_overflow_ratio_ &&
          environment_extents[1] <= template_plane.max_extent * max_plane_extent_overflow_ratio_)
        {
          has_extent_relation = true;
          break;
        }
      }
      if (!has_normal_relation || has_extent_relation) {
        continue;
      }
      result.ignored_cluster_ids.push_back(environment_plane.id);
      for (const std::uint32_t node_index : environment_plane.node_indices) {
        if (node_index < environment.nodes.size()) {
          result.ignored_node_indices.insert(static_cast<std::size_t>(node_index));
        }
      }
    }
    return result;
  }

  PlaneEvaluation evaluatePlaneClusters(
    const ais_gng_msgs::msg::TopologicalMap &environment,
    const PlaneFilterResult &plane_filter,
    double roll_deg,
    double pitch_deg,
    double yaw_deg) const
  {
    PlaneEvaluation result;
    result.template_node_to_environment_plane.assign(template_graph_.nodes.size(), -1);
    if (!enable_plane_cluster_evaluation_ || template_graph_.plane_clusters.empty() ||
      latest_plane_clusters_ == nullptr)
    {
      return result;
    }
    const std::uint32_t environment_frame = environment.frame_number;
    const std::uint32_t plane_frame = latest_plane_clusters_->frame_number;
    const std::uint32_t frame_lag = environment_frame >= plane_frame ?
      environment_frame - plane_frame : plane_frame - environment_frame;
    if (frame_lag > static_cast<std::uint32_t>(max_plane_cluster_frame_lag_)) {
      return result;
    }
    result.is_observed = true;
    std::vector<PairScore> candidates;
    for (std::size_t template_index = 0; template_index < template_graph_.plane_clusters.size(); ++template_index) {
      const auto &template_plane = template_graph_.plane_clusters[template_index];
      const Vec3 rotated_normal = rotateRpy(
        template_plane.normal, roll_deg, pitch_deg, yaw_deg + template_graph_.canonical_yaw_deg);
      for (std::size_t environment_index = 0;
        environment_index < latest_plane_clusters_->clusters.size(); ++environment_index)
      {
        const auto &environment_plane = latest_plane_clusters_->clusters[environment_index];
        if (std::find(
            plane_filter.ignored_cluster_ids.begin(), plane_filter.ignored_cluster_ids.end(),
            environment_plane.id) != plane_filter.ignored_cluster_ids.end())
        {
          continue;
        }
        const auto environment_extents = sortedExtents(
          static_cast<double>(environment_plane.extent_u), static_cast<double>(environment_plane.extent_v));
        if (environment_extents[0] <= 0.0 || environment_extents[1] <= 0.0) {
          continue;
        }
        const Vec3 environment_normal = normalize({
          environment_plane.normal.x, environment_plane.normal.y, environment_plane.normal.z});
        const double normal_angle_deg = std::acos(
          clampUnit(std::abs(dot(rotated_normal, environment_normal)))) * 180.0 / kPi;
        const double normal_score = descendingMembership(
          normal_angle_deg, max_normal_angle_full_deg_, max_plane_normal_angle_deg_);
        const double min_extent_score = intervalMembership(
          environment_extents[0] / template_plane.min_extent, min_plane_extent_allow_ratio_,
          min_plane_extent_full_match_ratio_, max_plane_extent_full_match_ratio_,
          max_plane_extent_overflow_ratio_);
        const double max_extent_score = intervalMembership(
          environment_extents[1] / template_plane.max_extent, min_plane_extent_allow_ratio_,
          min_plane_extent_full_match_ratio_, max_plane_extent_full_match_ratio_,
          max_plane_extent_overflow_ratio_);
        const double score = 0.70 * normal_score + 0.15 * min_extent_score + 0.15 * max_extent_score;
        if (score > 0.0) {
          candidates.push_back({environment_index, template_index, score});
        }
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const PairScore &first, const PairScore &second) {
      return first.score > second.score;
    });
    std::vector<bool> used_template(template_graph_.plane_clusters.size(), false);
    std::vector<bool> used_environment(latest_plane_clusters_->clusters.size(), false);
    double score_sum = 0.0;
    for (const PairScore &candidate : candidates) {
      if (used_template[candidate.template_index] || used_environment[candidate.environment_index]) {
        continue;
      }
      used_template[candidate.template_index] = true;
      used_environment[candidate.environment_index] = true;
      result.correspondences.push_back({
        candidate.template_index, candidate.environment_index, candidate.score});
      score_sum += candidate.score;
    }
    if (result.correspondences.empty()) {
      return result;
    }
    result.score = score_sum / static_cast<double>(result.correspondences.size());
    result.matched_ratio = static_cast<double>(result.correspondences.size()) /
      static_cast<double>(template_graph_.plane_clusters.size());
    result.support_score = 1.0 - std::exp(-score_sum / plane_support_score_scale_);
    for (const PlaneCorrespondence &correspondence : result.correspondences) {
      const auto &template_plane = template_graph_.plane_clusters[correspondence.template_index];
      const auto &environment_plane = latest_plane_clusters_->clusters[correspondence.environment_index];
      for (const std::size_t node_index : template_plane.node_indices) {
        if (result.template_node_to_environment_plane[node_index] < 0) {
          result.template_node_to_environment_plane[node_index] =
            static_cast<int>(correspondence.environment_index);
        }
      }
      for (const std::uint32_t node_index : environment_plane.node_indices) {
        if (node_index < environment.nodes.size()) {
          result.matched_environment_node_indices.insert(static_cast<std::size_t>(node_index));
        }
      }
    }
    return result;
  }

  ScaleEvaluation evaluateScale(
    const ais_gng_msgs::msg::TopologicalMap &environment,
    const std::vector<int> &template_to_environment,
    const std::unordered_set<std::uint32_t> &environment_edge_keys) const
  {
    std::vector<double> ratios;
    ratios.reserve(template_graph_.edges.size());
    for (const auto &[template_first, template_second] : template_graph_.edges) {
      const int environment_first = template_to_environment[template_first];
      const int environment_second = template_to_environment[template_second];
      if (environment_first < 0 || environment_second < 0) {
        continue;
      }
      if (environment_edge_keys.count(edgeKey(
          static_cast<std::size_t>(environment_first),
          static_cast<std::size_t>(environment_second))) == 0U)
      {
        continue;
      }
      const Vec3 &template_first_position = template_graph_.nodes[template_first].position;
      const Vec3 &template_second_position = template_graph_.nodes[template_second].position;
      const double template_dx = template_first_position.x - template_second_position.x;
      const double template_dy = template_first_position.y - template_second_position.y;
      const double template_dz = template_first_position.z - template_second_position.z;
      const double template_length = std::sqrt(
        template_dx * template_dx + template_dy * template_dy + template_dz * template_dz);
      if (template_length <= 1e-6) {
        continue;
      }
      const auto &environment_first_position = environment.nodes[environment_first].pos;
      const auto &environment_second_position = environment.nodes[environment_second].pos;
      const double environment_dx = static_cast<double>(environment_first_position.x) -
        static_cast<double>(environment_second_position.x);
      const double environment_dy = static_cast<double>(environment_first_position.y) -
        static_cast<double>(environment_second_position.y);
      const double environment_dz = static_cast<double>(environment_first_position.z) -
        static_cast<double>(environment_second_position.z);
      const double environment_length = std::sqrt(
        environment_dx * environment_dx + environment_dy * environment_dy +
        environment_dz * environment_dz);
      ratios.push_back(environment_length / template_length);
    }
    if (ratios.size() < static_cast<std::size_t>(min_scale_edge_num_)) {
      return {};
    }
    std::sort(ratios.begin(), ratios.end());
    const std::size_t middle = ratios.size() / 2U;
    const double ratio = ratios.size() % 2U == 0U ?
      (ratios[middle - 1U] + ratios[middle]) * 0.5 : ratios[middle];
    return {
      true,
      ratios.size(),
      ratio,
      intervalMembership(
        ratio, 1.0 - scale_tolerance_, 0.95,
        1.05, 1.0 + scale_tolerance_)};
  }

  json evaluateOrientation(
    const ais_gng_msgs::msg::TopologicalMap &environment,
    const std::vector<std::pair<std::size_t, std::size_t>> &environment_edges,
    double roll_deg,
    double pitch_deg,
    double yaw_deg) const
  {
    const PlaneFilterResult plane_filter = collectOversizedPlaneFilter(
      environment, roll_deg, pitch_deg, yaw_deg);
    const auto filtered_environment_edges = filterEnvironmentEdges(
      environment_edges, plane_filter.ignored_node_indices);
    const auto environment_neighbors = buildNeighbors(environment.nodes.size(), filtered_environment_edges);
    const PlaneEvaluation plane_evaluation = evaluatePlaneClusters(
      environment, plane_filter, roll_deg, pitch_deg, yaw_deg);
    std::vector<PairScore> pairs;
    pairs.reserve(environment.nodes.size() * template_graph_.nodes.size());
    for (std::size_t environment_index = 0; environment_index < environment.nodes.size(); ++environment_index) {
      if (plane_filter.ignored_node_indices.count(environment_index) > 0U) {
        continue;
      }
      for (std::size_t template_index = 0; template_index < template_graph_.nodes.size(); ++template_index) {
        const int expected_environment_plane =
          plane_evaluation.template_node_to_environment_plane[template_index];
        if (expected_environment_plane >= 0) {
          const auto &expected_plane = latest_plane_clusters_->clusters[
            static_cast<std::size_t>(expected_environment_plane)];
          if (std::find(
              expected_plane.node_indices.begin(), expected_plane.node_indices.end(),
              static_cast<std::uint32_t>(environment_index)) == expected_plane.node_indices.end())
          {
            continue;
          }
        } else if (!plane_evaluation.correspondences.empty() &&
          plane_evaluation.matched_environment_node_indices.count(environment_index) > 0U)
        {
          continue;
        }
        const double score = nodeScore(
          template_graph_.nodes[template_index], environment.nodes[environment_index],
          roll_deg, pitch_deg, yaw_deg);
        if (score >= kMinShapePairScore) {
          pairs.push_back({environment_index, template_index, score});
        }
      }
    }
    std::sort(pairs.begin(), pairs.end(), [](const PairScore &first, const PairScore &second) {
      return first.score > second.score;
    });
    std::vector<int> environment_to_template(environment.nodes.size(), -1);
    std::vector<int> template_to_environment(template_graph_.nodes.size(), -1);
    double node_score_sum = 0.0;
    for (const PairScore &pair : pairs) {
      if (environment_to_template[pair.environment_index] >= 0 ||
        template_to_environment[pair.template_index] >= 0)
      {
        continue;
      }
      environment_to_template[pair.environment_index] = static_cast<int>(pair.template_index);
      template_to_environment[pair.template_index] = static_cast<int>(pair.environment_index);
      node_score_sum += pair.score;
    }
    const std::size_t matched_node_num = static_cast<std::size_t>(std::count_if(
      template_to_environment.begin(), template_to_environment.end(), [](int value) {return value >= 0;}));
    const double node_score = matched_node_num == 0U ? 0.0 : node_score_sum / matched_node_num;

    std::unordered_set<std::uint32_t> environment_edge_keys;
    for (const auto &[first, second] : filtered_environment_edges) {
      environment_edge_keys.insert(edgeKey(first, second));
    }
    std::size_t supported_edge_num = 0U;
    std::size_t matched_edge_num = 0U;
    for (const auto &[first, second] : template_graph_.edges) {
      if (template_to_environment[first] < 0 || template_to_environment[second] < 0) {
        continue;
      }
      ++supported_edge_num;
      if (environment_edge_keys.count(edgeKey(
          static_cast<std::size_t>(template_to_environment[first]),
          static_cast<std::size_t>(template_to_environment[second]))) > 0U)
      {
        ++matched_edge_num;
      }
    }
    const double edge_score = supported_edge_num == 0U ? 0.0 :
      static_cast<double>(matched_edge_num) / supported_edge_num;

    std::unordered_set<std::size_t> contradiction_indices;
    std::size_t contradiction_edge_num = 0U;
    for (const auto &[first, second] : filtered_environment_edges) {
      const int first_template_index = environment_to_template[first];
      const int second_template_index = environment_to_template[second];
      if (first_template_index >= 0 && second_template_index >= 0 &&
        template_graph_.edge_keys.count(edgeKey(
          static_cast<std::size_t>(first_template_index),
          static_cast<std::size_t>(second_template_index))) == 0U)
      {
        ++contradiction_edge_num;
      }
    }
    for (std::size_t environment_index = 0; environment_index < environment.nodes.size(); ++environment_index) {
      if (plane_filter.ignored_node_indices.count(environment_index) > 0U) {
        continue;
      }
      if (!plane_evaluation.correspondences.empty() &&
        plane_evaluation.matched_environment_node_indices.count(environment_index) > 0U)
      {
        continue;
      }
      if (environment_to_template[environment_index] >= 0) {
        continue;
      }
      bool has_matched_neighbor = false;
      bool is_explainable = false;
      for (const std::size_t neighbor_index : environment_neighbors[environment_index]) {
        const int anchor_template_index = environment_to_template[neighbor_index];
        if (anchor_template_index < 0) {
          continue;
        }
        has_matched_neighbor = true;
        for (const std::size_t template_neighbor_index :
          template_graph_.neighbors[static_cast<std::size_t>(anchor_template_index)])
        {
          if (template_to_environment[template_neighbor_index] >= 0) {
            continue;
          }
          if (nodeScore(
              template_graph_.nodes[template_neighbor_index], environment.nodes[environment_index],
              roll_deg, pitch_deg, yaw_deg) >= kMinShapePairScore)
          {
            is_explainable = true;
            break;
          }
        }
        if (is_explainable) {
          break;
        }
      }
      if (has_matched_neighbor && !is_explainable) {
        contradiction_indices.insert(environment_index);
      }
    }
    std::uint64_t matched_point_num = 0U;
    for (const int environment_index : template_to_environment) {
      if (environment_index >= 0) {
        matched_point_num += pointSupport(environment.nodes[static_cast<std::size_t>(environment_index)]);
      }
    }
    std::uint64_t contradiction_point_num = 0U;
    for (const std::size_t environment_index : contradiction_indices) {
      contradiction_point_num += pointSupport(environment.nodes[environment_index]);
    }
    const double contradiction_point_ratio = matched_point_num + contradiction_point_num == 0U ? 1.0 :
      static_cast<double>(contradiction_point_num) /
      static_cast<double>(matched_point_num + contradiction_point_num);
    const ScaleEvaluation scale_evaluation =
      evaluateScale(environment, template_to_environment, environment_edge_keys);
    const double visible_ratio = static_cast<double>(matched_node_num) / template_graph_.nodes.size();
    const bool has_edge_evidence = supported_edge_num > 0U;
    const bool has_plane_evidence = !plane_evaluation.correspondences.empty();
    const double relation_score = std::max(
      has_edge_evidence ? edge_score : 0.0,
      has_plane_evidence ? plane_evaluation.support_score : 0.0);
    const double score = std::cbrt(std::max(0.0, node_score * relation_score * visible_ratio));
    const bool has_strong_plane_evidence = plane_evaluation.support_score >= min_plane_support_score_;
    const bool is_contradiction_falsified = contradiction_point_ratio > contradiction_limit_;
    const bool is_scale_falsified = scale_evaluation.is_observed && scale_evaluation.score <= 0.0;
    const bool is_falsified = is_contradiction_falsified || is_scale_falsified;

    json correspondences = json::array();
    for (std::size_t template_index = 0; template_index < template_to_environment.size(); ++template_index) {
      const int environment_index = template_to_environment[template_index];
      if (environment_index < 0) {
        continue;
      }
      correspondences.push_back({
        {"template_node_id", template_graph_.nodes[template_index].id},
        {"environment_node_id", environment.nodes[static_cast<std::size_t>(environment_index)].id}});
    }
    json plane_correspondences = json::array();
    for (const PlaneCorrespondence &correspondence : plane_evaluation.correspondences) {
      plane_correspondences.push_back({
        {"template_plane_cluster_id", template_graph_.plane_clusters[correspondence.template_index].id},
        {"environment_plane_cluster_id",
          latest_plane_clusters_->clusters[correspondence.environment_index].id},
        {"score", correspondence.score}});
    }
    return {
      {"template_id", template_id_},
      {"state", "candidate"},
      {"score", score},
      {"base_score", score},
      {"shape_score", node_score},
      {"relation_score", relation_score},
      {"visible_ratio", visible_ratio},
      {"roll_deg", roll_deg},
      {"pitch_deg", pitch_deg},
      {"yaw_deg", yaw_deg + template_graph_.canonical_yaw_deg},
      {"matched_node_num", matched_node_num},
      {"matched_node_ratio", static_cast<double>(matched_node_num) / template_graph_.nodes.size()},
      {"missing_node_ratio", 1.0 - static_cast<double>(matched_node_num) / template_graph_.nodes.size()},
      {"matched_edge_num", matched_edge_num},
      {"matched_edge_ratio", supported_edge_num == 0U ? 0.0 : edge_score},
      {"is_plane_cluster_observed", plane_evaluation.is_observed},
      {"matched_plane_cluster_num", plane_evaluation.correspondences.size()},
      {"matched_plane_cluster_ratio", plane_evaluation.matched_ratio},
      {"plane_score", plane_evaluation.correspondences.empty() ? 0.0 : plane_evaluation.score},
      {"plane_support_score", plane_evaluation.support_score},
      {"has_strong_plane_evidence", has_strong_plane_evidence},
      {"is_scale_observed", scale_evaluation.is_observed},
      {"scale_edge_num", scale_evaluation.edge_num},
      {"scale_ratio", scale_evaluation.is_observed ? json(scale_evaluation.ratio) : json(nullptr)},
      {"scale_score", scale_evaluation.is_observed ? json(scale_evaluation.score) : json(nullptr)},
      {"is_scale_falsified", is_scale_falsified},
      {"contradiction_node_num", contradiction_indices.size()},
      {"contradiction_edge_num", contradiction_edge_num},
      {"contradiction_point_num", contradiction_point_num},
      {"contradiction_point_ratio", contradiction_point_ratio},
      {"ignored_plane_node_num", plane_filter.ignored_node_indices.size()},
      {"ignored_plane_cluster_ids", plane_filter.ignored_cluster_ids},
      {"is_falsified", is_falsified},
      {"plane_correspondences", std::move(plane_correspondences)},
      {"correspondences", std::move(correspondences)}};
  }

  std::string template_id_;
  std::string template_dataset_path_;
  std::string environment_topic_;
  std::string plane_clusters_topic_;
  std::string candidate_topic_;
  double shape_tolerance_ = 0.35;
  double scale_tolerance_ = 0.30;
  double contradiction_limit_ = 0.20;
  double yaw_step_deg_ = 1.0;
  bool enable_roll_pitch_search_ = false;
  double roll_tolerance_deg_ = 0.0;
  double pitch_tolerance_deg_ = 0.0;
  double roll_pitch_step_deg_ = 1.0;
  int max_orientation_hypothesis_num_ = 1;
  double max_normal_angle_full_deg_ = 0.0;
  double max_normal_angle_partial_deg_ = 1.0;
  bool enable_rho_evaluation_ = true;
  double max_rho_dev_full_ratio_ = 0.0;
  double max_rho_dev_partial_ratio_ = 1.0;
  bool enable_scale_evaluation_ = false;
  double min_scale_allow_ratio_ = 0.0;
  double min_scale_full_match_ratio_ = 0.0;
  double max_scale_full_match_ratio_ = 0.0;
  double max_scale_allow_ratio_ = 0.0;
  double scale_weight_ = 0.0;
  int min_scale_edge_num_ = 0;
  double max_degree_dev_full_ = 0.0;
  double max_degree_dev_partial_ = 1.0;
  double normal_weight_ = 1.0;
  double rho_weight_ = 0.0;
  double degree_weight_ = 0.0;
  double edge_weight_ = 0.0;
  double min_node_score_ = 0.0;
  bool enable_contradiction_evaluation_ = true;
  double contradiction_weight_ = 0.0;
  double max_contradiction_point_ratio_ = 1.0;
  bool enable_oversized_plane_filter_ = true;
  double max_plane_normal_angle_deg_ = 0.0;
  double max_plane_extent_overflow_ratio_ = 1.0;
  int max_plane_cluster_frame_lag_ = 0;
  bool enable_plane_cluster_evaluation_ = true;
  double min_plane_extent_allow_ratio_ = 0.0;
  double min_plane_extent_full_match_ratio_ = 0.0;
  double max_plane_extent_full_match_ratio_ = 0.0;
  double plane_weight_ = 0.0;
  double plane_support_score_scale_ = 1.0;
  double min_plane_support_score_ = 1.0;
  TemplateGraph template_graph_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_publisher_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr environment_subscription_;
  rclcpp::Subscription<ais_gng_msgs::msg::PlaneClusterArray>::SharedPtr plane_clusters_subscription_;
  ais_gng_msgs::msg::PlaneClusterArray::SharedPtr latest_plane_clusters_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_sim::bridge::ObjectTemplateMatcherNode>(rclcpp::NodeOptions()));
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("object_template_matcher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
