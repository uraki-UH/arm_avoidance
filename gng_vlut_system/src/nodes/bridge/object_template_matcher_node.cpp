#include <ais_gng_msgs/msg/topological_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
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

constexpr double kPi = 3.14159265358979323846;

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct TemplateNode
{
  std::uint16_t id = 0;
  Vec3 normal;
  double rho = 0.0;
  std::size_t degree = 0;
};

struct TemplateGraph
{
  std::vector<TemplateNode> nodes;
  std::vector<std::pair<std::size_t, std::size_t>> edges;
  std::vector<std::vector<std::size_t>> neighbors;
  std::unordered_set<std::uint32_t> edge_keys;
  double canonical_yaw_deg = 0.0;
};

struct PairScore
{
  std::size_t environment_index = 0;
  std::size_t template_index = 0;
  double score = 0.0;
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
    graph.nodes.push_back({static_cast<std::uint16_t>(index), normal, node.value("rho", 0.0), 0U});
  }
  graph.neighbors.resize(graph.nodes.size());

  if (!graph_json.contains("edges") || !graph_json.at("edges").is_array()) {
    return graph;
  }
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
    candidate_topic_ = declare_parameter<std::string>("candidate_topic", "");
    enable_yaw_search_ = declare_parameter<bool>("enable_yaw_search", true);
    yaw_min_deg_ = declare_parameter<double>("yaw_min_deg", -180.0);
    yaw_max_deg_ = declare_parameter<double>("yaw_max_deg", 180.0);
    yaw_step_deg_ = declare_parameter<double>("yaw_step_deg", 10.0);
    enable_roll_pitch_search_ = declare_parameter<bool>("enable_roll_pitch_search", false);
    roll_min_deg_ = declare_parameter<double>("roll_min_deg", -8.0);
    roll_max_deg_ = declare_parameter<double>("roll_max_deg", 8.0);
    roll_step_deg_ = declare_parameter<double>("roll_step_deg", 4.0);
    pitch_min_deg_ = declare_parameter<double>("pitch_min_deg", -8.0);
    pitch_max_deg_ = declare_parameter<double>("pitch_max_deg", 8.0);
    pitch_step_deg_ = declare_parameter<double>("pitch_step_deg", 4.0);
    max_orientation_hypothesis_num_ = declare_parameter<int>("max_orientation_hypothesis_num", 1000);
    max_normal_angle_full_deg_ = declare_parameter<double>("max_normal_angle_full_deg", 12.0);
    max_normal_angle_partial_deg_ = declare_parameter<double>("max_normal_angle_partial_deg", 35.0);
    enable_rho_evaluation_ = declare_parameter<bool>("enable_rho_evaluation", true);
    max_rho_dev_full_ratio_ = declare_parameter<double>("max_rho_dev_full_ratio", 0.15);
    max_rho_dev_partial_ratio_ = declare_parameter<double>("max_rho_dev_partial_ratio", 0.45);
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

    if (template_id_.empty() || template_dataset_path_.empty() || environment_topic_.empty()) {
      throw std::runtime_error("template_id、template_dataset_path、environment_topological_map_topicの指定が必要です。");
    }
    if (candidate_topic_.empty()) {
      candidate_topic_ = "/" + template_id_ + "/object_template_match_candidates";
    }
    validateParameters();
    template_graph_ = loadTemplateGraph(template_dataset_path_);
    candidate_publisher_ = create_publisher<std_msgs::msg::String>(
      candidate_topic_, rclcpp::QoS(1).reliable().transient_local());
    environment_subscription_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      environment_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&ObjectTemplateMatcherNode::onEnvironmentMap, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "物体テンプレート照合開始: template=%s nodes=%zu edges=%zu input=%s",
      template_id_.c_str(), template_graph_.nodes.size(), template_graph_.edges.size(),
      environment_topic_.c_str());
  }

private:
  void validateParameters() const
  {
    const std::array<double, 13> values = {
      yaw_min_deg_, yaw_max_deg_, yaw_step_deg_, roll_min_deg_, roll_max_deg_, roll_step_deg_,
      pitch_min_deg_, pitch_max_deg_, pitch_step_deg_, min_node_score_, edge_weight_,
      contradiction_weight_, max_contradiction_point_ratio_};
    if (std::any_of(values.begin(), values.end(), [](double value) {return !isFinite(value);}) ||
      yaw_step_deg_ <= 0.0 || roll_step_deg_ <= 0.0 || pitch_step_deg_ <= 0.0 ||
      yaw_min_deg_ > yaw_max_deg_ || roll_min_deg_ > roll_max_deg_ || pitch_min_deg_ > pitch_max_deg_ ||
      max_orientation_hypothesis_num_ <= 0 || min_node_score_ < 0.0 || min_node_score_ > 1.0 ||
      edge_weight_ < 0.0 || edge_weight_ > 1.0 || normal_weight_ < 0.0 || rho_weight_ < 0.0 ||
      degree_weight_ < 0.0 || contradiction_weight_ < 0.0 || contradiction_weight_ > 1.0 ||
      max_contradiction_point_ratio_ < 0.0 || max_contradiction_point_ratio_ > 1.0)
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

  static std::vector<double> sampleAngles(double min_deg, double max_deg, double step_deg, bool enable)
  {
    if (!enable) {
      return {0.0};
    }
    std::vector<double> samples;
    for (double value = min_deg; value <= max_deg + 1e-9; value += step_deg) {
      samples.push_back(std::min(value, max_deg));
    }
    if (samples.empty() || samples.back() < max_deg - 1e-9) {
      samples.push_back(max_deg);
    }
    return samples;
  }

  double nodeScore(
    const TemplateNode &template_node,
    const ais_gng_msgs::msg::TopologicalNode &environment_node,
    std::size_t environment_degree,
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
    const double normal_score = descendingMembership(
      normal_angle_deg, max_normal_angle_full_deg_, max_normal_angle_partial_deg_);
    const double degree_score = descendingMembership(
      std::abs(static_cast<double>(template_node.degree) - static_cast<double>(environment_degree)),
      max_degree_dev_full_, max_degree_dev_partial_);
    const double environment_rho = static_cast<double>(environment_node.rho);
    const double rho_scale = std::max({std::abs(template_node.rho), std::abs(environment_rho), 1e-4});
    const double rho_score = descendingMembership(
      std::abs(template_node.rho - environment_rho) / rho_scale,
      max_rho_dev_full_ratio_, max_rho_dev_partial_ratio_);
    const double enabled_rho_weight = enable_rho_evaluation_ ? rho_weight_ : 0.0;
    const double weight_sum = normal_weight_ + enabled_rho_weight + degree_weight_;
    if (weight_sum <= 1e-9) {
      return 0.0;
    }
    return (normal_weight_ * normal_score + enabled_rho_weight * rho_score + degree_weight_ * degree_score) /
      weight_sum;
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
    const auto environment_neighbors = buildNeighbors(environment->nodes.size(), environment_edges);
    std::vector<std::size_t> environment_degrees(environment->nodes.size(), 0U);
    for (const auto &[first, second] : environment_edges) {
      ++environment_degrees[first];
      ++environment_degrees[second];
    }

    const auto yaw_samples = sampleAngles(yaw_min_deg_, yaw_max_deg_, yaw_step_deg_, enable_yaw_search_);
    const auto roll_samples = sampleAngles(
      roll_min_deg_, roll_max_deg_, roll_step_deg_, enable_roll_pitch_search_);
    const auto pitch_samples = sampleAngles(
      pitch_min_deg_, pitch_max_deg_, pitch_step_deg_, enable_roll_pitch_search_);
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
            *environment, environment_degrees, environment_edges, environment_neighbors,
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
      {"rejection_reason", "contradictory_point_support"}};
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

  json evaluateOrientation(
    const ais_gng_msgs::msg::TopologicalMap &environment,
    const std::vector<std::size_t> &environment_degrees,
    const std::vector<std::pair<std::size_t, std::size_t>> &environment_edges,
    const std::vector<std::vector<std::size_t>> &environment_neighbors,
    double roll_deg,
    double pitch_deg,
    double yaw_deg) const
  {
    std::vector<PairScore> pairs;
    pairs.reserve(environment.nodes.size() * template_graph_.nodes.size());
    for (std::size_t environment_index = 0; environment_index < environment.nodes.size(); ++environment_index) {
      for (std::size_t template_index = 0; template_index < template_graph_.nodes.size(); ++template_index) {
        const double score = nodeScore(
          template_graph_.nodes[template_index], environment.nodes[environment_index],
          environment_degrees[environment_index], roll_deg, pitch_deg, yaw_deg);
        if (score >= min_node_score_) {
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
    for (const auto &[first, second] : environment_edges) {
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
    for (const auto &[first, second] : environment_edges) {
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
              environment_degrees[environment_index], roll_deg, pitch_deg, yaw_deg) >= min_node_score_)
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
    const double base_score = (1.0 - edge_weight_) * node_score + edge_weight_ * edge_score;
    const double score = enable_contradiction_evaluation_ ?
      std::max(0.0, base_score - contradiction_weight_ * contradiction_point_ratio) : base_score;
    const bool is_falsified = enable_contradiction_evaluation_ &&
      contradiction_point_ratio > max_contradiction_point_ratio_;

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
    return {
      {"template_id", template_id_},
      {"state", "candidate"},
      {"score", score},
      {"base_score", base_score},
      {"roll_deg", roll_deg},
      {"pitch_deg", pitch_deg},
      {"yaw_deg", yaw_deg + template_graph_.canonical_yaw_deg},
      {"matched_node_num", matched_node_num},
      {"matched_node_ratio", static_cast<double>(matched_node_num) / template_graph_.nodes.size()},
      {"missing_node_ratio", 1.0 - static_cast<double>(matched_node_num) / template_graph_.nodes.size()},
      {"matched_edge_num", matched_edge_num},
      {"matched_edge_ratio", supported_edge_num == 0U ? 0.0 : edge_score},
      {"contradiction_node_num", contradiction_indices.size()},
      {"contradiction_edge_num", contradiction_edge_num},
      {"contradiction_point_num", contradiction_point_num},
      {"contradiction_point_ratio", contradiction_point_ratio},
      {"is_falsified", is_falsified},
      {"correspondences", std::move(correspondences)}};
  }

  std::string template_id_;
  std::string template_dataset_path_;
  std::string environment_topic_;
  std::string candidate_topic_;
  bool enable_yaw_search_ = true;
  double yaw_min_deg_ = 0.0;
  double yaw_max_deg_ = 0.0;
  double yaw_step_deg_ = 1.0;
  bool enable_roll_pitch_search_ = false;
  double roll_min_deg_ = 0.0;
  double roll_max_deg_ = 0.0;
  double roll_step_deg_ = 1.0;
  double pitch_min_deg_ = 0.0;
  double pitch_max_deg_ = 0.0;
  double pitch_step_deg_ = 1.0;
  int max_orientation_hypothesis_num_ = 1;
  double max_normal_angle_full_deg_ = 0.0;
  double max_normal_angle_partial_deg_ = 1.0;
  bool enable_rho_evaluation_ = true;
  double max_rho_dev_full_ratio_ = 0.0;
  double max_rho_dev_partial_ratio_ = 1.0;
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
  TemplateGraph template_graph_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_publisher_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr environment_subscription_;
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
