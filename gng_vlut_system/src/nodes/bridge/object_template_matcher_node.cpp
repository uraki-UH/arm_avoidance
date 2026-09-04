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
#include <map>
#include <memory>
#include <queue>
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
constexpr std::size_t kMaxRefinementTemplateNum = 8U;
constexpr std::size_t kMaxRetrievalCandidateNum = 8U;
constexpr std::size_t kMaxTemplatesPerRetrievalKey = 4U;
constexpr std::size_t kMaxPlaneTokenNumPerProbe = 24U;
constexpr std::size_t kMaxNonplaneRetrievalNodeNum = 64U;
constexpr std::size_t kMaxRetrievalNodeNum = 128U;
constexpr std::size_t kMaxPendingSynchronizedFrameNum = 64U;
constexpr int kNormalZBinNum = 12;
constexpr int kRhoBinNum = 10;
constexpr int kNeighborNormalBinNum = 10;
constexpr double kPlaneScaleProbeRatio = 1.5;

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
};

struct TemplatePlaneCluster
{
  std::uint32_t id = 0U;
  Vec3 normal;
  Vec3 centroid;
  bool has_centroid = false;
  double min_extent = 0.0;
  double max_extent = 0.0;
  std::vector<std::size_t> node_indices;
};

struct NonplaneComponent
{
  std::uint32_t id = 0U;
  std::vector<std::size_t> node_indices;
  std::unordered_set<std::size_t> anchor_plane_indices;
  std::size_t internal_edge_num = 0U;
};

struct TemplateGraph
{
  std::vector<TemplateNode> nodes;
  std::vector<std::pair<std::size_t, std::size_t>> edges;
  std::vector<std::vector<std::size_t>> neighbors;
  std::unordered_set<std::uint32_t> edge_keys;
  std::vector<TemplatePlaneCluster> plane_clusters;
  std::vector<NonplaneComponent> nonplane_components;
  double canonical_yaw_deg = 0.0;
  double roll_tolerance_deg = -1.0;
  double pitch_tolerance_deg = -1.0;
};

struct LoadedTemplate
{
  std::string id;
  TemplateGraph graph;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_publisher;
};

struct RetrievalCandidate
{
  std::size_t template_index = 0U;
  double score = 0.0;
};

struct RetrievalDescriptor
{
  int normal_z_bin = 0;
  int rho_bin = 0;
  int neighbor_normal_bin = 0;
};

struct RetrievalProbe
{
  std::uint64_t key = 0U;
  double score = 0.0;
};

struct PlaneRetrievalToken
{
  std::size_t template_index = 0U;
  double min_extent = 0.0;
  double max_extent = 0.0;
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
  bool is_relative_height_observed = false;
  std::size_t relative_height_pair_num = 0U;
  double relative_height_score = 0.0;
};

struct NonplaneEvaluation
{
  bool is_observed = false;
  std::size_t environment_component_num = 0U;
  double evidence_score = 0.0;
  double edge_fit_score = 0.0;
  json correspondences = json::array();
};

struct NonplaneEdgeFeature
{
  Vec3 direction;
  Vec3 midpoint;
  double length = 0.0;
  double normalized_length = 0.0;
  double normal_alignment = 0.0;
  double first_plane_dist = 0.0;
  double second_plane_dist = 0.0;
  double plane_scale = 0.0;
  bool has_plane_distances = false;
};

struct NonplaneEdgeFit
{
  double score = 0.0;
  std::size_t edge_num = 0U;
  std::size_t explained_edge_num = 0U;
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

int quantizeUnit(const double value, const int bin_num)
{
  const double bounded_value = std::max(0.0, std::min(1.0, value));
  return std::min(
    bin_num - 1,
    static_cast<int>(std::floor(bounded_value * static_cast<double>(bin_num))));
}

int quantizeRho(const double rho)
{
  return quantizeUnit(
    isFinite(rho) ? std::abs(rho) / (kPi * 0.5) : 0.0,
    kRhoBinNum);
}

std::uint64_t makeRetrievalKey(
  const std::uint8_t type, const int normal_z_bin, const int rho_bin,
  const int neighbor_normal_bin)
{
  return (static_cast<std::uint64_t>(type) << 24U) |
    (static_cast<std::uint64_t>(normal_z_bin) << 16U) |
    (static_cast<std::uint64_t>(rho_bin) << 8U) |
    static_cast<std::uint64_t>(neighbor_normal_bin);
}

RetrievalDescriptor makeRetrievalDescriptor(
  const Vec3 &normal, const double rho, const double neighbor_normal_alignment)
{
  return {
    quantizeUnit(std::abs(normalize(normal).z), kNormalZBinNum),
    quantizeRho(rho),
    quantizeUnit(neighbor_normal_alignment, kNeighborNormalBinNum)};
}

std::array<std::uint64_t, 2U> makeTemplateRetrievalKeys(
  const RetrievalDescriptor &descriptor)
{
  return {
    makeRetrievalKey(
      0U, descriptor.normal_z_bin, descriptor.rho_bin, 0),
    makeRetrievalKey(
      1U, descriptor.normal_z_bin, 0, descriptor.neighbor_normal_bin)};
}

std::vector<RetrievalProbe> makeEnvironmentRetrievalProbes(
  const RetrievalDescriptor &descriptor)
{
  std::vector<RetrievalProbe> probes;
  probes.reserve(54U);
  for (int normal_z_bin = std::max(0, descriptor.normal_z_bin - 1);
    normal_z_bin <= std::min(kNormalZBinNum - 1, descriptor.normal_z_bin + 1);
    ++normal_z_bin)
  {
    for (int rho_bin = std::max(0, descriptor.rho_bin - 1);
      rho_bin <= std::min(kRhoBinNum - 1, descriptor.rho_bin + 1);
      ++rho_bin)
    {
      const int bin_dev = std::abs(normal_z_bin - descriptor.normal_z_bin) +
        std::abs(rho_bin - descriptor.rho_bin);
      probes.push_back({
        makeRetrievalKey(0U, normal_z_bin, rho_bin, 0),
        1.0 / (1.0 + static_cast<double>(bin_dev))});
    }
    for (int neighbor_normal_bin = std::max(0, descriptor.neighbor_normal_bin - 1);
      neighbor_normal_bin <= std::min(
        kNeighborNormalBinNum - 1, descriptor.neighbor_normal_bin + 1);
      ++neighbor_normal_bin)
    {
      const int bin_dev = std::abs(normal_z_bin - descriptor.normal_z_bin) +
        std::abs(neighbor_normal_bin - descriptor.neighbor_normal_bin);
      probes.push_back({
        makeRetrievalKey(1U, normal_z_bin, 0, neighbor_normal_bin),
        1.0 / (1.0 + static_cast<double>(bin_dev))});
    }
  }
  std::sort(probes.begin(), probes.end(), [](const RetrievalProbe &first,
    const RetrievalProbe &second) {return first.key < second.key;});
  std::vector<RetrievalProbe> unique_probes;
  unique_probes.reserve(probes.size());
  for (const RetrievalProbe &probe : probes) {
    if (!unique_probes.empty() && unique_probes.back().key == probe.key) {
      unique_probes.back().score = std::max(unique_probes.back().score, probe.score);
    } else {
      unique_probes.push_back(probe);
    }
  }
  return unique_probes;
}

Vec3 rotateRpyRaw(const Vec3 &value, double roll_deg, double pitch_deg, double yaw_deg)
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
  return {
    std::cos(yaw) * pitch_x - std::sin(yaw) * pitch_y,
    std::sin(yaw) * pitch_x + std::cos(yaw) * pitch_y,
    pitch_z};
}

Vec3 rotateRpy(const Vec3 &value, double roll_deg, double pitch_deg, double yaw_deg)
{
  return normalize(rotateRpyRaw(value, roll_deg, pitch_deg, yaw_deg));
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

void deriveTemplateNonplaneComponents(TemplateGraph &graph)
{
  std::vector<std::unordered_set<std::size_t>> node_plane_indices(graph.nodes.size());
  for (std::size_t plane_index = 0U; plane_index < graph.plane_clusters.size(); ++plane_index) {
    for (const std::size_t node_index : graph.plane_clusters[plane_index].node_indices) {
      if (node_index < node_plane_indices.size()) {
        node_plane_indices[node_index].insert(plane_index);
      }
    }
  }

  std::vector<bool> is_visited(graph.nodes.size(), false);
  for (std::size_t root_index = 0U; root_index < graph.nodes.size(); ++root_index) {
    if (is_visited[root_index] || !node_plane_indices[root_index].empty()) {
      continue;
    }
    NonplaneComponent component;
    component.id = static_cast<std::uint32_t>(graph.nonplane_components.size());
    std::queue<std::size_t> pending;
    pending.push(root_index);
    is_visited[root_index] = true;
    while (!pending.empty()) {
      const std::size_t node_index = pending.front();
      pending.pop();
      component.node_indices.push_back(node_index);
      for (const std::size_t neighbor_index : graph.neighbors[node_index]) {
        if (!node_plane_indices[neighbor_index].empty()) {
          component.anchor_plane_indices.insert(
            node_plane_indices[neighbor_index].begin(), node_plane_indices[neighbor_index].end());
          continue;
        }
        if (!is_visited[neighbor_index]) {
          is_visited[neighbor_index] = true;
          pending.push(neighbor_index);
        }
      }
    }
    std::unordered_set<std::size_t> component_indices(
      component.node_indices.begin(), component.node_indices.end());
    for (const std::size_t node_index : component.node_indices) {
      for (const std::size_t neighbor_index : graph.neighbors[node_index]) {
        if (node_index < neighbor_index && component_indices.count(neighbor_index) > 0U) {
          ++component.internal_edge_num;
        }
      }
    }
    graph.nonplane_components.push_back(std::move(component));
  }
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
      node.value("rho", 0.0)});
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
      Vec3 centroid;
      bool has_centroid = false;
      if (plane_cluster.contains("centroid") && plane_cluster.at("centroid").is_array() &&
        plane_cluster.at("centroid").size() >= 3U &&
        plane_cluster.at("centroid").at(0).is_number() &&
        plane_cluster.at("centroid").at(1).is_number() &&
        plane_cluster.at("centroid").at(2).is_number())
      {
        centroid = {
          plane_cluster.at("centroid").at(0).get<double>(),
          plane_cluster.at("centroid").at(1).get<double>(),
          plane_cluster.at("centroid").at(2).get<double>()};
        has_centroid = isFinite(centroid.x) && isFinite(centroid.y) && isFinite(centroid.z);
      }
      if (!has_centroid && !node_indices.empty()) {
        for (const std::size_t node_index : node_indices) {
          const Vec3 &position = graph.nodes[node_index].position;
          centroid.x += position.x;
          centroid.y += position.y;
          centroid.z += position.z;
        }
        const double node_num = static_cast<double>(node_indices.size());
        centroid.x /= node_num;
        centroid.y /= node_num;
        centroid.z /= node_num;
        has_centroid = isFinite(centroid.x) && isFinite(centroid.y) && isFinite(centroid.z);
      }
      graph.plane_clusters.push_back({
        plane_cluster.value("id", static_cast<std::uint32_t>(index)),
        normalize({normal.at(0).get<double>(), normal.at(1).get<double>(), normal.at(2).get<double>()}),
        centroid, has_centroid, extents[0], extents[1], std::move(node_indices)});
    }
  }
  deriveTemplateNonplaneComponents(graph);
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
    template_ids_ = declare_parameter<std::vector<std::string>>(
      "template_ids", std::vector<std::string>{});
    template_dataset_paths_ = declare_parameter<std::vector<std::string>>(
      "template_dataset_paths", std::vector<std::string>{});
    template_roll_tolerance_degs_ = declare_parameter<std::vector<double>>(
      "template_roll_tolerance_degs", std::vector<double>{});
    template_pitch_tolerance_degs_ = declare_parameter<std::vector<double>>(
      "template_pitch_tolerance_degs", std::vector<double>{});
    environment_topic_ = declare_parameter<std::string>(
      "environment_topological_map_topic", "/topological_map");
    plane_clusters_topic_ = declare_parameter<std::string>("plane_clusters_topic", "/plane_clusters");
    candidate_topic_ = declare_parameter<std::string>("candidate_topic", "");
    candidate_topics_ = declare_parameter<std::vector<std::string>>(
      "candidate_topics", std::vector<std::string>{});
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
    normal_weight_ = declare_parameter<double>("normal_weight", 0.55);
    rho_weight_ = declare_parameter<double>("rho_weight", 0.20);
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
    enable_plane_relative_height_evaluation_ = declare_parameter<bool>(
      "enable_plane_relative_height_evaluation", true);
    max_plane_relative_height_dev_full_ = declare_parameter<double>(
      "max_plane_relative_height_dev_full", 0.015);
    max_plane_relative_height_dev_partial_ = declare_parameter<double>(
      "max_plane_relative_height_dev_partial", 0.060);
    plane_relative_height_weight_ = declare_parameter<double>(
      "plane_relative_height_weight", 0.25);
    enable_nonplane_component_evaluation_ = declare_parameter<bool>(
      "enable_nonplane_component_evaluation", true);
    min_nonplane_component_nodes_ = declare_parameter<int>("min_nonplane_component_nodes", 2);
    nonplane_weight_ = declare_parameter<double>("nonplane_weight", 0.20);
    nonplane_evidence_score_scale_ = declare_parameter<double>(
      "nonplane_evidence_score_scale", 1.50);
    min_nonplane_evidence_score_th_ = declare_parameter<double>(
      "min_nonplane_evidence_score_th", 0.55);

    if (environment_topic_.empty()) {
      throw std::runtime_error("environment_topological_map_topicの指定が必要です。");
    }
    validateParameters();
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&ObjectTemplateMatcherNode::onSetParameters, this, std::placeholders::_1));
    initializeTemplates();
    environment_subscription_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      environment_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&ObjectTemplateMatcherNode::onEnvironmentMap, this, std::placeholders::_1));
    plane_clusters_subscription_ = create_subscription<ais_gng_msgs::msg::PlaneClusterArray>(
      plane_clusters_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&ObjectTemplateMatcherNode::onPlaneClusters, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "物体テンプレート照合開始: templates=%zu input=%s",
      loaded_templates_.size(), environment_topic_.c_str());
  }

private:
  void initializeTemplates()
  {
    const bool has_multi_template_parameters = !template_ids_.empty() ||
      !template_dataset_paths_.empty() || !template_roll_tolerance_degs_.empty() ||
      !template_pitch_tolerance_degs_.empty() || !candidate_topics_.empty();
    std::vector<std::string> ids = template_ids_;
    std::vector<std::string> paths = template_dataset_paths_;
    std::vector<double> roll_tolerance_degs = template_roll_tolerance_degs_;
    std::vector<double> pitch_tolerance_degs = template_pitch_tolerance_degs_;
    std::vector<std::string> topics = candidate_topics_;
    if (has_multi_template_parameters) {
      if (ids.empty() || paths.empty() || ids.size() != paths.size()) {
        throw std::runtime_error("template_idsとtemplate_dataset_pathsの要素数一致が必要です。");
      }
    } else {
      if (template_id_.empty() || template_dataset_path_.empty()) {
        throw std::runtime_error(
                "template_idとtemplate_dataset_path、または複数テンプレート指定が必要です。");
      }
      ids = {template_id_};
      paths = {template_dataset_path_};
    }
    if (roll_tolerance_degs.empty()) {
      roll_tolerance_degs.assign(ids.size(), -1.0);
    }
    if (pitch_tolerance_degs.empty()) {
      pitch_tolerance_degs.assign(ids.size(), -1.0);
    }
    if (roll_tolerance_degs.size() != ids.size() || pitch_tolerance_degs.size() != ids.size()) {
      throw std::runtime_error("テンプレート別roll/pitch許容角の要素数一致が必要です。");
    }
    if (!topics.empty() && topics.size() != ids.size()) {
      throw std::runtime_error("candidate_topicsとtemplate_idsの要素数一致が必要です。");
    }
    if (topics.empty()) {
      topics.reserve(ids.size());
      for (const std::string &id : ids) {
        topics.push_back("/" + id + "/object_template_match_candidates");
      }
    }

    std::unordered_set<std::string> seen_ids;
    loaded_templates_.reserve(ids.size());
    for (std::size_t index = 0U; index < ids.size(); ++index) {
      if (ids[index].empty() || paths[index].empty() ||
        !seen_ids.insert(ids[index]).second || !isFinite(roll_tolerance_degs[index]) ||
        !isFinite(pitch_tolerance_degs[index]) || roll_tolerance_degs[index] < -1.0 ||
        roll_tolerance_degs[index] > 90.0 || pitch_tolerance_degs[index] < -1.0 ||
        pitch_tolerance_degs[index] > 90.0)
      {
        throw std::runtime_error("template_idまたはtemplate_dataset_pathが不正です。");
      }
      LoadedTemplate loaded_template;
      loaded_template.id = ids[index];
      loaded_template.graph = loadTemplateGraph(paths[index]);
      loaded_template.graph.roll_tolerance_deg = roll_tolerance_degs[index];
      loaded_template.graph.pitch_tolerance_deg = pitch_tolerance_degs[index];
      loaded_template.candidate_publisher = create_publisher<std_msgs::msg::String>(
        topics[index], rclcpp::QoS(1).reliable().transient_local());
      RCLCPP_INFO(
        get_logger(), "物体テンプレート読込: template=%s nodes=%zu edges=%zu planes=%zu",
        loaded_template.id.c_str(), loaded_template.graph.nodes.size(),
        loaded_template.graph.edges.size(), loaded_template.graph.plane_clusters.size());
      loaded_templates_.push_back(std::move(loaded_template));
    }
    buildTemplateRetrievalIndex();
    buildTemplatePlaneRetrievalIndex();
    activateTemplate(0U);
  }

  static double templateNeighborNormalAlignment(
    const TemplateGraph &graph, const std::size_t node_index)
  {
    const auto &neighbors = graph.neighbors[node_index];
    if (neighbors.empty()) {
      return 0.5;
    }
    double alignment_sum = 0.0;
    for (const std::size_t neighbor_index : neighbors) {
      alignment_sum += std::abs(dot(
        graph.nodes[node_index].normal, graph.nodes[neighbor_index].normal));
    }
    return alignment_sum / static_cast<double>(neighbors.size());
  }

  static double environmentNeighborNormalAlignment(
    const ais_gng_msgs::msg::TopologicalMap &environment,
    const std::vector<std::vector<std::size_t>> &neighbors, const std::size_t node_index)
  {
    if (neighbors[node_index].empty()) {
      return 0.5;
    }
    const Vec3 normal = normalize({
      environment.nodes[node_index].normal.x,
      environment.nodes[node_index].normal.y,
      environment.nodes[node_index].normal.z});
    double alignment_sum = 0.0;
    for (const std::size_t neighbor_index : neighbors[node_index]) {
      const Vec3 neighbor_normal = normalize({
        environment.nodes[neighbor_index].normal.x,
        environment.nodes[neighbor_index].normal.y,
        environment.nodes[neighbor_index].normal.z});
      alignment_sum += std::abs(dot(normal, neighbor_normal));
    }
    return alignment_sum / static_cast<double>(neighbors[node_index].size());
  }

  void buildTemplateRetrievalIndex()
  {
    template_retrieval_index_.clear();
    template_retrieval_index_.reserve(loaded_templates_.size() * 32U);
    for (std::size_t template_index = 0U; template_index < loaded_templates_.size(); ++template_index) {
      const TemplateGraph &graph = loaded_templates_[template_index].graph;
      std::unordered_set<std::uint64_t> unique_keys;
      unique_keys.reserve(graph.nodes.size() * 2U);
      for (std::size_t node_index = 0U; node_index < graph.nodes.size(); ++node_index) {
        const TemplateNode &node = graph.nodes[node_index];
        const RetrievalDescriptor descriptor = makeRetrievalDescriptor(
          node.normal, node.rho, templateNeighborNormalAlignment(graph, node_index));
        const auto keys = makeTemplateRetrievalKeys(descriptor);
        unique_keys.insert(keys.begin(), keys.end());
      }
      for (const std::uint64_t key : unique_keys) {
        template_retrieval_index_[key].push_back(template_index);
      }
    }
    for (auto &[key, template_indices] : template_retrieval_index_) {
      static_cast<void>(key);
      std::sort(template_indices.begin(), template_indices.end());
      template_indices.erase(
        std::unique(template_indices.begin(), template_indices.end()), template_indices.end());
    }
  }

  void buildTemplatePlaneRetrievalIndex()
  {
    template_plane_retrieval_tokens_.clear();
    for (std::size_t template_index = 0U; template_index < loaded_templates_.size(); ++template_index) {
      for (const TemplatePlaneCluster &plane_cluster : loaded_templates_[template_index].graph.plane_clusters) {
        template_plane_retrieval_tokens_.push_back({
          template_index, plane_cluster.min_extent, plane_cluster.max_extent});
      }
    }
    template_plane_min_extent_order_.resize(template_plane_retrieval_tokens_.size());
    template_plane_max_extent_order_.resize(template_plane_retrieval_tokens_.size());
    for (std::size_t token_index = 0U;
      token_index < template_plane_retrieval_tokens_.size(); ++token_index)
    {
      template_plane_min_extent_order_[token_index] = token_index;
      template_plane_max_extent_order_[token_index] = token_index;
    }
    std::sort(
      template_plane_min_extent_order_.begin(), template_plane_min_extent_order_.end(),
      [this](const std::size_t first_index, const std::size_t second_index) {
        return template_plane_retrieval_tokens_[first_index].min_extent <
               template_plane_retrieval_tokens_[second_index].min_extent;
      });
    std::sort(
      template_plane_max_extent_order_.begin(), template_plane_max_extent_order_.end(),
      [this](const std::size_t first_index, const std::size_t second_index) {
        return template_plane_retrieval_tokens_[first_index].max_extent <
               template_plane_retrieval_tokens_[second_index].max_extent;
      });
  }

  std::vector<std::size_t> findNearestPlaneTokenIndices(
    const std::vector<std::size_t> &order, const double target_extent,
    const bool use_min_extent) const
  {
    const auto lower = std::lower_bound(
      order.begin(), order.end(), target_extent,
      [this, use_min_extent](const std::size_t token_index, const double target) {
        const PlaneRetrievalToken &token = template_plane_retrieval_tokens_[token_index];
        return (use_min_extent ? token.min_extent : token.max_extent) < target;
      });
    std::ptrdiff_t left_index = std::distance(order.begin(), lower) - 1;
    std::size_t right_index = static_cast<std::size_t>(std::distance(order.begin(), lower));
    std::vector<std::size_t> nearest_indices;
    nearest_indices.reserve(kMaxPlaneTokenNumPerProbe);
    while (nearest_indices.size() < kMaxPlaneTokenNumPerProbe &&
      (left_index >= 0 || right_index < order.size()))
    {
      const bool has_left = left_index >= 0;
      const bool has_right = right_index < order.size();
      double left_distance = std::numeric_limits<double>::infinity();
      double right_distance = std::numeric_limits<double>::infinity();
      if (has_left) {
        const PlaneRetrievalToken &token = template_plane_retrieval_tokens_[
          order[static_cast<std::size_t>(left_index)]];
        left_distance = std::abs(std::log(
          (use_min_extent ? token.min_extent : token.max_extent) / target_extent));
      }
      if (has_right) {
        const PlaneRetrievalToken &token = template_plane_retrieval_tokens_[order[right_index]];
        right_distance = std::abs(std::log(
          (use_min_extent ? token.min_extent : token.max_extent) / target_extent));
      }
      if (left_distance <= right_distance) {
        nearest_indices.push_back(order[static_cast<std::size_t>(left_index)]);
        --left_index;
      } else {
        nearest_indices.push_back(order[right_index]);
        ++right_index;
      }
    }
    return nearest_indices;
  }

  std::vector<double> makePlaneScaleProbes() const
  {
    const double max_scale = 1.0 / min_plane_extent_allow_ratio_;
    std::vector<double> probes{1.0};
    for (double scale = kPlaneScaleProbeRatio;
      scale < max_scale - 1e-9;
      scale *= kPlaneScaleProbeRatio)
    {
      probes.push_back(scale);
    }
    if (probes.back() < max_scale) {
      probes.push_back(max_scale);
    }
    return probes;
  }

  std::vector<std::size_t> selectPlaneRetrievalTemplateIndices(
    const ais_gng_msgs::msg::PlaneClusterArray &plane_clusters) const
  {
    if (template_plane_retrieval_tokens_.empty() || plane_clusters.clusters.empty()) {
      return {};
    }
    std::unordered_map<std::size_t, double> template_score_sums;
    const std::vector<double> probes = makePlaneScaleProbes();
    for (const auto &environment_plane : plane_clusters.clusters) {
      const auto environment_extents = sortedExtents(
        static_cast<double>(environment_plane.extent_u), static_cast<double>(environment_plane.extent_v));
      if (!isFinite(environment_extents[0]) || !isFinite(environment_extents[1]) ||
        environment_extents[0] <= 0.0 || environment_extents[1] <= 0.0)
      {
        continue;
      }
      std::unordered_map<std::size_t, double> plane_template_scores;
      for (const double scale : probes) {
        const std::vector<std::size_t> min_indices = findNearestPlaneTokenIndices(
          template_plane_min_extent_order_, environment_extents[0] * scale, true);
        const std::vector<std::size_t> max_indices = findNearestPlaneTokenIndices(
          template_plane_max_extent_order_, environment_extents[1] * scale, false);
        std::unordered_set<std::size_t> nearby_token_indices(
          min_indices.begin(), min_indices.end());
        nearby_token_indices.insert(max_indices.begin(), max_indices.end());
        for (const std::size_t token_index : nearby_token_indices) {
          const PlaneRetrievalToken &token = template_plane_retrieval_tokens_[token_index];
          if (environment_extents[0] > token.min_extent * max_plane_extent_overflow_ratio_ ||
            environment_extents[1] > token.max_extent * max_plane_extent_overflow_ratio_)
          {
            continue;
          }
          const double min_score = intervalMembership(
            environment_extents[0] / token.min_extent, min_plane_extent_allow_ratio_,
            min_plane_extent_full_match_ratio_, max_plane_extent_full_match_ratio_,
            max_plane_extent_overflow_ratio_);
          const double max_score = intervalMembership(
            environment_extents[1] / token.max_extent, min_plane_extent_allow_ratio_,
            min_plane_extent_full_match_ratio_, max_plane_extent_full_match_ratio_,
            max_plane_extent_overflow_ratio_);
          const double score = 0.5 * (min_score + max_score);
          if (score <= 0.0) {
            continue;
          }
          auto insertion = plane_template_scores.emplace(token.template_index, score);
          if (!insertion.second) {
            insertion.first->second = std::max(insertion.first->second, score);
          }
        }
      }
      for (const auto &[template_index, score] : plane_template_scores) {
        template_score_sums[template_index] += score;
      }
    }
    std::vector<RetrievalCandidate> candidates;
    candidates.reserve(template_score_sums.size());
    const double plane_num = static_cast<double>(plane_clusters.clusters.size());
    for (const auto &[template_index, score_sum] : template_score_sums) {
      candidates.push_back({template_index, score_sum / plane_num});
    }
    std::sort(candidates.begin(), candidates.end(), [](const RetrievalCandidate &first,
      const RetrievalCandidate &second) {
      return first.score == second.score ? first.template_index < second.template_index :
             first.score > second.score;
    });
    if (candidates.size() > kMaxRefinementTemplateNum) {
      candidates.resize(kMaxRefinementTemplateNum);
    }
    std::vector<std::size_t> template_indices;
    template_indices.reserve(candidates.size());
    for (const RetrievalCandidate &candidate : candidates) {
      template_indices.push_back(candidate.template_index);
    }
    return template_indices;
  }

  static void appendEvenlySample(
    const std::vector<std::size_t> &source_indices, const std::size_t sample_num,
    std::vector<std::size_t> &target_indices, std::vector<bool> &is_selected)
  {
    if (sample_num == 0U || source_indices.empty()) {
      return;
    }
    for (std::size_t sample_index = 0U; sample_index < sample_num; ++sample_index) {
      const std::size_t source_index = source_indices[
        sample_index * source_indices.size() / sample_num];
      if (!is_selected[source_index]) {
        is_selected[source_index] = true;
        target_indices.push_back(source_index);
      }
    }
  }

  static std::vector<std::size_t> selectEnvironmentRetrievalNodes(
    const ais_gng_msgs::msg::TopologicalMap &environment)
  {
    std::vector<std::size_t> nonplane_indices;
    nonplane_indices.reserve(environment.nodes.size());
    std::vector<std::size_t> all_indices;
    all_indices.reserve(environment.nodes.size());
    for (std::size_t node_index = 0U; node_index < environment.nodes.size(); ++node_index) {
      all_indices.push_back(node_index);
      if (environment.nodes[node_index].nonplane_component_id !=
        std::numeric_limits<std::uint32_t>::max())
      {
        nonplane_indices.push_back(node_index);
      }
    }

    std::vector<std::size_t> selected_indices;
    selected_indices.reserve(kMaxRetrievalNodeNum);
    std::vector<bool> is_selected(environment.nodes.size(), false);
    appendEvenlySample(
      nonplane_indices,
      std::min(kMaxNonplaneRetrievalNodeNum, nonplane_indices.size()),
      selected_indices, is_selected);
    appendEvenlySample(
      all_indices,
      std::min(kMaxRetrievalNodeNum - selected_indices.size(), all_indices.size()),
      selected_indices, is_selected);
    for (std::size_t node_index = 0U;
      selected_indices.size() < kMaxRetrievalNodeNum && node_index < environment.nodes.size();
      ++node_index)
    {
      if (!is_selected[node_index]) {
        is_selected[node_index] = true;
        selected_indices.push_back(node_index);
      }
    }
    return selected_indices;
  }

  static void addRetrievalCandidate(
    std::vector<RetrievalCandidate> &candidates, const std::size_t template_index,
    const double score)
  {
    for (RetrievalCandidate &candidate : candidates) {
      if (candidate.template_index == template_index) {
        candidate.score += score;
        return;
      }
    }
    if (candidates.size() < kMaxRetrievalCandidateNum) {
      candidates.push_back({template_index, score});
      return;
    }
    const auto minimum = std::min_element(
      candidates.begin(), candidates.end(), [](const RetrievalCandidate &first,
      const RetrievalCandidate &second) {return first.score < second.score;});
    minimum->template_index = template_index;
    minimum->score += score;
  }

  static void addRetrievalCandidateMaximum(
    std::vector<RetrievalCandidate> &candidates, const std::size_t template_index,
    const double score)
  {
    for (RetrievalCandidate &candidate : candidates) {
      if (candidate.template_index == template_index) {
        candidate.score = std::max(candidate.score, score);
        return;
      }
    }
    if (candidates.size() < kMaxRetrievalCandidateNum) {
      candidates.push_back({template_index, score});
      return;
    }
    const auto minimum = std::min_element(
      candidates.begin(), candidates.end(), [](const RetrievalCandidate &first,
      const RetrievalCandidate &second) {return first.score < second.score;});
    if (score > minimum->score) {
      *minimum = {template_index, score};
    }
  }

  std::vector<std::size_t> selectRefinementTemplateIndices(
    const ais_gng_msgs::msg::TopologicalMap &environment,
    const std::vector<std::vector<std::size_t>> &environment_neighbors,
    const ais_gng_msgs::msg::PlaneClusterArray &plane_clusters) const
  {
    if (loaded_templates_.size() <= kMaxRefinementTemplateNum) {
      std::vector<std::size_t> indices;
      indices.reserve(loaded_templates_.size());
      for (std::size_t index = 0U; index < loaded_templates_.size(); ++index) {
        indices.push_back(index);
      }
      return indices;
    }
    const std::vector<std::size_t> plane_indices =
      selectPlaneRetrievalTemplateIndices(plane_clusters);
    if (!plane_indices.empty()) {
      return plane_indices;
    }
    std::vector<RetrievalCandidate> candidates;
    candidates.reserve(kMaxRetrievalCandidateNum);
    const std::vector<std::size_t> environment_indices =
      selectEnvironmentRetrievalNodes(environment);
    for (const std::size_t environment_index : environment_indices) {
      std::vector<RetrievalCandidate> local_candidates;
      local_candidates.reserve(kMaxRetrievalCandidateNum);
      const auto &node = environment.nodes[environment_index];
      const RetrievalDescriptor descriptor = makeRetrievalDescriptor(
        {node.normal.x, node.normal.y, node.normal.z}, static_cast<double>(node.rho),
        environmentNeighborNormalAlignment(environment, environment_neighbors, environment_index));
      for (const RetrievalProbe &probe : makeEnvironmentRetrievalProbes(descriptor)) {
        const auto lookup = template_retrieval_index_.find(probe.key);
        if (lookup == template_retrieval_index_.end() || lookup->second.empty()) {
          continue;
        }
        const std::vector<std::size_t> &template_indices = lookup->second;
        const std::size_t sample_num = std::min(
          kMaxTemplatesPerRetrievalKey, template_indices.size());
        const std::uint64_t seed =
          static_cast<std::uint64_t>(environment.frame_number) * 11400714819323198485ULL ^ probe.key;
        const std::size_t start_index = static_cast<std::size_t>(seed % template_indices.size());
        const double score = probe.score /
          std::sqrt(static_cast<double>(template_indices.size()));
        for (std::size_t offset = 0U; offset < sample_num; ++offset) {
          addRetrievalCandidateMaximum(
            local_candidates,
            template_indices[(start_index + offset) % template_indices.size()], score);
        }
      }
      for (const RetrievalCandidate &candidate : local_candidates) {
        addRetrievalCandidate(candidates, candidate.template_index, candidate.score);
      }
    }
    if (candidates.empty()) {
      return {static_cast<std::size_t>(environment.frame_number) % loaded_templates_.size()};
    }
    std::sort(candidates.begin(), candidates.end(), [](
      const RetrievalCandidate &first, const RetrievalCandidate &second) {
        return first.score == second.score ? first.template_index < second.template_index :
               first.score > second.score;
      });
    std::vector<std::size_t> selected_indices;
    selected_indices.reserve(kMaxRefinementTemplateNum);
    for (std::size_t index = 0U;
      index < candidates.size() && index < kMaxRefinementTemplateNum; ++index)
    {
      selected_indices.push_back(candidates[index].template_index);
    }
    return selected_indices;
  }

  void activateTemplate(const std::size_t index)
  {
    if (has_active_template_) {
      std::swap(template_graph_, loaded_templates_[active_template_index_].graph);
    }
    std::swap(template_graph_, loaded_templates_[index].graph);
    active_template_index_ = index;
    has_active_template_ = true;
    template_id_ = loaded_templates_[index].id;
    candidate_publisher_ = loaded_templates_[index].candidate_publisher;
  }

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
        if (name == "template_ids" || name == "template_dataset_paths" ||
          name == "template_roll_tolerance_degs" || name == "template_pitch_tolerance_degs" ||
          name == "candidate_topics")
        {
          throw std::runtime_error("テンプレート読込設定の変更にはノード再起動が必要です。");
        } else if (name == "shape_tolerance") stage(shape_tolerance_, parameter.as_double());
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
        else if (name == "normal_weight") stage(normal_weight_, parameter.as_double());
        else if (name == "rho_weight") stage(rho_weight_, parameter.as_double());
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
        } else if (name == "enable_plane_relative_height_evaluation") {
          stage(enable_plane_relative_height_evaluation_, parameter.as_bool());
        } else if (name == "max_plane_relative_height_dev_full") {
          stage(max_plane_relative_height_dev_full_, parameter.as_double());
        } else if (name == "max_plane_relative_height_dev_partial") {
          stage(max_plane_relative_height_dev_partial_, parameter.as_double());
        } else if (name == "plane_relative_height_weight") {
          stage(plane_relative_height_weight_, parameter.as_double());
        } else if (name == "enable_nonplane_component_evaluation") {
          stage(enable_nonplane_component_evaluation_, parameter.as_bool());
        } else if (name == "min_nonplane_component_nodes") {
          stage(min_nonplane_component_nodes_, parameter.as_int());
        } else if (name == "nonplane_weight") stage(nonplane_weight_, parameter.as_double());
        else if (name == "nonplane_evidence_score_scale") {
          stage(nonplane_evidence_score_scale_, parameter.as_double());
        } else if (name == "min_nonplane_evidence_score_th") {
          stage(min_nonplane_evidence_score_th_, parameter.as_double());
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
    const std::array<double, 30> values = {
      shape_tolerance_, scale_tolerance_, contradiction_limit_, yaw_step_deg_,
      roll_tolerance_deg_, pitch_tolerance_deg_, roll_pitch_step_deg_,
      min_node_score_, edge_weight_,
      contradiction_weight_, max_contradiction_point_ratio_, min_scale_allow_ratio_,
      min_scale_full_match_ratio_, max_scale_full_match_ratio_, max_scale_allow_ratio_, scale_weight_,
      max_plane_normal_angle_deg_, max_plane_extent_overflow_ratio_, min_plane_extent_allow_ratio_,
      min_plane_extent_full_match_ratio_, max_plane_extent_full_match_ratio_, plane_weight_,
      plane_support_score_scale_, min_plane_support_score_, max_plane_relative_height_dev_full_,
      max_plane_relative_height_dev_partial_, plane_relative_height_weight_, nonplane_weight_,
      nonplane_evidence_score_scale_, min_nonplane_evidence_score_th_};
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
      contradiction_weight_ < 0.0 || contradiction_weight_ > 1.0 ||
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
      max_plane_relative_height_dev_full_ < 0.0 ||
      max_plane_relative_height_dev_full_ >= max_plane_relative_height_dev_partial_ ||
      plane_relative_height_weight_ < 0.0 || plane_relative_height_weight_ > 1.0 ||
      min_nonplane_component_nodes_ <= 0 || nonplane_weight_ < 0.0 || nonplane_weight_ > 1.0 ||
      nonplane_evidence_score_scale_ <= 0.0 || min_nonplane_evidence_score_th_ < 0.0 ||
      min_nonplane_evidence_score_th_ > 1.0 ||
      (enable_oversized_plane_filter_ && plane_clusters_topic_.empty()))
    {
      throw std::runtime_error("物体テンプレート照合パラメータが不正です。");
    }
    if (max_normal_angle_full_deg_ < 0.0 ||
      max_normal_angle_full_deg_ >= max_normal_angle_partial_deg_ ||
      max_rho_dev_full_ratio_ < 0.0 ||
      max_rho_dev_full_ratio_ >= max_rho_dev_partial_ratio_)
    {
      throw std::runtime_error("ファジー評価範囲が不正です。");
    }
  }

  template <typename MessagePointer>
  static void limitPendingFrames(std::map<std::uint32_t, MessagePointer> &pending_messages)
  {
    while (pending_messages.size() > kMaxPendingSynchronizedFrameNum) {
      pending_messages.erase(pending_messages.begin());
    }
  }

  void evaluateSynchronizedEnvironment(
    const ais_gng_msgs::msg::TopologicalMap::SharedPtr &environment,
    const ais_gng_msgs::msg::PlaneClusterArray::SharedPtr &plane_clusters)
  {
    latest_plane_clusters_ = plane_clusters;
    const auto id_to_index = buildIdToIndex(*environment);
    const auto environment_edges = buildEnvironmentEdges(*environment, id_to_index);
    const auto environment_neighbors = buildNeighbors(environment->nodes.size(), environment_edges);
    std::vector<std::pair<std::size_t, json>> candidates;
    for (const std::size_t template_index :
      selectRefinementTemplateIndices(*environment, environment_neighbors, *plane_clusters))
    {
      activateTemplate(template_index);
      candidates.emplace_back(template_index, evaluateActiveTemplate(environment, environment_edges));
    }
    std::sort(candidates.begin(), candidates.end(), [](const std::pair<std::size_t, json> &first,
      const std::pair<std::size_t, json> &second) {
      return first.second.at("score").get<double>() > second.second.at("score").get<double>();
    });
    const double second_score = candidates.size() > 1U ?
      candidates[1U].second.at("score").get<double>() : 0.0;
    for (std::size_t rank = 0U; rank < candidates.size(); ++rank) {
      const std::size_t template_index = candidates[rank].first;
      json &candidate = candidates[rank].second;
      candidate["template_rank"] = rank + 1U;
      candidate["is_template_winner"] = rank == 0U;
      candidate["template_score_margin"] = rank == 0U ?
        candidate.at("score").get<double>() - second_score :
        candidate.at("score").get<double>() - candidates.front().second.at("score").get<double>();
      std_msgs::msg::String message;
      message.data = candidate.dump();
      loaded_templates_[template_index].candidate_publisher->publish(message);
    }
  }

  void onPlaneClusters(const ais_gng_msgs::msg::PlaneClusterArray::SharedPtr plane_clusters)
  {
    const auto environment = pending_environment_maps_.find(plane_clusters->frame_number);
    if (environment != pending_environment_maps_.end()) {
      const auto synchronized_environment = environment->second;
      pending_environment_maps_.erase(environment);
      evaluateSynchronizedEnvironment(synchronized_environment, plane_clusters);
      return;
    }
    pending_plane_clusters_[plane_clusters->frame_number] = plane_clusters;
    limitPendingFrames(pending_plane_clusters_);
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
    const auto plane_clusters = pending_plane_clusters_.find(environment->frame_number);
    if (plane_clusters != pending_plane_clusters_.end()) {
      const auto synchronized_plane_clusters = plane_clusters->second;
      pending_plane_clusters_.erase(plane_clusters);
      evaluateSynchronizedEnvironment(environment, synchronized_plane_clusters);
      return;
    }
    pending_environment_maps_[environment->frame_number] = environment;
    limitPendingFrames(pending_environment_maps_);
  }

  json evaluateActiveTemplate(
    const ais_gng_msgs::msg::TopologicalMap::SharedPtr &environment,
    const std::vector<std::pair<std::size_t, std::size_t>> &environment_edges)
  {
    const auto yaw_samples = sampleFullYaw(yaw_step_deg_);
    const double roll_tolerance_deg = template_graph_.roll_tolerance_deg >= 0.0 ?
      template_graph_.roll_tolerance_deg : roll_tolerance_deg_;
    const double pitch_tolerance_deg = template_graph_.pitch_tolerance_deg >= 0.0 ?
      template_graph_.pitch_tolerance_deg : pitch_tolerance_deg_;
    const bool enable_template_roll_pitch_search = enable_roll_pitch_search_ ||
      template_graph_.roll_tolerance_deg > 0.0 || template_graph_.pitch_tolerance_deg > 0.0;
    const auto roll_samples = sampleTolerance(
      roll_tolerance_deg, roll_pitch_step_deg_, enable_template_roll_pitch_search);
    const auto pitch_samples = sampleTolerance(
      pitch_tolerance_deg, roll_pitch_step_deg_, enable_template_roll_pitch_search);
    if (yaw_samples.size() * roll_samples.size() * pitch_samples.size() >
      static_cast<std::size_t>(max_orientation_hypothesis_num_))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "姿勢仮説数が上限を超えました: %zu", yaw_samples.size() * roll_samples.size() * pitch_samples.size());
      return {
        {"template_id", template_id_},
        {"state", "no_hypothesis"},
        {"score", 0.0},
        {"is_falsified", true},
        {"rejected_hypothesis_num", 0U},
        {"rejection_reason", "orientation_hypothesis_limit"}};
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
    return best_candidate;
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
    const double geometric_score = score_sum / static_cast<double>(result.correspondences.size());
    double relative_height_score_sum = 0.0;
    if (enable_plane_relative_height_evaluation_) {
      for (std::size_t first_index = 0U;
        first_index < result.correspondences.size(); ++first_index)
      {
        const PlaneCorrespondence &first_correspondence = result.correspondences[first_index];
        const TemplatePlaneCluster &first_template_plane =
          template_graph_.plane_clusters[first_correspondence.template_index];
        const auto &first_environment_plane =
          latest_plane_clusters_->clusters[first_correspondence.environment_index];
        if (!first_template_plane.has_centroid ||
          !isFinite(first_environment_plane.centroid.x) ||
          !isFinite(first_environment_plane.centroid.y) ||
          !isFinite(first_environment_plane.centroid.z))
        {
          continue;
        }
        for (std::size_t second_index = first_index + 1U;
          second_index < result.correspondences.size(); ++second_index)
        {
          const PlaneCorrespondence &second_correspondence = result.correspondences[second_index];
          const TemplatePlaneCluster &second_template_plane =
            template_graph_.plane_clusters[second_correspondence.template_index];
          const auto &second_environment_plane =
            latest_plane_clusters_->clusters[second_correspondence.environment_index];
          if (!second_template_plane.has_centroid ||
            !isFinite(second_environment_plane.centroid.x) ||
            !isFinite(second_environment_plane.centroid.y) ||
            !isFinite(second_environment_plane.centroid.z))
          {
            continue;
          }
          const Vec3 first_template_centroid = rotateRpyRaw(
            first_template_plane.centroid, roll_deg, pitch_deg,
            yaw_deg + template_graph_.canonical_yaw_deg);
          const Vec3 second_template_centroid = rotateRpyRaw(
            second_template_plane.centroid, roll_deg, pitch_deg,
            yaw_deg + template_graph_.canonical_yaw_deg);
          const double template_relative_height =
            first_template_centroid.z - second_template_centroid.z;
          const double environment_relative_height =
            static_cast<double>(first_environment_plane.centroid.z) -
            static_cast<double>(second_environment_plane.centroid.z);
          relative_height_score_sum += descendingMembership(
            std::abs(template_relative_height - environment_relative_height),
            max_plane_relative_height_dev_full_, max_plane_relative_height_dev_partial_);
          ++result.relative_height_pair_num;
        }
      }
    }
    if (result.relative_height_pair_num > 0U) {
      result.is_relative_height_observed = true;
      result.relative_height_score = relative_height_score_sum /
        static_cast<double>(result.relative_height_pair_num);
    }
    result.score = result.is_relative_height_observed ?
      (1.0 - plane_relative_height_weight_) * geometric_score +
      plane_relative_height_weight_ * result.relative_height_score : geometric_score;
    result.matched_ratio = static_cast<double>(result.correspondences.size()) /
      static_cast<double>(template_graph_.plane_clusters.size());
    const double evidence_score_sum = result.is_relative_height_observed ?
      score_sum * ((1.0 - plane_relative_height_weight_) +
      plane_relative_height_weight_ * result.relative_height_score) : score_sum;
    result.support_score = 1.0 - std::exp(-evidence_score_sum / plane_support_score_scale_);
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

  static bool hasIntersectingPlaneIndex(
    const std::unordered_set<std::size_t> &first,
    const std::unordered_set<std::size_t> &second)
  {
    for (const std::size_t index : first) {
      if (second.count(index) > 0U) {
        return true;
      }
    }
    return false;
  }

  static std::unordered_set<std::size_t> makeAttachmentPatchNodeIndices(
    const NonplaneComponent &component,
    const std::vector<std::vector<std::size_t>> &neighbors,
    const std::vector<std::unordered_set<std::size_t>> &node_plane_indices,
    const std::unordered_set<std::size_t> &anchor_plane_indices)
  {
    std::unordered_set<std::size_t> patch_node_indices(
      component.node_indices.begin(), component.node_indices.end());
    for (const std::size_t node_index : component.node_indices) {
      for (const std::size_t neighbor_index : neighbors[node_index]) {
        if (hasIntersectingPlaneIndex(node_plane_indices[neighbor_index], anchor_plane_indices)) {
          patch_node_indices.insert(neighbor_index);
        }
      }
    }
    return patch_node_indices;
  }

  static std::vector<std::pair<std::size_t, std::size_t>> makeAttachmentPatchEdges(
    const std::vector<std::pair<std::size_t, std::size_t>> &edges,
    const NonplaneComponent &component,
    const std::unordered_set<std::size_t> &patch_node_indices)
  {
    const std::unordered_set<std::size_t> component_node_indices(
      component.node_indices.begin(), component.node_indices.end());
    std::vector<std::pair<std::size_t, std::size_t>> patch_edges;
    patch_edges.reserve(edges.size());
    for (const auto &[first, second] : edges) {
      if (patch_node_indices.count(first) == 0U || patch_node_indices.count(second) == 0U ||
        (component_node_indices.count(first) == 0U && component_node_indices.count(second) == 0U))
      {
        continue;
      }
      patch_edges.emplace_back(first, second);
    }
    return patch_edges;
  }

  static void normalizeNonplaneEdgeLengths(std::vector<NonplaneEdgeFeature> &features)
  {
    if (features.empty()) {
      return;
    }
    std::vector<double> lengths;
    lengths.reserve(features.size());
    for (const NonplaneEdgeFeature &feature : features) {
      lengths.push_back(feature.length);
    }
    std::sort(lengths.begin(), lengths.end());
    const std::size_t middle = lengths.size() / 2U;
    const double median_length = lengths.size() % 2U == 0U ?
      (lengths[middle - 1U] + lengths[middle]) * 0.5 : lengths[middle];
    const double scale = std::max(median_length, 1e-6);
    for (NonplaneEdgeFeature &feature : features) {
      feature.normalized_length = feature.length / scale;
    }
  }

  std::pair<double, bool> templateAnchorPlaneDistance(
    const Vec3 &position, const std::unordered_set<std::size_t> &anchor_plane_indices) const
  {
    double min_distance = std::numeric_limits<double>::infinity();
    for (const std::size_t plane_index : anchor_plane_indices) {
      const TemplatePlaneCluster &plane = template_graph_.plane_clusters[plane_index];
      if (!plane.has_centroid) {
        continue;
      }
      const Vec3 difference{
        position.x - plane.centroid.x,
        position.y - plane.centroid.y,
        position.z - plane.centroid.z};
      min_distance = std::min(min_distance, std::abs(dot(difference, plane.normal)));
    }
    return {min_distance, std::isfinite(min_distance)};
  }

  std::pair<double, bool> environmentAnchorPlaneDistance(
    const geometry_msgs::msg::Point32 &position,
    const std::unordered_set<std::size_t> &anchor_plane_indices) const
  {
    double min_distance = std::numeric_limits<double>::infinity();
    for (const std::size_t plane_index : anchor_plane_indices) {
      const auto &plane = latest_plane_clusters_->clusters[plane_index];
      const Vec3 centroid{
        static_cast<double>(plane.centroid.x), static_cast<double>(plane.centroid.y),
        static_cast<double>(plane.centroid.z)};
      if (!isFinite(centroid.x) || !isFinite(centroid.y) || !isFinite(centroid.z)) {
        continue;
      }
      const Vec3 normal = normalize({
        static_cast<double>(plane.normal.x), static_cast<double>(plane.normal.y),
        static_cast<double>(plane.normal.z)});
      const Vec3 difference{
        static_cast<double>(position.x) - centroid.x,
        static_cast<double>(position.y) - centroid.y,
        static_cast<double>(position.z) - centroid.z};
      min_distance = std::min(min_distance, std::abs(dot(difference, normal)));
    }
    return {min_distance, std::isfinite(min_distance)};
  }

  double templateAnchorPlaneScale(
    const std::unordered_set<std::size_t> &anchor_plane_indices) const
  {
    double scale = 0.0;
    for (const std::size_t plane_index : anchor_plane_indices) {
      scale = std::max(scale, template_graph_.plane_clusters[plane_index].max_extent);
    }
    return std::max(scale, 0.03);
  }

  double environmentAnchorPlaneScale(
    const std::unordered_set<std::size_t> &anchor_plane_indices) const
  {
    double scale = 0.0;
    for (const std::size_t plane_index : anchor_plane_indices) {
      const auto &plane = latest_plane_clusters_->clusters[plane_index];
      scale = std::max(
        scale,
        std::max(std::abs(static_cast<double>(plane.extent_u)),
          std::abs(static_cast<double>(plane.extent_v))));
    }
    return std::max(scale, 0.03);
  }

  double templateEdgePlaneNormalAlignment(
    const Vec3 &direction, const std::unordered_set<std::size_t> &anchor_plane_indices) const
  {
    double alignment = 0.0;
    for (const std::size_t plane_index : anchor_plane_indices) {
      alignment = std::max(
        alignment, std::abs(dot(direction, template_graph_.plane_clusters[plane_index].normal)));
    }
    return alignment;
  }

  double environmentEdgePlaneNormalAlignment(
    const Vec3 &direction, const std::unordered_set<std::size_t> &anchor_plane_indices) const
  {
    double alignment = 0.0;
    for (const std::size_t plane_index : anchor_plane_indices) {
      const auto &plane = latest_plane_clusters_->clusters[plane_index];
      const Vec3 normal = normalize({
        static_cast<double>(plane.normal.x), static_cast<double>(plane.normal.y),
        static_cast<double>(plane.normal.z)});
      alignment = std::max(alignment, std::abs(dot(direction, normal)));
    }
    return alignment;
  }

  std::vector<NonplaneEdgeFeature> makeTemplateNonplaneEdgeFeatures(
    const std::vector<std::pair<std::size_t, std::size_t>> &edges,
    const std::unordered_set<std::size_t> &anchor_plane_indices) const
  {
    std::vector<NonplaneEdgeFeature> features;
    features.reserve(edges.size());
    for (const auto &[first, second] : edges) {
      const Vec3 &first_position = template_graph_.nodes[first].position;
      const Vec3 &second_position = template_graph_.nodes[second].position;
      const Vec3 difference{
        second_position.x - first_position.x,
        second_position.y - first_position.y,
        second_position.z - first_position.z};
      const double length = std::sqrt(
        difference.x * difference.x + difference.y * difference.y + difference.z * difference.z);
      if (length <= 1e-6) {
        continue;
      }
      const auto [first_plane_distance, has_first_plane_distance] =
        templateAnchorPlaneDistance(first_position, anchor_plane_indices);
      const auto [second_plane_distance, has_second_plane_distance] =
        templateAnchorPlaneDistance(second_position, anchor_plane_indices);
      const Vec3 direction{difference.x / length, difference.y / length, difference.z / length};
      features.push_back({
        direction,
        {(first_position.x + second_position.x) * 0.5,
          (first_position.y + second_position.y) * 0.5,
          (first_position.z + second_position.z) * 0.5},
        length,
        0.0,
        templateEdgePlaneNormalAlignment(direction, anchor_plane_indices),
        std::min(first_plane_distance, second_plane_distance),
        std::max(first_plane_distance, second_plane_distance),
        templateAnchorPlaneScale(anchor_plane_indices),
        has_first_plane_distance && has_second_plane_distance});
    }
    normalizeNonplaneEdgeLengths(features);
    return features;
  }

  std::vector<NonplaneEdgeFeature> makeEnvironmentNonplaneEdgeFeatures(
    const ais_gng_msgs::msg::TopologicalMap &environment,
    const std::vector<std::pair<std::size_t, std::size_t>> &edges,
    const std::unordered_set<std::size_t> &anchor_plane_indices) const
  {
    std::vector<NonplaneEdgeFeature> features;
    features.reserve(edges.size());
    for (const auto &[first, second] : edges) {
      const auto &first_position = environment.nodes[first].pos;
      const auto &second_position = environment.nodes[second].pos;
      const Vec3 difference{
        static_cast<double>(second_position.x) - static_cast<double>(first_position.x),
        static_cast<double>(second_position.y) - static_cast<double>(first_position.y),
        static_cast<double>(second_position.z) - static_cast<double>(first_position.z)};
      const double length = std::sqrt(
        difference.x * difference.x + difference.y * difference.y + difference.z * difference.z);
      if (length <= 1e-6) {
        continue;
      }
      const auto [first_plane_distance, has_first_plane_distance] =
        environmentAnchorPlaneDistance(first_position, anchor_plane_indices);
      const auto [second_plane_distance, has_second_plane_distance] =
        environmentAnchorPlaneDistance(second_position, anchor_plane_indices);
      const Vec3 direction{difference.x / length, difference.y / length, difference.z / length};
      features.push_back({
        direction,
        {(static_cast<double>(first_position.x) + static_cast<double>(second_position.x)) * 0.5,
          (static_cast<double>(first_position.y) + static_cast<double>(second_position.y)) * 0.5,
          (static_cast<double>(first_position.z) + static_cast<double>(second_position.z)) * 0.5},
        length,
        0.0,
        environmentEdgePlaneNormalAlignment(direction, anchor_plane_indices),
        std::min(first_plane_distance, second_plane_distance),
        std::max(first_plane_distance, second_plane_distance),
        environmentAnchorPlaneScale(anchor_plane_indices),
        has_first_plane_distance && has_second_plane_distance});
    }
    normalizeNonplaneEdgeLengths(features);
    return features;
  }

  std::pair<Vec3, bool> estimateAttachmentTranslation(
    const std::unordered_set<std::size_t> &template_anchor_plane_indices,
    const PlaneEvaluation &plane_evaluation,
    double roll_deg,
    double pitch_deg,
    double yaw_deg) const
  {
    Vec3 translation;
    std::size_t correspondence_num = 0U;
    for (const PlaneCorrespondence &correspondence : plane_evaluation.correspondences) {
      if (template_anchor_plane_indices.count(correspondence.template_index) == 0U) {
        continue;
      }
      const TemplatePlaneCluster &template_plane =
        template_graph_.plane_clusters[correspondence.template_index];
      const auto &environment_plane = latest_plane_clusters_->clusters[correspondence.environment_index];
      if (!template_plane.has_centroid || !isFinite(environment_plane.centroid.x) ||
        !isFinite(environment_plane.centroid.y) || !isFinite(environment_plane.centroid.z))
      {
        continue;
      }
      const Vec3 rotated_centroid = rotateRpyRaw(
        template_plane.centroid, roll_deg, pitch_deg, yaw_deg + template_graph_.canonical_yaw_deg);
      translation.x += static_cast<double>(environment_plane.centroid.x) - rotated_centroid.x;
      translation.y += static_cast<double>(environment_plane.centroid.y) - rotated_centroid.y;
      translation.z += static_cast<double>(environment_plane.centroid.z) - rotated_centroid.z;
      ++correspondence_num;
    }
    if (correspondence_num == 0U) {
      return {{}, false};
    }
    const double inverse_correspondence_num = 1.0 / static_cast<double>(correspondence_num);
    translation.x *= inverse_correspondence_num;
    translation.y *= inverse_correspondence_num;
    translation.z *= inverse_correspondence_num;
    return {translation, true};
  }

  double matchingPlaneScale(const PlaneEvaluation &plane_evaluation) const
  {
    double scale = 0.0;
    for (const PlaneCorrespondence &correspondence : plane_evaluation.correspondences) {
      const TemplatePlaneCluster &template_plane =
        template_graph_.plane_clusters[correspondence.template_index];
      const auto &environment_plane = latest_plane_clusters_->clusters[correspondence.environment_index];
      scale = std::max(scale, template_plane.max_extent);
      scale = std::max(
        scale,
        std::max(std::abs(static_cast<double>(environment_plane.extent_u)),
          std::abs(static_cast<double>(environment_plane.extent_v))));
    }
    return std::max(scale, 0.03);
  }

  double nonplaneNodePositionScore(
    const TemplateNode &template_node,
    const ais_gng_msgs::msg::TopologicalNode &environment_node,
    const Vec3 &translation,
    double plane_scale,
    double roll_deg,
    double pitch_deg,
    double yaw_deg) const
  {
    const Vec3 rotated_position = rotateRpyRaw(
      template_node.position, roll_deg, pitch_deg, yaw_deg + template_graph_.canonical_yaw_deg);
    const double dx = rotated_position.x + translation.x - static_cast<double>(environment_node.pos.x);
    const double dy = rotated_position.y + translation.y - static_cast<double>(environment_node.pos.y);
    const double dz = rotated_position.z + translation.z - static_cast<double>(environment_node.pos.z);
    return descendingMembership(
      std::sqrt(dx * dx + dy * dy + dz * dz) / plane_scale,
      0.05 + 0.10 * shape_tolerance_, 0.22 + 0.35 * shape_tolerance_);
  }

  double nonplaneEdgeFeatureScore(
    const NonplaneEdgeFeature &template_feature,
    const NonplaneEdgeFeature &environment_feature,
    const Vec3 &translation,
    bool has_translation,
    double roll_deg,
    double pitch_deg,
    double yaw_deg) const
  {
    const Vec3 rotated_direction = rotateRpy(
      template_feature.direction, roll_deg, pitch_deg, yaw_deg + template_graph_.canonical_yaw_deg);
    const double direction_score = descendingMembership(
      1.0 - std::abs(dot(rotated_direction, environment_feature.direction)),
      0.04 + 0.10 * shape_tolerance_, 0.25 + 0.50 * shape_tolerance_);
    const double length_score = descendingMembership(
      std::abs(std::log(std::max(template_feature.normalized_length, 1e-6) /
        std::max(environment_feature.normalized_length, 1e-6))),
      0.08 + 0.20 * shape_tolerance_, 0.35 + 0.70 * shape_tolerance_);
    const double normal_alignment_score = descendingMembership(
      std::abs(template_feature.normal_alignment - environment_feature.normal_alignment),
      0.04 + 0.10 * shape_tolerance_, 0.25 + 0.50 * shape_tolerance_);
    double score_sum = 0.25 * direction_score + 0.15 * length_score +
      0.15 * normal_alignment_score;
    double weight_sum = 0.55;
    if (has_translation) {
      const Vec3 rotated_midpoint = rotateRpyRaw(
        template_feature.midpoint, roll_deg, pitch_deg, yaw_deg + template_graph_.canonical_yaw_deg);
      const double midpoint_dx = rotated_midpoint.x + translation.x - environment_feature.midpoint.x;
      const double midpoint_dy = rotated_midpoint.y + translation.y - environment_feature.midpoint.y;
      const double midpoint_dz = rotated_midpoint.z + translation.z - environment_feature.midpoint.z;
      const double position_score = descendingMembership(
        std::sqrt(midpoint_dx * midpoint_dx + midpoint_dy * midpoint_dy + midpoint_dz * midpoint_dz) /
          std::max(template_feature.plane_scale, environment_feature.plane_scale),
        0.08 + 0.12 * shape_tolerance_, 0.35 + 0.50 * shape_tolerance_);
      score_sum += 0.25 * position_score;
      weight_sum += 0.25;
    }
    if (!template_feature.has_plane_distances || !environment_feature.has_plane_distances) {
      return score_sum / weight_sum;
    }
    const double first_distance_score = descendingMembership(
      std::abs(template_feature.first_plane_dist - environment_feature.first_plane_dist) /
        std::max(template_feature.plane_scale, environment_feature.plane_scale),
      0.10 + 0.20 * shape_tolerance_, 0.45 + 0.75 * shape_tolerance_);
    const double second_distance_score = descendingMembership(
      std::abs(template_feature.second_plane_dist - environment_feature.second_plane_dist) /
        std::max(template_feature.plane_scale, environment_feature.plane_scale),
      0.10 + 0.20 * shape_tolerance_, 0.45 + 0.75 * shape_tolerance_);
    score_sum += 0.225 * first_distance_score + 0.225 * second_distance_score;
    weight_sum += 0.45;
    return score_sum / weight_sum;
  }

  NonplaneEdgeFit evaluateNonplaneEdgeFit(
    const std::vector<NonplaneEdgeFeature> &template_features,
    const std::vector<NonplaneEdgeFeature> &environment_features,
    const Vec3 &translation,
    bool has_translation,
    double roll_deg,
    double pitch_deg,
    double yaw_deg) const
  {
    NonplaneEdgeFit result;
    result.edge_num = environment_features.size();
    if (template_features.empty() || environment_features.empty()) {
      return result;
    }
    double score_sum = 0.0;
    for (const NonplaneEdgeFeature &environment_feature : environment_features) {
      double best_score = 0.0;
      for (const NonplaneEdgeFeature &template_feature : template_features) {
        best_score = std::max(
          best_score,
          nonplaneEdgeFeatureScore(
            template_feature, environment_feature, translation, has_translation,
            roll_deg, pitch_deg, yaw_deg));
      }
      score_sum += best_score;
      if (best_score >= kMinShapePairScore) {
        ++result.explained_edge_num;
      }
    }
    result.score = score_sum / static_cast<double>(environment_features.size());
    return result;
  }

  NonplaneEvaluation evaluateNonplaneComponents(
    const ais_gng_msgs::msg::TopologicalMap &environment,
    const std::vector<std::pair<std::size_t, std::size_t>> &environment_edges,
    const PlaneEvaluation &plane_evaluation,
    double roll_deg,
    double pitch_deg,
    double yaw_deg) const
  {
    NonplaneEvaluation result;
    if (!enable_nonplane_component_evaluation_ || template_graph_.nonplane_components.empty() ||
      plane_evaluation.correspondences.empty() || latest_plane_clusters_ == nullptr)
    {
      return result;
    }
    std::vector<std::unordered_set<std::size_t>> template_node_plane_indices(
      template_graph_.nodes.size());
    for (std::size_t plane_index = 0U; plane_index < template_graph_.plane_clusters.size(); ++plane_index) {
      for (const std::size_t node_index : template_graph_.plane_clusters[plane_index].node_indices) {
        if (node_index < template_node_plane_indices.size()) {
          template_node_plane_indices[node_index].insert(plane_index);
        }
      }
    }
    std::vector<std::unordered_set<std::size_t>> environment_node_plane_indices(environment.nodes.size());
    for (std::size_t plane_index = 0U; plane_index < latest_plane_clusters_->clusters.size(); ++plane_index) {
      for (const std::uint32_t node_index : latest_plane_clusters_->clusters[plane_index].node_indices) {
        if (node_index < environment_node_plane_indices.size()) {
          environment_node_plane_indices[node_index].insert(plane_index);
        }
      }
    }

    std::unordered_map<std::uint32_t, std::size_t> component_index_by_id;
    std::vector<NonplaneComponent> environment_components;
    for (std::size_t node_index = 0U; node_index < environment.nodes.size(); ++node_index) {
      const std::uint32_t component_id = environment.nodes[node_index].nonplane_component_id;
      if (component_id == ais_gng_msgs::msg::TopologicalNode::NONPLANE_COMPONENT_NONE) {
        continue;
      }
      const auto [iterator, is_inserted] = component_index_by_id.emplace(
        component_id, environment_components.size());
      if (is_inserted) {
        environment_components.push_back({component_id, {}, {}, 0U});
      }
      environment_components[iterator->second].node_indices.push_back(node_index);
    }
    for (const auto &[first, second] : environment_edges) {
      const std::uint32_t first_component_id = environment.nodes[first].nonplane_component_id;
      const std::uint32_t second_component_id = environment.nodes[second].nonplane_component_id;
      const auto first_component = component_index_by_id.find(first_component_id);
      const auto second_component = component_index_by_id.find(second_component_id);
      if (first_component != component_index_by_id.end() && second_component != component_index_by_id.end() &&
        first_component->second == second_component->second)
      {
        ++environment_components[first_component->second].internal_edge_num;
      }
      if (first_component != component_index_by_id.end()) {
        environment_components[first_component->second].anchor_plane_indices.insert(
          environment_node_plane_indices[second].begin(), environment_node_plane_indices[second].end());
      }
      if (second_component != component_index_by_id.end()) {
        environment_components[second_component->second].anchor_plane_indices.insert(
          environment_node_plane_indices[first].begin(), environment_node_plane_indices[first].end());
      }
    }
    environment_components.erase(
      std::remove_if(
        environment_components.begin(), environment_components.end(),
        [this](const NonplaneComponent &component) {
          return component.node_indices.size() < static_cast<std::size_t>(min_nonplane_component_nodes_);
        }),
      environment_components.end());
    result.environment_component_num = environment_components.size();
    const auto environment_neighbors = buildNeighbors(environment.nodes.size(), environment_edges);

    double evidence_sum = 0.0;
    for (const NonplaneComponent &template_component : template_graph_.nonplane_components) {
      if (template_component.node_indices.size() < static_cast<std::size_t>(min_nonplane_component_nodes_) ||
        template_component.anchor_plane_indices.empty())
      {
        continue;
      }
      std::unordered_set<std::size_t> expected_environment_plane_indices;
      for (const PlaneCorrespondence &correspondence : plane_evaluation.correspondences) {
        if (template_component.anchor_plane_indices.count(correspondence.template_index) > 0U) {
          expected_environment_plane_indices.insert(correspondence.environment_index);
        }
      }
      if (expected_environment_plane_indices.empty()) {
        continue;
      }
      const auto template_patch_node_indices = makeAttachmentPatchNodeIndices(
        template_component, template_graph_.neighbors, template_node_plane_indices,
        template_component.anchor_plane_indices);
      const auto template_patch_edges = makeAttachmentPatchEdges(
        template_graph_.edges, template_component, template_patch_node_indices);
      const auto template_features = makeTemplateNonplaneEdgeFeatures(
        template_patch_edges, template_component.anchor_plane_indices);
      if (template_features.empty()) {
        continue;
      }
      const auto [translation, has_translation] = estimateAttachmentTranslation(
        template_component.anchor_plane_indices, plane_evaluation, roll_deg, pitch_deg, yaw_deg);

      const NonplaneComponent *best_environment_component = nullptr;
      NonplaneEdgeFit best_edge_fit;
      double best_component_score = 0.0;
      std::size_t best_anchor_num = 0U;
      std::size_t best_environment_patch_edge_num = 0U;
      for (const NonplaneComponent &environment_component : environment_components) {
        std::size_t matching_anchor_num = 0U;
        for (const std::size_t anchor_plane_index : environment_component.anchor_plane_indices) {
          if (expected_environment_plane_indices.count(anchor_plane_index) > 0U) {
            ++matching_anchor_num;
          }
        }
        if (matching_anchor_num == 0U) {
          continue;
        }
        const auto environment_patch_node_indices = makeAttachmentPatchNodeIndices(
          environment_component, environment_neighbors,
          environment_node_plane_indices, expected_environment_plane_indices);
        const auto environment_patch_edges = makeAttachmentPatchEdges(
          environment_edges, environment_component, environment_patch_node_indices);
        const auto environment_features = makeEnvironmentNonplaneEdgeFeatures(
          environment, environment_patch_edges, expected_environment_plane_indices);
        const NonplaneEdgeFit edge_fit = evaluateNonplaneEdgeFit(
          template_features, environment_features, translation, has_translation,
          roll_deg, pitch_deg, yaw_deg);
        if (edge_fit.edge_num == 0U) {
          continue;
        }
        const double anchor_score = static_cast<double>(matching_anchor_num) /
          static_cast<double>(expected_environment_plane_indices.size());
        const double edge_evidence_score = edge_fit.score *
          (1.0 - std::exp(-static_cast<double>(edge_fit.edge_num) / 3.0));
        const double component_score = anchor_score * edge_evidence_score;
        if (component_score > best_component_score) {
          best_component_score = component_score;
          best_environment_component = &environment_component;
          best_edge_fit = edge_fit;
          best_anchor_num = matching_anchor_num;
          best_environment_patch_edge_num = environment_patch_edges.size();
        }
      }
      if (best_environment_component == nullptr) {
        continue;
      }
      evidence_sum += best_component_score;
      result.edge_fit_score = std::max(result.edge_fit_score, best_edge_fit.score);
      json environment_plane_cluster_ids = json::array();
      for (const std::size_t plane_index : best_environment_component->anchor_plane_indices) {
        if (expected_environment_plane_indices.count(plane_index) > 0U) {
          environment_plane_cluster_ids.push_back(latest_plane_clusters_->clusters[plane_index].id);
        }
      }
      result.correspondences.push_back({
        {"template_component_id", template_component.id},
        {"template_node_num", template_component.node_indices.size()},
        {"template_patch_edge_num", template_patch_edges.size()},
        {"environment_component_id", best_environment_component->id},
        {"environment_node_num", best_environment_component->node_indices.size()},
        {"environment_patch_edge_num", best_environment_patch_edge_num},
        {"matching_anchor_num", best_anchor_num},
        {"environment_plane_cluster_ids", std::move(environment_plane_cluster_ids)},
        {"edge_fit_score", best_edge_fit.score},
        {"explained_edge_num", best_edge_fit.explained_edge_num},
        {"observed_edge_num", best_edge_fit.edge_num},
        {"unexplained_edge_ratio", 1.0 - best_edge_fit.score},
        {"score", best_component_score}});
    }
    result.is_observed = !result.correspondences.empty();
    result.evidence_score = 1.0 - std::exp(-evidence_sum / nonplane_evidence_score_scale_);
    return result;
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
    std::unordered_set<std::size_t> matched_template_plane_indices;
    matched_template_plane_indices.reserve(plane_evaluation.correspondences.size());
    for (const PlaneCorrespondence &correspondence : plane_evaluation.correspondences) {
      matched_template_plane_indices.insert(correspondence.template_index);
    }
    const auto [plane_translation, has_plane_translation] = estimateAttachmentTranslation(
      matched_template_plane_indices, plane_evaluation, roll_deg, pitch_deg, yaw_deg);
    const double matching_plane_scale = matchingPlaneScale(plane_evaluation);
    const NonplaneEvaluation nonplane_evaluation = evaluateNonplaneComponents(
      environment, filtered_environment_edges, plane_evaluation, roll_deg, pitch_deg, yaw_deg);
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
        double score = nodeScore(
          template_graph_.nodes[template_index], environment.nodes[environment_index],
          roll_deg, pitch_deg, yaw_deg);
        if (expected_environment_plane < 0 && has_plane_translation) {
          const double position_score = nonplaneNodePositionScore(
            template_graph_.nodes[template_index], environment.nodes[environment_index],
            plane_translation, matching_plane_scale, roll_deg, pitch_deg, yaw_deg);
          score = 0.40 * score + 0.60 * position_score;
        }
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
    const double base_score = std::cbrt(std::max(0.0, node_score * relation_score * visible_ratio));
    const double score = std::min(1.0, base_score + nonplane_weight_ * nonplane_evaluation.evidence_score);
    const bool has_strong_plane_evidence = plane_evaluation.support_score >= min_plane_support_score_;
    const bool has_nonplane_evidence = nonplane_evaluation.is_observed &&
      nonplane_evaluation.evidence_score >= min_nonplane_evidence_score_th_;
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
      {"base_score", base_score},
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
      {"is_plane_relative_height_observed", plane_evaluation.is_relative_height_observed},
      {"plane_relative_height_pair_num", plane_evaluation.relative_height_pair_num},
      {"plane_relative_height_score", plane_evaluation.relative_height_score},
      {"has_strong_plane_evidence", has_strong_plane_evidence},
      {"is_nonplane_component_observed", nonplane_evaluation.is_observed},
      {"template_nonplane_component_num", template_graph_.nonplane_components.size()},
      {"environment_nonplane_component_num", nonplane_evaluation.environment_component_num},
      {"nonplane_evidence_score", nonplane_evaluation.evidence_score},
      {"nonplane_edge_fit_score", nonplane_evaluation.edge_fit_score},
      {"has_nonplane_evidence", has_nonplane_evidence},
      {"nonplane_correspondences", std::move(nonplane_evaluation.correspondences)},
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
  std::vector<std::string> template_ids_;
  std::vector<std::string> template_dataset_paths_;
  std::vector<double> template_roll_tolerance_degs_;
  std::vector<double> template_pitch_tolerance_degs_;
  std::string environment_topic_;
  std::string plane_clusters_topic_;
  std::string candidate_topic_;
  std::vector<std::string> candidate_topics_;
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
  double normal_weight_ = 1.0;
  double rho_weight_ = 0.0;
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
  bool enable_plane_relative_height_evaluation_ = true;
  double max_plane_relative_height_dev_full_ = 0.0;
  double max_plane_relative_height_dev_partial_ = 1.0;
  double plane_relative_height_weight_ = 0.0;
  bool enable_nonplane_component_evaluation_ = true;
  int min_nonplane_component_nodes_ = 2;
  double nonplane_weight_ = 0.0;
  double nonplane_evidence_score_scale_ = 1.0;
  double min_nonplane_evidence_score_th_ = 1.0;
  TemplateGraph template_graph_;
  std::vector<LoadedTemplate> loaded_templates_;
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> template_retrieval_index_;
  std::vector<PlaneRetrievalToken> template_plane_retrieval_tokens_;
  std::vector<std::size_t> template_plane_min_extent_order_;
  std::vector<std::size_t> template_plane_max_extent_order_;
  std::size_t active_template_index_ = 0U;
  bool has_active_template_ = false;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_publisher_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr environment_subscription_;
  rclcpp::Subscription<ais_gng_msgs::msg::PlaneClusterArray>::SharedPtr plane_clusters_subscription_;
  ais_gng_msgs::msg::PlaneClusterArray::SharedPtr latest_plane_clusters_;
  std::map<std::uint32_t, ais_gng_msgs::msg::TopologicalMap::SharedPtr> pending_environment_maps_;
  std::map<std::uint32_t, ais_gng_msgs::msg::PlaneClusterArray::SharedPtr> pending_plane_clusters_;
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
