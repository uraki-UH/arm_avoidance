#include <zlib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace
{

using json = nlohmann::json;

constexpr double kPi = 3.14159265358979323846;
constexpr double kPairBinSize = 0.008;
constexpr double kTranslationBinSize = 0.015;
constexpr double kYawBinDeg = 5.0;
constexpr double kMinPairXyDist = 0.012;
constexpr double kMaxPairXyDist = 0.180;
constexpr int kPairBinDev = 1;
constexpr std::size_t kMaxIndexedPairNumPerTemplateKey = 8U;
constexpr std::size_t kMaxPoseNumPerTemplate = 4U;
constexpr double kStrictSupportDistTh = 0.0125;
constexpr double kRelaxedSupportDistTh = 0.025;
constexpr double kPlaneToNonplaneSupportWeight = 0.5;
constexpr std::size_t kTopResultNum = 20U;

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Graph
{
  std::string template_id;
  std::vector<Vec3> nodes;
  std::vector<std::pair<std::size_t, std::size_t>> edges;
  std::vector<bool> is_plane_node;
};

struct DirectedPair
{
  std::size_t first_index = 0U;
  std::size_t second_index = 0U;
  int xy_dist_bin = 0;
  int z_dev_bin = 0;
  double bearing_rad = 0.0;
};

struct IndexedPair
{
  std::size_t template_index = 0U;
  DirectedPair pair;
};

struct VoteKey
{
  std::size_t template_index = 0U;
  int yaw_bin = 0;
  int x_bin = 0;
  int y_bin = 0;
  int z_bin = 0;

  bool operator==(const VoteKey &other) const
  {
    return template_index == other.template_index && yaw_bin == other.yaw_bin &&
      x_bin == other.x_bin && y_bin == other.y_bin && z_bin == other.z_bin;
  }
};

struct VoteKeyHash
{
  std::size_t operator()(const VoteKey &key) const
  {
    std::size_t result = key.template_index;
    for (const int value : {key.yaw_bin, key.x_bin, key.y_bin, key.z_bin}) {
      result ^= std::hash<int>{}(value) + 0x9e3779b9U + (result << 6U) + (result >> 2U);
    }
    return result;
  }
};

struct VoteResult
{
  std::size_t template_index = 0U;
  std::uint32_t vote_num = 0U;
  double yaw_deg = 0.0;
  Vec3 translation;
};

struct ScoredResult
{
  VoteResult vote_result;
  double strict_support_ratio = 0.0;
  double strict_compatible_support_ratio = 0.0;
  double strict_nonplane_to_plane_support_ratio = 0.0;
  double support_ratio = 0.0;
  double compatible_support_ratio = 0.0;
  double nonplane_to_plane_support_ratio = 0.0;
  double template_support_ratio = 0.0;
  double precise_support = 0.0;
  double balanced_support = 0.0;
  double mean_dist = 0.0;
};

bool isFinite(const double value)
{
  return std::isfinite(value);
}

Vec3 subtract(const Vec3 &first, const Vec3 &second)
{
  return {first.x - second.x, first.y - second.y, first.z - second.z};
}

double wrapDeg(double value)
{
  while (value >= 180.0) {
    value -= 360.0;
  }
  while (value < -180.0) {
    value += 360.0;
  }
  return value;
}

int quantize(const double value, const double bin_size)
{
  return static_cast<int>(std::llround(value / bin_size));
}

double calculateBalancedSupport(const double support_ratio, const double template_support_ratio)
{
  const double denominator = support_ratio + template_support_ratio;
  return denominator > 0.0 ? 2.0 * support_ratio * template_support_ratio / denominator : 0.0;
}

Vec3 transformByYaw(const Vec3 &point, const VoteResult &result)
{
  const double yaw_rad = result.yaw_deg * kPi / 180.0;
  return {
    std::cos(yaw_rad) * point.x - std::sin(yaw_rad) * point.y + result.translation.x,
    std::sin(yaw_rad) * point.x + std::cos(yaw_rad) * point.y + result.translation.y,
    point.z + result.translation.z};
}

ScoredResult scoreResult(
  const Graph &query, const Graph &template_graph, const VoteResult &vote_result)
{
  std::vector<Vec3> transformed_nodes;
  transformed_nodes.reserve(query.nodes.size());
  std::size_t strict_supported_node_num = 0U;
  std::size_t strict_nonplane_to_plane_supported_node_num = 0U;
  double strict_compatible_supported_node_num = 0.0;
  std::size_t supported_node_num = 0U;
  std::size_t nonplane_to_plane_supported_node_num = 0U;
  double compatible_supported_node_num = 0.0;
  double sum_dist = 0.0;
  for (std::size_t query_node_index = 0U; query_node_index < query.nodes.size(); ++query_node_index) {
    const Vec3 &query_node = query.nodes[query_node_index];
    const Vec3 transformed_node = transformByYaw(query_node, vote_result);
    transformed_nodes.push_back(transformed_node);
    double min_dist_squared = std::numeric_limits<double>::infinity();
    std::size_t min_template_node_index = 0U;
    for (std::size_t template_node_index = 0U;
      template_node_index < template_graph.nodes.size(); ++template_node_index)
    {
      const Vec3 &template_node = template_graph.nodes[template_node_index];
      const Vec3 delta = subtract(transformed_node, template_node);
      const double dist_squared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
      if (dist_squared < min_dist_squared) {
        min_dist_squared = dist_squared;
        min_template_node_index = template_node_index;
      }
    }
    const double min_dist = std::sqrt(min_dist_squared);
    const bool query_is_plane = query.is_plane_node[query_node_index];
    const bool template_is_plane = template_graph.is_plane_node[min_template_node_index];
    const double compatible_weight = query_is_plane && !template_is_plane ?
      kPlaneToNonplaneSupportWeight : 1.0;
    if (min_dist <= kStrictSupportDistTh) {
      ++strict_supported_node_num;
      strict_compatible_supported_node_num += compatible_weight;
      if (!query_is_plane && template_is_plane) {
        ++strict_nonplane_to_plane_supported_node_num;
      }
    }
    if (min_dist <= kRelaxedSupportDistTh) {
      ++supported_node_num;
      if (!query_is_plane && template_is_plane) {
        ++nonplane_to_plane_supported_node_num;
      }
      compatible_supported_node_num += compatible_weight;
    }
    sum_dist += std::min(min_dist, kRelaxedSupportDistTh);
  }
  std::size_t supported_template_node_num = 0U;
  for (const Vec3 &template_node : template_graph.nodes) {
    double min_dist_squared = std::numeric_limits<double>::infinity();
    for (const Vec3 &transformed_node : transformed_nodes) {
      const Vec3 delta = subtract(template_node, transformed_node);
      const double dist_squared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
      min_dist_squared = std::min(min_dist_squared, dist_squared);
    }
    if (std::sqrt(min_dist_squared) <= kRelaxedSupportDistTh) {
      ++supported_template_node_num;
    }
  }
  const double strict_support_ratio =
    static_cast<double>(strict_supported_node_num) / static_cast<double>(query.nodes.size());
  const double strict_compatible_support_ratio =
    strict_compatible_supported_node_num / static_cast<double>(query.nodes.size());
  const double strict_nonplane_to_plane_support_ratio =
    static_cast<double>(strict_nonplane_to_plane_supported_node_num) /
    static_cast<double>(query.nodes.size());
  const double support_ratio =
    static_cast<double>(supported_node_num) / static_cast<double>(query.nodes.size());
  const double compatible_support_ratio =
    compatible_supported_node_num / static_cast<double>(query.nodes.size());
  const double nonplane_to_plane_support_ratio =
    static_cast<double>(nonplane_to_plane_supported_node_num) /
    static_cast<double>(query.nodes.size());
  const double template_support_ratio = static_cast<double>(supported_template_node_num) /
    static_cast<double>(template_graph.nodes.size());
  return {
    vote_result,
    strict_support_ratio,
    strict_compatible_support_ratio,
    strict_nonplane_to_plane_support_ratio,
    support_ratio,
    compatible_support_ratio,
    nonplane_to_plane_support_ratio,
    template_support_ratio,
    calculateBalancedSupport(strict_compatible_support_ratio, template_support_ratio),
    calculateBalancedSupport(compatible_support_ratio, template_support_ratio),
    sum_dist / static_cast<double>(query.nodes.size())};
}

std::uint64_t makePairKey(const int xy_dist_bin, const int z_dev_bin)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(xy_dist_bin)) << 32U) |
    static_cast<std::uint32_t>(z_dev_bin);
}

std::string readDatasetText(const std::filesystem::path &dataset_path)
{
  if (dataset_path.extension() != ".gz") {
    std::ifstream stream(dataset_path);
    if (!stream) {
      throw std::runtime_error("テンプレートJSONを開けません: " + dataset_path.string());
    }
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  }

  gzFile file = gzopen(dataset_path.c_str(), "rb");
  if (file == nullptr) {
    throw std::runtime_error("テンプレートgzipを開けません: " + dataset_path.string());
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
    throw std::runtime_error("テンプレートgzipの読込失敗: " + dataset_path.string());
  }
  return text;
}

Graph loadGraph(const std::filesystem::path &dataset_path)
{
  const json root = json::parse(readDatasetText(dataset_path));
  const json *template_root = &root;
  if (root.contains("gng_template") && root.at("gng_template").is_object()) {
    template_root = &root.at("gng_template");
  }
  if (!template_root->contains("gng") || !template_root->at("gng").is_object()) {
    throw std::runtime_error("GNGがありません: " + dataset_path.string());
  }
  const json &gng = template_root->at("gng");
  if (!gng.contains("nodes") || !gng.at("nodes").is_array()) {
    throw std::runtime_error("GNG node配列がありません: " + dataset_path.string());
  }

  Graph graph;
  graph.template_id = root.value(
    "template_id", template_root->value("template_id", dataset_path.stem().string()));
  std::vector<std::size_t> source_to_node_index;
  source_to_node_index.reserve(gng.at("nodes").size());
  for (const json &node_json : gng.at("nodes")) {
    source_to_node_index.push_back(std::numeric_limits<std::size_t>::max());
    if (!node_json.is_object()) {
      continue;
    }
    const Vec3 position = {
      node_json.value("x", std::numeric_limits<double>::quiet_NaN()),
      node_json.value("y", std::numeric_limits<double>::quiet_NaN()),
      node_json.value("z", std::numeric_limits<double>::quiet_NaN())};
    if (isFinite(position.x) && isFinite(position.y) && isFinite(position.z)) {
      source_to_node_index.back() = graph.nodes.size();
      graph.nodes.push_back(position);
    }
  }
  if (graph.nodes.size() < 3U) {
    throw std::runtime_error("GNG node数が不足しています: " + dataset_path.string());
  }
  graph.is_plane_node.assign(graph.nodes.size(), false);
  if (gng.contains("plane_clusters") && gng.at("plane_clusters").is_array()) {
    for (const json &plane_cluster_json : gng.at("plane_clusters")) {
      if (!plane_cluster_json.is_object() || !plane_cluster_json.contains("idx") ||
        !plane_cluster_json.at("idx").is_array())
      {
        continue;
      }
      for (const json &source_index_json : plane_cluster_json.at("idx")) {
        if (!source_index_json.is_number_unsigned()) {
          continue;
        }
        const std::size_t source_index = source_index_json.get<std::size_t>();
        if (source_index >= source_to_node_index.size()) {
          continue;
        }
        const std::size_t node_index = source_to_node_index[source_index];
        if (node_index < graph.is_plane_node.size()) {
          graph.is_plane_node[node_index] = true;
        }
      }
    }
  }

  std::set<std::pair<std::size_t, std::size_t>> edge_set;
  if (gng.contains("edges") && gng.at("edges").is_array()) {
    for (const json &edge_json : gng.at("edges")) {
      if (!edge_json.is_array() || edge_json.size() < 2U ||
        !edge_json.at(0).is_number_unsigned() || !edge_json.at(1).is_number_unsigned())
      {
        continue;
      }
      const std::size_t first_index = edge_json.at(0).get<std::size_t>();
      const std::size_t second_index = edge_json.at(1).get<std::size_t>();
      if (first_index >= graph.nodes.size() || second_index >= graph.nodes.size() ||
        first_index == second_index)
      {
        continue;
      }
      edge_set.emplace(
        std::min(first_index, second_index), std::max(first_index, second_index));
    }
  }
  graph.edges.assign(edge_set.begin(), edge_set.end());
  if (graph.edges.empty()) {
    throw std::runtime_error("GNG edgeがありません: " + dataset_path.string());
  }
  return graph;
}

std::vector<DirectedPair> makePairs(const Graph &graph)
{
  std::vector<DirectedPair> pairs;
  pairs.reserve(graph.edges.size() * 2U);
  const auto append_pair = [&graph, &pairs](
      const std::size_t first_index, const std::size_t second_index) {
      const Vec3 delta = subtract(graph.nodes[second_index], graph.nodes[first_index]);
      const double xy_dist = std::hypot(delta.x, delta.y);
      if (xy_dist < kMinPairXyDist || xy_dist > kMaxPairXyDist) {
        return;
      }
      pairs.push_back({
        first_index,
        second_index,
        quantize(xy_dist, kPairBinSize),
        quantize(delta.z, kPairBinSize),
        std::atan2(delta.y, delta.x)});
    };
  for (const auto &[first_index, second_index] : graph.edges) {
    append_pair(first_index, second_index);
    append_pair(second_index, first_index);
  }
  return pairs;
}

bool hasTemplateSuffix(const std::string &name)
{
  constexpr std::string_view gzip_suffix = "_gng_template.json.gz";
  constexpr std::string_view json_suffix = "_gng_template.json";
  return (name.size() >= gzip_suffix.size() &&
    name.compare(name.size() - gzip_suffix.size(), gzip_suffix.size(), gzip_suffix) == 0) ||
    (name.size() >= json_suffix.size() &&
    name.compare(name.size() - json_suffix.size(), json_suffix.size(), json_suffix) == 0);
}

std::vector<std::filesystem::path> findTemplatePaths(const std::filesystem::path &dataset_dir)
{
  std::vector<std::filesystem::path> paths;
  for (const auto &entry : std::filesystem::directory_iterator(dataset_dir)) {
    if (entry.is_regular_file() && hasTemplateSuffix(entry.path().filename().string())) {
      paths.push_back(entry.path());
    }
  }
  std::sort(paths.begin(), paths.end());
  return paths;
}

void printUsage()
{
  std::cerr <<
    "使用法: yaw_pair_vote_retrieval dataset_dir query_dataset [除外template_id ...]" << std::endl;
}

}  // 無名namespace

int main(int argc, char **argv)
{
  if (argc < 3) {
    printUsage();
    return 1;
  }
  try {
    const std::filesystem::path dataset_dir(argv[1]);
    const Graph query = loadGraph(argv[2]);
    std::set<std::string> excluded_ids;
    excluded_ids.insert(query.template_id);
    for (int index = 3; index < argc; ++index) {
      excluded_ids.insert(argv[index]);
    }

    std::vector<Graph> templates;
    std::unordered_map<std::uint64_t, std::vector<IndexedPair>> pair_index;
    for (const auto &template_path : findTemplatePaths(dataset_dir)) {
      Graph graph = loadGraph(template_path);
      if (excluded_ids.count(graph.template_id) > 0U) {
        continue;
      }
      const std::size_t template_index = templates.size();
      for (const DirectedPair &pair : makePairs(graph)) {
        pair_index[makePairKey(pair.xy_dist_bin, pair.z_dev_bin)].push_back(
          {template_index, pair});
      }
      templates.push_back(std::move(graph));
    }
    if (templates.empty()) {
      throw std::runtime_error("照合対象テンプレートがありません。");
    }

    for (auto &[pair_key, indexed_pairs] : pair_index) {
      static_cast<void>(pair_key);
      std::sort(indexed_pairs.begin(), indexed_pairs.end(), [](
          const IndexedPair &first, const IndexedPair &second) {
          if (first.template_index != second.template_index) {
            return first.template_index < second.template_index;
          }
          if (first.pair.first_index != second.pair.first_index) {
            return first.pair.first_index < second.pair.first_index;
          }
          return first.pair.second_index < second.pair.second_index;
        });
      std::vector<IndexedPair> selected_pairs;
      selected_pairs.reserve(indexed_pairs.size());
      for (std::size_t begin_index = 0U; begin_index < indexed_pairs.size();) {
        std::size_t end_index = begin_index + 1U;
        while (end_index < indexed_pairs.size() &&
          indexed_pairs[end_index].template_index == indexed_pairs[begin_index].template_index)
        {
          ++end_index;
        }
        const std::size_t group_size = end_index - begin_index;
        const std::size_t selected_num = std::min(kMaxIndexedPairNumPerTemplateKey, group_size);
        for (std::size_t selected_index = 0U; selected_index < selected_num; ++selected_index) {
          selected_pairs.push_back(indexed_pairs[begin_index +
            selected_index * group_size / selected_num]);
        }
        begin_index = end_index;
      }
      indexed_pairs = std::move(selected_pairs);
    }

    std::unordered_map<VoteKey, std::uint32_t, VoteKeyHash> votes;
    const std::vector<DirectedPair> query_pairs = makePairs(query);
    for (const DirectedPair &query_pair : query_pairs) {
      for (int xy_offset = -kPairBinDev; xy_offset <= kPairBinDev; ++xy_offset) {
        for (int z_offset = -kPairBinDev; z_offset <= kPairBinDev; ++z_offset) {
          const auto index_iterator = pair_index.find(makePairKey(
            query_pair.xy_dist_bin + xy_offset, query_pair.z_dev_bin + z_offset));
          if (index_iterator == pair_index.end()) {
            continue;
          }
          for (const IndexedPair &indexed_pair : index_iterator->second) {
            const double yaw_deg = wrapDeg((indexed_pair.pair.bearing_rad - query_pair.bearing_rad) *
                180.0 / kPi);
            const int yaw_bin = quantize(yaw_deg, kYawBinDeg);
            const double yaw_rad = static_cast<double>(yaw_bin) * kYawBinDeg * kPi / 180.0;
            const Vec3 &query_anchor = query.nodes[query_pair.first_index];
            const Vec3 &template_anchor =
              templates[indexed_pair.template_index].nodes[indexed_pair.pair.first_index];
            const Vec3 rotated_query_anchor = {
              std::cos(yaw_rad) * query_anchor.x - std::sin(yaw_rad) * query_anchor.y,
              std::sin(yaw_rad) * query_anchor.x + std::cos(yaw_rad) * query_anchor.y,
              query_anchor.z};
            const Vec3 translation = subtract(template_anchor, rotated_query_anchor);
            ++votes[{
              indexed_pair.template_index,
              yaw_bin,
              quantize(translation.x, kTranslationBinSize),
              quantize(translation.y, kTranslationBinSize),
              quantize(translation.z, kTranslationBinSize)}];
          }
        }
      }
    }

    std::unordered_map<std::size_t, std::vector<VoteResult>> template_results;
    template_results.reserve(templates.size());
    for (const auto &[key, vote_num] : votes) {
      const VoteResult result = {
        key.template_index,
        vote_num,
        static_cast<double>(key.yaw_bin) * kYawBinDeg,
        {
          static_cast<double>(key.x_bin) * kTranslationBinSize,
          static_cast<double>(key.y_bin) * kTranslationBinSize,
          static_cast<double>(key.z_bin) * kTranslationBinSize}};
      template_results[key.template_index].push_back(result);
    }

    std::vector<ScoredResult> results;
    results.reserve(template_results.size());
    for (auto &[template_index, candidate_results] : template_results) {
      std::sort(candidate_results.begin(), candidate_results.end(), [](
          const VoteResult &first, const VoteResult &second) {
          return first.vote_num == second.vote_num ? first.yaw_deg < second.yaw_deg :
                 first.vote_num > second.vote_num;
        });
      const std::size_t candidate_num = std::min(kMaxPoseNumPerTemplate, candidate_results.size());
      ScoredResult best_result = scoreResult(query, templates[template_index], candidate_results.front());
      for (std::size_t candidate_index = 1U; candidate_index < candidate_num; ++candidate_index) {
        const ScoredResult score = scoreResult(
          query, templates[template_index], candidate_results[candidate_index]);
        if (score.strict_compatible_support_ratio > best_result.strict_compatible_support_ratio ||
          (score.strict_compatible_support_ratio == best_result.strict_compatible_support_ratio &&
          score.mean_dist < best_result.mean_dist))
        {
          best_result = score;
        }
      }
      results.push_back(best_result);
    }
    std::sort(results.begin(), results.end(), [](const ScoredResult &first, const ScoredResult &second) {
      return first.precise_support == second.precise_support ?
             (first.strict_compatible_support_ratio == second.strict_compatible_support_ratio ?
             (first.mean_dist == second.mean_dist ?
             first.vote_result.template_index < second.vote_result.template_index :
             first.mean_dist < second.mean_dist) :
             first.strict_compatible_support_ratio > second.strict_compatible_support_ratio) :
             first.precise_support > second.precise_support;
    });

    const std::size_t plane_node_num = static_cast<std::size_t>(std::count(
        query.is_plane_node.begin(), query.is_plane_node.end(), true));
    std::cout << "query=" << query.template_id << " nodes=" << query.nodes.size() <<
      " plane_nodes=" << plane_node_num << " pairs=" << query_pairs.size() << std::endl;
    std::cout << "rank template strict_support strict_compatible_support "
      "strict_nonplane_to_plane_support support compatible_support nonplane_to_plane_support "
      "template_support precise_support balanced_support mean_dist votes yaw_deg tx ty tz" << std::endl;
    for (std::size_t index = 0U; index < results.size() && index < kTopResultNum; ++index) {
      const ScoredResult &result = results[index];
      const VoteResult &vote_result = result.vote_result;
      std::cout << index + 1U << ' ' << templates[vote_result.template_index].template_id << ' ' <<
        result.strict_support_ratio << ' ' << result.strict_compatible_support_ratio << ' ' <<
        result.strict_nonplane_to_plane_support_ratio << ' ' << result.support_ratio << ' ' <<
        result.compatible_support_ratio << ' ' <<
        result.nonplane_to_plane_support_ratio << ' ' << result.template_support_ratio << ' ' <<
        result.precise_support << ' ' << result.balanced_support << ' ' << result.mean_dist << ' ' <<
        vote_result.vote_num << ' ' <<
        vote_result.yaw_deg << ' ' << vote_result.translation.x << ' ' <<
        vote_result.translation.y << ' ' << vote_result.translation.z << std::endl;
    }
  } catch (const std::exception &error) {
    std::cerr << "評価失敗: " << error.what() << std::endl;
    return 2;
  }
  return 0;
}
