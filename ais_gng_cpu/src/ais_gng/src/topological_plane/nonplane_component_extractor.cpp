#include "ais_gng/topological_plane/nonplane_component_extractor.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace fuzzrobo::topological_plane::nonplane
{
namespace
{

class disjoint_set
{
public:
  explicit disjoint_set(const std::size_t size)
  : parent_(size), rank_(size, 0U)
  {
    std::iota(parent_.begin(), parent_.end(), 0U);
  }

  std::size_t find(const std::size_t index)
  {
    std::size_t root = index;
    while (parent_[root] != root) {
      root = parent_[root];
    }
    std::size_t current = index;
    while (parent_[current] != current) {
      const std::size_t next = parent_[current];
      parent_[current] = root;
      current = next;
    }
    return root;
  }

  void unite(const std::size_t first, const std::size_t second)
  {
    std::size_t first_root = find(first);
    std::size_t second_root = find(second);
    if (first_root == second_root) {
      return;
    }
    if (rank_[first_root] < rank_[second_root]) {
      std::swap(first_root, second_root);
    }
    parent_[second_root] = first_root;
    if (rank_[first_root] == rank_[second_root]) {
      ++rank_[first_root];
    }
  }

private:
  std::vector<std::size_t> parent_;
  std::vector<std::uint8_t> rank_;
};

bool is_valid_edge(
  const ais_gng_msgs::msg::TopologicalMap &map, const std::size_t first,
  const std::size_t second)
{
  return first < map.nodes.size() && second < map.nodes.size();
}

}  // 無名名前空間

extraction_result extract_components(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &plane_clusters,
  const extractor_options &options)
{
  extraction_result result;
  result.map.header = map.header;
  result.map.frame_number = map.frame_number;
  const std::size_t node_count = map.nodes.size();
  if (node_count == 0U) {
    return result;
  }

  std::vector<int> plane_owner(node_count, -1);
  for (std::size_t cluster_index = 0U; cluster_index < plane_clusters.clusters.size(); ++cluster_index) {
    for (const std::uint32_t node_index : plane_clusters.clusters[cluster_index].node_indices) {
      if (node_index < node_count && plane_owner[node_index] < 0) {
        plane_owner[node_index] = static_cast<int>(cluster_index);
      }
    }
  }

  disjoint_set components(node_count);
  for (std::size_t edge_index = 0U; edge_index + 1U < map.edges.size(); edge_index += 2U) {
    const std::size_t first = map.edges[edge_index];
    const std::size_t second = map.edges[edge_index + 1U];
    if (!is_valid_edge(map, first, second) || plane_owner[first] >= 0 || plane_owner[second] >= 0) {
      continue;
    }
    components.unite(first, second);
  }

  std::vector<std::size_t> component_size(node_count, 0U);
  for (std::size_t node_index = 0U; node_index < node_count; ++node_index) {
    if (plane_owner[node_index] < 0) {
      ++component_size[components.find(node_index)];
    }
  }

  const std::size_t min_component_nodes = std::max<std::size_t>(1U, options.min_component_nodes);
  const std::size_t invalid_component = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> component_index_by_root(node_count, invalid_component);
  std::vector<std::size_t> output_index_by_source(node_count, invalid_component);
  result.map.nodes.reserve(node_count);

  for (std::size_t node_index = 0U; node_index < node_count; ++node_index) {
    if (plane_owner[node_index] >= 0) {
      continue;
    }
    const std::size_t root = components.find(node_index);
    if (component_size[root] < min_component_nodes) {
      continue;
    }
    std::size_t component_index = component_index_by_root[root];
    if (component_index == invalid_component) {
      component_index = result.map.clusters.size();
      component_index_by_root[root] = component_index;
      result.map.clusters.emplace_back();
      auto &cluster = result.map.clusters.back();
      cluster.id = static_cast<std::uint32_t>(component_index);
      cluster.label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
      cluster.label_inferred = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
      cluster.label_reliability = 1.0F;
      cluster.semantic_label = ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT;
      cluster.semantic_reliability = 0.0F;
      cluster.frame = map.frame_number;
      cluster.quat.w = 1.0;
      cluster.match = 1.0F;
      cluster.nodes.reserve(component_size[root]);
    }
    const std::size_t output_index = result.map.nodes.size();
    output_index_by_source[node_index] = output_index;
    result.map.nodes.push_back(map.nodes[node_index]);
    result.map.clusters[component_index].nodes.push_back(
      static_cast<std::uint16_t>(output_index));
  }

  for (std::size_t edge_index = 0U; edge_index + 1U < map.edges.size(); edge_index += 2U) {
    const std::size_t first = map.edges[edge_index];
    const std::size_t second = map.edges[edge_index + 1U];
    if (!is_valid_edge(map, first, second)) {
      continue;
    }
    const std::size_t first_output = output_index_by_source[first];
    const std::size_t second_output = output_index_by_source[second];
    if (first_output != invalid_component && second_output != invalid_component) {
      result.map.edges.push_back(static_cast<std::uint16_t>(first_output));
      result.map.edges.push_back(static_cast<std::uint16_t>(second_output));
      continue;
    }
    const bool first_is_output = first_output != invalid_component;
    const bool second_is_output = second_output != invalid_component;
    if (first_is_output == second_is_output) {
      continue;
    }
    const std::size_t source_index = first_is_output ? first : second;
    const std::size_t plane_index = first_is_output ? second : first;
    const int plane_cluster_index = plane_owner[plane_index];
    if (plane_cluster_index < 0) {
      continue;
    }
    const std::size_t root = components.find(source_index);
    const std::size_t component_index = component_index_by_root[root];
    if (component_index == invalid_component) {
      continue;
    }
    result.plane_anchor_edges.push_back({
      static_cast<std::uint32_t>(component_index),
      static_cast<std::uint32_t>(source_index),
      static_cast<std::uint32_t>(plane_index),
      plane_clusters.clusters[static_cast<std::size_t>(plane_cluster_index)].id});
  }

  for (auto &cluster : result.map.clusters) {
    if (cluster.nodes.empty()) {
      continue;
    }
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double max_z = std::numeric_limits<double>::lowest();
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    for (const std::uint16_t node_index : cluster.nodes) {
      const auto &node = result.map.nodes[node_index];
      min_x = std::min(min_x, static_cast<double>(node.pos.x));
      min_y = std::min(min_y, static_cast<double>(node.pos.y));
      min_z = std::min(min_z, static_cast<double>(node.pos.z));
      max_x = std::max(max_x, static_cast<double>(node.pos.x));
      max_y = std::max(max_y, static_cast<double>(node.pos.y));
      max_z = std::max(max_z, static_cast<double>(node.pos.z));
      sum_x += node.pos.x;
      sum_y += node.pos.y;
      sum_z += node.pos.z;
    }
    const double count = static_cast<double>(cluster.nodes.size());
    cluster.pos.x = static_cast<float>(sum_x / count);
    cluster.pos.y = static_cast<float>(sum_y / count);
    cluster.pos.z = static_cast<float>(sum_z / count);
    cluster.scale.x = static_cast<float>(max_x - min_x);
    cluster.scale.y = static_cast<float>(max_y - min_y);
    cluster.scale.z = static_cast<float>(max_z - min_z);
  }
  return result;
}

}  // namespace fuzzrobo::topological_plane::nonplane
