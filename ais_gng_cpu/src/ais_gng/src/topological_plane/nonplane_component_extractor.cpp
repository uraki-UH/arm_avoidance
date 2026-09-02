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
      component_index = result.components.size();
      component_index_by_root[root] = component_index;
      result.components.emplace_back();
      auto &component = result.components.back();
      component.id = static_cast<std::uint32_t>(component_index);
      component.node_indices.reserve(component_size[root]);
    }
    result.components[component_index].node_indices.push_back(
      static_cast<std::uint32_t>(node_index));
  }

  return result;
}

}  // namespace fuzzrobo::topological_plane::nonplane
