#pragma once

#include <ais_gng_msgs/msg/topological_map.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace fuzzrobo::topological_query
{

enum class RelationMode : uint8_t
{
  GraphEdges = 0,
  ClusterMembership = 1,
  SameNodeLabel = 2,
};

struct QueryOptions
{
  RelationMode relation_mode = RelationMode::GraphEdges;
  double max_euclidean_distance = 0.5;
  std::size_t max_hops = std::numeric_limits<std::size_t>::max();
  bool include_seed_nodes = true;
};

struct ReachableNode
{
  std::size_t seed_index = 0;
  std::size_t node_index = 0;
  std::size_t hops = 0;
  double euclidean_distance = 0.0;
};

using NeighborResolver = std::function<void(
  std::size_t node_index,
  const ais_gng_msgs::msg::TopologicalMap &map,
  std::vector<std::size_t> &neighbors)>;

std::string toString(RelationMode mode);
std::optional<RelationMode> relationModeFromString(const std::string &text);

std::vector<std::size_t> collectSemanticSeeds(
  const ais_gng_msgs::msg::TopologicalMap &map,
  uint8_t semantic_label = ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE);

std::vector<ReachableNode> queryReachableNodes(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const std::vector<std::size_t> &seed_node_indices,
  const QueryOptions &options = {},
  const NeighborResolver &resolver = {});

std::vector<ReachableNode> queryReachableNodesBySemanticLabel(
  const ais_gng_msgs::msg::TopologicalMap &map,
  uint8_t semantic_label,
  const QueryOptions &options = {},
  const NeighborResolver &resolver = {});

std::unordered_map<std::size_t, ReachableNode> queryReachableNodeMap(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const std::vector<std::size_t> &seed_node_indices,
  const QueryOptions &options = {},
  const NeighborResolver &resolver = {});

}  // namespace fuzzrobo::topological_query

