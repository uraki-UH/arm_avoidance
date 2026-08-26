#include <ais_gng/topological_query/topological_reachability.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
#include <unordered_set>

namespace fuzzrobo::topological_query
{
namespace
{

struct SearchState
{
  std::size_t seed_index = 0;
  std::size_t node_index = 0;
  std::size_t hops = 0;
  std::array<double, 3> seed_pos{0.0, 0.0, 0.0};
};

using Adjacency = std::vector<std::vector<std::size_t>>;

std::array<double, 3> toPoint(const ais_gng_msgs::msg::TopologicalNode &node)
{
  return {
    static_cast<double>(node.pos.x),
    static_cast<double>(node.pos.y),
    static_cast<double>(node.pos.z)
  };
}

double euclideanDistance(
  const std::array<double, 3> &lhs,
  const std::array<double, 3> &rhs)
{
  const double dx = lhs[0] - rhs[0];
  const double dy = lhs[1] - rhs[1];
  const double dz = lhs[2] - rhs[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool isValidIndex(const ais_gng_msgs::msg::TopologicalMap &map, std::size_t index)
{
  return index < map.nodes.size();
}

void addUndirectedEdge(
  Adjacency &adjacency,
  std::size_t lhs,
  std::size_t rhs)
{
  if (lhs >= adjacency.size() || rhs >= adjacency.size() || lhs == rhs) {
    return;
  }
  adjacency[lhs].push_back(rhs);
  adjacency[rhs].push_back(lhs);
}

Adjacency buildGraphEdgeAdjacency(const ais_gng_msgs::msg::TopologicalMap &map)
{
  Adjacency adjacency(map.nodes.size());
  for (std::size_t i = 0; i + 1 < map.edges.size(); i += 2) {
    const std::size_t lhs = static_cast<std::size_t>(map.edges[i]);
    const std::size_t rhs = static_cast<std::size_t>(map.edges[i + 1]);
    addUndirectedEdge(adjacency, lhs, rhs);
  }
  return adjacency;
}

Adjacency buildClusterAdjacency(const ais_gng_msgs::msg::TopologicalMap &map)
{
  Adjacency adjacency(map.nodes.size());
  for (const auto &cluster : map.clusters) {
    if (cluster.nodes.empty()) {
      continue;
    }
    const std::size_t hub = static_cast<std::size_t>(cluster.nodes.front());
    for (const auto node_id : cluster.nodes) {
      const std::size_t node_index = static_cast<std::size_t>(node_id);
      addUndirectedEdge(adjacency, hub, node_index);
    }
  }
  return adjacency;
}

Adjacency buildLabelAdjacency(const ais_gng_msgs::msg::TopologicalMap &map)
{
  Adjacency adjacency(map.nodes.size());
  std::unordered_map<uint8_t, std::size_t> label_hubs;
  for (std::size_t i = 0; i < map.nodes.size(); ++i) {
    const auto label = map.nodes[i].label;
    const auto [it, inserted] = label_hubs.emplace(label, i);
    if (!inserted) {
      addUndirectedEdge(adjacency, it->second, i);
    }
  }
  return adjacency;
}

Adjacency buildAdjacency(
  const ais_gng_msgs::msg::TopologicalMap &map,
  RelationMode mode,
  const NeighborResolver &resolver)
{
  if (resolver) {
    Adjacency adjacency(map.nodes.size());
    for (std::size_t i = 0; i < map.nodes.size(); ++i) {
      std::vector<std::size_t> neighbors;
      resolver(i, map, neighbors);
      for (const auto neighbor : neighbors) {
        addUndirectedEdge(adjacency, i, neighbor);
      }
    }
    return adjacency;
  }

  switch (mode) {
    case RelationMode::GraphEdges:
      return buildGraphEdgeAdjacency(map);
    case RelationMode::ClusterMembership:
      return buildClusterAdjacency(map);
    case RelationMode::SameNodeLabel:
      return buildLabelAdjacency(map);
  }

  return buildGraphEdgeAdjacency(map);
}

uint64_t makeVisitedKey(std::size_t seed_index, std::size_t node_index)
{
  return (static_cast<uint64_t>(seed_index) << 32) | static_cast<uint64_t>(node_index);
}

}  // namespace

std::string toString(RelationMode mode)
{
  switch (mode) {
    case RelationMode::GraphEdges:
      return "graph_edges";
    case RelationMode::ClusterMembership:
      return "cluster_membership";
    case RelationMode::SameNodeLabel:
      return "same_node_label";
  }
  return "graph_edges";
}

std::optional<RelationMode> relationModeFromString(const std::string &text)
{
  if (text == "graph_edges" || text == "edges") {
    return RelationMode::GraphEdges;
  }
  if (text == "cluster_membership" || text == "clusters") {
    return RelationMode::ClusterMembership;
  }
  if (text == "same_node_label" || text == "label") {
    return RelationMode::SameNodeLabel;
  }
  return std::nullopt;
}

std::vector<std::size_t> collectSemanticSeeds(
  const ais_gng_msgs::msg::TopologicalMap &map,
  uint8_t semantic_label)
{
  std::vector<std::size_t> seeds;
  seeds.reserve(map.nodes.size());
  for (std::size_t i = 0; i < map.nodes.size(); ++i) {
    if (map.nodes[i].semantic_label == semantic_label) {
      seeds.push_back(i);
    }
  }
  return seeds;
}

std::unordered_map<std::size_t, ReachableNode> queryReachableNodeMap(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const std::vector<std::size_t> &seed_node_indices,
  const QueryOptions &options,
  const NeighborResolver &resolver)
{
  std::unordered_map<std::size_t, ReachableNode> best_nodes;
  if (map.nodes.empty() || seed_node_indices.empty()) {
    return best_nodes;
  }

  const auto adjacency = buildAdjacency(map, options.relation_mode, resolver);
  std::queue<SearchState> queue;
  std::unordered_set<uint64_t> visited;

  for (std::size_t seed_i = 0; seed_i < seed_node_indices.size(); ++seed_i) {
    const auto seed_index = seed_node_indices[seed_i];
    if (!isValidIndex(map, seed_index)) {
      continue;
    }
    const auto seed_pos = toPoint(map.nodes[seed_index]);
    queue.push(SearchState{seed_index, seed_index, 0, seed_pos});
    visited.insert(makeVisitedKey(seed_index, seed_index));

    best_nodes.emplace(seed_index, ReachableNode{
      seed_index,
      seed_index,
      0,
      0.0
    });
  }

  while (!queue.empty()) {
    const auto current = queue.front();
    queue.pop();

    if (!isValidIndex(map, current.node_index)) {
      continue;
    }
    if (current.hops >= options.max_hops) {
      continue;
    }

    for (const auto neighbor : adjacency[current.node_index]) {
      if (!isValidIndex(map, neighbor)) {
        continue;
      }

      const auto neighbor_pos = toPoint(map.nodes[neighbor]);
      const double distance = euclideanDistance(current.seed_pos, neighbor_pos);
      if (distance > options.max_euclidean_distance) {
        continue;
      }

      const auto visited_key = makeVisitedKey(current.seed_index, neighbor);
      if (!visited.insert(visited_key).second) {
        continue;
      }

      const ReachableNode candidate{
        current.seed_index,
        neighbor,
        current.hops + 1,
        distance
      };
      const auto [it, inserted] = best_nodes.emplace(neighbor, candidate);
      if (!inserted && distance < it->second.euclidean_distance) {
        it->second = candidate;
      }

      queue.push(SearchState{
        current.seed_index,
        neighbor,
        current.hops + 1,
        current.seed_pos
      });
    }
  }

  return best_nodes;
}

std::vector<ReachableNode> queryReachableNodes(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const std::vector<std::size_t> &seed_node_indices,
  const QueryOptions &options,
  const NeighborResolver &resolver)
{
  const auto node_map = queryReachableNodeMap(map, seed_node_indices, options, resolver);
  std::vector<ReachableNode> result;
  result.reserve(node_map.size());
  for (const auto &entry : node_map) {
    if (!options.include_seed_nodes && entry.second.hops == 0) {
      continue;
    }
    result.push_back(entry.second);
  }
  std::sort(result.begin(), result.end(), [](const ReachableNode &lhs, const ReachableNode &rhs) {
    if (lhs.euclidean_distance != rhs.euclidean_distance) {
      return lhs.euclidean_distance < rhs.euclidean_distance;
    }
    if (lhs.hops != rhs.hops) {
      return lhs.hops < rhs.hops;
    }
    return lhs.node_index < rhs.node_index;
  });
  return result;
}

std::vector<ReachableNode> queryReachableNodesBySemanticLabel(
  const ais_gng_msgs::msg::TopologicalMap &map,
  uint8_t semantic_label,
  const QueryOptions &options,
  const NeighborResolver &resolver)
{
  return queryReachableNodes(map, collectSemanticSeeds(map, semantic_label), options, resolver);
}

}  // namespace fuzzrobo::topological_query

