#include "ais_gng/topological_plane/nonplane_component_extractor.hpp"

#include <gtest/gtest.h>

#include <cstdint>
namespace
{

using ais_gng_msgs::msg::PlaneCluster;
using ais_gng_msgs::msg::PlaneClusterArray;
using ais_gng_msgs::msg::TopologicalMap;
using fuzzrobo::topological_plane::nonplane::extract_components;
using fuzzrobo::topological_plane::nonplane::extractor_options;

void append_node(TopologicalMap &map, const float x)
{
  ais_gng_msgs::msg::TopologicalNode node;
  node.id = static_cast<std::uint16_t>(map.nodes.size());
  node.pos.x = x;
  map.nodes.push_back(node);
}

void append_edge(TopologicalMap &map, const std::uint16_t first, const std::uint16_t second)
{
  map.edges.push_back(first);
  map.edges.push_back(second);
}

PlaneCluster make_plane_cluster(const std::uint32_t id, const std::uint32_t node_index)
{
  PlaneCluster cluster;
  cluster.id = id;
  cluster.node_indices.push_back(node_index);
  return cluster;
}

}  // 無名名前空間

// 2平面をまたぐ残余成分を、単一成分として保持する。
TEST(NonplaneComponentExtractor, PreservesBridgeAcrossMultiplePlaneClusters)
{
  TopologicalMap map;
  map.frame_number = 7U;
  for (std::size_t index = 0U; index < 5U; ++index) {
    append_node(map, static_cast<float>(index));
  }
  append_edge(map, 0U, 2U);
  append_edge(map, 2U, 3U);
  append_edge(map, 3U, 4U);
  append_edge(map, 4U, 1U);

  PlaneClusterArray plane_clusters;
  plane_clusters.frame_number = map.frame_number;
  plane_clusters.clusters.push_back(make_plane_cluster(101U, 0U));
  plane_clusters.clusters.push_back(make_plane_cluster(202U, 1U));

  extractor_options options;
  options.min_component_nodes = 2U;
  const auto result = extract_components(map, plane_clusters, options);

  ASSERT_EQ(result.components.size(), 1U);
  EXPECT_EQ(result.components.front().node_indices.size(), 3U);
}

// 平面nodeを通る2成分の誤った併合を防ぐ。
TEST(NonplaneComponentExtractor, DoesNotTraverseThroughPlaneClusterNode)
{
  TopologicalMap map;
  map.frame_number = 8U;
  for (std::size_t index = 0U; index < 3U; ++index) {
    append_node(map, static_cast<float>(index));
  }
  append_edge(map, 0U, 1U);
  append_edge(map, 1U, 2U);

  PlaneClusterArray plane_clusters;
  plane_clusters.frame_number = map.frame_number;
  plane_clusters.clusters.push_back(make_plane_cluster(303U, 1U));

  extractor_options options;
  options.min_component_nodes = 1U;
  const auto result = extract_components(map, plane_clusters, options);

  ASSERT_EQ(result.components.size(), 2U);
  EXPECT_EQ(result.components[0].node_indices.size(), 1U);
  EXPECT_EQ(result.components[1].node_indices.size(), 1U);
}
