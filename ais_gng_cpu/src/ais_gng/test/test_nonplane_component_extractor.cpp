#include "ais_gng/topological_plane/nonplane_component_extractor.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>
namespace
{

using ais_gng_msgs::msg::PlaneCluster;
using ais_gng_msgs::msg::PlaneClusterArray;
using ais_gng_msgs::msg::TopologicalMap;
using fuzzrobo::topological_plane::nonplane::extract_components;

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

// 2平面をまたぐ残余成分の単一成分としての保持。
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

  const auto result = extract_components(map, plane_clusters);

  ASSERT_EQ(result.components.size(), 1U);
  EXPECT_EQ(result.components.front().node_indices.size(), 3U);
}

// 平面nodeを通る2成分の誤併合防止。
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

  const auto result = extract_components(map, plane_clusters);

  ASSERT_EQ(result.components.size(), 2U);
  EXPECT_EQ(result.components[0].node_indices.size(), 1U);
  EXPECT_EQ(result.components[1].node_indices.size(), 1U);
}

// 平面所属・非平面連結成分・平面にだけ接続する単独点・孤立点の排他的な全被覆。
TEST(nonplane_component_extractor, covers_all_nodes_without_overlap)
{
  TopologicalMap map;
  for (std::size_t idx = 0; idx < 8; ++idx) {
    append_node(map, static_cast<float>(idx));
    map.nodes.back().id = static_cast<std::uint16_t>(100 + idx * 3);
  }
  append_edge(map, 0, 2);
  append_edge(map, 2, 3);
  append_edge(map, 3, 1);
  append_edge(map, 0, 4);
  append_edge(map, 6, 7);
  append_edge(map, 5, 999);
  map.edges.push_back(5);  // 不完全な末尾エッジの無視。
  PlaneClusterArray planes;
  planes.clusters.push_back(make_plane_cluster(10, 0));
  planes.clusters.push_back(make_plane_cluster(20, 1));
  const auto result = extract_components(map, planes);
  ASSERT_EQ(result.components.size(), 4U);
  EXPECT_EQ(result.components[0].node_indices, (std::vector<std::uint32_t>{2, 3}));
  EXPECT_EQ(result.components[1].node_indices, (std::vector<std::uint32_t>{4}));
  EXPECT_EQ(result.components[2].node_indices, (std::vector<std::uint32_t>{5}));
  EXPECT_EQ(result.components[3].node_indices, (std::vector<std::uint32_t>{6, 7}));
  std::vector<std::size_t> counts(map.nodes.size(), 0);
  for (const auto &plane : planes.clusters) {
    for (const auto idx : plane.node_indices) ++counts[idx];
  }
  for (std::size_t idx = 0; idx < result.components.size(); ++idx) {
    EXPECT_EQ(result.components[idx].id, idx);
    for (const auto node_idx : result.components[idx].node_indices) ++counts[node_idx];
  }
  for (const auto count : counts) EXPECT_EQ(count, 1U);
}

// 平面・エッジがない場合の全単独成分保持。
TEST(nonplane_component_extractor, retains_all_isolated_nodes)
{
  TopologicalMap map;
  for (std::size_t idx = 0; idx < 5; ++idx) append_node(map, static_cast<float>(idx));
  const auto result = extract_components(map, PlaneClusterArray{});
  ASSERT_EQ(result.components.size(), map.nodes.size());
  for (std::size_t idx = 0; idx < map.nodes.size(); ++idx) {
    EXPECT_EQ(result.components[idx].node_indices,
      (std::vector<std::uint32_t>{static_cast<std::uint32_t>(idx)}));
  }
}

// 空入力・全平面所属時の非平面成分なし。
TEST(nonplane_component_extractor, handles_empty_and_all_plane_maps)
{
  TopologicalMap map;
  PlaneClusterArray planes;
  EXPECT_TRUE(extract_components(map, planes).components.empty());
  append_node(map, 0);
  planes.clusters.push_back(make_plane_cluster(10, 0));
  EXPECT_TRUE(extract_components(map, planes).components.empty());
}

// 接続消失・平面所属変更後の単独成分再構築。
TEST(nonplane_component_extractor, rebuilds_singletons_after_changes)
{
  TopologicalMap map;
  for (std::size_t idx = 0; idx < 3; ++idx) append_node(map, static_cast<float>(idx));
  append_edge(map, 0, 1);
  append_edge(map, 1, 2);
  EXPECT_EQ(extract_components(map, PlaneClusterArray{}).components.size(), 1U);
  map.edges.clear();
  EXPECT_EQ(extract_components(map, PlaneClusterArray{}).components.size(), 3U);
  PlaneClusterArray planes;
  planes.clusters.push_back(make_plane_cluster(10, 1));
  const auto result = extract_components(map, planes);
  ASSERT_EQ(result.components.size(), 2U);
  EXPECT_EQ(result.components[0].node_indices, (std::vector<std::uint32_t>{0}));
  EXPECT_EQ(result.components[1].node_indices, (std::vector<std::uint32_t>{2}));
}
