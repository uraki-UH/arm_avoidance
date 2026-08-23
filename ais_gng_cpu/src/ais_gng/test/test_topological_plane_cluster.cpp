#include <ais_gng/topological_plane/topological_plane_cluster.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

namespace
{

ais_gng_msgs::msg::TopologicalMap makePlanarGrid()
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.header.frame_id = "map";
  map.frame_number = 7U;
  for (int y = 0; y < 3; ++y) {
    for (int x = 0; x < 3; ++x) {
      ais_gng_msgs::msg::TopologicalNode node;
      node.id = static_cast<std::uint16_t>(map.nodes.size());
      node.frame = 7U;
      node.pos.x = static_cast<float>(x);
      node.pos.y = static_cast<float>(y);
      node.pos.z = 0.0F;
      node.normal.z = 1.0F;
      map.nodes.push_back(node);
    }
  }
  const auto connect = [&map](std::size_t first, std::size_t second) {
      map.edges.push_back(static_cast<std::uint16_t>(first));
      map.edges.push_back(static_cast<std::uint16_t>(second));
    };
  for (std::size_t y = 0; y < 3U; ++y) {
    for (std::size_t x = 0; x < 3U; ++x) {
      const std::size_t index = y * 3U + x;
      if (x + 1U < 3U) {
        connect(index, index + 1U);
      }
      if (y + 1U < 3U) {
        connect(index, index + 3U);
      }
    }
  }
  return map;
}

ais_gng_msgs::msg::TopologicalMap makeLine()
{
  ais_gng_msgs::msg::TopologicalMap map;
  for (int index = 0; index < 5; ++index) {
    ais_gng_msgs::msg::TopologicalNode node;
    node.id = static_cast<std::uint16_t>(index);
    node.pos.x = static_cast<float>(index);
    map.nodes.push_back(node);
    if (index > 0) {
      map.edges.push_back(static_cast<std::uint16_t>(index - 1));
      map.edges.push_back(static_cast<std::uint16_t>(index));
    }
  }
  return map;
}

TEST(TopologicalPlaneCluster, ExtractsPlanarGrid)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  const auto result = extractor.extract(makePlanarGrid());

  ASSERT_EQ(result.clusters.clusters.size(), 1U);
  const auto &cluster = result.clusters.clusters.front();
  EXPECT_EQ(cluster.node_indices.size(), 9U);
  EXPECT_EQ(cluster.boundary.size(), 4U);
  EXPECT_NEAR(cluster.area, 4.0F, 1.0e-4F);
  EXPECT_NEAR(std::abs(cluster.normal.z), 1.0F, 1.0e-4F);
  EXPECT_NEAR(cluster.planarity, 1.0F, 1.0e-4F);
  EXPECT_NEAR(cluster.residual_ratio, 0.0F, 1.0e-4F);
}

TEST(TopologicalPlaneCluster, RejectsOneDimensionalGraph)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  const auto result = extractor.extract(makeLine());

  EXPECT_TRUE(result.clusters.clusters.empty());
  EXPECT_EQ(result.statistics.locally_planar_node_count, 0U);
}

}  // namespace
