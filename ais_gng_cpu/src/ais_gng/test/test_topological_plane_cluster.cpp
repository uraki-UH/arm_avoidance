#include <ais_gng/topological_plane/topological_plane_cluster.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

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
      node.label = ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN;
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

ais_gng_msgs::msg::TopologicalMap makePlanarRing()
{
  ais_gng_msgs::msg::TopologicalMap map;
  constexpr std::size_t kNodeCount = 8U;
  constexpr double kPi = 3.14159265358979323846;
  for (std::size_t index = 0U; index < kNodeCount; ++index) {
    const double angle = 2.0 * kPi * static_cast<double>(index) /
      static_cast<double>(kNodeCount);
    ais_gng_msgs::msg::TopologicalNode node;
    node.id = static_cast<std::uint16_t>(index);
    node.pos.x = static_cast<float>(std::cos(angle));
    node.pos.y = static_cast<float>(std::sin(angle));
    node.normal.z = 1.0F;
    node.label = ais_gng_msgs::msg::TopologicalMap::WALL;
    map.nodes.push_back(node);
    map.edges.push_back(static_cast<std::uint16_t>(index));
    map.edges.push_back(static_cast<std::uint16_t>((index + 1U) % kNodeCount));
  }
  return map;
}

ais_gng_msgs::msg::TopologicalMap makePlanarRingWithSingleTail()
{
  auto map = makePlanarRing();
  ais_gng_msgs::msg::TopologicalNode node;
  node.id = static_cast<std::uint16_t>(map.nodes.size());
  node.pos.x = 2.0F;
  node.normal.z = 1.0F;
  node.label = ais_gng_msgs::msg::TopologicalMap::WALL;
  map.nodes.push_back(node);
  map.edges.push_back(0U);
  map.edges.push_back(static_cast<std::uint16_t>(map.nodes.size() - 1U));
  return map;
}

ais_gng_msgs::msg::TopologicalMap makePlanarGridAsSingleChain()
{
  auto map = makePlanarGrid();
  map.edges.clear();
  const std::vector<std::size_t> chain{0U, 1U, 2U, 5U, 4U, 3U, 6U, 7U, 8U};
  for (std::size_t index = 1U; index < chain.size(); ++index) {
    map.edges.push_back(static_cast<std::uint16_t>(chain[index - 1U]));
    map.edges.push_back(static_cast<std::uint16_t>(chain[index]));
  }
  return map;
}

ais_gng_msgs::msg::TopologicalMap makeAdjacentCoplanarPatches()
{
  ais_gng_msgs::msg::TopologicalMap map;
  const auto connect = [&map](std::size_t first, std::size_t second) {
      map.edges.push_back(static_cast<std::uint16_t>(first));
      map.edges.push_back(static_cast<std::uint16_t>(second));
    };
  for (std::size_t patch = 0U; patch < 2U; ++patch) {
    const std::size_t offset = patch * 9U;
    for (std::size_t y = 0U; y < 3U; ++y) {
      for (std::size_t x = 0U; x < 3U; ++x) {
        ais_gng_msgs::msg::TopologicalNode node;
        node.id = static_cast<std::uint16_t>(map.nodes.size());
        node.pos.x = static_cast<float>(patch * 3U + x);
        node.pos.y = static_cast<float>(y);
        node.label = patch == 0U ? ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN :
          ais_gng_msgs::msg::TopologicalMap::WALL;
        // 固有ベクトルの符号反転は同じ平面を表すため、二つのラベルにまたがっても
        // 一つの面として連結できることを確認する。
        node.normal.z = patch == 0U ? 1.0F : -1.0F;
        map.nodes.push_back(node);
        const std::size_t index = offset + y * 3U + x;
        if (x > 0U) {
          connect(index - 1U, index);
        }
        if (y > 0U) {
          connect(index - 3U, index);
        }
      }
    }
  }
  connect(5U, 12U);
  return map;
}

ais_gng_msgs::msg::TopologicalMap makePerpendicularPlanarGrids()
{
  ais_gng_msgs::msg::TopologicalMap map;
  const auto connect = [&map](std::size_t first, std::size_t second) {
      map.edges.push_back(static_cast<std::uint16_t>(first));
      map.edges.push_back(static_cast<std::uint16_t>(second));
    };
  for (std::size_t y = 0U; y < 3U; ++y) {
    for (std::size_t x = 0U; x < 3U; ++x) {
      ais_gng_msgs::msg::TopologicalNode node;
      node.id = static_cast<std::uint16_t>(map.nodes.size());
      node.pos.x = static_cast<float>(x);
      node.pos.y = static_cast<float>(y);
      node.normal.z = 1.0F;
      node.label = ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN;
      map.nodes.push_back(node);
      const std::size_t index = y * 3U + x;
      if (x > 0U) {
        connect(index - 1U, index);
      }
      if (y > 0U) {
        connect(index - 3U, index);
      }
    }
  }
  const std::size_t wall_offset = map.nodes.size();
  for (std::size_t z = 0U; z < 3U; ++z) {
    for (std::size_t y = 0U; y < 3U; ++y) {
      ais_gng_msgs::msg::TopologicalNode node;
      node.id = static_cast<std::uint16_t>(map.nodes.size());
      node.pos.x = 3.0F;
      node.pos.y = static_cast<float>(y);
      node.pos.z = static_cast<float>(z);
      node.normal.x = 1.0F;
      node.label = ais_gng_msgs::msg::TopologicalMap::WALL;
      map.nodes.push_back(node);
      const std::size_t index = wall_offset + z * 3U + y;
      if (y > 0U) {
        connect(index - 1U, index);
      }
      if (z > 0U) {
        connect(index - 3U, index);
      }
    }
  }
  connect(5U, wall_offset + 4U);
  return map;
}

TEST(TopologicalPlaneCluster, ExtractsPlanarGrid)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  const auto result = extractor.extract(makePlanarGrid());

  ASSERT_EQ(result.clusters.clusters.size(), 1U);
  const auto &cluster = result.clusters.clusters.front();
  EXPECT_EQ(cluster.node_indices.size(), 9U);
  EXPECT_EQ(cluster.source_label, ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN);
  EXPECT_TRUE(cluster.boundary.empty());
  EXPECT_EQ(cluster.support_edges.size(), 24U);
  EXPECT_FLOAT_EQ(cluster.area, 0.0F);
  EXPECT_NEAR(std::abs(cluster.normal.z), 1.0F, 1.0e-4F);
  EXPECT_NEAR(cluster.planarity, 1.0F, 1.0e-4F);
  EXPECT_NEAR(cluster.residual_ratio, 0.0F, 1.0e-4F);
  EXPECT_EQ(result.statistics.clustered_node_count, 9U);
  EXPECT_EQ(result.statistics.clustered_terrain_node_count, 9U);
  EXPECT_EQ(result.statistics.clustered_default_node_count, 0U);
  EXPECT_EQ(result.statistics.clustered_wall_node_count, 0U);
  EXPECT_EQ(result.statistics.clustered_unknown_node_count, 0U);
}

TEST(TopologicalPlaneCluster, ClustersConnectedUnknownNodesBySurfaceNormal)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  auto mixed_map = makePlanarGrid();
  for (std::size_t index = 0; index < mixed_map.nodes.size(); ++index) {
    mixed_map.nodes[index].label =
      ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN;
  }
  mixed_map.nodes[0].label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
  mixed_map.nodes[1].label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
  mixed_map.nodes[4].label = ais_gng_msgs::msg::TopologicalMap::WALL;
  const auto mixed_result = extractor.extract(mixed_map);
  ASSERT_EQ(mixed_result.clusters.clusters.size(), 1U);
  EXPECT_EQ(mixed_result.clusters.clusters.front().node_indices.size(), 9U);
  EXPECT_EQ(mixed_result.statistics.clustered_terrain_node_count, 6U);
  EXPECT_EQ(mixed_result.statistics.clustered_wall_node_count, 1U);
  EXPECT_EQ(mixed_result.statistics.clustered_unknown_node_count, 2U);

  for (auto &node : mixed_map.nodes) {
    node.label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
  }
  const auto unknown_result = extractor.extract(mixed_map);
  ASSERT_EQ(unknown_result.clusters.clusters.size(), 1U);
  EXPECT_EQ(unknown_result.clusters.clusters.front().node_indices.size(), 9U);
  EXPECT_EQ(unknown_result.clusters.clusters.front().source_label,
    ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT);
  EXPECT_EQ(unknown_result.statistics.clustered_unknown_node_count, 9U);
}

TEST(TopologicalPlaneCluster, KeepsWallConnectedWhenNormalSignsAlternate)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  auto wall_map = makePlanarGrid();
  for (std::size_t index = 0; index < wall_map.nodes.size(); ++index) {
    auto &node = wall_map.nodes[index];
    node.label = ais_gng_msgs::msg::TopologicalMap::WALL;
    node.normal.y = 0.0F;
    node.normal.z = index % 2U == 0U ? 1.0F : -1.0F;
  }
  const auto result = extractor.extract(wall_map);

  ASSERT_EQ(result.clusters.clusters.size(), 1U);
  EXPECT_EQ(result.clusters.clusters.front().node_indices.size(), 9U);
}

TEST(TopologicalPlaneCluster, IncludesUnknownNodesInSurfaceComponent)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  auto wall_map = makePlanarGrid();
  for (auto &node : wall_map.nodes) {
    node.label = ais_gng_msgs::msg::TopologicalMap::WALL;
  }
  wall_map.nodes[0].label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
  wall_map.nodes[1].label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
  const auto result = extractor.extract(wall_map);

  ASSERT_EQ(result.clusters.clusters.size(), 1U);
  EXPECT_EQ(result.clusters.clusters.front().node_indices.size(), 9U);
}

TEST(TopologicalPlaneCluster, BridgesSingleUnknownLabelGapOnPlane)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  auto map = makePlanarGrid();
  // 中央列だけが一時的に UNKNOWN_OBJECT になっても、左右の同一平面を分断しない。
  map.nodes[1].label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
  map.nodes[4].label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
  map.nodes[7].label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
  const auto result = extractor.extract(map);

  ASSERT_EQ(result.clusters.clusters.size(), 1U);
  EXPECT_EQ(result.clusters.clusters.front().node_indices.size(), 9U);
  EXPECT_EQ(result.clusters.clusters.front().source_label,
    ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN);
}

TEST(TopologicalPlaneCluster, MergesCoplanarLabelsThroughSingleGngEdge)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  const auto result = extractor.extract(makeAdjacentCoplanarPatches());

  ASSERT_EQ(result.clusters.clusters.size(), 1U);
  EXPECT_EQ(result.clusters.clusters.front().node_indices.size(), 18U);
}

TEST(TopologicalPlaneCluster, MergesCoplanarPatchesThroughSingleGngEdge)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  auto map = makeAdjacentCoplanarPatches();
  for (auto &node : map.nodes) {
    node.label = ais_gng_msgs::msg::TopologicalMap::WALL;
  }
  const auto result = extractor.extract(map);

  ASSERT_EQ(result.clusters.clusters.size(), 1U);
  EXPECT_EQ(result.clusters.clusters.front().node_indices.size(), 18U);
}

TEST(TopologicalPlaneCluster, SeparatesPerpendicularPlanesDespiteGngConnection)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  const auto result = extractor.extract(makePerpendicularPlanarGrids());

  ASSERT_EQ(result.clusters.clusters.size(), 2U);
  EXPECT_EQ(result.clusters.clusters[0].node_indices.size(), 9U);
  EXPECT_EQ(result.clusters.clusters[1].node_indices.size(), 9U);
}

TEST(TopologicalPlaneCluster, RejectsOneDimensionalGraph)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  const auto result = extractor.extract(makeLine());

  EXPECT_TRUE(result.clusters.clusters.empty());
  EXPECT_EQ(result.statistics.locally_planar_node_count, 0U);
}

TEST(TopologicalPlaneCluster, RejectsPlanarRingByEdgeDensity)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  const auto result = extractor.extract(makePlanarRing());

  EXPECT_TRUE(result.clusters.clusters.empty());
}

TEST(TopologicalPlaneCluster, RejectsPlanarRingWithSingleTailByEdgeDensity)
{
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor;
  const auto result = extractor.extract(makePlanarRingWithSingleTail());

  EXPECT_TRUE(result.clusters.clusters.empty());
  EXPECT_GT(result.statistics.line_like_component_count, 0U);
}

TEST(TopologicalPlaneCluster, RetainsTrackedPlaneAcrossTransientLabelLoss)
{
  fuzzrobo::topological_plane::PlaneClusterOptions options;
  options.min_cluster_nodes = 7U;
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor(options);
  fuzzrobo::topological_plane::PersistentPlaneClusterTracker tracker(options);

  const auto initial_map = makePlanarGrid();
  const auto initial = tracker.update(extractor.extract(initial_map).clusters, initial_map);
  ASSERT_EQ(initial.clusters.size(), 1U);
  const std::uint32_t persistent_id = initial.clusters.front().id;

  auto wall_transition_map = initial_map;
  ++wall_transition_map.frame_number;
  for (auto &node : wall_transition_map.nodes) {
    node.label = ais_gng_msgs::msg::TopologicalMap::WALL;
  }
  const auto transitioned = tracker.update(
    extractor.extract(wall_transition_map).clusters, wall_transition_map);
  ASSERT_EQ(transitioned.clusters.size(), 1U);
  EXPECT_EQ(transitioned.clusters.front().id, persistent_id);
  EXPECT_EQ(transitioned.clusters.front().source_label, ais_gng_msgs::msg::TopologicalMap::WALL);

  auto label_flicker_map = initial_map;
  label_flicker_map.frame_number = wall_transition_map.frame_number + 1U;
  for (auto &node : label_flicker_map.nodes) {
    node.label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
  }
  ais_gng_msgs::msg::PlanarClusterArray maintenance;
  maintenance.header = label_flicker_map.header;
  maintenance.frame_number = label_flicker_map.frame_number;
  const auto held = tracker.update(maintenance, label_flicker_map);

  ASSERT_EQ(held.clusters.size(), 1U);
  EXPECT_EQ(held.clusters.front().id, persistent_id);
  EXPECT_EQ(held.clusters.front().node_indices.size(), 9U);
  EXPECT_EQ(
    held.clusters.front().source_label, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT);
}

TEST(TopologicalPlaneCluster, ReturnsToSeedWhenMembersFallBelowMinimum)
{
  fuzzrobo::topological_plane::PlaneClusterOptions options;
  options.min_cluster_nodes = 7U;
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor(options);
  fuzzrobo::topological_plane::PersistentPlaneClusterTracker tracker(options);
  auto map = makePlanarGrid();
  const auto extracted = extractor.extract(map).clusters;

  ++map.frame_number;
  ASSERT_EQ(tracker.update(extracted, map).clusters.size(), 1U);

  auto moved_map = map;
  ++moved_map.frame_number;
  for (std::size_t index = 0U; index < 3U; ++index) {
    moved_map.nodes[index].pos.z = 2.0F;
  }
  ais_gng_msgs::msg::PlanarClusterArray maintenance;
  maintenance.header = moved_map.header;
  maintenance.frame_number = moved_map.frame_number;
  EXPECT_TRUE(tracker.update(maintenance, moved_map).clusters.empty());

  // 最小数未満の追跡クラスタは残留せず、復帰時には抽出器の種から即座に再誕生する。
  auto restored_map = map;
  ++restored_map.frame_number;
  EXPECT_EQ(tracker.update(extracted, restored_map).clusters.size(), 1U);
}

TEST(TopologicalPlaneCluster, ReturnsToSeedWhenCovarianceBecomesLineLike)
{
  fuzzrobo::topological_plane::PlaneClusterOptions options;
  options.min_cluster_nodes = 7U;
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor(options);
  fuzzrobo::topological_plane::PersistentPlaneClusterTracker tracker(options);
  const auto planar_map = makePlanarGrid();
  const auto initial = tracker.update(extractor.extract(planar_map).clusters, planar_map);
  ASSERT_EQ(initial.clusters.size(), 1U);
  const std::uint32_t initial_id = initial.clusters.front().id;

  auto line_like_map = planar_map;
  ++line_like_map.frame_number;
  for (std::size_t index = 0U; index < line_like_map.nodes.size(); ++index) {
    line_like_map.nodes[index].pos.x = static_cast<float>(index);
    line_like_map.nodes[index].pos.y = 0.0F;
    line_like_map.nodes[index].pos.z = 0.0F;
  }
  ais_gng_msgs::msg::PlanarClusterArray maintenance;
  maintenance.header = line_like_map.header;
  maintenance.frame_number = line_like_map.frame_number;
  EXPECT_TRUE(tracker.update(maintenance, line_like_map).clusters.empty());

  auto restored_map = planar_map;
  restored_map.frame_number = line_like_map.frame_number + 1U;
  const auto restored = tracker.update(extractor.extract(restored_map).clusters, restored_map);
  ASSERT_EQ(restored.clusters.size(), 1U);
  EXPECT_NE(restored.clusters.front().id, initial_id);
}

TEST(TopologicalPlaneCluster, RetainsTrackedClusterWhileSupportIsTemporarilySingleChain)
{
  fuzzrobo::topological_plane::PlaneClusterOptions options;
  options.min_cluster_nodes = 7U;
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor(options);
  fuzzrobo::topological_plane::PersistentPlaneClusterTracker tracker(options);
  auto map = makePlanarGrid();

  ++map.frame_number;
  const auto initial = tracker.update(extractor.extract(map).clusters, map);
  ASSERT_EQ(initial.clusters.size(), 1U);
  const std::uint32_t persistent_id = initial.clusters.front().id;

  auto chained_map = makePlanarGridAsSingleChain();
  chained_map.frame_number = map.frame_number + 1U;
  ais_gng_msgs::msg::PlanarClusterArray maintenance;
  maintenance.header = chained_map.header;
  maintenance.frame_number = chained_map.frame_number;
  const auto chained = tracker.update(maintenance, chained_map);
  ASSERT_EQ(chained.clusters.size(), 1U);
  EXPECT_EQ(chained.clusters.front().id, persistent_id);
  EXPECT_EQ(chained.clusters.front().node_indices.size(), 9U);

  ++map.frame_number;
  const auto restored = tracker.update(extractor.extract(map).clusters, map);
  ASSERT_EQ(restored.clusters.size(), 1U);
  EXPECT_EQ(restored.clusters.front().id, persistent_id);
  EXPECT_EQ(restored.clusters.front().node_indices.size(), 9U);
}

TEST(TopologicalPlaneCluster, AssignsEachNodeToOnlyOnePersistentCluster)
{
  fuzzrobo::topological_plane::PlaneClusterOptions options;
  options.min_cluster_nodes = 7U;
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor(options);
  fuzzrobo::topological_plane::PersistentPlaneClusterTracker tracker(options);
  auto map = makePlanarGrid();
  auto frame_clusters = extractor.extract(map).clusters;
  ASSERT_EQ(frame_clusters.clusters.size(), 1U);
  auto duplicate = frame_clusters.clusters.front();
  duplicate.id = 2U;
  frame_clusters.clusters.push_back(duplicate);

  ais_gng_msgs::msg::PlanarClusterArray output;
  for (std::size_t update = 0U; update < 4U; ++update) {
    ++map.frame_number;
    frame_clusters.frame_number = map.frame_number;
    output = tracker.update(frame_clusters, map);
  }

  ASSERT_EQ(output.clusters.size(), 1U);
  EXPECT_EQ(output.clusters.front().node_indices.size(), 9U);
}

TEST(TopologicalPlaneCluster, MergesPersistentTracksByTransferringNodeOwnership)
{
  fuzzrobo::topological_plane::PlaneClusterOptions options;
  options.min_cluster_nodes = 7U;
  fuzzrobo::topological_plane::TopologicalPlaneClusterExtractor extractor(options);
  fuzzrobo::topological_plane::PersistentPlaneClusterTracker tracker(options);
  auto map = makeAdjacentCoplanarPatches();
  for (auto &node : map.nodes) {
    node.label = ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN;
  }
  // 二本の接続があれば、一つの同ラベル平面として抽出される。
  map.edges.push_back(8U);
  map.edges.push_back(15U);
  const auto extracted = extractor.extract(map).clusters;
  ASSERT_EQ(extracted.clusters.size(), 1U);

  auto first_patch = extracted.clusters.front();
  auto second_patch = extracted.clusters.front();
  first_patch.node_indices.clear();
  second_patch.node_indices.clear();
  for (std::uint32_t index = 0U; index < 9U; ++index) {
    first_patch.node_indices.push_back(index);
    second_patch.node_indices.push_back(index + 9U);
  }
  ais_gng_msgs::msg::PlanarClusterArray split_frame;
  split_frame.header = extracted.header;
  split_frame.clusters = {first_patch, second_patch};

  ais_gng_msgs::msg::PlanarClusterArray split_output;
  for (std::size_t update = 0U; update < 3U; ++update) {
    ++map.frame_number;
    split_frame.frame_number = map.frame_number;
    split_output = tracker.update(split_frame, map);
  }
  ASSERT_EQ(split_output.clusters.size(), 2U);
  const std::uint32_t surviving_id = std::min(
    split_output.clusters[0].id, split_output.clusters[1].id);

  auto merged_frame = extracted;
  ++map.frame_number;
  merged_frame.frame_number = map.frame_number;
  const auto merged_output = tracker.update(merged_frame, map);

  ASSERT_EQ(merged_output.clusters.size(), 1U);
  EXPECT_EQ(merged_output.clusters.front().id, surviving_id);
  EXPECT_EQ(merged_output.clusters.front().node_indices.size(), 18U);
}

TEST(TopologicalPlaneCluster, ExtendsPersistentClusterThroughSharedNode)
{
  fuzzrobo::topological_plane::PlaneClusterOptions options;
  options.min_cluster_nodes = 7U;
  fuzzrobo::topological_plane::PersistentPlaneClusterTracker tracker(options);
  auto map = makeAdjacentCoplanarPatches();

  auto first_patch = ais_gng_msgs::msg::PlanarCluster{};
  first_patch.id = 1U;
  first_patch.source_label = ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN;
  first_patch.local_spacing = 1.0F;
  first_patch.normal.z = 1.0;
  for (std::uint32_t index = 0U; index < 9U; ++index) {
    first_patch.node_indices.push_back(index);
  }
  auto second_patch = first_patch;
  second_patch.id = 2U;
  second_patch.node_indices.clear();
  // 共有ノードが一つでもあれば、既存クラスタの継続として平面内の未所属ノードを加える。
  second_patch.node_indices.push_back(8U);
  for (std::uint32_t index = 9U; index < 17U; ++index) {
    second_patch.node_indices.push_back(index);
  }

  ais_gng_msgs::msg::PlanarClusterArray first_frame;
  first_frame.header = map.header;
  first_frame.clusters = {first_patch};
  for (std::size_t update = 0U; update < 3U; ++update) {
    ++map.frame_number;
    first_frame.frame_number = map.frame_number;
    tracker.update(first_frame, map);
  }

  ais_gng_msgs::msg::PlanarClusterArray second_frame;
  second_frame.header = map.header;
  second_frame.clusters = {second_patch};
  ais_gng_msgs::msg::PlanarClusterArray output;
  for (std::size_t update = 0U; update < 3U; ++update) {
    ++map.frame_number;
    second_frame.frame_number = map.frame_number;
    output = tracker.update(second_frame, map);
  }

  ASSERT_EQ(output.clusters.size(), 1U);
  EXPECT_EQ(output.clusters.front().id, 1U);
  EXPECT_EQ(output.clusters.front().node_indices.size(), 17U);
}

}  // 無名名前空間
