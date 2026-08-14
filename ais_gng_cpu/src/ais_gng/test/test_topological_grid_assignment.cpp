#include <ais_gng/topological_grid/topological_grid_assignment.hpp>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <gtest/gtest.h>

#include <cstdint>
#include <unordered_set>

namespace
{

ais_gng_msgs::msg::TopologicalNode makeNode(
  float x, float y, float z, std::uint8_t label)
{
  ais_gng_msgs::msg::TopologicalNode node;
  node.pos.x = x;
  node.pos.y = y;
  node.pos.z = z;
  node.label = label;
  return node;
}

TEST(TopologicalGridAssignment, DefaultsToTenMillimeterCells)
{
  const fuzzrobo::topological_grid::GridSpec spec;
  EXPECT_DOUBLE_EQ(spec.cell_size, 0.01);
}

TEST(TopologicalGridAssignment, ExcludesConfiguredLabelsAndMergesDuplicateCells)
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::DEFAULT),
    makeNode(0.009F, 0.009F, 0.009F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(0.008F, 0.004F, 0.002F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(0.021F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.041F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN),
    makeNode(0.061F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::HUMAN),
    makeNode(0.081F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::CAR),
  };

  fuzzrobo::topological_grid::VoxelizationOptions options;
  options.excluded_labels = {
    ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN,
    ais_gng_msgs::msg::TopologicalMap::HUMAN,
    ais_gng_msgs::msg::TopologicalMap::CAR,
  };
  options.require_input_points = false;
  const auto result = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{0.02, 0.0, 0.0, 0.0}, options);

  ASSERT_EQ(result.included_node_count, 4U);
  ASSERT_EQ(result.excluded_node_count, 3U);
  ASSERT_EQ(result.voxels.size(), 2U);
  EXPECT_EQ(result.voxels[0].cell.x, 0);
  EXPECT_EQ(
    result.voxels[0].label,
    ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT);
  EXPECT_EQ(result.voxels[0].node_count, 3U);
  EXPECT_EQ(result.voxels[1].cell.x, 1);
  EXPECT_EQ(result.voxels[1].label, ais_gng_msgs::msg::TopologicalMap::WALL);
}

TEST(TopologicalGridAssignment, UsesLowestLabelToBreakDominantLabelTie)
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.002F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::DEFAULT),
  };

  fuzzrobo::topological_grid::VoxelizationOptions options;
  options.excluded_labels.clear();
  options.require_input_points = false;
  const auto result = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{0.02, 0.0, 0.0, 0.0}, options);

  ASSERT_EQ(result.voxels.size(), 1U);
  EXPECT_EQ(result.voxels.front().label, ais_gng_msgs::msg::TopologicalMap::DEFAULT);
}

TEST(TopologicalGridAssignment, UsesFloorForNegativeCoordinates)
{
  const auto node = makeNode(
    -0.001F, -0.021F, 0.039F, ais_gng_msgs::msg::TopologicalMap::DEFAULT);
  const auto cell = fuzzrobo::topological_grid::nodeToGridCell(
    node, fuzzrobo::topological_grid::GridSpec{0.02, 0.0, 0.0, 0.0});

  EXPECT_EQ(cell.x, -1);
  EXPECT_EQ(cell.y, -2);
  EXPECT_EQ(cell.z, 1);
}

TEST(TopologicalGridAssignment, IncludesEveryNonExcludedLabelWithCurrentPointSupport)
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(
      0.001F, 0.001F, 0.001F,
      ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(
      0.021F, 0.001F, 0.001F,
      ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(
      0.041F, 0.001F, 0.001F,
      ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(
      0.061F, 0.001F, 0.001F,
      ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN),
  };

  fuzzrobo::topological_grid::GridPointCounts point_counts;
  point_counts.emplace(fuzzrobo::topological_grid::GridCell{2, 0, 0}, 2U);
  point_counts.emplace(fuzzrobo::topological_grid::GridCell{4, 0, 0}, 1U);
  const auto result = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{},
    fuzzrobo::topological_grid::VoxelizationOptions{}, &point_counts);

  EXPECT_EQ(result.included_node_count, 2U);
  EXPECT_EQ(result.excluded_node_count, 1U);
  EXPECT_EQ(result.unsupported_node_count, 1U);
  ASSERT_EQ(result.label_voxels.size(), 3U);
  EXPECT_EQ(result.label_voxels.front().input_point_count, 0U);
  ASSERT_EQ(result.voxels.size(), 2U);
  EXPECT_EQ(result.voxels.front().cell.x, 2);
  EXPECT_EQ(result.voxels.front().input_point_count, 2U);
  EXPECT_EQ(result.voxels.back().label, ais_gng_msgs::msg::TopologicalMap::WALL);
}

TEST(TopologicalGridAssignment, CountsTwentySixConnectedNeighbors)
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(
      0.001F, 0.001F, 0.001F,
      ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(
      0.011F, 0.011F, 0.011F,
      ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(
      0.031F, 0.001F, 0.001F,
      ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
  };

  fuzzrobo::topological_grid::GridPointCounts point_counts;
  point_counts.emplace(fuzzrobo::topological_grid::GridCell{0, 0, 0}, 1U);
  point_counts.emplace(fuzzrobo::topological_grid::GridCell{1, 1, 1}, 1U);
  point_counts.emplace(fuzzrobo::topological_grid::GridCell{3, 0, 0}, 1U);
  const auto result = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{},
    fuzzrobo::topological_grid::VoxelizationOptions{}, &point_counts);

  ASSERT_EQ(result.voxels.size(), 3U);
  EXPECT_EQ(result.voxels[0].neighbor_count, 1U);
  EXPECT_EQ(result.voxels[1].neighbor_count, 1U);
  EXPECT_EQ(result.voxels[2].neighbor_count, 0U);
  EXPECT_EQ(result.isolated_voxel_count, 1U);
}

TEST(TopologicalGridAssignment, CountsGngNeighborsWithoutPointSupport)
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(
      0.001F, 0.001F, 0.001F,
      ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(
      0.011F, 0.001F, 0.001F,
      ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  fuzzrobo::topological_grid::GridPointCounts point_counts;
  point_counts.emplace(fuzzrobo::topological_grid::GridCell{0, 0, 0}, 1U);

  const auto result = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{},
    fuzzrobo::topological_grid::VoxelizationOptions{}, &point_counts);

  ASSERT_EQ(result.label_voxels.size(), 2U);
  EXPECT_EQ(result.label_voxels[0].neighbor_count, 1U);
  EXPECT_EQ(result.label_voxels[1].neighbor_count, 1U);
  ASSERT_EQ(result.voxels.size(), 1U);
  EXPECT_EQ(result.isolated_voxel_count, 0U);
}

TEST(TopologicalGridAssignment, CountsLabelAndPointInputHistorySeparately)
{
  using fuzzrobo::topological_grid::GridCell;
  using fuzzrobo::topological_grid::LabeledGridVoxel;
  using fuzzrobo::topological_grid::TemporalVoxelFilter;
  using fuzzrobo::topological_grid::TemporalVoxelFilterConfig;

  TemporalVoxelFilter filter(TemporalVoxelFilterConfig{100, 3, 3, 5, 5});
  const LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 2, 2, 1};
  const LabeledGridVoxel label_only{GridCell{1, 2, 3}, 3, 2, 0, 1};

  EXPECT_TRUE(filter.update({supported}).empty());
  EXPECT_TRUE(filter.update({label_only}).empty());
  EXPECT_TRUE(filter.update({supported}).empty());
  fuzzrobo::topological_grid::GridPointCounts point_counts;
  point_counts.emplace(GridCell{1, 2, 3}, 1U);
  EXPECT_TRUE(filter.update({}, true, 1, &point_counts).empty());

  const auto stable = filter.update({supported});
  ASSERT_EQ(stable.size(), 1U);
  EXPECT_EQ(stable.front().history_sample_count, 5U);
  EXPECT_EQ(stable.front().label_history_count, 4U);
  EXPECT_EQ(stable.front().point_input_history_count, 4U);
  const auto retained_without_current_points = filter.update({label_only});
  ASSERT_EQ(retained_without_current_points.size(), 1U);
  EXPECT_EQ(retained_without_current_points.front().input_point_count, 0U);
}

TEST(TopologicalGridAssignment, RemovesIsolatedVoxelWhenPointHistoryAgesOut)
{
  using fuzzrobo::topological_grid::GridCell;
  using fuzzrobo::topological_grid::LabeledGridVoxel;
  using fuzzrobo::topological_grid::TemporalVoxelFilter;
  using fuzzrobo::topological_grid::TemporalVoxelFilterConfig;

  TemporalVoxelFilter filter(TemporalVoxelFilterConfig{5, 2, 2, 2, 2});
  const LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 1, 1, 0};
  const LabeledGridVoxel label_only{GridCell{1, 2, 3}, 3, 1, 0, 0};

  EXPECT_TRUE(filter.update({supported}).empty());
  ASSERT_EQ(filter.update({supported}).size(), 1U);
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(filter.update({label_only}).size(), 1U);
  }
  EXPECT_TRUE(filter.update({label_only}).empty());
}

TEST(TopologicalGridAssignment, KeepsNonIsolatedVoxelWithoutPointHistory)
{
  using fuzzrobo::topological_grid::GridCell;
  using fuzzrobo::topological_grid::LabeledGridVoxel;
  using fuzzrobo::topological_grid::TemporalVoxelFilter;
  using fuzzrobo::topological_grid::TemporalVoxelFilterConfig;

  TemporalVoxelFilter filter(TemporalVoxelFilterConfig{5, 2, 2, 2, 2});
  const LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 1, 1, 1};
  const LabeledGridVoxel label_only{GridCell{1, 2, 3}, 3, 1, 0, 1};

  EXPECT_TRUE(filter.update({supported}).empty());
  ASSERT_EQ(filter.update({supported}).size(), 1U);
  std::vector<LabeledGridVoxel> retained;
  for (int i = 0; i < 6; ++i) {
    retained = filter.update({label_only});
    ASSERT_EQ(retained.size(), 1U);
  }
  EXPECT_EQ(retained.front().point_input_history_count, 0U);
}

TEST(TopologicalGridAssignment, KeepsConfirmedVoxelAfterLabelHistoryAgesOut)
{
  using fuzzrobo::topological_grid::GridCell;
  using fuzzrobo::topological_grid::LabeledGridVoxel;
  using fuzzrobo::topological_grid::TemporalVoxelFilter;
  using fuzzrobo::topological_grid::TemporalVoxelFilterConfig;

  TemporalVoxelFilter filter(TemporalVoxelFilterConfig{5, 2, 2, 2, 2, 5});
  const LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 1, 1, 1};

  EXPECT_TRUE(filter.update({supported}).empty());
  ASSERT_EQ(filter.update({supported}).size(), 1U);
  std::vector<LabeledGridVoxel> retained;
  for (int i = 0; i < 5; ++i) {
    retained = filter.update({});
    ASSERT_EQ(retained.size(), 1U);
  }
  EXPECT_EQ(retained.front().label_history_count, 0U);
  EXPECT_TRUE(filter.update({}).empty());
}

TEST(TopologicalGridAssignment, KeepsOccupancyAcrossIncludedLabelChanges)
{
  using fuzzrobo::topological_grid::GridCell;
  using fuzzrobo::topological_grid::LabeledGridVoxel;
  using fuzzrobo::topological_grid::TemporalVoxelFilter;
  using fuzzrobo::topological_grid::TemporalVoxelFilterConfig;

  TemporalVoxelFilter filter(TemporalVoxelFilterConfig{100, 2, 2, 2, 2});
  const GridCell cell{1, 2, 3};
  EXPECT_TRUE(filter.update({LabeledGridVoxel{cell, 2, 1, 1, 1}}).empty());
  const auto changed = filter.update({LabeledGridVoxel{cell, 3, 1, 1, 1}});
  ASSERT_EQ(changed.size(), 1U);
  EXPECT_EQ(changed.front().label, 2);
  const auto stable = filter.update({LabeledGridVoxel{cell, 3, 1, 1, 1}});
  ASSERT_EQ(stable.size(), 1U);
  EXPECT_EQ(stable.front().label_history_count, 3U);
  EXPECT_EQ(stable.front().point_input_history_count, 3U);
}

TEST(TopologicalGridAssignment, IsolatedVoxelUsesHigherHistoryCounts)
{
  using fuzzrobo::topological_grid::GridCell;
  using fuzzrobo::topological_grid::LabeledGridVoxel;
  using fuzzrobo::topological_grid::TemporalVoxelFilter;
  using fuzzrobo::topological_grid::TemporalVoxelFilterConfig;

  TemporalVoxelFilter filter(TemporalVoxelFilterConfig{100, 3, 3, 5, 5});
  const LabeledGridVoxel isolated{GridCell{1, 2, 3}, 3, 1, 1, 0};

  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(filter.update({isolated}).empty());
  }
  ASSERT_EQ(filter.update({isolated}).size(), 1U);
  for (int i = 0; i < 10; ++i) {
    const auto retained = filter.update({});
    ASSERT_EQ(retained.size(), 1U);
    EXPECT_EQ(retained.front().node_count, 0U);
  }
  EXPECT_TRUE(filter.update({}).empty());
  EXPECT_EQ(filter.trackedVoxelCount(), 1U);

  for (int i = 0; i < 89; ++i) {
    EXPECT_TRUE(filter.update({}).empty());
  }
  EXPECT_EQ(filter.trackedVoxelCount(), 0U);
}

TEST(TopologicalGridAssignment, BridgesShortVoxelPositionJitter)
{
  using fuzzrobo::topological_grid::GridCell;
  using fuzzrobo::topological_grid::LabeledGridVoxel;
  using fuzzrobo::topological_grid::TemporalVoxelFilter;
  using fuzzrobo::topological_grid::TemporalVoxelFilterConfig;

  TemporalVoxelFilter filter(TemporalVoxelFilterConfig{100, 2, 2, 2, 2, 3});
  const LabeledGridVoxel original{GridCell{0, 0, 0}, 3, 1, 1, 1};
  const LabeledGridVoxel moved{GridCell{1, 0, 0}, 3, 1, 1, 1};

  EXPECT_TRUE(filter.update({original}).empty());
  ASSERT_EQ(filter.update({original}).size(), 1U);
  const auto first_moved = filter.update({moved});
  ASSERT_EQ(first_moved.size(), 1U);
  EXPECT_EQ(first_moved.front().cell.x, 0);
  const auto second_moved = filter.update({moved});
  ASSERT_EQ(second_moved.size(), 2U);
  const auto third_moved = filter.update({moved});
  ASSERT_EQ(third_moved.size(), 2U);
  const auto settled = filter.update({moved});
  ASSERT_EQ(settled.size(), 1U);
  EXPECT_EQ(settled.front().cell.x, 1);
}

TEST(TopologicalGridAssignment, FillsBetweenStableVoxelsUsingGngEdge)
{
  using namespace fuzzrobo::topological_grid;
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.041F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  map.edges = {0, 1};
  const std::vector<LabeledGridVoxel> direct{
    LabeledGridVoxel{GridCell{0, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{4, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
  };

  const auto result = inferVoxelsFromStableVoxelEdges(
    map, GridSpec{}, direct, VoxelizationOptions{}.excluded_labels);

  EXPECT_EQ(result.input_edge_count, 1U);
  EXPECT_EQ(result.voxel_edge_count, 1U);
  ASSERT_EQ(result.voxels.size(), 3U);
  EXPECT_EQ(result.voxels[0].cell.x, 1);
  EXPECT_EQ(result.voxels[1].cell.x, 2);
  EXPECT_EQ(result.voxels[2].cell.x, 3);
  EXPECT_EQ(result.voxels[1].edge_support_count, 1U);
}

TEST(TopologicalGridAssignment, RequiresBothEdgeEndpointsToBeStableDirectVoxels)
{
  using namespace fuzzrobo::topological_grid;
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.021F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.041F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  map.edges = {0, 1, 1, 2};
  const std::vector<LabeledGridVoxel> direct{
    LabeledGridVoxel{GridCell{0, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{2, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
  };

  const auto result = inferVoxelsFromStableVoxelEdges(
    map, GridSpec{}, direct, VoxelizationOptions{}.excluded_labels);

  EXPECT_EQ(result.voxel_edge_count, 1U);
  EXPECT_EQ(result.inactive_endpoint_edge_count, 1U);
  ASSERT_EQ(result.voxels.size(), 1U);
  EXPECT_EQ(result.voxels.front().cell.x, 1);
}

TEST(TopologicalGridAssignment, RejectsExcludedDuplicateAndLongVoxelEdges)
{
  using namespace fuzzrobo::topological_grid;
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.021F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.041F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN),
    makeNode(0.201F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  map.edges = {0, 1, 1, 0, 0, 2, 0, 3};
  const std::vector<LabeledGridVoxel> direct{
    LabeledGridVoxel{GridCell{0, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{2, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{4, 0, 0}, ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN},
    LabeledGridVoxel{GridCell{20, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
  };

  const auto result = inferVoxelsFromStableVoxelEdges(
    map, GridSpec{}, direct, VoxelizationOptions{}.excluded_labels,
    EdgeInferenceOptions{true, 0.05});

  EXPECT_EQ(result.voxel_edge_count, 1U);
  EXPECT_EQ(result.duplicate_voxel_edge_count, 1U);
  EXPECT_EQ(result.excluded_edge_count, 1U);
  EXPECT_EQ(result.overlength_edge_count, 1U);
  ASSERT_EQ(result.voxels.size(), 1U);
}

TEST(TopologicalGridAssignment, DirectVoxelWinsWhenMergingInferredVoxels)
{
  using namespace fuzzrobo::topological_grid;
  const LabeledGridVoxel direct{GridCell{1, 0, 0}, 2, 3, 4, 5};
  LabeledGridVoxel overlapping{GridCell{1, 0, 0}, 3};
  overlapping.edge_support_count = 2;
  LabeledGridVoxel inferred{GridCell{2, 0, 0}, 3};
  inferred.edge_support_count = 1;

  const auto merged = mergeDirectAndInferredVoxels({direct}, {overlapping, inferred});

  ASSERT_EQ(merged.size(), 2U);
  EXPECT_EQ(merged.front().label, 2);
  EXPECT_EQ(merged.front().edge_support_count, 0U);
  EXPECT_EQ(merged.back().cell.x, 2);
}

}  // namespace
