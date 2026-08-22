#include <ais_gng/topological_grid/topological_grid_assignment.hpp>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <gtest/gtest.h>

#include <algorithm>
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

TEST(TopologicalGridAssignment, ExcludesAmbiguousNodeGenerationFromIdentityTracking)
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.021F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  map.nodes[0].id = 7;
  map.nodes[1].id = 7;
  map.nodes[0].frame = 10;
  map.nodes[1].frame = 10;
  fuzzrobo::topological_grid::VoxelizationOptions options;
  options.require_input_points = false;

  const auto result = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{}, options);

  EXPECT_TRUE(result.eligible_nodes.empty());
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

TEST(TopologicalGridAssignment, MarksOnlyFourCellUnknownGngComponentsAsObjectEvents)
{
  using namespace fuzzrobo::topological_grid;
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(0.011F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(0.021F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(0.031F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
  };
  map.edges = {0, 1, 1, 2, 2, 3};
  GridPointCounts point_counts;
  for (int x = 0; x < 4; ++x) {
    point_counts.emplace(GridCell{x, 0, 0}, 1U);
  }

  const auto result = voxelizeNodes(map, GridSpec{}, VoxelizationOptions{}, &point_counts);

  EXPECT_EQ(result.unknown_component_count, 1U);
  EXPECT_EQ(result.unknown_component_event_count, 1U);
  EXPECT_EQ(result.unknown_component_event_node_count, 4U);
  EXPECT_EQ(result.unknown_component_event_voxel_count, 4U);
  ASSERT_EQ(result.label_voxels.size(), 4U);
  EXPECT_TRUE(std::all_of(
      result.label_voxels.begin(), result.label_voxels.end(),
      [](const LabeledGridVoxel &voxel) {
        return voxel.unknown_component_evaluated && voxel.unknown_component_event &&
               voxel.unknown_component_event_id != 0U;
      }));

  map.nodes.pop_back();
  map.edges = {0, 1, 1, 2};
  const auto too_small = voxelizeNodes(map, GridSpec{}, VoxelizationOptions{}, &point_counts);
  EXPECT_EQ(too_small.unknown_component_count, 1U);
  EXPECT_EQ(too_small.unknown_component_event_count, 0U);
  EXPECT_TRUE(std::none_of(
      too_small.label_voxels.begin(), too_small.label_voxels.end(),
      [](const LabeledGridVoxel &voxel) {return voxel.unknown_component_event;}));
}

TEST(TopologicalGridAssignment, AutoPointSupportUsesGngInputAssignmentsAcrossCellBoundaries)
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.019F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(0.041F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  map.nodes[0].inpcl_ids = {10, 11};
  fuzzrobo::topological_grid::GridPointCounts point_counts;
  point_counts.emplace(fuzzrobo::topological_grid::GridCell{0, 0, 0}, 2U);
  fuzzrobo::topological_grid::VoxelizationOptions options;
  options.point_support_mode = fuzzrobo::topological_grid::PointSupportMode::Auto;

  const auto result = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{0.01, 0.0, 0.0, 0.0},
    options, &point_counts);

  EXPECT_EQ(result.included_node_count, 1U);
  EXPECT_EQ(result.unsupported_node_count, 1U);
  ASSERT_EQ(result.voxels.size(), 1U);
  EXPECT_EQ(result.voxels.front().cell.x, 1);
  EXPECT_EQ(result.voxels.front().input_point_count, 2U);
}

TEST(TopologicalGridAssignment, AutoPointSupportFallsBackPerCellWhenGngIdsAreSparse)
{
  using namespace fuzzrobo::topological_grid;
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(0.041F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  map.nodes[0].inpcl_ids = {10U, 11U};
  GridPointCounts point_counts;
  point_counts.emplace(GridCell{4, 0, 0}, 3U);
  VoxelizationOptions options;
  options.point_support_mode = PointSupportMode::Auto;
  options.point_support_radius_m = 0.0;

  const auto result = voxelizeNodes(
    map, GridSpec{0.01, 0.0, 0.0, 0.0}, options, &point_counts);

  ASSERT_EQ(result.voxels.size(), 2U);
  EXPECT_EQ(result.voxels[0].cell, (GridCell{0, 0, 0}));
  EXPECT_EQ(result.voxels[0].input_point_count, 2U);
  EXPECT_EQ(result.voxels[1].cell, (GridCell{4, 0, 0}));
  EXPECT_EQ(result.voxels[1].input_point_count, 3U);
}

TEST(TopologicalGridAssignment, AutoPointSupportFallsBackToMetricRadius)
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.021F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(0.061F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  fuzzrobo::topological_grid::GridPointCounts point_counts;
  point_counts.emplace(fuzzrobo::topological_grid::GridCell{0, 0, 0}, 3U);
  fuzzrobo::topological_grid::VoxelizationOptions options;
  options.point_support_mode = fuzzrobo::topological_grid::PointSupportMode::Auto;
  options.point_support_radius_m = 0.02;

  const auto result = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{0.01, 0.0, 0.0, 0.0},
    options, &point_counts);

  EXPECT_EQ(result.included_node_count, 1U);
  EXPECT_EQ(result.unsupported_node_count, 1U);
  ASSERT_EQ(result.voxels.size(), 1U);
  EXPECT_EQ(result.voxels.front().cell.x, 2);
  EXPECT_EQ(result.voxels.front().input_point_count, 3U);
}

TEST(TopologicalGridAssignment, CachedRadiusSupportMatchesExactCubeAcrossTileBoundaries)
{
  using namespace fuzzrobo::topological_grid;
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(-0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
    makeNode(0.071F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
  };
  GridPointCounts point_counts;
  // Cell -1 with a radius of two crosses tiles [-8, -1] and [0, 7].
  point_counts.emplace(GridCell{-3, 0, 0}, 7U);
  point_counts.emplace(GridCell{1, 0, 0}, 11U);
  point_counts.emplace(GridCell{-1, -2, 2}, 13U);
  // Cell 7 crosses tiles [0, 7] and [8, 15].
  point_counts.emplace(GridCell{5, 0, 0}, 2U);
  point_counts.emplace(GridCell{9, 0, 0}, 3U);
  point_counts.emplace(GridCell{7, 2, -2}, 5U);
  // Force the tiled cache path without contributing to either query.
  for (int index = 0; index < 128; ++index) {
    point_counts.emplace(GridCell{100 + index, 100, 100}, 1U);
  }
  VoxelizationOptions options;
  options.require_input_points = false;
  options.point_support_mode = PointSupportMode::Radius;
  options.point_support_radius_m = 0.02;

  const auto result = voxelizeNodes(map, GridSpec{}, options, &point_counts);

  ASSERT_EQ(result.label_voxels.size(), 2U);
  EXPECT_EQ(result.label_voxels[0].cell, (GridCell{-1, 0, 0}));
  EXPECT_EQ(result.label_voxels[0].input_point_count, 31U);
  EXPECT_EQ(result.label_voxels[1].cell, (GridCell{7, 0, 0}));
  EXPECT_EQ(result.label_voxels[1].input_point_count, 10U);
}

TEST(TopologicalGridAssignment, SameCellPointSupportKeepsLegacyBehavior)
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.021F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
  };
  fuzzrobo::topological_grid::GridPointCounts point_counts;
  point_counts.emplace(fuzzrobo::topological_grid::GridCell{0, 0, 0}, 3U);
  fuzzrobo::topological_grid::VoxelizationOptions options;
  options.point_support_mode = fuzzrobo::topological_grid::PointSupportMode::SameCell;

  const auto result = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{0.01, 0.0, 0.0, 0.0},
    options, &point_counts);

  EXPECT_EQ(result.included_node_count, 0U);
  EXPECT_EQ(result.unsupported_node_count, 1U);
  EXPECT_TRUE(result.voxels.empty());
}

TEST(TopologicalGridAssignment, MetricNeighborRadiusIsStableAcrossGridSizes)
{
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.021F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  fuzzrobo::topological_grid::VoxelizationOptions options;
  options.require_input_points = false;
  options.neighbor_radius_m = 0.025;

  const auto coarse = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{0.01, 0.0, 0.0, 0.0}, options);
  const auto fine = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{0.005, 0.0, 0.0, 0.0}, options);

  ASSERT_EQ(coarse.voxels.size(), 2U);
  ASSERT_EQ(fine.voxels.size(), 2U);
  EXPECT_EQ(coarse.isolated_voxel_count, 0U);
  EXPECT_EQ(fine.isolated_voxel_count, 0U);
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

TEST(TopologicalGridAssignment, ActivatesFromContinuousTemporalEvidence)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.history_window_size = 10;
  config.time_constant_sec = 0.10;
  config.activation_score = 0.65;
  config.retention_score = 0.30;
  TemporalVoxelFilter filter(config);
  const LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 1, 1, 1};

  EXPECT_TRUE(filter.update({supported}).empty());
  const auto stable = filter.update({supported});
  ASSERT_EQ(stable.size(), 1U);
  EXPECT_GT(stable.front().temporal_stability_score, config.activation_score);
  EXPECT_DOUBLE_EQ(stable.front().point_support_score, 1.0);
}

TEST(TopologicalGridAssignment, KeepsObjectEvidenceAcrossTerrainRelabeling)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.history_window_size = 10;
  config.time_constant_sec = 0.10;
  config.activation_score = 0.65;
  config.retention_score = 0.30;
  TemporalVoxelFilter filter(config);

  std::vector<LabeledGridVoxel> object_events;
  std::vector<LabeledGridVoxel> terrain_observations;
  for (int x = 0; x < 4; ++x) {
    LabeledGridVoxel object_event{GridCell{x, 2, 3},
      ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT, 1, 1, 1};
    object_event.unknown_component_evaluated = true;
    object_event.unknown_component_event = true;
    object_event.unknown_component_event_id = 1U;
    auto terrain = object_event;
    terrain.label = ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN;
    terrain.unknown_component_event = false;
    terrain.unknown_component_event_id = 0U;
    object_events.push_back(object_event);
    terrain_observations.push_back(terrain);
  }

  EXPECT_TRUE(filter.update(object_events, true, 1, nullptr, nullptr, 0.10).empty());
  const auto stable = filter.update(
    terrain_observations, true, 1, nullptr, nullptr, 0.10);
  ASSERT_EQ(stable.size(), 4U);
  EXPECT_TRUE(std::all_of(
      stable.begin(), stable.end(), [](const LabeledGridVoxel &voxel) {
        return voxel.label == ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT &&
               voxel.unknown_component_active_cell_count == 4U;
      }));

  GridVisibilityStates free_space;
  for (std::size_t index = 1; index < terrain_observations.size(); ++index) {
    free_space.emplace(terrain_observations[index].cell, VoxelVisibilityState::Free);
  }
  EXPECT_TRUE(filter.update(
      {terrain_observations.front()}, true, 1, nullptr, nullptr, 0.20, &free_space).empty());

  LabeledGridVoxel isolated_unknown{GridCell{5, 2, 3},
    ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT, 1, 1, 1};
  isolated_unknown.unknown_component_evaluated = true;
  TemporalVoxelFilter noise_filter(config);
  EXPECT_TRUE(noise_filter.update(
      {isolated_unknown}, true, 1, nullptr, nullptr, 0.20).empty());
  EXPECT_EQ(noise_filter.trackedVoxelCount(), 0U);
}

TEST(TopologicalGridAssignment, DoesNotUseConnectivityForTemporalActivation)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.10;
  config.activation_score = 0.65;
  config.retention_score = 0.30;
  TemporalVoxelFilter filter(config);
  const LabeledGridVoxel isolated{GridCell{1, 2, 3}, 3, 1, 1, 0};

  EXPECT_TRUE(filter.update({isolated}).empty());
  ASSERT_EQ(filter.update({isolated}).size(), 1U);
}

TEST(TopologicalGridAssignment, RejectsSparseCellRelativeToLocalPointDensity)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.10;
  config.activation_score = 0.65;
  config.retention_score = 0.30;
  TemporalVoxelFilter filter(config);
  const LabeledGridVoxel sparse{GridCell{0, 0, 0}, 3, 1, 1, 0};
  const LabeledGridVoxel dense{GridCell{1, 0, 0}, 3, 1, 9, 1};

  for (int i = 0; i < 5; ++i) {
    const auto stable = filter.update({sparse, dense});
    const auto sparse_it = std::find_if(
      stable.begin(), stable.end(), [](const LabeledGridVoxel &voxel) {
        return voxel.cell == GridCell{0, 0, 0};
      });
    EXPECT_EQ(sparse_it, stable.end());
  }
}

TEST(TopologicalGridAssignment, DecaysAndRemovesAfterAnObservedCellDisappears)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.50;
  config.activation_score = 0.65;
  config.retention_score = 0.30;
  TemporalVoxelFilter filter(config);
  const LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 1, 1, 1};

  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(filter.update({supported}).empty());
  }
  ASSERT_EQ(filter.update({supported}).size(), 1U);
  const auto retained_once = filter.update({});
  ASSERT_EQ(retained_once.size(), 1U);
  EXPECT_EQ(retained_once.front().node_count, 0U);
  ASSERT_EQ(filter.update({}).size(), 1U);
  ASSERT_EQ(filter.update({}).size(), 1U);
  EXPECT_TRUE(filter.update({}).empty());
}

TEST(TopologicalGridAssignment, RetainsSingleDropoutButRejectsRepeatedFlicker)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.20;
  config.activation_score = 0.60;
  config.retention_score = 0.35;
  const LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 1, 1, 1};

  TemporalVoxelFilter stable_filter(config);
  for (int i = 0; i < 10; ++i) {
    stable_filter.update({supported}, true, 1, nullptr, nullptr, 0.05);
  }
  const auto temporarily_missing =
    stable_filter.update({}, true, 1, nullptr, nullptr, 0.05);
  ASSERT_EQ(temporarily_missing.size(), 1U);

  TemporalVoxelFilter flicker_filter(config);
  for (int i = 0; i < 20; ++i) {
    const auto flickering = i % 2 == 0
      ? flicker_filter.update({supported}, true, 1, nullptr, nullptr, 0.05)
      : flicker_filter.update({}, true, 1, nullptr, nullptr, 0.05);
    EXPECT_TRUE(flickering.empty());
  }
}

TEST(TopologicalGridAssignment, UsesElapsedSecondsInsteadOfUpdateCount)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.20;
  config.activation_score = 0.60;
  config.retention_score = 0.30;
  const LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 1, 1, 1};

  TemporalVoxelFilter one_update_filter(config);
  const auto one_update =
    one_update_filter.update({supported}, true, 1, nullptr, nullptr, 0.20);
  ASSERT_EQ(one_update.size(), 1U);

  TemporalVoxelFilter two_update_filter(config);
  EXPECT_TRUE(two_update_filter.update({supported}, true, 1, nullptr, nullptr, 0.10).empty());
  const auto two_updates =
    two_update_filter.update({supported}, true, 1, nullptr, nullptr, 0.10);
  ASSERT_EQ(two_updates.size(), 1U);
  EXPECT_NEAR(
    one_update.front().temporal_stability_score,
    two_updates.front().temporal_stability_score,
    1.0e-9);
}

TEST(TopologicalGridAssignment, HoldsOccludedAndOutOfViewVoxelsButDeletesVisibleFreeSpace)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.10;
  config.activation_score = 0.60;
  config.retention_score = 0.30;
  const LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 1, 1, 1};

  const auto make_active_filter = [&config, &supported]() {
      TemporalVoxelFilter filter(config);
      EXPECT_EQ(filter.update({supported}, true, 1, nullptr, nullptr, 0.20).size(), 1U);
      return filter;
    };

  auto out_of_view_filter = make_active_filter();
  GridVisibilityStates out_of_view;
  out_of_view.emplace(supported.cell, VoxelVisibilityState::OutOfView);
  const auto held_out_of_view = out_of_view_filter.update(
    {}, true, 1, nullptr, nullptr, 0.50, &out_of_view);
  ASSERT_EQ(held_out_of_view.size(), 1U);
  EXPECT_NEAR(held_out_of_view.front().temporal_stability_score, 0.8646647, 1.0e-6);

  auto occluded_filter = make_active_filter();
  GridVisibilityStates occluded;
  occluded.emplace(supported.cell, VoxelVisibilityState::Occluded);
  EXPECT_EQ(occluded_filter.update({}, true, 1, nullptr, nullptr, 0.50, &occluded).size(), 1U);

  auto unknown_filter = make_active_filter();
  GridVisibilityStates unknown;
  unknown.emplace(supported.cell, VoxelVisibilityState::Unknown);
  EXPECT_EQ(unknown_filter.update({}, true, 1, nullptr, nullptr, 0.50, &unknown).size(), 1U);

  auto free_filter = make_active_filter();
  GridVisibilityStates free_space;
  free_space.emplace(supported.cell, VoxelVisibilityState::Free);
  EXPECT_TRUE(free_filter.update({}, true, 1, nullptr, nullptr, 0.50, &free_space).empty());
}

TEST(TopologicalGridAssignment, NormalDriftSuppressesOtherwisePointSupportedVoxel)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.10;
  config.activation_score = 0.60;
  config.retention_score = 0.30;
  const LabeledGridVoxel stationary{GridCell{1, 2, 3}, 3, 1, 1, 1};

  TemporalVoxelFilter stationary_filter(config);
  ASSERT_EQ(stationary_filter.update({stationary}, true, 1, nullptr, nullptr, 0.20).size(), 1U);
  EXPECT_EQ(stationary_filter.update({stationary}, true, 1, nullptr, nullptr, 0.20).size(), 1U);

  LabeledGridVoxel normal_motion = stationary;
  normal_motion.normal_drift_score = 1.0;
  TemporalVoxelFilter moving_filter(config);
  ASSERT_EQ(moving_filter.update({stationary}, true, 1, nullptr, nullptr, 0.20).size(), 1U);
  EXPECT_TRUE(moving_filter.update(
      {normal_motion}, true, 1, nullptr, nullptr, 0.50).empty());
}

TEST(TopologicalGridAssignment, IgnoresSubthresholdNormalDrift)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.10;
  config.activation_score = 0.60;
  config.retention_score = 0.30;
  const LabeledGridVoxel stationary{GridCell{1, 2, 3}, 3, 1, 1, 1};
  auto small_normal_motion = stationary;
  small_normal_motion.normal_drift_score = 0.49;

  TemporalVoxelFilter filter(config);
  ASSERT_EQ(filter.update({stationary}, true, 1, nullptr, nullptr, 0.20).size(), 1U);
  const auto stable = filter.update(
    {small_normal_motion}, true, 1, nullptr, nullptr, 0.50);
  ASSERT_EQ(stable.size(), 1U);
  EXPECT_GT(stable.front().temporal_stability_score, 0.99);
}

TEST(TopologicalGridAssignment, HoldsActiveGngStructureAcrossPointCloudDropout)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.10;
  config.activation_score = 0.60;
  config.retention_score = 0.30;
  const LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 1, 1, 1};
  const LabeledGridVoxel no_points{GridCell{1, 2, 3}, 3, 1, 0, 1};

  TemporalVoxelFilter dropout_filter(config);
  ASSERT_EQ(dropout_filter.update({supported}, true, 1, nullptr, nullptr, 0.20).size(), 1U);
  const auto held = dropout_filter.update({no_points}, true, 1, nullptr, nullptr, 1.00);
  ASSERT_EQ(held.size(), 1U);
  EXPECT_TRUE(held.front().retained_by_gng_structure);
  EXPECT_NEAR(held.front().temporal_stability_score, 0.8646647, 1.0e-6);

  TemporalVoxelFilter free_space_filter(config);
  ASSERT_EQ(free_space_filter.update({supported}, true, 1, nullptr, nullptr, 0.20).size(), 1U);
  GridVisibilityStates free_space;
  free_space.emplace(no_points.cell, VoxelVisibilityState::Free);
  EXPECT_TRUE(free_space_filter.update(
      {no_points}, true, 1, nullptr, nullptr, 1.00, &free_space).empty());

  TemporalVoxelFilter moving_filter(config);
  ASSERT_EQ(moving_filter.update({supported}, true, 1, nullptr, nullptr, 0.20).size(), 1U);
  auto strong_normal_motion = no_points;
  strong_normal_motion.normal_drift_score = 1.0;
  EXPECT_TRUE(moving_filter.update(
      {strong_normal_motion}, true, 1, nullptr, nullptr, 1.00).empty());
}

TEST(TopologicalGridAssignment, HoldsMissingVoxelWhenFormerGngNeighborhoodIsStatic)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.10;
  config.activation_score = 0.60;
  config.retention_score = 0.30;
  LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 1, 1, 1};
  const NodeIdentity identity{7, 10};
  supported.node_observations.push_back(
    NodeObservation{identity, 0.0, 0.0, 0.0, 3});

  NodeLocalStructureStates static_neighborhood;
  static_neighborhood.emplace(identity, NodeLocalStructureState::Static);
  TemporalVoxelFilter static_filter(config);
  ASSERT_EQ(static_filter.update({supported}, true, 1, nullptr, nullptr, 0.20).size(), 1U);
  const auto held = static_filter.update(
    {}, true, 1, nullptr, nullptr, 1.00, nullptr, &static_neighborhood);
  ASSERT_EQ(held.size(), 1U);
  EXPECT_TRUE(held.front().retained_by_local_structure);

  TemporalVoxelFilter free_space_filter(config);
  ASSERT_EQ(free_space_filter.update({supported}, true, 1, nullptr, nullptr, 0.20).size(), 1U);
  GridVisibilityStates free_space;
  free_space.emplace(supported.cell, VoxelVisibilityState::Free);
  EXPECT_TRUE(free_space_filter.update(
      {}, true, 1, nullptr, nullptr, 1.00, &free_space, &static_neighborhood).empty());

  NodeLocalStructureStates moving_neighborhood;
  moving_neighborhood.emplace(identity, NodeLocalStructureState::Moving);
  TemporalVoxelFilter moving_filter(config);
  ASSERT_EQ(moving_filter.update({supported}, true, 1, nullptr, nullptr, 0.20).size(), 1U);
  EXPECT_TRUE(moving_filter.update(
      {}, true, 1, nullptr, nullptr, 1.00, nullptr, &moving_neighborhood).empty());

  NodeLocalStructureStates ambiguous_neighborhood;
  ambiguous_neighborhood.emplace(identity, NodeLocalStructureState::Ambiguous);
  TemporalVoxelFilter ambiguous_filter(config);
  ASSERT_EQ(ambiguous_filter.update({supported}, true, 1, nullptr, nullptr, 0.20).size(), 1U);
  EXPECT_EQ(ambiguous_filter.update(
      {}, true, 1, nullptr, nullptr, 1.00, nullptr, &ambiguous_neighborhood).size(), 1U);
}

TEST(TopologicalGridAssignment, RequiresCurrentSupportButNotAnArbitraryHistoryCount)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.001;
  config.activation_score = 0.65;
  config.retention_score = 0.30;
  TemporalVoxelFilter filter(config);
  const LabeledGridVoxel label_only{GridCell{1, 2, 3}, 3, 1, 0, 1};
  const LabeledGridVoxel supported{GridCell{1, 2, 3}, 3, 1, 1, 1};

  EXPECT_TRUE(filter.update({label_only}).empty());
  EXPECT_TRUE(filter.update({label_only}).empty());
  EXPECT_TRUE(filter.update({supported}).empty());
  const auto stable = filter.update({supported});
  ASSERT_EQ(stable.size(), 1U);
  EXPECT_EQ(stable.front().label_history_count, 4U);
  EXPECT_EQ(stable.front().point_input_history_count, 2U);
}

TEST(TopologicalGridAssignment, RetainsByNearbyNodeIdentityOnlyWhenExplicitlyEnabled)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.time_constant_sec = 0.10;
  config.activation_score = 0.65;
  config.retention_score = 0.30;
  config.node_identity_retention_enabled = true;
  config.node_identity_max_displacement = 0.02;
  TemporalVoxelFilter filter(config);
  LabeledGridVoxel tracked{GridCell{0, 0, 0}, 3, 1, 1, 1};
  tracked.node_observations.push_back(NodeObservation{NodeIdentity{7, 10}, 0.0, 0.0, 0.0, 3});
  NodeObservationMap current_nodes;
  current_nodes.emplace(
    NodeIdentity{7, 10}, NodeObservation{NodeIdentity{7, 10}, 0.01, 0.0, 0.0, 3});

  EXPECT_TRUE(filter.update({tracked}, true, 1, nullptr, &current_nodes).empty());
  ASSERT_EQ(filter.update({tracked}, true, 1, nullptr, &current_nodes).size(), 1U);
  for (int i = 0; i < 4; ++i) {
    const auto retained = filter.update({}, true, 1, nullptr, &current_nodes);
    ASSERT_EQ(retained.size(), 1U);
    EXPECT_TRUE(retained.front().retained_by_node_identity);
  }
}

TEST(TopologicalGridAssignment, PreservesHistoryWhenAUniqueNodeMovesAcrossFineCells)
{
  using namespace fuzzrobo::topological_grid;
  TemporalVoxelFilterConfig config;
  config.history_window_size = 10;
  config.time_constant_sec = 0.10;
  config.activation_score = 0.65;
  config.retention_score = 0.30;
  config.node_identity_history_migration_enabled = true;
  config.node_identity_max_displacement = 0.02;
  TemporalVoxelFilter filter(config);
  LabeledGridVoxel original{GridCell{0, 0, 0}, 3, 1, 1, 1};
  original.node_observations.push_back(
    NodeObservation{NodeIdentity{7, 10}, 0.001, 0.0, 0.0, 3});
  LabeledGridVoxel moved{GridCell{9, 0, 0}, 3, 1, 1, 1};
  moved.node_observations.push_back(
    NodeObservation{NodeIdentity{7, 10}, 0.010, 0.0, 0.0, 3});

  EXPECT_TRUE(filter.update({original}).empty());
  ASSERT_EQ(filter.update({original}).size(), 1U);
  const auto stable = filter.update({moved});
  ASSERT_EQ(stable.size(), 1U);
  EXPECT_EQ(stable.front().cell.x, 9);
  EXPECT_EQ(stable.front().history_sample_count, 3U);
  EXPECT_EQ(filter.trackedVoxelCount(), 1U);
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

TEST(TopologicalGridAssignment, UsesGngEdgesInsteadOfVoxelAdjacencyForConnectivity)
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
  EdgeInferenceOptions options;
  options.maximum_edge_length = 0.0;

  const auto edge_result = inferVoxelsFromStableVoxelEdges(
    map, GridSpec{}, direct, VoxelizationOptions{}.excluded_labels, options);
  const auto split = splitVoxelsByGngConnectivity(direct, edge_result);

  ASSERT_EQ(edge_result.voxels.size(), 3U);
  ASSERT_EQ(edge_result.connected_endpoint_cells.size(), 2U);
  EXPECT_EQ(split.connected_voxels.size(), 2U);
  EXPECT_TRUE(split.isolated_voxels.empty());
}

TEST(TopologicalGridAssignment, KeepsLocallyRetainedHistoricalVoxelInMainOutput)
{
  using namespace fuzzrobo::topological_grid;
  LabeledGridVoxel retained{GridCell{3, 2, 1},
    ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT};
  retained.retained_by_local_structure = true;
  const auto split = splitVoxelsByGngConnectivity({retained}, EdgeVoxelizationResult{});

  ASSERT_EQ(split.connected_voxels.size(), 1U);
  EXPECT_TRUE(split.isolated_voxels.empty());
}

TEST(TopologicalGridAssignment, RejectsOnlyRelativeGngEdgeOutlierInAdaptiveMode)
{
  using namespace fuzzrobo::topological_grid;
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.011F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.021F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.501F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  map.edges = {0, 1, 1, 2, 2, 3};
  const std::vector<LabeledGridVoxel> direct{
    LabeledGridVoxel{GridCell{0, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{1, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{2, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{50, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
  };
  EdgeInferenceOptions options;
  options.maximum_edge_length = 0.0;

  const auto result = inferVoxelsFromStableVoxelEdges(
    map, GridSpec{}, direct, VoxelizationOptions{}.excluded_labels, options);

  EXPECT_EQ(result.voxel_edge_count, 2U);
  EXPECT_EQ(result.overlength_edge_count, 1U);
  EXPECT_EQ(result.connected_endpoint_cells.size(), 3U);
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

TEST(TopologicalGridAssignment, EmitsOnlyPointSupportedInferredEdgeCells)
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
  GridPointCounts point_counts;
  point_counts.emplace(GridCell{2, 0, 0}, 1U);
  EdgeInferenceOptions options;
  options.require_point_support_for_output = true;

  const auto result = inferVoxelsFromStableVoxelEdges(
    map, GridSpec{}, direct, VoxelizationOptions{}.excluded_labels, options, &point_counts);

  ASSERT_EQ(result.voxels.size(), 1U);
  EXPECT_EQ(result.voxels.front().cell, (GridCell{2, 0, 0}));
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

TEST(TopologicalGridAssignment, FillsValidatedGngTriangleArea)
{
  using namespace fuzzrobo::topological_grid;
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.041F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.001F, 0.041F, 0.001F, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT),
  };
  for (auto &node : map.nodes) {
    node.normal.z = 1.0F;
  }
  map.edges = {0, 1, 1, 2, 2, 0};
  const std::vector<LabeledGridVoxel> direct{
    LabeledGridVoxel{GridCell{0, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{4, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{0, 4, 0}, ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT},
  };
  TriangleInferenceOptions options;
  options.maximum_edge_length = 0.10;

  const auto result = inferVoxelsFromStableVoxelTriangles(
    map, GridSpec{}, direct, VoxelizationOptions{}.excluded_labels, options);

  EXPECT_EQ(result.candidate_triangle_count, 1U);
  EXPECT_EQ(result.accepted_triangle_count, 1U);
  const auto interior = std::find_if(
    result.voxels.begin(), result.voxels.end(),
    [](const LabeledGridVoxel &voxel) {return voxel.cell == GridCell{1, 1, 0};});
  ASSERT_NE(interior, result.voxels.end());
  EXPECT_EQ(interior->triangle_support_count, 1U);
  EXPECT_EQ(interior->label, ais_gng_msgs::msg::TopologicalMap::WALL);
}

TEST(TopologicalGridAssignment, RequiresClosedThreeEdgeCycleForTriangle)
{
  using namespace fuzzrobo::topological_grid;
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.041F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.001F, 0.041F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  map.edges = {0, 1, 1, 2};
  const std::vector<LabeledGridVoxel> direct{
    LabeledGridVoxel{GridCell{0, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{4, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{0, 4, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
  };

  const auto result = inferVoxelsFromStableVoxelTriangles(
    map, GridSpec{}, direct, VoxelizationOptions{}.excluded_labels);

  EXPECT_EQ(result.candidate_triangle_count, 0U);
  EXPECT_TRUE(result.voxels.empty());
}

TEST(TopologicalGridAssignment, EmitsOnlyPointSupportedTriangleCells)
{
  using namespace fuzzrobo::topological_grid;
  ais_gng_msgs::msg::TopologicalMap map;
  map.nodes = {
    makeNode(0.001F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.041F, 0.001F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
    makeNode(0.001F, 0.041F, 0.001F, ais_gng_msgs::msg::TopologicalMap::WALL),
  };
  for (auto &node : map.nodes) {
    node.normal.z = 1.0F;
  }
  map.edges = {0, 1, 1, 2, 2, 0};
  const std::vector<LabeledGridVoxel> direct{
    LabeledGridVoxel{GridCell{0, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{4, 0, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
    LabeledGridVoxel{GridCell{0, 4, 0}, ais_gng_msgs::msg::TopologicalMap::WALL},
  };
  GridPointCounts point_counts;
  point_counts.emplace(GridCell{1, 1, 0}, 1U);
  TriangleInferenceOptions options;
  options.maximum_edge_length = 0.10;
  options.require_point_support_for_output = true;

  const auto result = inferVoxelsFromStableVoxelTriangles(
    map, GridSpec{}, direct, VoxelizationOptions{}.excluded_labels, options, &point_counts);

  ASSERT_EQ(result.voxels.size(), 1U);
  EXPECT_EQ(result.voxels.front().cell, (GridCell{1, 1, 0}));
}

TEST(TopologicalGridAssignment, DirectVoxelWinsWhenMergingInferredVoxels)
{
  using namespace fuzzrobo::topological_grid;
  const LabeledGridVoxel direct{GridCell{1, 0, 0}, 2, 3, 4, 5};
  LabeledGridVoxel overlapping{GridCell{1, 0, 0}, 3};
  overlapping.edge_support_count = 2;
  overlapping.triangle_support_count = 3;
  LabeledGridVoxel inferred{GridCell{2, 0, 0}, 3};
  inferred.edge_support_count = 1;

  const auto merged = mergeDirectAndInferredVoxels({direct}, {overlapping, inferred});

  ASSERT_EQ(merged.size(), 2U);
  EXPECT_EQ(merged.front().label, 2);
  EXPECT_EQ(merged.front().edge_support_count, 2U);
  EXPECT_EQ(merged.front().triangle_support_count, 3U);
  EXPECT_EQ(merged.back().cell.x, 2);
}

TEST(TopologicalGridAssignment, SplitsIsolatedVoxelsFromGraspCandidates)
{
  using namespace fuzzrobo::topological_grid;
  const LabeledGridVoxel isolated{GridCell{0, 0, 0}, 3, 1, 1, 0};
  const LabeledGridVoxel connected{GridCell{1, 0, 0}, 3, 1, 1, 1};

  const auto split = splitVoxelsByIsolation({isolated, connected});

  ASSERT_EQ(split.connected_voxels.size(), 1U);
  EXPECT_EQ(split.connected_voxels.front().cell, connected.cell);
  ASSERT_EQ(split.isolated_voxels.size(), 1U);
  EXPECT_EQ(split.isolated_voxels.front().cell, isolated.cell);
}

TEST(TopologicalGridAssignment, AggregatesPointCountsOnMetricActivityGrid)
{
  using namespace fuzzrobo::topological_grid;
  GridPointCounts fine;
  fine.emplace(GridCell{0, 0, 0}, 2U);
  fine.emplace(GridCell{1, 0, 0}, 3U);
  fine.emplace(GridCell{4, 0, 0}, 7U);
  const GridSpec fine_spec{0.005, 0.0, 0.0, 0.0};
  const GridSpec activity_spec{0.02, 0.0, 0.0, 0.0};

  const auto aggregated = aggregatePointCounts(fine, fine_spec, activity_spec);

  ASSERT_EQ(aggregated.size(), 2U);
  EXPECT_EQ(aggregated.at(GridCell{0, 0, 0}), 5U);
  EXPECT_EQ(aggregated.at(GridCell{1, 0, 0}), 7U);
}

TEST(TopologicalGridAssignment, PointActivitySchedulerSlowsStaticInputContinuously)
{
  using namespace fuzzrobo::topological_grid;
  PointActivitySchedulerConfig config;
  config.minimum_update_interval = 1;
  config.maximum_update_interval = 5;
  config.top_fraction = 0.5;
  config.warmup_update_count = 1;
  PointActivityScheduler scheduler(config);
  const GridPointCounts points{{GridCell{0, 0, 0}, 3U}};

  EXPECT_TRUE(scheduler.update(points).should_process);
  for (std::size_t update = 1; update < 5; ++update) {
    const auto decision = scheduler.update(points);
    EXPECT_FALSE(decision.should_process);
    EXPECT_DOUBLE_EQ(decision.activity_score, 0.0);
    EXPECT_EQ(decision.desired_update_interval, 5U);
  }
  EXPECT_TRUE(scheduler.update(points).should_process);
}

TEST(TopologicalGridAssignment, PointActivitySchedulerImmediatelyProcessesNewRegion)
{
  using namespace fuzzrobo::topological_grid;
  PointActivitySchedulerConfig config;
  config.minimum_update_interval = 1;
  config.maximum_update_interval = 10;
  config.top_fraction = 0.5;
  config.warmup_update_count = 1;
  PointActivityScheduler scheduler(config);
  GridPointCounts points{{GridCell{0, 0, 0}, 3U}};
  ASSERT_TRUE(scheduler.update(points).should_process);

  points.emplace(GridCell{10, 0, 0}, 4U);
  const auto decision = scheduler.update(points);

  EXPECT_TRUE(decision.should_process);
  EXPECT_DOUBLE_EQ(decision.activity_score, 1.0);
  EXPECT_EQ(decision.desired_update_interval, 1U);
}

TEST(TopologicalGridAssignment, SuppressesPlanarUnknownComponentAtDifferentNodeDensities)
{
  using namespace fuzzrobo::topological_grid;
  const auto make_map = [](double scale) {
      ais_gng_msgs::msg::TopologicalMap map;
      for (int component = 0; component < 2; ++component) {
        for (int y = 0; y < 3; ++y) {
          for (int x = 0; x < 3; ++x) {
            auto node = makeNode(
              static_cast<float>((component * 6 + x) * scale),
              static_cast<float>(y * scale),
              static_cast<float>(component == 1 && x == 1 && y == 1 ? scale : 0.0),
              ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT);
            node.normal.z = 1.0F;
            map.nodes.push_back(node);
          }
        }
      }
      for (int component = 0; component < 2; ++component) {
        const int base = component * 9;
        for (int y = 0; y < 3; ++y) {
          for (int x = 0; x < 3; ++x) {
            const int index = base + y * 3 + x;
            if (x + 1 < 3) {
              map.edges.push_back(static_cast<std::uint16_t>(index));
              map.edges.push_back(static_cast<std::uint16_t>(index + 1));
            }
            if (y + 1 < 3) {
              map.edges.push_back(static_cast<std::uint16_t>(index));
              map.edges.push_back(static_cast<std::uint16_t>(index + 3));
            }
          }
        }
      }
      return map;
    };
  VoxelizationOptions options;
  options.require_input_points = false;
  options.unknown_shape_filter_enabled = true;
  options.shape_neighborhood_hops = 2;
  options.shape_minimum_neighbors = 3;
  options.shape_seed_expansion_scale = 2.5;

  const auto dense = voxelizeNodes(make_map(0.01), GridSpec{0.005}, options);
  const auto sparse = voxelizeNodes(make_map(0.10), GridSpec{0.05}, options);

  EXPECT_EQ(dense.shape_candidate_node_count, 18U);
  EXPECT_GT(dense.shape_seed_node_count, 0U);
  EXPECT_GT(dense.shape_rejected_node_count, 0U);
  EXPECT_GT(dense.shape_retained_node_count, 0U);
  EXPECT_EQ(dense.shape_seed_node_count, sparse.shape_seed_node_count);
  EXPECT_EQ(dense.shape_retained_node_count, sparse.shape_retained_node_count);
  EXPECT_EQ(dense.shape_rejected_node_count, sparse.shape_rejected_node_count);
}

}  // namespace
