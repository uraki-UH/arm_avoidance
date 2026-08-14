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

TEST(TopologicalGridAssignment, DefaultsToTwentyMillimeterCells)
{
  const fuzzrobo::topological_grid::GridSpec spec;
  EXPECT_DOUBLE_EQ(spec.cell_size, 0.02);
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

  const std::unordered_set<std::uint8_t> excluded{
    ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN,
    ais_gng_msgs::msg::TopologicalMap::HUMAN,
    ais_gng_msgs::msg::TopologicalMap::CAR,
  };
  const auto result = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{0.02, 0.0, 0.0, 0.0}, excluded);

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

  const auto result = fuzzrobo::topological_grid::voxelizeNodes(
    map, fuzzrobo::topological_grid::GridSpec{0.02, 0.0, 0.0, 0.0}, {});

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

TEST(TopologicalGridAssignment, PublishesOnlyTemporallyStableVoxels)
{
  using fuzzrobo::topological_grid::GridCell;
  using fuzzrobo::topological_grid::LabeledGridVoxel;
  using fuzzrobo::topological_grid::TemporalVoxelFilter;

  TemporalVoxelFilter filter(3, 1);
  const LabeledGridVoxel object{GridCell{1, 2, 3}, 3, 2};

  EXPECT_TRUE(filter.update({object}).empty());
  EXPECT_TRUE(filter.update({object}).empty());
  ASSERT_EQ(filter.update({object}).size(), 1U);

  EXPECT_TRUE(filter.update({}).empty());
  ASSERT_EQ(filter.update({object}).size(), 1U);
}

TEST(TopologicalGridAssignment, ResetsStabilityWhenLabelChanges)
{
  using fuzzrobo::topological_grid::GridCell;
  using fuzzrobo::topological_grid::LabeledGridVoxel;
  using fuzzrobo::topological_grid::TemporalVoxelFilter;

  TemporalVoxelFilter filter(2, 0);
  const GridCell cell{1, 2, 3};
  EXPECT_TRUE(filter.update({LabeledGridVoxel{cell, 2, 1}}).empty());
  EXPECT_TRUE(filter.update({LabeledGridVoxel{cell, 3, 1}}).empty());
  ASSERT_EQ(filter.update({LabeledGridVoxel{cell, 3, 1}}).size(), 1U);
}

}  // namespace
