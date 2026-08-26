#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include <Eigen/Geometry>

#include "nodes/bridge/reachability_voxel_accumulator.hpp"

namespace robot_sim::bridge
{
namespace
{

TEST(reachability_bounds_test, includes_boundary_faces)
{
  reachability_bounds bounds;
  bounds.enable_filter = true;
  bounds.min_corner = Eigen::Vector3d(-1.0, -2.0, -3.0);
  bounds.max_corner = Eigen::Vector3d(1.0, 2.0, 3.0);

  EXPECT_TRUE(bounds.contains(bounds.min_corner));
  EXPECT_TRUE(bounds.contains(bounds.max_corner));
  EXPECT_FALSE(bounds.contains(Eigen::Vector3d(1.01, 0.0, 0.0)));
}

TEST(reachability_bounds_test, rejects_reversed_range)
{
  reachability_bounds bounds;
  bounds.enable_filter = true;
  bounds.min_corner = Eigen::Vector3d(1.0, 0.0, 0.0);
  bounds.max_corner = Eigen::Vector3d(0.0, 1.0, 1.0);

  EXPECT_THROW(bounds.validate(), std::invalid_argument);
}

TEST(reachability_bounds_test, expands_each_axis_by_margin)
{
  reachability_bounds bounds;
  bounds.enable_filter = true;
  bounds.min_corner = Eigen::Vector3d(-1.0, -1.0, -1.0);
  bounds.max_corner = Eigen::Vector3d(1.0, 1.0, 1.0);
  bounds.margin = Eigen::Vector3d(0.5, 0.2, 0.1);

  EXPECT_TRUE(bounds.contains(Eigen::Vector3d(1.5, 1.2, 1.1)));
  EXPECT_FALSE(bounds.contains(Eigen::Vector3d(1.51, 0.0, 0.0)));
  EXPECT_FALSE(bounds.contains(Eigen::Vector3d(0.0, -1.21, 0.0)));
}

TEST(reachability_bounds_test, rejects_negative_margin)
{
  reachability_bounds bounds;
  bounds.enable_filter = true;
  bounds.min_corner = Eigen::Vector3d::Constant(-1.0);
  bounds.max_corner = Eigen::Vector3d::Constant(1.0);
  bounds.margin = Eigen::Vector3d(0.1, -0.1, 0.1);

  EXPECT_THROW(bounds.validate(), std::invalid_argument);
}

TEST(reachability_voxel_accumulator_test, aggregates_only_transformed_reachable_points)
{
  robot_sim::analysis::VoxelIdCodec codec(0.1);
  codec.setIndexingParams(42, 21, 0, 1000000L);

  reachability_bounds bounds;
  bounds.enable_filter = true;
  bounds.min_corner = Eigen::Vector3d(0.0, -0.5, -0.5);
  bounds.max_corner = Eigen::Vector3d(1.0, 0.5, 0.5);

  Eigen::Isometry3d source_to_target = Eigen::Isometry3d::Identity();
  source_to_target.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);

  reachability_voxel_accumulator accumulator(codec, bounds, 8000000U);
  accumulator.begin_frame(4);
  accumulator.add_point(Eigen::Vector3d(0.01, 0.01, 0.01), source_to_target);
  accumulator.add_point(Eigen::Vector3d(0.02, 0.02, 0.02), source_to_target);
  accumulator.add_point(Eigen::Vector3d(0.60, 0.0, 0.0), source_to_target);
  accumulator.add_point(
    Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
    source_to_target);

  const auto stats = accumulator.stats();
  const auto voxel_ids = accumulator.finish_voxel_ids();

  ASSERT_EQ(voxel_ids.size(), 1U);
  EXPECT_EQ(stats.input_point_count, 4U);
  EXPECT_EQ(stats.accepted_point_count, 2U);
  EXPECT_EQ(stats.outside_point_count, 1U);
  EXPECT_EQ(stats.nonfinite_point_count, 1U);
  EXPECT_TRUE(accumulator.uses_dense_bitmap());
}

TEST(reachability_voxel_accumulator_test, aggregates_target_frame_points_without_transform)
{
  robot_sim::analysis::VoxelIdCodec codec(0.1);
  codec.setIndexingParams(42, 21, 0, 1000000L);

  reachability_bounds bounds;
  bounds.enable_filter = true;
  bounds.min_corner = Eigen::Vector3d(0.0, -0.5, -0.5);
  bounds.max_corner = Eigen::Vector3d(1.0, 0.5, 0.5);

  reachability_voxel_accumulator accumulator(codec, bounds, 8000000U);
  accumulator.begin_frame(4);
  accumulator.add_point_in_target_frame(Eigen::Vector3d(0.51, 0.01, 0.01));
  accumulator.add_point_in_target_frame(Eigen::Vector3d(0.52, 0.02, 0.02));
  accumulator.add_point_in_target_frame(Eigen::Vector3d(1.01, 0.0, 0.0));
  accumulator.add_point_in_target_frame(
    Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));

  const auto stats = accumulator.stats();
  const auto voxel_ids = accumulator.finish_voxel_ids();

  ASSERT_EQ(voxel_ids.size(), 1U);
  EXPECT_EQ(stats.input_point_count, 4U);
  EXPECT_EQ(stats.accepted_point_count, 2U);
  EXPECT_EQ(stats.outside_point_count, 1U);
  EXPECT_EQ(stats.nonfinite_point_count, 1U);
}

TEST(reachability_voxel_accumulator_test, reuses_dense_bitmap_between_frames)
{
  robot_sim::analysis::VoxelIdCodec codec(0.1);
  codec.setIndexingParams(42, 21, 0, 1000000L);

  reachability_bounds bounds;
  bounds.enable_filter = true;
  bounds.min_corner = Eigen::Vector3d::Constant(-0.5);
  bounds.max_corner = Eigen::Vector3d::Constant(0.5);

  reachability_voxel_accumulator accumulator(codec, bounds, 8000000U);
  accumulator.begin_frame(2);
  accumulator.add_point(Eigen::Vector3d(0.01, 0.01, 0.01), Eigen::Isometry3d::Identity());
  accumulator.add_point(Eigen::Vector3d(0.02, 0.02, 0.02), Eigen::Isometry3d::Identity());
  EXPECT_EQ(accumulator.finish_voxel_ids().size(), 1U);

  accumulator.begin_frame(1);
  accumulator.add_point(Eigen::Vector3d(0.01, 0.01, 0.01), Eigen::Isometry3d::Identity());
  EXPECT_EQ(accumulator.finish_voxel_ids().size(), 1U);
  EXPECT_EQ(accumulator.stats().input_point_count, 1U);
}

TEST(reachability_voxel_accumulator_test, falls_back_to_reusable_hash)
{
  robot_sim::analysis::VoxelIdCodec codec(0.1);
  codec.setIndexingParams(42, 21, 0, 1000000L);

  reachability_bounds bounds;
  bounds.enable_filter = true;
  bounds.min_corner = Eigen::Vector3d::Constant(-0.5);
  bounds.max_corner = Eigen::Vector3d::Constant(0.5);

  reachability_voxel_accumulator accumulator(codec, bounds, 0U);
  accumulator.begin_frame(2);
  accumulator.add_point(Eigen::Vector3d(0.01, 0.01, 0.01), Eigen::Isometry3d::Identity());
  accumulator.add_point(Eigen::Vector3d(0.02, 0.02, 0.02), Eigen::Isometry3d::Identity());

  EXPECT_FALSE(accumulator.uses_dense_bitmap());
  EXPECT_EQ(accumulator.finish_voxel_ids().size(), 1U);
}

}  // 無名namespace終端
}  // robot_sim::bridge namespace終端
