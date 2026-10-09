#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include <Eigen/Geometry>

#include "core/indexing/reachability_voxel_accumulator.hpp"

namespace robot_sim::indexing
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

TEST(reachability_voxel_accumulator_test, shared_cells_classify_once_and_preserve_points_outside_roi)
{
  robot_sim::analysis::VoxelIdCodec codec(0.1);
  codec.setIndexingParams(42, 21, 0, 1000000L);
  reachability_bounds bounds;
  bounds.enable_filter = true;
  bounds.min_corner.setConstant(-0.5);
  bounds.max_corner.setConstant(0.5);
  for (std::size_t max_dense : {0U, 8000000U}) {
    reachability_voxel_accumulator accumulator(codec, bounds, max_dense, true);
    accumulator.begin_frame(5);
    std::size_t num_checks = 0;
    const auto classify = [&](long) {++num_checks; return true;};
    accumulator.add_shared_point({0.01, 0.01, 0.01}, 0, [] {return false;}, classify);
    accumulator.add_shared_point({0.02, 0.02, 0.02}, 1, [] {return false;}, classify);
    accumulator.add_shared_point({1.01, 0.01, 0.01}, 2, [] {return true;}, classify);
    accumulator.add_shared_point({2.01, 0.01, 0.01}, 3, [] {return false;}, classify);
    accumulator.add_shared_point({NAN, 0, 0}, 4, [] {return true;}, classify);
    const auto frame = accumulator.point_membership();
    ASSERT_EQ(frame->cells.size(), 1U);
    EXPECT_EQ(num_checks, 2U);
    EXPECT_EQ(accumulator.finish_voxel_ids().size(), 1U);
    EXPECT_EQ(frame->point_cells[0], frame->point_cells[1]);
    EXPECT_TRUE(frame->is_self_point(2));
    EXPECT_FALSE(frame->is_self_point(3));
    EXPECT_EQ(frame->point_cells[4], voxel_idx::roi_point_membership::no_cell);
    // 前フレーム保持中のバッファ切替とラベル初期化
    accumulator.begin_frame(1);
    accumulator.add_shared_point({0.01, 0.01, 0.01}, 0, [] {return false;}, [](long) {return false;});
    EXPECT_TRUE(frame->is_self_point(0));
    EXPECT_FALSE(accumulator.point_membership()->is_self_point(0));
    EXPECT_EQ(accumulator.finish_voxel_ids().size(), 1U);
  }
}

TEST(reachability_voxel_accumulator_test, self_cells_reuse_dense_occupancy_and_update_without_point_lookup)
{
  robot_sim::analysis::VoxelIdCodec codec(0.1);
  codec.setIndexingParams(42, 21, 0, 1000000L);
  reachability_bounds bounds;
  bounds.enable_filter = true;
  bounds.min_corner.setConstant(-0.5); bounds.max_corner.setConstant(0.5);
  reachability_voxel_accumulator accumulator(codec, bounds, 8000000U, true);
  accumulator.set_self_cells(std::vector<long>{codec.toFlatId({0, 0, 0})});
  const auto no_lookup = [](long) {ADD_FAILURE() << "ROIセルの重複照合"; return false;};
  for (int iter = 0; iter < 3; ++iter) {
    accumulator.begin_frame(2);
    accumulator.add_shared_point({.01, .01, .01}, 0, [] {return false;}, no_lookup);
    accumulator.add_shared_point({-.11, .01, .01}, 1, [] {return false;}, no_lookup);
    const auto &ids = accumulator.finish_voxel_ids();
    ASSERT_EQ(ids.size(), 2U);
    EXPECT_TRUE(std::is_sorted(ids.begin(), ids.end()));
    EXPECT_TRUE(accumulator.point_membership()->is_self_point(0));
    EXPECT_FALSE(accumulator.point_membership()->is_self_point(1));
  }
  const auto previous = accumulator.point_membership();
  accumulator.set_self_cells(std::vector<long>{});
  accumulator.begin_frame(1);
  accumulator.add_shared_point({.01, .01, .01}, 0, [] {return false;}, no_lookup);
  EXPECT_FALSE(accumulator.point_membership()->is_self_point(0));
  EXPECT_TRUE(previous->is_self_point(0));
}

TEST(reachability_voxel_accumulator_test, indexed_roi_reuses_registration_and_labels_outside_roi)
{
  robot_sim::analysis::VoxelIdCodec codec(0.1);
  codec.setIndexingParams(42, 21, 0, 1000000L);
  reachability_bounds bounds;
  bounds.enable_filter = true;
  bounds.min_corner.setConstant(-.5); bounds.max_corner.setConstant(.5);
  const std::vector<Eigen::Vector3f> points{
    {5, 0, 0}, {.01f, 0, 0}, {NAN, 0, 0}, {1.01f, 0, 0}, {.02f, 0, 0}, {-.11f, 0, 0}};
  voxel_idx::world_point_bucket_index world(.2);
  world.begin_frame(points.size());
  for (std::uint32_t idx = 0; idx < points.size(); ++idx) {world.add_point(points[idx], idx);}
  for (std::size_t max_dense : {0U, 8000000U}) {
    reachability_voxel_accumulator direct(codec, bounds, max_dense, true);
    reachability_voxel_accumulator indexed(codec, bounds, max_dense, true);
    const std::vector<long> mask{codec.toFlatId({0, 0, 0}), codec.toFlatId({10, 0, 0})};
    direct.set_self_cells(mask); indexed.set_self_cells(mask);
    direct.begin_frame(points.size()); indexed.begin_frame(points.size());
    std::size_t num_registered = 0, num_self_checks = 0;
    const auto add = [&](auto &accumulator, const Eigen::Vector3f &value, std::uint32_t idx) {
      const Eigen::Vector3d point = value.cast<double>();
      accumulator.add_shared_point(point, idx, [&] {return point.x() >= 1. && point.x() < 1.1;},
        [&](long id) {
          ++num_self_checks;
          return std::find(mask.begin(), mask.end(), id) != mask.end();
        });
    };
    for (std::uint32_t idx = 0; idx < points.size(); ++idx) {add(direct, points[idx], idx);}
    num_self_checks = 0;
    world.query_aabb_with_source({-.5, -.5, -.5}, {1.1, .5, .5},
      [&](const Eigen::Vector3f &point, std::uint32_t idx) {++num_registered; add(indexed, point, idx);});
    EXPECT_EQ(num_registered, 4U);
    EXPECT_EQ(num_self_checks, max_dense ? 1U : 3U);
    EXPECT_EQ(direct.finish_voxel_ids(), indexed.finish_voxel_ids());
    EXPECT_EQ(direct.point_membership()->point_cells, indexed.point_membership()->point_cells);
    EXPECT_TRUE(indexed.point_membership()->is_self_point(3));
    EXPECT_FALSE(indexed.point_membership()->is_self_point(0));
  }
}

}  // 無名namespace終端
}  // robot_sim::indexing namespace終端
