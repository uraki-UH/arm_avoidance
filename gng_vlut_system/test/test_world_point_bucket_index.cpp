#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include "nodes/bridge/world_point_bucket_index.hpp"

namespace robot_sim::bridge
{
namespace
{

TEST(world_point_bucket_index_test, rejects_invalid_bucket_size)
{
  EXPECT_THROW(world_point_bucket_index(0.0), std::invalid_argument);
  EXPECT_THROW(world_point_bucket_index(-0.1), std::invalid_argument);
}

TEST(world_point_bucket_index_test, indexes_negative_and_positive_coordinates)
{
  world_point_bucket_index index(0.2);
  index.begin_frame(4U);
  index.add_point(Eigen::Vector3f(-0.21F, 0.0F, 0.0F));
  index.add_point(Eigen::Vector3f(-0.01F, 0.0F, 0.0F));
  index.add_point(Eigen::Vector3f(0.01F, 0.0F, 0.0F));
  index.add_point(Eigen::Vector3f(0.21F, 0.0F, 0.0F));

  EXPECT_EQ(index.point_num(), 4U);
  EXPECT_EQ(index.bucket_num(), 4U);
}

TEST(world_point_bucket_index_test, returns_only_points_inside_query_bounds)
{
  world_point_bucket_index index(0.2);
  index.begin_frame(5U);
  index.add_point(Eigen::Vector3f(-0.11F, 0.0F, 0.0F));
  index.add_point(Eigen::Vector3f(-0.09F, 0.0F, 0.0F));
  index.add_point(Eigen::Vector3f(0.0F, 0.0F, 0.0F));
  index.add_point(Eigen::Vector3f(0.09F, 0.0F, 0.0F));
  index.add_point(Eigen::Vector3f(0.11F, 0.0F, 0.0F));

  std::vector<Eigen::Vector3f> selected_points;
  const auto stats = index.query_aabb(
    Eigen::Vector3d(-0.1, -0.1, -0.1),
    Eigen::Vector3d(0.1, 0.1, 0.1),
    [&selected_points](const Eigen::Vector3f &point) {
      selected_points.push_back(point);
    });

  EXPECT_EQ(selected_points.size(), 3U);
  EXPECT_EQ(stats.accepted_point_num, 3U);
  EXPECT_GE(stats.candidate_point_num, stats.accepted_point_num);
}

TEST(world_point_bucket_index_test, clears_points_between_frames)
{
  world_point_bucket_index index(0.2);
  index.begin_frame(1U);
  index.add_point(Eigen::Vector3f(0.0F, 0.0F, 0.0F));
  ASSERT_EQ(index.point_num(), 1U);

  index.begin_frame(0U);
  std::size_t selected_point_num = 0U;
  index.query_aabb(
    Eigen::Vector3d::Constant(-1.0),
    Eigen::Vector3d::Constant(1.0),
    [&selected_point_num](const Eigen::Vector3f &) {
      ++selected_point_num;
    });

  EXPECT_EQ(index.point_num(), 0U);
  EXPECT_EQ(index.bucket_num(), 0U);
  EXPECT_EQ(selected_point_num, 0U);
}

}  // 無名namespace終端
}  // robot_sim::bridge namespace終端
