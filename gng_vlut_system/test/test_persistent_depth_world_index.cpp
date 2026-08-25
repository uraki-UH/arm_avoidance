#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>

#include "nodes/bridge/persistent_depth_world_index.hpp"

namespace robot_sim::bridge
{
namespace
{

depth_camera_intrinsics make_intrinsics()
{
  depth_camera_intrinsics intrinsics;
  intrinsics.width = 2U;
  intrinsics.height = 1U;
  intrinsics.fx = 1.0F;
  intrinsics.fy = 1.0F;
  intrinsics.cx = 0.0F;
  intrinsics.cy = 0.0F;
  return intrinsics;
}

std::vector<Eigen::Vector3f> query_all(const persistent_depth_world_index &index)
{
  std::vector<Eigen::Vector3f> points;
  index.query_aabb(
    Eigen::Vector3d::Constant(-10.0), Eigen::Vector3d::Constant(10.0),
    [&points](const Eigen::Vector3f &point) {
      points.push_back(point);
    });
  return points;
}

TEST(persistent_depth_world_index_test, rejects_invalid_configuration)
{
  persistent_depth_world_index_config config;
  config.bucket_size = 0.0;
  EXPECT_THROW((void)persistent_depth_world_index{config}, std::invalid_argument);

  config.bucket_size = 0.2;
  config.free_confirmation_num = 0U;
  EXPECT_THROW((void)persistent_depth_world_index{config}, std::invalid_argument);
}

TEST(persistent_depth_world_index_test, inserts_and_queries_depth_pixels)
{
  persistent_depth_world_index_config config;
  config.bucket_size = 0.2;
  persistent_depth_world_index index(config);
  EXPECT_TRUE(index.set_camera_intrinsics(make_intrinsics()));

  const auto stats = index.update(
    std::vector<std::uint16_t>{1000U, 2000U}, Eigen::Isometry3f::Identity());
  const std::vector<Eigen::Vector3f> points = query_all(index);

  EXPECT_TRUE(stats.is_rebuild);
  EXPECT_EQ(stats.inserted_point_num, 2U);
  EXPECT_EQ(index.point_num(), 2U);
  ASSERT_EQ(points.size(), 2U);
  EXPECT_TRUE(points[0].isApprox(Eigen::Vector3f(0.0F, 0.0F, 1.0F)));
  EXPECT_TRUE(points[1].isApprox(Eigen::Vector3f(2.0F, 0.0F, 2.0F)));
}

TEST(persistent_depth_world_index_test, updates_point_when_depth_change_reaches_threshold)
{
  persistent_depth_world_index_config config;
  config.bucket_size = 0.2;
  config.depth_update_mm_th = 10U;
  persistent_depth_world_index index(config);
  index.set_camera_intrinsics(make_intrinsics());
  index.update(std::vector<std::uint16_t>{1000U, 0U}, Eigen::Isometry3f::Identity());

  const auto unchanged_stats = index.update(
    std::vector<std::uint16_t>{1005U, 0U}, Eigen::Isometry3f::Identity());
  ASSERT_EQ(query_all(index).size(), 1U);
  EXPECT_EQ(unchanged_stats.updated_point_num, 0U);
  EXPECT_EQ(unchanged_stats.unchanged_point_num, 1U);
  EXPECT_NEAR(query_all(index).front().z(), 1.0F, 1.0e-6F);

  const auto updated_stats = index.update(
    std::vector<std::uint16_t>{1015U, 0U}, Eigen::Isometry3f::Identity());
  ASSERT_EQ(query_all(index).size(), 1U);
  EXPECT_EQ(updated_stats.updated_point_num, 1U);
  EXPECT_NEAR(query_all(index).front().z(), 1.015F, 1.0e-6F);
}

TEST(persistent_depth_world_index_test, removes_only_after_free_confirmation)
{
  persistent_depth_world_index_config config;
  config.bucket_size = 0.2;
  config.free_confirmation_num = 3U;
  persistent_depth_world_index index(config);
  index.set_camera_intrinsics(make_intrinsics());
  index.update(std::vector<std::uint16_t>{1000U, 0U}, Eigen::Isometry3f::Identity());

  const auto first_free_stats = index.update(
    std::vector<std::uint16_t>{0U, 0U}, Eigen::Isometry3f::Identity());
  const auto second_free_stats = index.update(
    std::vector<std::uint16_t>{0U, 0U}, Eigen::Isometry3f::Identity());
  const auto third_free_stats = index.update(
    std::vector<std::uint16_t>{0U, 0U}, Eigen::Isometry3f::Identity());

  EXPECT_EQ(first_free_stats.deferred_free_point_num, 1U);
  EXPECT_EQ(second_free_stats.deferred_free_point_num, 1U);
  EXPECT_EQ(third_free_stats.removed_point_num, 1U);
  EXPECT_EQ(index.point_num(), 0U);
}

TEST(persistent_depth_world_index_test, rebuilds_when_camera_transform_changes)
{
  persistent_depth_world_index_config config;
  config.bucket_size = 0.2;
  persistent_depth_world_index index(config);
  index.set_camera_intrinsics(make_intrinsics());
  index.update(std::vector<std::uint16_t>{1000U, 0U}, Eigen::Isometry3f::Identity());

  Eigen::Isometry3f camera_to_world = Eigen::Isometry3f::Identity();
  camera_to_world.translation().x() = 1.0F;
  const auto stats = index.update(
    std::vector<std::uint16_t>{1000U, 0U}, camera_to_world);

  ASSERT_TRUE(stats.is_rebuild);
  ASSERT_EQ(index.point_num(), 1U);
  ASSERT_EQ(query_all(index).size(), 1U);
  EXPECT_TRUE(query_all(index).front().isApprox(Eigen::Vector3f(1.0F, 0.0F, 1.0F)));
}

}  // 無名namespace終端
}  // robot_sim::bridge namespace終端
