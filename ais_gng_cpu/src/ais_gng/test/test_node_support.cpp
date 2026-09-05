#include <ais_gng/node_support.hpp>
#include "../../gng_cpu/src/utils/node.hpp"
#include <gtest/gtest.h>

using namespace fuzzrobo::node_support;

TEST(node_support, mixture_preserves_between_mean_variance)
{
  node_moments moments;
  moments.add_input(Vec3f(0, 0, 0), 0.5, 1);
  moments.add_input(Vec3f(2, 0, 0), 0.5, 1);
  EXPECT_DOUBLE_EQ(moments.mean[0], 1);
  EXPECT_DOUBLE_EQ(moments.covariance[0], 1);
}

TEST(node_support, weighted_cumulative_and_zero_weight)
{
  node_moments moments;
  moments.add_input(Vec3f(0, 0, 0), 0, 1);
  moments.add_input(Vec3f(3, 0, 0), 0, 0.5);
  EXPECT_DOUBLE_EQ(moments.count, 1.5);
  EXPECT_NEAR(moments.mean[0], 1, 1e-12);
  EXPECT_NEAR(moments.covariance[0], 2, 1e-12);
  moments.add_input(Vec3f(100, 0, 0), 0.5, 0);
  EXPECT_DOUBLE_EQ(moments.count, 1.5);
}

TEST(node_support, reuse_resets_both_statistics)
{
  Node node;
  node.winner_stats.add_residual(Vec3f(1, 2, 3));
  node.support_stats.add_input(Vec3f(1, 2, 3), 0.01, 1);
  node.init(1, 0.08f, 0.008f);
  EXPECT_DOUBLE_EQ(node.winner_stats.count, 0);
  EXPECT_DOUBLE_EQ(node.support_stats.count, 0);
  EXPECT_DOUBLE_EQ(node.support_stats.mean[0], 0);
}

TEST(node_support, fixed_scale_and_degenerate_axes)
{
  gng_node_statistics stats;
  EXPECT_FALSE(make_ellipsoid(stats, {}).has_support);
  stats.support_weight_sum = 1;
  options config;
  const auto shape = make_ellipsoid(stats, config);
  EXPECT_TRUE(shape.has_support);
  EXPECT_NEAR(config.base_scale * shape.axis_std.maxCoeff(), 0.004, 1e-12);
  config.base_scale = 0.5;
  EXPECT_TRUE(shape.moment.isApprox(make_ellipsoid(stats, config).moment));
  config.second_weight = -1;
  EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(node_support, api_statistics_without_event_capture)
{
  ASSERT_TRUE(gng_setParameter("node.covariance_enabled", 0, 1));
  ASSERT_TRUE(gng_setParameter("node.enable_support", 0, 1));
  gng_setParameter("node.learning_num", 0, 100);
  gng_setParameter("node.num_max", 0, 100);
  gng_setParameter("input.local_coordinates", 0, 1);
  ASSERT_EQ(gng_init(), SUCCESS);
  std::vector<Vec3> points;
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {points.push_back({x * 0.08f, y * 0.08f, 1});}
  }
  LiDAR_Config lidar; lidar.point_step = sizeof(Vec3);
  gng_setTrainingEventCapture(0);
  gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size(), &lidar);
  gng_exec();
  uint32_t event_num = 99;
  EXPECT_EQ(gng_getTrainingEvents(&event_num), nullptr);
  EXPECT_EQ(event_num, 0U);
  const auto map = gng_getTopologicalMap();
  double count = 0, weight = 0;
  for (uint32_t idx = 0; idx < map.node_num; ++idx) {
    const auto stats = gng_get_node_statistics(map.nodes[idx].id);
    count += stats.winner_point_count;
    weight += stats.support_weight_sum;
  }
  EXPECT_GT(count, 0);
  EXPECT_GT(weight, count);
  EXPECT_DOUBLE_EQ(gng_get_node_statistics(65535).winner_point_count, 0);
  // 記録を有効化した別フレームとの勝者数・重み照合。
  ASSERT_EQ(gng_init(), SUCCESS);
  gng_setTrainingEventMaxWinnerRank(2);
  gng_setTrainingEventCapture(1);
  gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size(), &lidar);
  gng_exec();
  const auto *events = gng_getTrainingEvents(&event_num);
  const auto recorded_map = gng_getTopologicalMap();
  for (uint32_t idx = 0; idx < recorded_map.node_num; ++idx) {
    const auto &node = recorded_map.nodes[idx];
    double first_num = 0, expected_weight = 0;
    for (uint32_t event_idx = 0; event_idx < event_num; ++event_idx) {
      const auto &event = events[event_idx];
      if (event.winner_node_id != node.id || event.winner_node_frame != node.frame) {continue;}
      first_num += event.winner_rank == 1;
      expected_weight += event.winner_rank == 1 ? 1 : 0.5;
    }
    const auto stats = gng_get_node_statistics(node.id);
    EXPECT_DOUBLE_EQ(stats.winner_point_count, first_num);
    EXPECT_DOUBLE_EQ(stats.support_weight_sum, expected_weight);
  }
  gng_setTrainingEventCapture(0);
  ASSERT_TRUE(gng_setParameter("node.covariance_enabled", 0, 0));
  for (uint32_t idx = 0; idx < recorded_map.node_num; ++idx) {
    const auto stats = gng_get_node_statistics(recorded_map.nodes[idx].id);
    EXPECT_DOUBLE_EQ(stats.winner_point_count, 0);
  }
  ASSERT_TRUE(gng_setParameter("node.enable_support", 0, 0));
}
