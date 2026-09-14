#include <gtest/gtest.h>

#include "core/common/evaluation_metric_serialization.hpp"
#include "core/planning/topological_map_avoidance_helpers.hpp"

namespace {

TEST(candidate_metric_availability, provisional_metrics_remain_invalid)
{
  using namespace robot_sim::planning::topological_map_avoidance;
  auto gng = std::make_shared<GNGType>(2, 3, nullptr);
  auto &node = gng->nodeAt(0);
  node.id = 0;
  node.weight_angle = Eigen::VectorXf::Constant(2, 0.2F);
  node.weight_coord = Eigen::Vector3f(0.1F, 0.2F, 0.3F);
  node.status.joint_limit_score = 0.8F;
  node.status.manip_info.valid = true;
  node.status.manip_info.manipulability = 0.4;

  // 候補選択・姿勢・経路を維持したまま、暫定指標だけを未計算として配信
  const auto metrics = buildGraspCandidateMetricArray(
    rclcpp::Time(1, 0), "world", "test_robot", "base", 0, 0,
    {0}, {{0, {0}}}, gng, nullptr, {"joint_a", "joint_b"});
  ASSERT_EQ(metrics.candidates.size(), 1U);
  const auto &candidate = metrics.candidates.front();
  EXPECT_TRUE(candidate.selected);
  EXPECT_TRUE(candidate.feasible);
  EXPECT_EQ(candidate.goal_node_id, 0);
  EXPECT_EQ(candidate.path_node_ids, std::vector<int32_t>({0}));
  ASSERT_EQ(candidate.final_joint_state.position.size(), 2U);
  EXPECT_NEAR(candidate.final_joint_state.position[0], 0.2, 1e-6);
  EXPECT_NEAR(candidate.end_effector_pose.position.x, 0.1, 1e-6);
  EXPECT_FLOAT_EQ(candidate.position_manipulability, 0.4F);
  EXPECT_TRUE(std::isnan(candidate.joint_limit_margin_min));
  EXPECT_TRUE(std::isnan(candidate.joint_limit_margin_mean));
  EXPECT_TRUE(std::isnan(candidate.estimated_energy));
  EXPECT_TRUE(std::isnan(candidate.estimated_duration));

  const auto serialized = robot_sim::common::buildCandidateEvaluationMetrics(
    rclcpp::Time(1, 0), "test", "candidate", "/test/metrics", metrics);
  for (const auto *name : {"joint_limit_margin_min", "joint_limit_margin_mean",
      "estimated_energy", "estimated_duration"}) {
    const auto it = std::find(serialized.sample_metric_ids.begin(),
      serialized.sample_metric_ids.end(), name);
    ASSERT_NE(it, serialized.sample_metric_ids.end());
    const auto idx = std::distance(serialized.sample_metric_ids.begin(), it);
    EXPECT_FALSE(serialized.sample_metric_valid[idx]);
    EXPECT_TRUE(std::isnan(serialized.sample_metric_scalar_values[idx]));
  }
}

}  // 無名名前空間
