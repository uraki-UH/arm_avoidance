#include <gtest/gtest.h>

#include "core/common/evaluation_metric_serialization.hpp"
#include "core/planning/topological_map_avoidance_helpers.hpp"
#include "core/planning/joint_linf_cost.hpp"

namespace {

struct planning_graph {
  using node_type = GNG::NeuronNode<Eigen::VectorXf, Eigen::Vector3f>;
  std::vector<node_type> nodes{5};
  std::vector<std::vector<int>> neighbors{{1, 2}, {0, 3, 4}, {0, 3}, {1, 2}, {1}};

  planning_graph() {
    const float angles[] = {0.0F, 0.4F, -0.25F, 1.0F, 0.4F};
    for (int idx = 0; idx < 5; ++idx) {
      auto &node = nodes[idx];
      node.id = idx;
      node.weight_angle = Eigen::VectorXf::Constant(1, angles[idx]);
      node.status.active = true;
      node.status.self_collision_free = true;
      node.status.is_colliding = idx == 4;
      node.status.is_danger = false;
    }
  }

  std::size_t getMaxNodeNum() const { return nodes.size(); }
  const node_type &nodeAt(int idx) const { return nodes[idx]; }
  const std::vector<int> &getNeighborsAngle(int idx) const { return neighbors[idx]; }
  bool isEdgeActive(int, int, int) const { return true; }
  template <typename callback_type>
  void forEachActiveValid(callback_type callback) const {
    for (const auto &node : nodes) {
      if (node.id >= 0 && node.status.active && node.status.self_collision_free) {
        callback(node.id, node);
      }
    }
  }
};

TEST(candidate_path_planning, safety_constraints_without_repulsion)
{
  planning_graph graph;
  planning::GngDijkstraPlanner<Eigen::VectorXf, Eigen::Vector3f, planning_graph> planner;
  planner.setCostEvaluator(
      std::make_shared<planning::JointLInfCost<Eigen::VectorXf, Eigen::Vector3f>>());
  planner.setAvoidCollisions(true);
  planner.setAvoidDanger(true);
  planner.setStrictGoalCollisionCheck(true);

  // 既存の実行系では、衝突ノード隣接による迂回コストの保持
  EXPECT_EQ(planner.planToAnyNode(0, {3}, graph).second, std::vector<int>({0, 2, 3}));
  planner.set_enable_safety_penalty(false);
  EXPECT_EQ(planner.planToAnyNode(0, {3}, graph).second, std::vector<int>({0, 1, 3}));
  EXPECT_EQ(planner.planNodeIndices(0, 3, graph), std::vector<int>({0, 1, 3}));

  // 危険ノードの最終目標許可と、通過禁止の独立性
  graph.nodes[3].status.is_danger = true;
  EXPECT_FALSE(planner.planToAnyNode(0, {3}, graph, true).second.empty());
  EXPECT_TRUE(planner.planToAnyNode(0, {3}, graph, false).second.empty());
  graph.nodes[1].status.is_danger = true;
  EXPECT_EQ(planner.planToAnyNode(0, {3}, graph, true).second, std::vector<int>({0, 2, 3}));
  graph.nodes[2].status.is_colliding = true;
  EXPECT_TRUE(planner.planToAnyNode(0, {3}, graph, true).second.empty());
}

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
