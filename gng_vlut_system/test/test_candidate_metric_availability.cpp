#include <gtest/gtest.h>

#include "core/common/evaluation_metric_serialization.hpp"
#include "core/planning/topological_map_avoidance_helpers.hpp"
#include "core/planning/joint_linf_cost.hpp"
#include "core/planning/gng_dijkstra_planner.hpp"

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

TEST(candidate_path_planning, retreat_goal_requires_direct_safe_neighbors)
{
  using robot_sim::planning::topological_map_avoidance::has_safe_retreat_neighbors;
  planning_graph graph;
  // 自身は安全でも隣接4が衝突中の候補1は除外。二次隣接だけの候補3は許可
  EXPECT_FALSE(has_safe_retreat_neighbors(graph, 1));
  EXPECT_TRUE(has_safe_retreat_neighbors(graph, 3));
  std::vector<int> goals{1, 3};
  goals.erase(std::remove_if(goals.begin(), goals.end(), [&](int id) {
    return !has_safe_retreat_neighbors(graph, id);
  }), goals.end());
  planning::GngDijkstraPlanner<Eigen::VectorXf, Eigen::Vector3f, planning_graph> planner;
  planner.setCostEvaluator(std::make_shared<planning::JointLInfCost<Eigen::VectorXf, Eigen::Vector3f>>());
  planner.setAvoidCollisions(true);
  planner.setAvoidDanger(true);
  planner.setStrictGoalCollisionCheck(true);
  planner.set_enable_safety_penalty(false);
  // 中間ノード1の隣接まで禁止せず、安全なノード3への退避経路を維持
  EXPECT_EQ(planner.planToAnyNode(0, goals, graph).second, std::vector<int>({0, 1, 3}));
  graph.nodes[2].status.is_danger = true;
  EXPECT_FALSE(has_safe_retreat_neighbors(graph, 3));
  graph.nodes[2].status.is_danger = false;
  EXPECT_TRUE(has_safe_retreat_neighbors(graph, 3));
  graph.nodes[2].status.active = false;
  EXPECT_FALSE(has_safe_retreat_neighbors(graph, 3));
  graph.nodes[2].status.active = true;
  graph.nodes[3].status.is_danger = true;
  EXPECT_FALSE(has_safe_retreat_neighbors(graph, 3));
  graph.nodes[3].status.is_danger = false;
  graph.neighbors[3].push_back(99);
  EXPECT_FALSE(has_safe_retreat_neighbors(graph, 3));
  EXPECT_FALSE(has_safe_retreat_neighbors(graph, -1));
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

namespace {
class injected_graph_planner final
    : public robot_sim::planning::topological_map_avoidance::graph_planner_type {
public:
  robot_sim::planning::graph_paths plan(
      const robot_sim::planning::topological_map_avoidance::GNGType &,
      const robot_sim::planning::graph_plan_request &request) override {
    EXPECT_EQ(request.start_ids, std::vector<int>({0}));
    EXPECT_EQ(request.goal_ids, std::vector<int>({1}));
    if (has_path) return {{0, {{1, {0, 1}}}}};
    return {};
  }
  bool has_path = true;
};

TEST(graph_planner_components, injected_backend_reaches_candidate_selection) {
  using namespace robot_sim::planning::topological_map_avoidance;
  auto graph = std::make_shared<GNGType>(2, 3, nullptr);
  for (int idx = 0; idx < 2; ++idx) {
    auto &node = graph->nodeAt(idx);
    node.id = idx;
    node.status.active = true;
    node.status.self_collision_free = true;
    node.weight_angle = Eigen::VectorXf::Constant(2, idx);
  }
  injected_graph_planner planner;
  int selected_start = -1;
  std::unordered_map<int, std::vector<int>> by_goal;
  std::vector<std::vector<int>> paths;
  const auto current = Eigen::VectorXf::Zero(2).eval();
  const auto result = planFromStartCandidates(
      graph, planner, current, {0}, {1}, selected_start, by_goal, paths, false);
  EXPECT_EQ(result.first, 1);
  EXPECT_EQ(result.second, std::vector<int>({0, 1}));
  EXPECT_EQ(selected_start, 0);
  planner.has_path = false;
  const auto failure = planFromStartCandidates(
      graph, planner, current, {0}, {1}, selected_start, by_goal, paths, false);
  EXPECT_EQ(failure.first, -1);
  EXPECT_EQ(selected_start, -1);
  EXPECT_TRUE(paths.empty());
  EXPECT_TRUE(by_goal.empty());
}
}  // 無名名前空間の終端
