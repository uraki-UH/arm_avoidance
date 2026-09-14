#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <random>
#include <set>

#include "core/planning/gng_dijkstra_planner.hpp"
#include "core/planning/joint_linf_cost.hpp"

namespace {
struct batch_graph {
  using node_type = GNG::NeuronNode<Eigen::VectorXf, Eigen::Vector3f>;
  std::vector<node_type> nodes;
  std::vector<std::vector<int>> neighbors;
  std::set<std::pair<int, int>> disabled_edges;

  explicit batch_graph(int num) : nodes(num), neighbors(num) {
    for (int idx = 0; idx < num; ++idx) {
      nodes[idx].id = idx;
      nodes[idx].weight_angle = Eigen::VectorXf::Constant(1, idx % 7);
      nodes[idx].status.active = true;
      nodes[idx].status.self_collision_free = true;
      nodes[idx].status.is_colliding = false;
      nodes[idx].status.is_danger = false;
    }
  }
  std::size_t getMaxNodeNum() const { return nodes.size(); }
  const node_type &nodeAt(int idx) const { return nodes[idx]; }
  const std::vector<int> &getNeighborsAngle(int idx) const { return neighbors[idx]; }
  bool isEdgeActive(int from, int to, int) const {
    return disabled_edges.count({from, to}) == 0;
  }
  template <typename callback_type>
  void forEachActiveValid(callback_type callback) const {
    for (const auto &node : nodes) callback(node.id, node);
  }
};
using planner_type = planning::GngDijkstraPlanner<Eigen::VectorXf, Eigen::Vector3f, batch_graph>;

planner_type make_planner() {
  planner_type planner;
  planner.setCostEvaluator(
      std::make_shared<planning::JointLInfCost<Eigen::VectorXf, Eigen::Vector3f>>());
  planner.set_enable_safety_penalty(false);
  return planner;
}

void compare_paths(planner_type &planner, const batch_graph &graph,
                   int start_id, const std::vector<int> &goals, bool allow_danger_goal) {
  auto paths = planner.plan_to_each_node(start_id, goals, graph, allow_danger_goal);
  auto cached = planner.plan_from_each_start({start_id, (start_id + 1) % static_cast<int>(graph.nodes.size())}, goals,
                                             graph, allow_danger_goal);
  for (int goal_id : goals) {
    EXPECT_EQ(paths[goal_id], planner.planToAnyNode(start_id, {goal_id}, graph, allow_danger_goal).second)
        << "start=" << start_id << " goal=" << goal_id;
  }
  for (auto &[id, goal_paths] : cached) {
    for (int goal_id : goals) {
      EXPECT_EQ(goal_paths[goal_id], planner.planToAnyNode(id, {goal_id}, graph, allow_danger_goal).second);
    }
  }
}

TEST(candidate_path_batch, edge_cache_scope) {
  struct counting_cost : planning::JointLInfCost<Eigen::VectorXf, Eigen::Vector3f> {
    int num_calls = 0;
    float evaluate(const batch_graph::node_type &from, const batch_graph::node_type &to) override {
      ++num_calls;
      return planning::JointLInfCost<Eigen::VectorXf, Eigen::Vector3f>::evaluate(from, to);
    }
  };
  batch_graph graph(5);
  graph.neighbors = {{1}, {2}, {3}, {}, {0}};
  auto cost = std::make_shared<counting_cost>();
  auto planner = make_planner();
  planner.setCostEvaluator(cost);
  planner.setAvoidCollisions(true);
  auto paths = planner.plan_from_each_start({0, 4, 0, -1, 99}, {2, 3}, graph);
  EXPECT_EQ(paths.size(), 2U);
  EXPECT_EQ(paths[4][3], std::vector<int>({4, 0, 1, 2, 3}));
  EXPECT_EQ(cost->num_calls, 4);
  EXPECT_TRUE(planner.plan_from_each_start({}, {3}, graph).empty());
  EXPECT_TRUE(planner.plan_from_each_start({0, 4}, {}, graph).empty());
  EXPECT_EQ(planner.plan_from_each_start({0}, {3}, graph)[0][3], paths[0][3]);
  // 次の計画への古いエッジ状態・ノード状態・関節コストの持ち越し防止
  graph.disabled_edges.insert({1, 2});
  EXPECT_TRUE(planner.plan_from_each_start({0, 4}, {2, 3}, graph)[0][3].empty());
  graph.disabled_edges.clear();
  graph.nodes[1].status.is_danger = true;
  EXPECT_TRUE(planner.plan_from_each_start({0, 4}, {2, 3}, graph)[4][3].empty());
  graph.nodes[1].status.is_danger = false;
  graph.nodes[2].weight_angle[0] = 30.0F;
  const int num_before = cost->num_calls;
  EXPECT_EQ(planner.plan_from_each_start({0, 4}, {2, 3}, graph)[0][3], paths[0][3]);
  EXPECT_EQ(cost->num_calls - num_before, 4);
  graph.neighbors[1].clear();
  EXPECT_TRUE(planner.plan_from_each_start({0, 4}, {3}, graph)[4][3].empty());
}

TEST(candidate_path_batch, terminal_exceptions_and_updates) {
  batch_graph graph(5);
  graph.neighbors = {{1}, {2}, {3}, {}, {}};
  auto planner = make_planner();
  planner.setAvoidCollisions(true);
  graph.nodes[1].status.is_danger = true;
  auto paths = planner.plan_to_each_node(0, {1, 2, 3}, graph);
  EXPECT_EQ(paths[1], std::vector<int>({0, 1}));
  EXPECT_TRUE(paths[2].empty());
  EXPECT_TRUE(paths[3].empty());
  graph.nodes[1].status.is_danger = false;
  EXPECT_EQ(planner.plan_to_each_node(0, {2, 3}, graph)[3], std::vector<int>({0, 1, 2, 3}));
  graph.nodes[1].status.is_colliding = true;
  compare_paths(planner, graph, 0, {0, 1, 2, 3, 4, -1, 99, 1}, true);
  EXPECT_TRUE(planner.plan_to_each_node(0, {1, 2}, graph)[2].empty());
  planner.setStrictGoalCollisionCheck(true);
  EXPECT_TRUE(planner.plan_to_each_node(0, {1, 2}, graph)[1].empty());
  // 衝突開始点からの脱出と、非活性エッジの遮断
  EXPECT_EQ(planner.plan_to_each_node(1, {2, 3}, graph)[3], std::vector<int>({1, 2, 3}));
  graph.disabled_edges.insert({1, 2});
  EXPECT_TRUE(planner.plan_to_each_node(1, {2, 3}, graph)[3].empty());
  EXPECT_TRUE(planner.plan_to_each_node(-1, {1, 2}, graph)[1].empty());
  EXPECT_TRUE(planner.plan_to_each_node(99, {1, 2}, graph)[1].empty());
  EXPECT_TRUE(planner.plan_to_each_node(0, {}, graph).empty());
}

TEST(candidate_path_batch, matches_individual_paths_with_ties_and_safety_modes) {
  std::mt19937 rng(914);
  for (int iter = 0; iter < 20; ++iter) {
    batch_graph graph(40);
    for (int idx = 0; idx < 40; ++idx) {
      graph.nodes[idx].status.is_colliding = rng() % 9 == 0;
      graph.nodes[idx].status.is_danger = rng() % 7 == 0;
      graph.nodes[idx].status.active = rng() % 13 != 0;
      graph.nodes[idx].status.self_collision_free = rng() % 17 != 0;
      for (int to = 0; to < 40; ++to) {
        if (to != idx && rng() % 10 == 0) graph.neighbors[idx].push_back(to);
      }
    }
    std::vector<int> goals{0, 0, -1, 40};
    for (int idx = 1; idx < 40; ++idx) goals.push_back(idx);
    for (int mode = 0; mode < 32; ++mode) {
      auto planner = make_planner();
      planner.setAvoidCollisions(mode & 1);
      planner.setAvoidDanger(mode & 2);
      planner.setStrictGoalCollisionCheck(mode & 4);
      planner.set_enable_safety_penalty(mode & 8);
      compare_paths(planner, graph, iter % 40, goals, mode & 16);
    }
  }
}

TEST(candidate_path_batch, benchmark_10000_nodes) {
  batch_graph graph(10000);
  std::mt19937 rng(915);
  for (int idx = 0; idx < 10000; ++idx) {
    graph.nodes[idx].weight_angle = Eigen::VectorXf::Constant(1, rng() % 10000 * 0.001F);
    graph.neighbors[idx].push_back((idx + 1) % 10000);
    for (int j = 0; j < 5; ++j) graph.neighbors[idx].push_back(rng() % 10000);
  }
  const std::vector<int> goals{9000, 9100, 9200, 9300, 9400, 9500, 9600, 9700};
  auto planner = make_planner();
  planner.setAvoidCollisions(true);
  std::vector<std::unordered_map<int, std::vector<int>>> expected(5);
  std::size_t num_individual = 0;
  const auto begin = std::chrono::steady_clock::now();
  for (int start_id = 0; start_id < 5; ++start_id) {
    for (int goal_id : goals) {
      expected[start_id][goal_id] = planner.planToAnyNode(start_id, {goal_id}, graph).second;
      num_individual += planner.getLastStats().visited_nodes;
    }
  }
  const auto middle = std::chrono::steady_clock::now();
  std::size_t num_shared = 0;
  for (int start_id = 0; start_id < 5; ++start_id) {
    EXPECT_EQ(planner.plan_to_each_node(start_id, goals, graph), expected[start_id]);
    num_shared += planner.getLastStats().visited_nodes;
  }
  const auto end = std::chrono::steady_clock::now();
  EXPECT_LT(num_shared, num_individual);
  std::cout << "individual_ms=" << std::chrono::duration<double, std::milli>(middle - begin).count()
            << " shared_ms=" << std::chrono::duration<double, std::milli>(end - middle).count()
            << " individual_visited=" << num_individual << " shared_visited=" << num_shared << '\n';
}

TEST(candidate_path_batch, actual_robot_graph) {
  const std::string path = "/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin";
  if (!std::ifstream(path).good()) GTEST_SKIP() << "実ロボットGNGデータなし";
  using graph_type = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
  graph_type graph(7, 3, nullptr);
  ASSERT_TRUE(graph.load(path));
  planning::GngDijkstraPlanner<Eigen::VectorXf, Eigen::Vector3f, graph_type> planner;
  planner.setCostEvaluator(
      std::make_shared<planning::JointLInfCost<Eigen::VectorXf, Eigen::Vector3f>>());
  planner.setAvoidCollisions(true);
  planner.set_enable_safety_penalty(false);
  std::vector<int> safe_ids;
  graph.forEachActiveValid([&](int id, const auto &node) {
    if (node.status.self_collision_free && !node.status.is_colliding && !node.status.is_danger) {
      safe_ids.push_back(id);
    }
  });
  ASSERT_GT(safe_ids.size(), 100U);
  std::vector<int> goals;
  for (int idx = 1; idx <= 8; ++idx) goals.push_back(safe_ids[idx * safe_ids.size() / 9]);
  double individual_ms = 0.0;
  double shared_ms = 0.0;
  std::vector<int> starts;
  std::unordered_map<int, std::unordered_map<int, std::vector<int>>> expected_by_start;
  for (int start_idx = 0; start_idx < 5; ++start_idx) {
    const int start_id = safe_ids[start_idx];
    starts.push_back(start_id);
    std::unordered_map<int, std::vector<int>> expected;
    auto begin = std::chrono::steady_clock::now();
    for (int goal_id : goals) expected[goal_id] = planner.planToAnyNode(start_id, {goal_id}, graph).second;
    auto middle = std::chrono::steady_clock::now();
    auto actual = planner.plan_to_each_node(start_id, goals, graph);
    auto end = std::chrono::steady_clock::now();
    for (int goal_id : goals) EXPECT_EQ(actual[goal_id], expected[goal_id]);
    expected_by_start[start_id] = std::move(expected);
    individual_ms += std::chrono::duration<double, std::milli>(middle - begin).count();
    shared_ms += std::chrono::duration<double, std::milli>(end - middle).count();
  }
  const auto begin = std::chrono::steady_clock::now();
  auto cached = planner.plan_from_each_start(starts, goals, graph);
  const double cached_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  for (int start_id : starts) {
    for (int goal_id : goals) EXPECT_EQ(cached[start_id][goal_id], expected_by_start[start_id][goal_id]);
  }
  std::cout << "actual_graph_individual_ms=" << individual_ms << " shared_ms=" << shared_ms
            << " cached_ms=" << cached_ms << '\n';
}
}  // 無名名前空間
