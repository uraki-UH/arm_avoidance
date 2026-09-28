#pragma once

#include "planning/graph_planner.hpp"
#include "planning/gng_dijkstra_planner.hpp"

namespace robot_sim::planning {

// 既存Dijkstraの設定と探索方式を閉じ込める実装部品
template <typename angle_type, typename coord_type, typename graph_type>
class dijkstra_graph_planner final : public graph_planner<graph_type> {
public:
  dijkstra_graph_planner(
      const graph_type &graph, const graph_planner_options &options,
      std::shared_ptr<::planning::ICostEvaluator<angle_type, coord_type>> cost,
      bool enable_batch)
      : enable_batch_(enable_batch) {
    planner_.setCostEvaluator(std::move(cost));
    planner_.setAvoidCollisions(options.enable_collision_check);
    planner_.setAvoidDanger(options.enable_danger_check);
    planner_.set_enable_safety_penalty(options.enable_safety_penalty);
    planner_.setStrictGoalCollisionCheck(options.enable_strict_goal_check);
    if (options.enable_static_graph && enable_batch_) {
      planner_.prepare_static_graph(graph);
    }
  }

  graph_paths plan(const graph_type &graph,
                   const graph_plan_request &request) override {
    if (enable_batch_) {
      return planner_.plan_from_each_start(request.start_ids, request.goal_ids,
                                          graph, request.allow_danger_goal);
    }
    graph_paths result;
    for (int start_id : request.start_ids) {
      for (int goal_id : request.goal_ids) {
        result[start_id][goal_id] = planner_.planToAnyNode(
            start_id, {goal_id}, graph, request.allow_danger_goal).second;
      }
    }
    return result;
  }

private:
  ::planning::GngDijkstraPlanner<angle_type, coord_type, graph_type> planner_;
  bool enable_batch_;
};

}  // 名前空間robot_sim::planningの終端
