#pragma once

#include <unordered_map>
#include <vector>

namespace robot_sim::planning {

// 到達不能な組合せは省略または空経路。開始・終端を含むGNGノード列
using graph_paths = std::unordered_map<int, std::unordered_map<int, std::vector<int>>>;

struct graph_plan_request {
  const std::vector<int> &start_ids;
  const std::vector<int> &goal_ids;
  bool allow_danger_goal = true;
};

struct graph_planner_options {
  bool enable_collision_check = true;
  bool enable_danger_check = true;
  bool enable_safety_penalty = true;
  bool enable_strict_goal_check = true;
  bool enable_static_graph = false;
};

// 呼出し中のグラフ更新は利用側で排他。計画器の所有権は呼出し側
// 関節補間・時刻付与・実行制御を含まないグラフ経路計画の境界
template <typename graph_type>
class graph_planner {
public:
  virtual ~graph_planner() = default;
  virtual graph_paths plan(const graph_type &graph,
                           const graph_plan_request &request) = 0;
};

}  // 名前空間robot_sim::planningの終端
