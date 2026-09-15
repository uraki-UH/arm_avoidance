#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <unordered_map>
#include <vector>

namespace planning {

// 固定の有向グラフ上の再開可能な最短経路木。安全状態変更時の全木失効
class static_path_index {
public:
  struct edge {
    int to;
    float cost;
  };

  static_path_index(std::vector<std::size_t> offsets, std::vector<edge> edges)
      : offsets_(std::move(offsets)), edges_(std::move(edges)) {}

  void update_access(std::vector<uint8_t> can_enter, const std::vector<int> &starts,
                     const std::vector<int> &goals) {
    if (can_enter != can_enter_) {
      can_enter_ = std::move(can_enter);
      trees_.clear();
    }
    for (auto it = trees_.begin(); it != trees_.end();) {
      if (std::find(starts.begin(), starts.end(), it->first.first) == starts.end() ||
          (it->first.second >= 0 && std::find(goals.begin(), goals.end(), it->first.second) == goals.end())) {
        it = trees_.erase(it);
      } else {
        ++it;
      }
    }
    for (int start : starts) {
      if (start < 0 || start >= static_cast<int>(can_enter_.size())) continue;
      trees_.try_emplace(std::make_pair(start, -1));
      for (int goal : goals) {
        if (goal >= 0 && goal < static_cast<int>(can_enter_.size()) && !can_enter_[goal]) {
          trees_.try_emplace(std::make_pair(start, goal));
        }
      }
    }
  }

  bool needs_search(int start, const std::vector<int> &goals, int terminal = -1) const {
    if (goals.empty()) return false;
    const auto &tree = trees_.at({start, terminal});
    if (tree.dist.empty()) return true;
    if (tree.queue.empty()) return false;
    return std::any_of(goals.begin(), goals.end(), [&](int goal) { return !tree.is_settled[goal]; });
  }

  std::unordered_map<int, std::vector<int>> paths(
      int start, const std::vector<int> &goals, std::size_t &num_visited, int terminal = -1) {
    if (goals.empty()) return {};
    auto &tree = trees_.at({start, terminal});
    const std::size_t num_nodes = can_enter_.size();
    if (tree.dist.empty()) {
      tree.dist.assign(num_nodes, std::numeric_limits<float>::infinity());
      tree.parent.assign(num_nodes, -1);
      tree.is_settled.assign(num_nodes, false);
      tree.dist[start] = 0.0f;
      tree.queue.push({start, 0.0f});
    }
    std::vector<bool> is_goal(num_nodes, false);
    std::size_t num_remaining = 0;
    for (int goal : goals) {
      if (!tree.is_settled[goal] && !is_goal[goal]) {
        is_goal[goal] = true;
        ++num_remaining;
      }
    }
    while (num_remaining && !tree.queue.empty()) {
      const auto current = tree.queue.top();
      tree.queue.pop();
      ++num_visited;
      if (current.dist > tree.dist[current.id]) continue;
      tree.is_settled[current.id] = true;
      if (is_goal[current.id]) {
        is_goal[current.id] = false;
        --num_remaining;
      }
      if (current.id == terminal) break;
      // 再開時にも同じキュー順を維持するため、終点の隣接更新まで実施
      for (std::size_t idx = offsets_[current.id]; idx < offsets_[current.id + 1]; ++idx) {
        const auto &next = edges_[idx];
        if (!can_enter_[next.to] && next.to != terminal) continue;
        const float dist = current.dist + next.cost;
        if ((tree.parent[next.to] == -1 && next.to != start) || dist < tree.dist[next.to]) {
          tree.dist[next.to] = dist;
          tree.parent[next.to] = current.id;
          tree.queue.push({next.to, dist});
        }
      }
    }
    std::unordered_map<int, std::vector<int>> result;
    for (int goal : goals) {
      auto &path = result[goal];
      if (!tree.is_settled[goal] || !path.empty()) continue;
      for (int at = goal; at != start; at = tree.parent[at]) path.push_back(at);
      path.push_back(start);
      std::reverse(path.begin(), path.end());
    }
    return result;
  }

private:
  struct queue_node {
    int id;
    float dist;
    bool operator>(const queue_node &other) const { return dist > other.dist; }
  };
  struct search_tree {
    std::vector<float> dist;
    std::vector<int> parent;
    std::vector<bool> is_settled;
    std::priority_queue<queue_node, std::vector<queue_node>, std::greater<queue_node>> queue;
  };
  std::vector<std::size_t> offsets_;
  std::vector<edge> edges_;
  std::vector<uint8_t> can_enter_;
  std::map<std::pair<int, int>, search_tree> trees_;
};
}  // namespace planning
