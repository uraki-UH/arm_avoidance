#pragma once

#include "planner/RRT/ik_rrt_planner.hpp"
#include "planning/path_planner.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include "safety_engine/runtime/safety_management.hpp"
#include "safety_engine/indexing/ispatial_index.hpp"
#include "planning/static_path_index.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <future>
#include <thread>
#include <map>
#include <limits>
#include <queue>
#include <unordered_map>

namespace planning {

/**
 * GNGグラフ上でのダイクストラアルゴリズムの実装。
 * この実装はジェネリックで、ノード間の移動コストを定義するために
 * ICostEvaluator を使用する。
 */
template <typename T_angle, typename T_coord, typename T_GNG>
class GngDijkstraPlanner : public IPathPlanner<T_angle, T_coord, T_GNG> {
  struct cached_edge {
    bool has_value = false;
    bool is_active = false;
    float cost = 0.0f;
  };
  using edge_cache = std::vector<std::vector<cached_edge>>;

public:
  struct Stats {
    size_t visited_nodes = 0;
  };

  GngDijkstraPlanner() = default;

  Stats getLastStats() const { return stats_; }

  /**
   * コスト評価クラスを設定する。
   */
  void setCostEvaluator(
      std::shared_ptr<ICostEvaluator<T_angle, T_coord>> evaluator) override {
    evaluator_ = evaluator;
    static_graph_.reset();
  }

  // 固定モデル読込後の探索索引構築。重み・エッジ変更時は再構築が必要
  bool prepare_static_graph(const T_GNG &gng) {
    static_graph_.reset();
    if (!evaluator_) return false;
    std::vector<std::size_t> offsets(gng.getMaxNodeNum() + 1, 0);
    std::vector<static_path_index::edge> edges;
    for (std::size_t from = 0; from < gng.getMaxNodeNum(); ++from) {
      for (int to : gng.getNeighborsAngle(from)) {
        if (!gng.isEdgeActive(from, to, 0)) continue;
        const auto cost = evaluator_->static_edge_cost(gng.nodeAt(from), gng.nodeAt(to));
        if (!cost || !std::isfinite(*cost) || *cost < 0.0f) return false;
        edges.push_back({to, *cost});
      }
      offsets[from + 1] = edges.size();
    }
    static_graph_ = std::make_unique<static_path_index>(std::move(offsets), std::move(edges));
    static_gng_ = &gng;
    static_num_nodes_ = gng.getMaxNodeNum();
    return true;
  }

  /**
   * スタートとゴール姿勢間のパスを計画する。
   * @param start スタートのジョイント値（ノードに正確に対応しない場合もある）
   * @param goal ゴールのジョイント値（ノードに正確に対応しない場合もある）
   * @param gng パス計画に使用するGNGグラフ
   * @return ジョイント構成のシーケンス（パス）。パスが見つからない場合は空。
   */
  std::vector<T_angle> plan(const T_angle &start, const T_angle &goal,
                            const T_GNG &gng) override {
    // 1. スタートとゴールに最も近いノードを見つける
    int start_id = findNearestNode(start, gng);
    int goal_id = findNearestNode(goal, gng);

    if (start_id == -1 || goal_id == -1)
      return {};

    // 2. ノード間のパスを計画する
    std::vector<int> node_ids = planNodeIndices(start_id, goal_id, gng);
    if (node_ids.empty())
      return {};

    // 3. ノードインデックスをジョイント構成に変換する
    std::vector<T_angle> path;
    path.reserve(node_ids.size() + 1);
    for (int id : node_ids) {
      path.push_back(gng.nodeAt(id).weight_angle);
    }

    // 最後に正確な目標ポスチャ（goal）を追加して、到達精度を向上させる
    if (!path.empty() && (path.back() - goal).norm() > 1e-4) {
      path.push_back(goal);
    }

    return path;
  }

  /**
   * 衝突回避モードを設定する。
   */
  void setAvoidCollisions(bool enable) { avoid_collisions_ = enable; }

  void setAvoidDanger(bool enable) { avoid_danger_ = enable; }

  void set_enable_safety_penalty(bool enable_safety_penalty) {
    enable_safety_penalty_ = enable_safety_penalty;
  }

  /**
   * ゴールノードの厳格な衝突チェックを設定する。
   */
  void setStrictGoalCollisionCheck(bool enable) {
    strict_goal_collision_check_ = enable;
  }

  /**
   * 特定のノードID間のパスを計画する。
   */
  std::vector<int> planNodeIndices(int start_id, int goal_id,
                                   const T_GNG &gng) override {
    if (!evaluator_)
      return {};
    if (start_id == goal_id)
      return {start_id};

    stats_.visited_nodes = 0;

    struct NodeInfo {
      int id;
      float dist;
      bool operator>(const NodeInfo &other) const { return dist > other.dist; }
    };

    std::priority_queue<NodeInfo, std::vector<NodeInfo>, std::greater<NodeInfo>>
        pq;
    std::map<int, float> min_dist;
    std::map<int, int> parent;

    pq.push({start_id, 0.0f});
    min_dist[start_id] = 0.0f;

    // Search Limit to prevent freeze
    size_t max_scan_nodes = gng.getMaxNodeNum() * 5;

    while (!pq.empty()) {
      NodeInfo current = pq.top();
      pq.pop();
      stats_.visited_nodes++;

      if (stats_.visited_nodes > max_scan_nodes) {
        // Abort search if taking too long
        break;
      }

      if (current.id == goal_id)
        break;
      if (current.dist > min_dist[current.id])
        continue;

      // デュアルスペースGNGでは、モーションプランニングに主に角度空間の近傍を使用する
      for (int neighbor_id : gng.getNeighborsAngle(current.id)) {
        const auto &v = gng.nodeAt(neighbor_id);

        // 有効かつ活性なノードのみを探索対象にする
        if (!v.status.self_collision_free || !v.status.active)
          continue;

        // [衝突回避] ノード安全性判定 (衝突しているノードは遮断)
        if (avoid_collisions_) {
          // 隣接ノードが衝突している場合は原則遮断
          if (v.status.is_colliding) {
            // ただし、ゴールノードかつ厳格チェックOFFなら
            if (neighbor_id == goal_id && !strict_goal_collision_check_) {
              // 許容
            } else {
              continue;
            }
          }
          if (avoid_danger_ && v.status.is_danger) {
            continue;
          }
          // Note: 現在のノード (current.id) が衝突していても、移動先
          // (neighbor_id) が安全なら移動を許可する（Escape）。
        }

        // エッジが活性であることを確認
        if (!gng.isEdgeActive(current.id, neighbor_id, 0))
          continue;

        const auto &u = gng.nodeAt(current.id);
        float step_cost = evaluator_->evaluate(u, v);
        
        // 実行系向けの隣接危険ノード数による追加コスト
        float safety_penalty = 0.0f;
        if (avoid_collisions_ && enable_safety_penalty_) {
          for (int nv_id : gng.getNeighborsAngle(neighbor_id)) {
            if (gng.nodeAt(nv_id).status.is_colliding) {
              safety_penalty += 2.0f; // 1つ隣接衝突があるごとに大幅なコスト増
            }
            if (avoid_danger_ && gng.nodeAt(nv_id).status.is_danger) {
              safety_penalty += 1.0f;
            }
          }
        }
        
        float new_dist = current.dist + step_cost + safety_penalty;

        if (min_dist.find(neighbor_id) == min_dist.end() ||
            new_dist < min_dist[neighbor_id]) {
          min_dist[neighbor_id] = new_dist;
          parent[neighbor_id] = current.id;
          pq.push({neighbor_id, new_dist});
        }
      }
    }

    // Check if goal was actually reached
    if (min_dist.find(goal_id) == min_dist.end()) {
      return {}; // Path not found
    }

    std::vector<int> path;
    for (int at = goal_id; at != start_id; at = parent[at]) {
      path.push_back(at);
    }
    path.push_back(start_id);
    std::reverse(path.begin(), path.end());

    return path;
  }

  /**
   * 複数ターゲットに対する一括パス計画 (One-to-Many Dijkstra)
   * 開始ノードから探索を開始し、指定されたターゲットノードのいずれかに到達した時点で
   * その経路を返します（もしくは全てのターゲットまでの経路を計算することも可能ですが、
   * ここでは「最も近いターゲット」への経路を見つける
   * @param start_id 開始ノードID
   * @param candidate_goal_ids ターゲット候補ノードIDのリスト
   * @param gng GNGグラフ
   * @return {到達したゴールID, パス(ID列)} のペア。見つからなければ {-1, empty}
   */
  std::pair<int, std::vector<int>>
  planToAnyNode(int start_id, const std::vector<int> &candidate_goal_ids,
                const T_GNG &gng, bool allow_danger_goal = true) {
    return search_node_paths(start_id, candidate_goal_ids, gng, allow_danger_goal, nullptr);
  }

  // 同じ安全条件のゴール間で共有した探索。例外的な終点許可・危険度コストは個別探索
  std::unordered_map<int, std::vector<int>> plan_to_each_node(
      int start_id, const std::vector<int> &goal_ids,
      const T_GNG &gng, bool allow_danger_goal = true) {
    return plan_to_each_node_cached(start_id, goal_ids, gng, allow_danger_goal, nullptr);
  }

  // 固定索引の再利用と独立木の並列探索。非対応評価器は計画内のエッジコスト共有
  std::unordered_map<int, std::unordered_map<int, std::vector<int>>> plan_from_each_start(
      const std::vector<int> &start_ids, const std::vector<int> &goal_ids,
      const T_GNG &gng, bool allow_danger_goal = true) {
    std::unordered_map<int, std::unordered_map<int, std::vector<int>>> paths;
    stats_.visited_nodes = 0;
    if (start_ids.empty() || goal_ids.empty() || !evaluator_) return paths;
    const bool can_use_static = static_graph_ && static_gng_ == &gng &&
        static_num_nodes_ == gng.getMaxNodeNum() && !(avoid_collisions_ && enable_safety_penalty_);
    std::vector<uint8_t> can_enter;
    if (can_use_static) {
      can_enter.resize(gng.getMaxNodeNum());
      for (std::size_t idx = 0; idx < can_enter.size(); ++idx) {
        const auto &status = gng.nodeAt(idx).status;
        can_enter[idx] = status.active && status.self_collision_free &&
            (!avoid_collisions_ || (!status.is_colliding && !(avoid_danger_ && status.is_danger)));
      }
      static_graph_->update_access(can_enter, start_ids, goal_ids);
    }
    edge_cache cache(!can_use_static && start_ids.size() > 1 ? gng.getMaxNodeNum() : 0);
    using indexed_result = std::pair<std::unordered_map<int, std::vector<int>>, std::size_t>;
    struct search_job {
      int start;
      int terminal;
      std::vector<int> goals;
    };
    std::vector<search_job> jobs;
    std::size_t num_visited = 0;
    auto add_job = [&](int start, std::vector<int> goals, int terminal) {
      if (goals.empty()) return;
      if (static_graph_->needs_search(start, goals, terminal)) {
        jobs.push_back({start, terminal, std::move(goals)});
      } else {
        auto result = static_graph_->paths(start, goals, num_visited, terminal);
        paths.at(start).insert(std::make_move_iterator(result.begin()), std::make_move_iterator(result.end()));
      }
    };
    for (int start_id : start_ids) {
      if (start_id < 0 || start_id >= static_cast<int>(gng.getMaxNodeNum()) ||
          paths.count(start_id)) continue;
      if (can_use_static) {
        std::vector<int> shared_goals;
        std::vector<int> exception_goals;
        for (int goal : goal_ids) {
          if (goal < 0 || goal >= static_cast<int>(can_enter.size())) continue;
          if (can_enter[goal] || goal == start_id) {
            shared_goals.push_back(goal);
          } else {
            const auto &status = gng.nodeAt(goal).status;
            if (status.active && status.self_collision_free &&
                !(avoid_collisions_ && ((status.is_colliding && strict_goal_collision_check_) ||
                   (status.is_danger && avoid_danger_ && !allow_danger_goal))) &&
                std::find(exception_goals.begin(), exception_goals.end(), goal) == exception_goals.end()) {
              exception_goals.push_back(goal);
            }
          }
        }
        paths.emplace(start_id, std::unordered_map<int, std::vector<int>>{});
        add_job(start_id, std::move(shared_goals), -1);
        // 例外終点ごとに独立した木。別の危険終点の通過許可への転用なし
        for (int goal : exception_goals) add_job(start_id, {goal}, goal);
      } else {
        paths.emplace(start_id,
            plan_to_each_node_cached(start_id, goal_ids, gng, allow_danger_goal,
                                     cache.empty() ? nullptr : &cache));
        num_visited += stats_.visited_nodes;
      }
    }
    std::vector<indexed_result> results(jobs.size());
    std::atomic<std::size_t> next_job{0};
    auto search = [&]() {
      for (std::size_t idx; (idx = next_job.fetch_add(1, std::memory_order_relaxed)) < jobs.size();) {
        const auto &job = jobs[idx];
        results[idx].first = static_graph_->paths(job.start, job.goals, results[idx].second, job.terminal);
      }
    };
    // ワーカー数をCPU数までに制限。同じ木の同時更新と再利用時のスレッド起動なし
    const auto num_workers = std::min<std::size_t>(jobs.size(), std::max(1U, std::thread::hardware_concurrency()));
    std::vector<std::future<void>> workers;
    for (std::size_t idx = 1; idx < num_workers; ++idx) workers.push_back(std::async(std::launch::async, search));
    search();
    for (auto &worker : workers) worker.get();
    for (std::size_t idx = 0; idx < jobs.size(); ++idx) {
      auto &result = results[idx];
      num_visited += result.second;
      paths.at(jobs[idx].start).insert(std::make_move_iterator(result.first.begin()),
                                       std::make_move_iterator(result.first.end()));
    }
    stats_.visited_nodes = num_visited;
    return paths;
  }

private:
  std::unordered_map<int, std::vector<int>> plan_to_each_node_cached(
      int start_id, const std::vector<int> &goal_ids,
      const T_GNG &gng, bool allow_danger_goal, edge_cache *cache) {
    std::unordered_map<int, std::vector<int>> paths;
    std::vector<int> shared_goal_ids;
    std::size_t num_visited = 0;
    for (int goal_id : goal_ids) {
      if (goal_id < 0 || goal_id >= static_cast<int>(gng.getMaxNodeNum())) {
        continue;
      }
      const auto &node = gng.nodeAt(goal_id);
      if (avoid_collisions_ && (enable_safety_penalty_ || node.status.is_colliding ||
                               (avoid_danger_ && node.status.is_danger))) {
        if (paths.count(goal_id)) continue;
        auto result = search_node_paths(start_id, {goal_id}, gng, allow_danger_goal, nullptr, cache);
        num_visited += stats_.visited_nodes;
        paths.emplace(goal_id, std::move(result.second));
      } else {
        shared_goal_ids.push_back(goal_id);
      }
    }
    if (!shared_goal_ids.empty()) {
      search_node_paths(start_id, shared_goal_ids, gng, allow_danger_goal, &paths, cache);
      num_visited += stats_.visited_nodes;
    }
    stats_.visited_nodes = num_visited;
    return paths;
  }

  std::pair<int, std::vector<int>> search_node_paths(
      int start_id, const std::vector<int> &candidate_goal_ids,
      const T_GNG &gng, bool allow_danger_goal,
      std::unordered_map<int, std::vector<int>> *all_paths, edge_cache *cache = nullptr) {
    stats_.visited_nodes = 0;
    if (!evaluator_ || start_id < 0 || start_id >= static_cast<int>(gng.getMaxNodeNum()))
      return {-1, {}};

    // ゴール判定用セット
    std::vector<bool> is_goal(gng.getMaxNodeNum(), false);
    std::size_t num_remaining = 0;
    for (int gid : candidate_goal_ids) {
      if (gid >= 0 && gid < (int)gng.getMaxNodeNum() && !is_goal[gid]) {
        is_goal[gid] = true;
        ++num_remaining;
      }
    }
    if (num_remaining == 0)
      return {-1, {}};

    // すでにゴールにいる場合
    if (is_goal[start_id] && !all_paths)
      return {start_id, {start_id}};

    stats_.visited_nodes = 0;

    struct NodeInfo {
      int id;
      float dist;
      bool operator>(const NodeInfo &other) const { return dist > other.dist; }
    };

    std::priority_queue<NodeInfo, std::vector<NodeInfo>, std::greater<NodeInfo>>
        pq;
    std::vector<float> min_dist(gng.getMaxNodeNum(), std::numeric_limits<float>::infinity());
    std::vector<int> parent(gng.getMaxNodeNum(), -1);

    pq.push({start_id, 0.0f});
    min_dist[start_id] = 0.0f;

    int reached_goal_id = -1;

    while (!pq.empty()) {
      NodeInfo current = pq.top();
      pq.pop();
      stats_.visited_nodes++;

      // ゴール判定
      if (is_goal[current.id]) {
        reached_goal_id = current.id;
        if (!all_paths) break;
        auto &path = (*all_paths)[current.id];
        for (int at = current.id; at != start_id; at = parent[at]) {
          path.push_back(at);
        }
        path.push_back(start_id);
        std::reverse(path.begin(), path.end());
        is_goal[current.id] = false;
        if (--num_remaining == 0) break;
      }

      if (current.dist > min_dist[current.id])
        continue;

      const auto &neighbors = gng.getNeighborsAngle(current.id);
      if (cache) (*cache)[current.id].resize(neighbors.size());
      for (std::size_t neighbor_idx = 0; neighbor_idx < neighbors.size(); ++neighbor_idx) {
        const int neighbor_id = neighbors[neighbor_idx];
        const auto &v = gng.nodeAt(neighbor_id);
        if (!v.status.self_collision_free || !v.status.active)
          continue;

        if (avoid_collisions_) {
          bool v_colliding = v.status.is_colliding;
          bool v_danger = v.status.is_danger;

          if (v_colliding) {
            // 終点以外の衝突ノードへの進入禁止
            if (!is_goal[neighbor_id])
              continue;

            // 厳格チェック時の終点衝突禁止
            if (strict_goal_collision_check_)
              continue;
          }
          if (avoid_danger_ && v_danger && !(allow_danger_goal && is_goal[neighbor_id])) {
            continue;
          }

          // 衝突中の開始ノードから安全ノードへの脱出許可
        }

        float step_cost;
        if (cache) {
          auto &edge = (*cache)[current.id][neighbor_idx];
          if (!edge.has_value) {
            edge.is_active = gng.isEdgeActive(current.id, neighbor_id, 0);
            if (edge.is_active) edge.cost = evaluator_->evaluate(gng.nodeAt(current.id), v);
            edge.has_value = true;
          }
          if (!edge.is_active) continue;
          step_cost = edge.cost;
        } else {
          if (!gng.isEdgeActive(current.id, neighbor_id, 0)) continue;
          step_cost = evaluator_->evaluate(gng.nodeAt(current.id), v);
        }

        // 実行系向けの隣接危険ノード数による追加コスト
        float safety_penalty = 0.0f;
        if (avoid_collisions_ && enable_safety_penalty_) {
          for (int nv_id : gng.getNeighborsAngle(neighbor_id)) {
            if (gng.nodeAt(nv_id).status.is_colliding) {
              safety_penalty += 2.0f;
            }
            if (avoid_danger_ && gng.nodeAt(nv_id).status.is_danger &&
                !(allow_danger_goal && is_goal[nv_id])) {
              safety_penalty += 1.0f;
            }
          }
        }

        float new_dist = current.dist + step_cost + safety_penalty;

        if ((parent[neighbor_id] == -1 && neighbor_id != start_id) ||
            new_dist < min_dist[neighbor_id]) {
          min_dist[neighbor_id] = new_dist;
          parent[neighbor_id] = current.id;
          pq.push({neighbor_id, new_dist});
        }
      }
    }

    if (all_paths || reached_goal_id == -1)
      return {-1, {}};

    // パス再構築
    std::vector<int> path;
    for (int at = reached_goal_id; at != start_id; at = parent[at]) {
      path.push_back(at);
    }
    path.push_back(start_id);
    std::reverse(path.begin(), path.end());

    return {reached_goal_id, path};
  }

public:
  /**
   * 指定されたポスチャに最も近いノードを見つける。
   */
  int findNearestNode(const T_angle &posture, const T_GNG &gng) const {
    float min_dist = 1e10f;
    int nearest_id = -1;

    gng.forEachActiveValid([&](int i, const auto &node) {
      int dim = std::min((int)node.weight_angle.size(), (int)posture.size());
      float d = (node.weight_angle.head(dim) - posture.head(dim)).norm();
      if (d < min_dist) {
        min_dist = d;
        nearest_id = i;
      }
    });
    return nearest_id;
  }

  /**
   * Reachabilityを考慮した最近傍ノードの探索。
   * @param posture 現在の姿勢 $q$
   * @param gng GNGグラフ
   * @param checker 有効性チェッカー (Local Pathの検証に使用)
   * @param num_candidates 候補として抽出する近傍ノード数
   * @param check_steps 直線補間時の分割ステップ数
   */
  /**
   * VLUT (Voxel Look-Up Table) をフル活用した超高速な最近傍探索。
   * 到達可能性チェック (Reachability) も VLUT ベースの判定に切り替えることで、
   * ODE による重いメッシュ干渉判定を完全に排除します。
   */
  int findNearestReachableNode(
      const T_angle &posture, const Eigen::Vector3d &eef_pos, const T_GNG &gng,
      const robot_sim::analysis::ISpatialIndex *spatial_index,
      const robot_sim::simulation::SafetyStateManager *safety_manager,
      int check_steps = 5) const {
    if (!spatial_index || !safety_manager) {
        // Fallback to slow scan if safety manager is missing
        return findNearestReachableNode(posture, eef_pos, gng, spatial_index, nullptr, check_steps);
    }

    // 1. ボクセルから候補ノード ID 群を O(1) で取得
    std::vector<int> candidates = spatial_index->getNodesInVoxel(eef_pos);
    if (candidates.empty()) {
        // 近傍ボクセルをチェックするなどの拡張も考えられるが、
        // 現状は安全側に倒してフルスキャンへのフォールバックを許容
        return -1;
    }

    // 2. 候補ノードの中から関節空間で最も近いものを探す
    int nearest_id = -1;
    float min_dist = 1e10f;
    int dim = posture.size();

    for (int id : candidates) {
        if (id < 0 || (size_t)id >= gng.getMaxNodeNum()) continue;
        const auto &node = gng.nodeAt(id);
        if (!node.status.self_collision_free || !node.status.active) continue;

        float d = (node.weight_angle.head(dim) - posture).norm();
        if (d < min_dist) {
            // 到達可能性チェック (VLUTベース)
            bool reachable = true;
            for (int step = 1; step <= check_steps; ++step) {
                float t = (float)step / (float)check_steps;
                T_angle q_step = posture * (1.0f - t) + node.weight_angle.head(dim) * t;
                
                // 補間点のEEF位置を求める (軽量なFKが望ましい)
                // ここではノードの weight_coord を補間して代用する (近似)
                // 正確には kinematic_chain_->calculateFK() が必要だが、
                // 速度を優先し、C-Space 直線が Work-Space でも概ね直線であると仮定
                Eigen::Vector3d pos_step = eef_pos * (1.0f - t) + node.weight_coord.template cast<double>() * t;

                if (safety_manager->isCollidingAt(spatial_index, pos_step)) {
                    reachable = false;
                    break;
                }
            }

            if (reachable) {
                min_dist = d;
                nearest_id = id;
            }
        }
    }

    return nearest_id;
  }

  /**
   * Reachabilityを考慮した最近傍ノードの探索 (デフォルト版)。
   */
  int findNearestReachableNode(
      const T_angle &posture, const T_GNG &gng,
      const robot_sim::planner::StateValidityChecker *checker,
      int num_candidates = 5, int check_steps = 5) const {
    if (!checker)
      return findNearestNode(posture, gng);

    struct NodeDist {
      int id;
      float dist;
      bool operator<(const NodeDist &other) const { return dist < other.dist; }
    };

    std::vector<NodeDist> candidates;
    gng.forEachActiveValid([&](int i, const auto &node) {
      int dim = std::min((int)node.weight_angle.size(), (int)posture.size());
      float d = (node.weight_angle.head(dim) - posture.head(dim)).norm();
      candidates.push_back({i, d});
    });

    if (candidates.empty())
      return -1;

    // 距離の近い順にソートして上位候補を検証
    int limit = std::min((int)candidates.size(), num_candidates);
    std::partial_sort(candidates.begin(), candidates.begin() + limit,
                      candidates.end());

    for (int i = 0; i < limit; ++i) {
      int id = candidates[i].id;
      const auto &target_q = gng.nodeAt(id).weight_angle;

      // 直線補間によるLocal Pathの衝突チェック
      bool reachable = true;
      int dim = std::min((int)posture.size(), (int)target_q.size());
      for (int step = 1; step <= check_steps; ++step) {
        float t = (float)step / (float)check_steps;
        T_angle interpolated_q = posture.head(dim) * (1.0f - t) + target_q.head(dim) * t;
        if (!checker->isValid(interpolated_q.template cast<double>())) {
          reachable = false;
          break;
        }
      }

      if (reachable) {
        return id; // 最も近くて到達可能なノードを返す
      }
    }

    return -1; // 到達可能なノードが見つからない
  }

private:
  std::shared_ptr<ICostEvaluator<T_angle, T_coord>> evaluator_;
  std::unique_ptr<static_path_index> static_graph_;
  const T_GNG *static_gng_ = nullptr;
  std::size_t static_num_nodes_ = 0;
  bool avoid_collisions_ = false;
  bool enable_safety_penalty_ = true;
  bool avoid_danger_ = true;
  bool strict_goal_collision_check_ = false; // Added
  Stats stats_;
};

} // namespace planning
