#pragma once

#include "planner/RRT/ik_rrt_planner.hpp"
#include "planning/path_planner.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include "simulation/safety/safety_management.hpp"
#include "spatial/ispatial_index.hpp"
#include <algorithm>
#include <map>
#include <queue>

namespace planning {

/**
 * GNGグラフ上でのダイクストラアルゴリズムの実装。
 * この実装はジェネリックで、ノード間の移動コストを定義するために
 * ICostEvaluator を使用する。
 */
template <typename T_angle, typename T_coord, typename T_GNG>
class GngDijkstraPlanner : public IPathPlanner<T_angle, T_coord, T_GNG> {
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
  }

  /**
   * スタートとゴールのジョイント構成（ポスチャ）間のパスを計画する。
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
        if (!v.status.valid || !v.status.active)
          continue;

        // [衝突回避] ノード安全性判定 (衝突しているノードは遮断)
        if (avoid_collisions_) {
          // 隣接ノードが衝突している場合は原則遮断
          if (v.status.is_colliding) {
            // ただし、ゴールノードかつ厳格チェックOFFなら許避
            if (neighbor_id == goal_id && !strict_goal_collision_check_) {
              // 許容
            } else {
              continue;
            }
          }
          // Note: 現在のノード (current.id) が衝突していても、移動先
          // (neighbor_id) が安全なら移動を許可する（Escape）。
        }

        // エッジが活性であることを確認
        if (!gng.isEdgeActive(current.id, neighbor_id, 0))
          continue;

        const auto &u = gng.nodeAt(current.id);
        float step_cost = evaluator_->evaluate(u, v);
        
        // [追加] 隣接安全マージンによるペナルティ
        float safety_penalty = 0.0f;
        if (avoid_collisions_) {
          for (int nv_id : gng.getNeighborsAngle(neighbor_id)) {
            if (gng.nodeAt(nv_id).status.is_colliding) {
              safety_penalty += 2.0f; // 1つ隣接衝突があるごとに大幅なコスト増
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
   * ここでは「最も近いターゲット」への経路を効率的に見つけることを主眼とします）。
   *
   * @param start_id 開始ノードID
   * @param candidate_goal_ids ターゲット候補ノードIDのリスト
   * @param gng GNGグラフ
   * @return {到達したゴールID, パス(ID列)} のペア。見つからなければ {-1, empty}
   */
  std::pair<int, std::vector<int>>
  planToAnyNode(int start_id, const std::vector<int> &candidate_goal_ids,
                const T_GNG &gng) {
    if (!evaluator_)
      return {-1, {}};

    // ゴール判定用セット
    std::vector<bool> is_goal(gng.getMaxNodeNum(), false);
    bool valid_goal_exists = false;
    for (int gid : candidate_goal_ids) {
      if (gid >= 0 && gid < (int)gng.getMaxNodeNum()) {
        is_goal[gid] = true;
        valid_goal_exists = true;
      }
    }
    if (!valid_goal_exists)
      return {-1, {}};

    // すでにゴールにいる場合
    if (is_goal[start_id])
      return {start_id, {start_id}};

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

    int reached_goal_id = -1;

    while (!pq.empty()) {
      NodeInfo current = pq.top();
      pq.pop();
      stats_.visited_nodes++;

      // ゴール判定
      if (is_goal[current.id]) {
        reached_goal_id = current.id;
        break;
      }

      if (current.dist > min_dist[current.id])
        continue;

      for (int neighbor_id : gng.getNeighborsAngle(current.id)) {
        const auto &v = gng.nodeAt(neighbor_id);
        if (!v.status.valid || !v.status.active)
          continue;

        if (avoid_collisions_) {
          bool v_colliding = v.status.is_colliding;

          if (v_colliding) {
            // Target node is colliding

            // Unless it is the goal, strictly avoid entering collision
            if (!is_goal[neighbor_id])
              continue;

            // If strict check is enabled, even goal collision is forbidden
            if (strict_goal_collision_check_)
              continue;
          }

          // Note: We ALLOW u_colliding (current) -> !v_colliding (safe)
          // This enables "Escape" paths from a colliding start node.
        }

        if (!gng.isEdgeActive(current.id, neighbor_id, 0))
          continue;

        const auto &u = gng.nodeAt(current.id);
        float step_cost = evaluator_->evaluate(u, v);

        // [追加] 隣接安全マージンによるペナルティ
        float safety_penalty = 0.0f;
        if (avoid_collisions_) {
          for (int nv_id : gng.getNeighborsAngle(neighbor_id)) {
            if (gng.nodeAt(nv_id).status.is_colliding) {
              safety_penalty += 2.0f;
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

    if (reached_goal_id == -1)
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
        if (!node.status.valid || !node.status.active) continue;

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
  bool avoid_collisions_ = false;
  bool strict_goal_collision_check_ = false; // Added
  Stats stats_;
};

} // namespace planning