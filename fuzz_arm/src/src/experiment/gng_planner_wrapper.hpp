#pragma once
#include "experiment/i_planner.hpp"
#include "gng/GrowingNeuralGas_offline2.hpp"
#include "kinematics/state_adapter.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include "planning/gng_dijkstra_planner.hpp"
#include "simulation/core/simulation_state.hpp"

namespace robot_sim {
namespace experiment {

using GNGType = GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f>;

/**
 * @brief 既存のGNG+Dijkstraプランナーのラップ実装
 */
class GNGPlannerWrapper : public IPlanner {
public:
  GNGPlannerWrapper(
      std::shared_ptr<planning::GngDijkstraPlanner<Eigen::VectorXf,
                                                   Eigen::Vector3f, GNGType>>
          planner,
      const GNGType &gng, const kinematics::KinematicChain &chain,
      kinematics::JointStateAdapter *adapter,
      robot_sim::simulation::SimulationState &state,
      const planner::StateValidityChecker *validity_checker = nullptr)
      : planner_(planner), gng_(gng), chain_(chain), adapter_(adapter),
        state_(state), validity_checker_(validity_checker) {}

  PlanStats getLastPlanStats() const override { return last_stats_; }

  std::vector<Eigen::VectorXd>
  plan(const Eigen::VectorXd &start_q,
       const Eigen::Vector3d &target_pos) override {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Sync strict check setting from state
    if (planner_) {
      planner_->setStrictGoalCollisionCheck(state_.strict_goal_collision_check);
    }

    // 1. ビーム探索による目標候補ノードの選定
    // (全走査を回避し、トポロジカルに到達可能なノードを優先する)
    struct CandidateNode {
      int id;
      float dist;
    };
    std::vector<CandidateNode> candidates;

    // 起点となるスタートノードを検索
    // (ここも前のフレームの情報をキャッシュするとさらに速い)
    Eigen::VectorXf start_q_f = start_q.cast<float>();
    int start_node_id = -1;
    float min_dist_s = 1e10;
    for (int i = 0; i < (int)gng_.getMaxNodeNum(); ++i) {
      const auto &node = gng_.nodeAt(i);
      if (node.id != -1 && node.status.active && node.status.valid) {
        // [Follow Adapter] start_q_f and node.weight_angle are both 7D logical
        float d = (node.weight_angle - start_q_f).norm();
        if (d < min_dist_s) {
          min_dist_s = d;
          start_node_id = i;
        }
      }
    }

    if (start_node_id != -1) {
      std::deque<int> beam;
      beam.push_back(start_node_id);
      std::unordered_set<int> visited;
      visited.insert(start_node_id);

      // ビーム幅や探索ステップ数
      int beam_width = std::max(5, state_.gng_multi_target_count * 2);
      int max_steps = 100;

      Eigen::Vector3f target_f = target_pos.cast<float>();

      for (int step = 0; step < max_steps && !beam.empty(); ++step) {
        std::vector<CandidateNode> frontier;
        for (int node_id : beam) {
          // 隣接ノードを展開
          for (int neighbor_id : gng_.getNeighborsAngle(node_id)) {
            if (visited.find(neighbor_id) == visited.end()) {
              const auto &n = gng_.nodeAt(neighbor_id);
              if (n.id != -1 && n.status.active && n.status.valid &&
                  !n.status.is_colliding) {
                float d = (n.weight_coord - target_f).norm();
                frontier.push_back({neighbor_id, d});
                visited.insert(neighbor_id);
                // すべての訪れたノードを候補として蓄積
                candidates.push_back({neighbor_id, d});
              }
            }
          }
        }

        // 次のビームを選択 (距離が近い順)
        std::sort(frontier.begin(), frontier.end(),
                  [](const CandidateNode &a, const CandidateNode &b) {
                    return a.dist < b.dist;
                  });

        beam.clear();
        int count = std::min((int)frontier.size(), beam_width);
        for (int i = 0; i < count; ++i) {
          beam.push_back(frontier[i].id);
        }
      }

      // 最初に見つけたスタートノード自体も候補に加える
      float d_start =
          (gng_.nodeAt(start_node_id).weight_coord - target_f).norm();
      candidates.push_back({start_node_id, d_start});
    }

    // 安全性・距離順に最終ソート
    std::sort(candidates.begin(), candidates.end(),
              [&](const CandidateNode &a, const CandidateNode &b) {
                // 1. 危険フラグがないものを優先
                bool a_danger = gng_.nodeAt(a.id).status.is_danger;
                bool b_danger = gng_.nodeAt(b.id).status.is_danger;
                if (a_danger != b_danger)
                  return b_danger; // aが安全ならtrue
                // 2. 距離が近いものを優先
                return a.dist < b.dist;
              });

    if (candidates.empty()) {
      return {};
    }

    // 上位N件へのパスを計算して可視化用に保存
    std::vector<Eigen::VectorXd> best_path;
    path_indices_.clear();
    int max_candidates =
        std::min((int)candidates.size(), state_.gng_multi_target_count);

    if (start_node_id == -1)
      return {};

    // 候補ゴールノードIDリスト作成
    std::vector<int> candidate_goal_ids;
    for (int k = 0; k < max_candidates; ++k) {
      candidate_goal_ids.push_back(candidates[k].id);
    }

    // 一括探索実行 (One-to-Many)
    auto result_pair =
        planner_->planToAnyNode(start_node_id, candidate_goal_ids, gng_);
    int found_goal_id = result_pair.first;
    path_indices_ = result_pair.second;

    // default.
    bool enable_automatic_escape = false;

    bool is_escaping = false;

    if (enable_automatic_escape && found_goal_id == -1) {
      // Automatic Escape Mode DISABLED by user request
      /*
      const auto &start_node = gng_.nodeAt(start_node_id);
      if (start_node.status.is_colliding) {
        int best_safe_id = -1;
        // BFS Logic ... (Disabled)
      }
      */
    }

    state_.current_path_node_ids = path_indices_;

    size_t total_visited_nodes = planner_->getLastStats().visited_nodes;

    if (found_goal_id != -1) {
      // パスを関節角列に変換
      std::vector<Eigen::VectorXd> path_d;
      // 連続性を確保するため、現在の位置をパスの起点として必ず追加する
      path_d.push_back(start_q);

      for (size_t i = 0; i < path_indices_.size(); ++i) {
        Eigen::VectorXd node_q =
            gng_.nodeAt(path_indices_[i]).weight_angle.cast<double>();

        // Backtracking Prevention Logic
        if (i == 0) {
          // 1. Skip if too close (existing logic)
          if ((node_q - start_q).norm() < 0.2) {
            continue;
          }

          // 2. Skip if we are effectively "ahead" of this node towards the next
          // node This prevents the "infinite loop" where the robot goes back to
          // the nearest node, then moves forward, then replans back to the
          // nearest node again.
          if (path_indices_.size() > 1) {
            Eigen::VectorXd next_node_q =
                gng_.nodeAt(path_indices_[1]).weight_angle.cast<double>();
            Eigen::VectorXd vec_edge = next_node_q - node_q;
            Eigen::VectorXd vec_progress = start_q - node_q;

            double edge_len_sq = vec_edge.squaredNorm();
            if (edge_len_sq > 1e-6) {
              double t = vec_progress.dot(vec_edge) / edge_len_sq;
              // If t > 0, we have made some progress along the edge (0 -> 1)
              // If t < 1, we are still between 0 and 1 (not yet passed 1)
              // We skip node 0 if we are "significantly" ahead (e.g., > 10% or
              // > small distance) But for robustness, even t > 0 might be
              // enough cause to skip node 0 IF we are already on the path
              // segment. Let's use a small threshold to avoid skipping if we
              // are just "at" the node.
              if (t > 0.1) {
                // std::cout << "[GNGPlanner] Skipping back-node " <<
                // path_indices_[0] << " (progress: " << t << ")" << std::endl;
                continue;
              }
            }
          }
        }
        path_d.push_back(node_q);
      }

      // ゴールIK補正 (Escape時はスキップ)
      // found_goal_id に対するデカルト目標への厳密なIK解へ補間
      if (!is_escaping) {
        // std::cout << "[GNGPlanner] Refining goal with IK..." << std::endl;
        // IK refined goal calculation
        Eigen::VectorXd goal_q =
            gng_.nodeAt(found_goal_id).weight_angle.cast<double>();
        std::vector<double> ik_solution;
        // JointStateAdapter::toPhysicalVec handles the expansion to 8D physical state
        if (chain_.inverseKinematicsAt(
                chain_.getNumJoints() + 1, target_pos,
                adapter_->toPhysicalVec(goal_q),
                100, 1e-4, ik_solution)) {
          Eigen::VectorXd exact_goal_q_phys = Eigen::Map<Eigen::VectorXd>(
              ik_solution.data(), ik_solution.size());
          // Back to 6D planning state
          Eigen::VectorXd exact_goal_q = adapter_->toPlanning(exact_goal_q_phys);
          // パスの最後を置き換え、または追加
          if ((path_d.back() - exact_goal_q).norm() > 1e-4) {
            path_d.push_back(exact_goal_q);
          } else {
            path_d.back() = exact_goal_q;
          }
        }
      } else {
      }

      best_path = path_d;

      // 可視化データ (採用されたパス)
      state_.candidate_paths_viz.clear();
      robot_sim::simulation::CandidatePathVisualization viz;
      for (const auto &q : path_d)
        viz.path.push_back(q.cast<float>());
      viz.is_selected = true;
      viz.color = Eigen::Vector3f(0.0f, 1.0f, 0.0f); // Green
      state_.candidate_paths_viz.push_back(viz);
    }

    // (Optimization) 他の候補へのパス可視化は省略、または必要なら別途探索
    auto end_time = std::chrono::high_resolution_clock::now();
    last_planning_time_ms_ =
        std::chrono::duration<double, std::milli>(end_time - start_time)
            .count();

    last_stats_.planning_time_ms = last_planning_time_ms_;
    last_stats_.node_count = best_path.size(); // Path length
    // Use collision_check_count field to store visited nodes for GNG (graph
    // search metric)
    last_stats_.collision_check_count = total_visited_nodes;
    last_stats_.total_check_time_us = 0; // GNG uses pre-computed checks
    last_stats_.avg_check_time_us = 0;
    last_stats_.success = !best_path.empty();
    // GNG stats could be populated if GngDijkstraPlanner exposed them

    if (!best_path.empty()) {
    }

    return best_path;
  }

  std::string getName() const override { return "GNG_ReactivePlanner"; }

  double measureManipulabilityOverhead(const Eigen::VectorXd &state) override {
    // RRTPlannerWrapperと同じ計算を行い、公平に比較する
    auto start = std::chrono::high_resolution_clock::now();

    std::vector<double> q_vec(state.data(), state.data() + state.size());
    auto J = chain_.calculateJacobianAt(chain_.getTotalDOF() - 1, q_vec);
    (void)J;

    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
  }

  bool isPathValid(const Eigen::VectorXd &current_q,
                   size_t progress_index) const override {
    if (path_indices_.empty())
      return true;

    // 0. 現在姿勢のチェック (厳密に行う)
    // ユーザー指摘: 「判定が甘い」原因の一つはここを無視していたこと
    // Unused parameter 'current_q' warning 解消
    if (this->validity_checker_ &&
        !this->validity_checker_->isValid(current_q)) {

      return false; // Collision detected at current position -> Replan
    }

    if (path_indices_.empty())
      return true;

    // 1. 現在位置から次の目標ノードまでの補間チェック
    // GNGのパスインデックス配列内での探索探索開始位置
    size_t start_idx = (progress_index >= path_indices_.size())
                           ? path_indices_.size()
                           : progress_index;

    // もしパスが既に終わりならOK
    if (start_idx >= path_indices_.size())
      return true;

    // 現在位置 -> 次のノード への補間チェック (分解能0.05rad程度)
    Eigen::VectorXd next_q =
        gng_.nodeAt(path_indices_[start_idx]).weight_angle.cast<double>();
    if (this->validity_checker_) {
      // 解像度
      double step = 0.05;

      // [Follow Adapter] both are now 7D logical
      double dist = (next_q - current_q).norm();
      int steps = std::max(1, (int)(dist / step));
      for (int k = 1; k <= steps; ++k) {
        double t = (double)k / steps;
        Eigen::VectorXd p = current_q + t * (next_q - current_q);
        if (!this->validity_checker_->isValid(p)) {
          return false;
        }
      }
    }

    // 2. 将来のパスノードのチェック (GNGフラグベース + 必要なら実チェック)
    // GNGの場合はフラグチェックのみで非常に軽量なため、パスの全ノードをチェックする
    for (size_t i = start_idx; i < path_indices_.size(); ++i) {
      int id = path_indices_[i];
      const auto &node = gng_.nodeAt(id);

      // Nodeのフラグを見る (広域チェック)
      if (node.id != -1 && node.status.is_colliding) {
        // ゴールノードのみの衝突であれば、ほぼ到達していると見なして許容する
        if (i == path_indices_.size() - 1) {
          if (!state_.strict_goal_collision_check) {
            continue;
          }
        }
        return false;
      }
    }
    return true;
  }

private:
  std::shared_ptr<
      planning::GngDijkstraPlanner<Eigen::VectorXf, Eigen::Vector3f, GNGType>>
      planner_;
  const GNGType &gng_;
  const kinematics::KinematicChain &chain_;
  kinematics::JointStateAdapter *adapter_;
  robot_sim::simulation::SimulationState &state_; // Reference to state
  const planner::StateValidityChecker *validity_checker_;
  PlanStats last_stats_;
  double last_planning_time_ms_ = 0.0;
  std::vector<int> path_indices_;
};

} // namespace experiment
} // namespace robot_sim
