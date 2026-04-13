#pragma once
#include "control/trajectory_safety_manager.hpp"
#include "experiment/i_planner.hpp"
#include "kinematics/state_adapter.hpp"
#include "planner/RRT/ik_rrt_planner.hpp"
#include "simulation/core/simulation_state.hpp"
#include <iostream>

namespace robot_sim {
namespace experiment {

/**
 * @brief RRTベースラインのラップ実装 (IK-RRT Connect版)
 */
class RRTPlannerWrapper : public IPlanner {
public:
  RRTPlannerWrapper(const kinematics::KinematicChain &chain,
                    planner::StateValidityChecker *checker,
                    kinematics::JointStateAdapter *adapter,
                    robot_sim::simulation::SimulationState &state,
                    const planner::RRTParams &params = planner::RRTParams())
      : rrt_(chain, params), chain_(chain), checker_(checker), adapter_(adapter), state_(state) {}

  PlanStats getLastPlanStats() const override { return last_stats_; }

  std::vector<Eigen::VectorXd>
  plan(const Eigen::VectorXd &start_q,
       const Eigen::Vector3d &target_pos) override {
    if (!checker_) {
      std::cerr << "[RRTPlannerWrapper] Error: Checker is null!" << std::endl;
      last_stats_ = PlanStats(); // Reset
      return {};
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    last_stats_ = PlanStats(); // Reset
    last_stats_.success = false;

    auto path = rrt_.plan(start_q, target_pos, *checker_);
    current_path_.assign(path.begin(), path.end());

    auto end_time = std::chrono::high_resolution_clock::now();
    last_planning_time_ms_ =
        std::chrono::duration<double, std::milli>(end_time - start_time)
            .count();

    if (!path.empty()) {
      if (last_planning_time_ms_ > 100.0) { // Only log if significant
        std::cout << "[RRTPlanner] Path found! Nodes: " << path.size()
                  << " Time: " << last_planning_time_ms_ << "ms" << std::endl;
      }

      // Update Stats
      last_stats_.planning_time_ms = last_planning_time_ms_;
      last_stats_.node_count = path.size();
      last_stats_.collision_check_count =
          rrt_.getCollisionCheckCount(*checker_);
      last_stats_.total_check_time_us =
          rrt_.getTotalCollisionCheckTimeUs(*checker_);
      last_stats_.avg_check_time_us =
          rrt_.getAverageCollisionCheckTimeUs(*checker_);
      last_stats_.success = true;

      // 可視化用に追加 (副作用)
      state_.candidate_paths_viz.clear();

      robot_sim::simulation::CandidatePathVisualization viz;
      for (const auto &q : path) {
        viz.path.push_back(q.cast<float>());
      }
      viz.is_selected = true;
      viz.color = Eigen::Vector3f(0.0f, 1.0f, 0.0f); // Green
      state_.candidate_paths_viz.push_back(viz);
    }

    return path;
  }

  std::string getName() const override { return "IK-RRT_Connect"; }

  double measureManipulabilityOverhead(const Eigen::VectorXd &state) override {
    auto start = std::chrono::high_resolution_clock::now();
    // JointStateAdapter::toPhysicalVec expansion for 8-DOF kinematic chain
    auto J = chain_.calculateJacobianAt(
        chain_.getTotalDOF() - 1, adapter_->toPhysicalVec(state));
    (void)J;
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
  }

  bool isPathValid(const Eigen::VectorXd &current_q,
                   size_t progress_index) const override {
    if (!checker_ || current_path_.empty())
      return true;

    // 0. 現在姿勢のチェック
    // [Follow Adapter] current_q is now guaranteed 7D logical state
    if (!checker_->isValid(current_q)) {
      return false;
    }

    // 1. 将来の衝突チェック
    static robot_sim::control::TrajectorySafetyManager safety_manager;
    // Note: Implicit default config (resolution=0.05, strict=true)

    size_t start_idx = progress_index + 1;
    // Look ahead a reasonable amount, or to the end?
    // User wanted "management of lookahead".
    // Here we use the safety object, but logic of *window* is still local.
    // Ideally SafetyManager::checkPathSafety(path, checker, start_idx,
    // lookahead) For now, manually loop to use checkSegmentSafety for
    // interpolation.

    size_t max_lookahead = 100;
    size_t end_idx = std::min(current_path_.size(), start_idx + max_lookahead);

    for (size_t i = start_idx; i < end_idx; ++i) {
      // Direct Waypoint Check
      if (!checker_->isValid(current_path_[i])) {
        // Optimization: if it's the *very last point*, maybe GNG allows it?
        // But RRT should reach a valid goal.
        return false;
      }

      // Interpolated Segment Check
      // (Strict check between nodes)
      if (i > start_idx) {
        if (!safety_manager.checkSegmentSafety(current_path_[i - 1],
                                               current_path_[i], checker_)) {
          return false;
        }
      }
    }
    return true;
  }

  double getLastPlanningTime() const { return last_planning_time_ms_; }

private:
  planner::IKRRTPlanner rrt_;
  const kinematics::KinematicChain &chain_;
  planner::StateValidityChecker *checker_;
  kinematics::JointStateAdapter *adapter_;
  robot_sim::simulation::SimulationState &state_; // Reference to state
  double last_planning_time_ms_ = 0.0;
  PlanStats last_stats_;
  std::deque<Eigen::VectorXd> current_path_;
};

} // namespace experiment
} // namespace robot_sim
