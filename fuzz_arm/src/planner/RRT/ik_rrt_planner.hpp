#pragma once

#include "kinematics/kinematic_chain.hpp"
#include "planner/RRT/rrt_connect_planner.hpp"
#include "planner/RRT/rrt_params.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <vector>

namespace robot_sim {
namespace planner {

/**
 * @brief IK-RRT Planner implementation based on Berenson et al. (2008).
 * Supports Cartesian goals via multi-IK sampling and RRT-Connect.
 */
class IKRRTPlanner {
public:
  IKRRTPlanner(const kinematics::KinematicChain &chain,
               const RRTParams &params = RRTParams())
      : chain_(chain), params_(params), connect_planner_(chain, params) {}

  /**
   * @brief Plan a path from start joint angles to a target Cartesian position.
   * @param start_q Initial joint angles.
   * @param target_pos Target position in world coordinates.
   * @param checker State validity checker (collisions, etc.)
   * @param override_params Optional parameters to override defaults (e.g., for
   * fast tracking).
   * @return A vector of joint angle states representing the path. Empty if
   * failed.
   */
  std::vector<Eigen::VectorXd>
  plan(const Eigen::VectorXd &start_q, const Eigen::Vector3d &target_pos,
       const StateValidityChecker &checker,
       const RRTParams *override_params = nullptr) {
    // [Fix] Ensure start_q matches the internal DOF to prevent Eigen assertion crashes
    int dof = chain_.getTotalDOF();
    Eigen::VectorXd s = start_q;
    if (s.size() != dof) {
      s = start_q.head(std::min((int)start_q.size(), dof));
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    last_planning_time_ms_ = 0.0;
    checker.resetMetrics();

    // 1. Multi-IK Sampling for multiple goal candidates
    std::vector<Eigen::VectorXd> goal_states =
        sampleGoalStates(s, target_pos, checker, override_params);

    if (goal_states.empty()) {
      auto end_time = std::chrono::high_resolution_clock::now();
      last_planning_time_ms_ =
          std::chrono::duration<double, std::milli>(end_time - start_time)
              .count();
      return {}; // Failed to find any valid IK solution
    }

    // 2. Delegate to plan with pre-sampled goals
    return plan(s, goal_states, checker, override_params);
  }

  /**
   * @brief Plan a path using pre-sampled goal states.
   */
  std::vector<Eigen::VectorXd>
  plan(const Eigen::VectorXd &start_q,
       const std::vector<Eigen::VectorXd> &goal_states,
       const StateValidityChecker &checker,
       const RRTParams *override_params = nullptr) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (goal_states.empty()) {
      last_planning_time_ms_ = 0.0;
      return {};
    }

    // Initialize Two Trees for RRT-Connect
    std::vector<RRTConnectPlanner::Node> start_tree;
    start_tree.push_back({start_q, -1});

    std::vector<RRTConnectPlanner::Node> goal_tree;
    for (const auto &gq : goal_states) {
      goal_tree.push_back({gq, -1});
    }

    // Delegate to RRTConnect solver
    auto path =
        connect_planner_.solve(start_tree, goal_tree, checker, override_params);

    auto end_time = std::chrono::high_resolution_clock::now();
    last_planning_time_ms_ =
        std::chrono::duration<double, std::milli>(end_time - start_time)
            .count();

    return path;
  }

  /**
   * @brief Get the computation time of the last plan() call in milliseconds.
   */
  double getLastPlanningTimeMs() const { return last_planning_time_ms_; }

private:
  const kinematics::KinematicChain &chain_;
  RRTParams params_;
  RRTConnectPlanner connect_planner_;
  double last_planning_time_ms_ = 0.0;

public:
  /**
   * @brief Sample valid goal states using IK with random seeds.
   */
  std::vector<Eigen::VectorXd>
  sampleGoalStates(const Eigen::VectorXd &start_q,
                   const Eigen::Vector3d &target_pos,
                   const StateValidityChecker &checker,
                   const RRTParams *override_params = nullptr) {
    std::vector<Eigen::VectorXd> valid_goals;
    const RRTParams &p = override_params ? *override_params : params_;

    int attempts = 0;
    int max_attempts = p.max_ik_samples * 10;

    auto ik_start_time = std::chrono::high_resolution_clock::now();
    const double ik_timeout_ms =
        10.0; // Hard timeout for IK sampling in Real-Time

    // First attempt: Use current posture as seed for continuity
    std::vector<double> current_seed(start_q.data(),
                                     start_q.data() + start_q.size());
    std::vector<double> sol;
    if (chain_.inverseKinematicsAt(chain_.getNumJoints() + 1, target_pos,
                                   current_seed, p.max_ik_iterations,
                                   p.goal_pos_tolerance, sol)) {
      Eigen::VectorXd q_goal =
          Eigen::Map<Eigen::VectorXd>(sol.data(), sol.size());
      if (checker.isValid(q_goal))
        valid_goals.push_back(q_goal);
    }

    while (valid_goals.size() < (size_t)p.max_ik_samples &&
           attempts < max_attempts) {
      attempts++;

      // Time Check
      auto now = std::chrono::high_resolution_clock::now();
      if (std::chrono::duration<double, std::milli>(now - ik_start_time)
              .count() > ik_timeout_ms) {
        break;
      }

      std::vector<double> seed_vec = chain_.sampleRandomJointValues();
      std::vector<double> solution;
      if (chain_.inverseKinematicsAt(chain_.getNumJoints() + 1, target_pos,
                                     seed_vec, p.max_ik_iterations,
                                     p.goal_pos_tolerance, solution)) {
        Eigen::VectorXd q_goal =
            Eigen::Map<Eigen::VectorXd>(solution.data(), solution.size());
        if (checker.isValid(q_goal))
          valid_goals.push_back(q_goal);
      }
    }

    // Sort valid goals by distance to start_q to prefer continuity
    if (!valid_goals.empty()) {
      // [Fix] Enforce dimension consistency for physical/planning space boundary
      int dof = chain_.getTotalDOF();
      Eigen::VectorXd start_q_fixed = start_q;
      if (start_q_fixed.size() != dof) {
        start_q_fixed = start_q.head(std::min((int)start_q.size(), dof));
      }

      std::sort(valid_goals.begin(), valid_goals.end(),
                [&](const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
                  return (a - start_q_fixed).squaredNorm() <
                         (b - start_q_fixed).squaredNorm();
                });
    }

    return valid_goals;
  }

public:
  /**
   * @brief Get the total number of collision checks performed during the last
   * plan().
   */
  size_t getCollisionCheckCount(const StateValidityChecker &checker) const {
    return checker.getCheckCount();
  }

  /**
   * @brief Get the total time spent on collision checks during the last plan()
   * in microseconds.
   */
  double
  getTotalCollisionCheckTimeUs(const StateValidityChecker &checker) const {
    return checker.getTotalCheckTimeUs();
  }

  /**
   * @brief Get the average time per collision check in microseconds.
   */
  double
  getAverageCollisionCheckTimeUs(const StateValidityChecker &checker) const {
    size_t count = checker.getCheckCount();
    return (count > 0) ? (checker.getTotalCheckTimeUs() / count) : 0.0;
  }
};

} // namespace planner
} // namespace robot_sim
