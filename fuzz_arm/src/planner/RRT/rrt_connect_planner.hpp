#pragma once

#include "kinematics/kinematic_chain.hpp"
#include "planner/RRT/rrt_params.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <random>
#include <vector>

namespace robot_sim {
namespace planner {

/**
 * @brief Standalone RRT-Connect Planner implementation.
 */
class RRTConnectPlanner {
public:
  struct Node {
    Eigen::VectorXd q;
    int parent_idx;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  RRTConnectPlanner(const kinematics::KinematicChain &chain,
                    const RRTParams &params = RRTParams())
      : chain_(chain), params_(params) {
    dof_ = chain_.getTotalDOF();
    std::random_device rd;
    rng_.seed(rd());
  }

  /**
   * @brief Plan a path between two joint configurations.
   */
  std::vector<Eigen::VectorXd> plan(const Eigen::VectorXd &start_q,
                                    const Eigen::VectorXd &goal_q,
                                    const StateValidityChecker &checker) {
    std::vector<Node> start_tree;
    start_tree.push_back({start_q, -1});

    std::vector<Node> goal_tree;
    goal_tree.push_back({goal_q, -1});

    return solve(start_tree, goal_tree, checker);
  }

  /**
   * @brief Core RRT-Connect solver that can take pre-initialized trees (e.g.,
   * for multi-root).
   */
  std::vector<Eigen::VectorXd>
  solve(std::vector<Node> &tree_a, std::vector<Node> &tree_b,
        const StateValidityChecker &checker,
        const RRTParams *override_params = nullptr) {
    bool swapped = false;
    const RRTParams &p = override_params ? *override_params : params_;

    auto start_time = std::chrono::steady_clock::now();

    for (int iter = 0; iter < p.max_iterations; ++iter) {
      // Time Limit Check
      auto current_time = std::chrono::steady_clock::now();
      double elapsed_ms =
          std::chrono::duration<double, std::milli>(current_time - start_time)
              .count();
      if (elapsed_ms > p.max_planning_time_ms) {
        // Timeout! Return empty path to prevent sim freeze.
        return {};
      }

      Eigen::VectorXd q_rand = sampleRandomConfig();

      // Extend tree A towards q_rand
      int new_idx_a = extend(tree_a, q_rand, checker, override_params);
      if (new_idx_a != -1) {
        // Try to connect the other tree to the new node in tree A
        int connect_idx_b =
            connect(tree_b, tree_a[new_idx_a].q, checker, override_params);
        if (connect_idx_b != -1) {
          // Path found!
          return buildPath(tree_a, new_idx_a, tree_b, connect_idx_b, swapped);
        }
      }

      // Swap trees for balance
      std::swap(tree_a, tree_b);
      swapped = !swapped;
    }

    return {};
  }

  Eigen::VectorXd sampleRandomConfig() {
    std::vector<double> q_vec = chain_.sampleRandomJointValues();
    return Eigen::Map<Eigen::VectorXd>(q_vec.data(), q_vec.size());
  }

  int extend(std::vector<Node> &tree, const Eigen::VectorXd &q_target,
             const StateValidityChecker &checker,
             const RRTParams *override_params = nullptr) {
    int nearest_idx = findNearest(tree, q_target);
    const Eigen::VectorXd &q_nearest = tree[nearest_idx].q;
    const RRTParams &p = override_params ? *override_params : params_;

    Eigen::VectorXd diff = q_target - q_nearest;
    double dist = diff.norm();
    if (dist < 1e-6)
      return nearest_idx;

    Eigen::VectorXd q_new =
        q_nearest + (diff / dist) * std::min(p.step_size, dist);

    // Hard clamp to joint limits for safety/stability
    std::vector<double> q_new_v(q_new.data(), q_new.data() + q_new.size());
    chain_.clampToLimits(q_new_v);
    q_new = Eigen::Map<Eigen::VectorXd>(q_new_v.data(), q_new_v.size());

    if (checker.isValid(q_new)) {
      // Simulate manipulability computation overhead if enabled
      if (p.simulate_manipulability_overhead) {
        std::vector<double> q_vec(q_new.data(), q_new.data() + q_new.size());
        Eigen::MatrixXd J =
            chain_.calculateJacobianAt(chain_.getNumJoints() + 1, q_vec);
        // Compute manipulability measure (sqrt(det(J*J^T))) to add realism
        double m = std::sqrt((J * J.transpose()).determinant());
        (void)m; // Suppress unused variable warning
      }

      tree.push_back({q_new, nearest_idx});
      return (int)tree.size() - 1;
    }
    return -1;
  }

  int connect(std::vector<Node> &tree, const Eigen::VectorXd &q_target,
              const StateValidityChecker &checker,
              const RRTParams *override_params = nullptr) {
    int last_idx = (int)tree.size() - 1;
    while (true) {
      int new_idx = extend(tree, q_target, checker, override_params);
      if (new_idx == -1)
        break;

      last_idx = new_idx;
      if ((tree[last_idx].q - q_target).norm() < 1e-6) {
        return last_idx;
      }
    }
    return -1;
  }

  int findNearest(const std::vector<Node> &tree, const Eigen::VectorXd &q) {
    int best_idx = 0;
    double min_dist_sq = (tree[0].q - q).squaredNorm();
    for (size_t i = 1; i < tree.size(); ++i) {
      double d_sq = (tree[i].q - q).squaredNorm();
      if (d_sq < min_dist_sq) {
        min_dist_sq = d_sq;
        best_idx = (int)i;
      }
    }
    return best_idx;
  }

  std::vector<Eigen::VectorXd> buildPath(const std::vector<Node> &tree_a,
                                         int idx_a,
                                         const std::vector<Node> &tree_b,
                                         int idx_b, bool swapped) {
    std::vector<Eigen::VectorXd> path_a, path_b;

    // Backtrack tree A
    int curr = idx_a;
    while (curr != -1) {
      path_a.push_back(tree_a[curr].q);
      curr = tree_a[curr].parent_idx;
    }
    std::reverse(path_a.begin(), path_a.end());

    // Backtrack tree B
    curr = idx_b;
    while (curr != -1) {
      path_b.push_back(tree_b[curr].q);
      curr = tree_b[curr].parent_idx;
    }

    if (swapped) {
      // tree_a was originally goal_tree, tree_b was originally start_tree
      // path_b is start->meeting, path_a is goal->meeting
      std::reverse(path_b.begin(), path_b.end());
      path_b.insert(path_b.end(), path_a.begin(), path_a.end());
      return path_b;
    } else {
      // tree_a was originally start_tree, tree_b was originally goal_tree
      // path_a is start->meeting, path_b is goal->meeting
      path_a.insert(path_a.end(), path_b.begin(), path_b.end());
      return path_a;
    }
  }

private:
  const kinematics::KinematicChain &chain_;
  RRTParams params_;
  int dof_;
  std::mt19937 rng_;
};

} // namespace planner
} // namespace robot_sim
