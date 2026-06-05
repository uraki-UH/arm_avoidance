#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include "planner/RRT/ik_rrt_planner.hpp"
#include "planner/RRT/rrt_params.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include "planning/gng_dijkstra_planner.hpp"
#include "planning/joint_linf_cost.hpp"
#include "safety_engine/gng/GrowingNeuralGas.hpp"

namespace robot_sim::planning::topological_map_avoidance {

using GNGType = ::GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
using PlannerType =
    ::planning::GngDijkstraPlanner<Eigen::VectorXf, Eigen::Vector3f, GNGType>;

static inline uint8_t pathLabelFromStatus(const ::GNG::Status &status) {
  if (status.is_colliding) {
    return 2;
  }
  if (status.is_danger) {
    return 3;
  }
  return 1;
}

static inline std::vector<Eigen::VectorXf>
buildBridgePath(const Eigen::VectorXf &from_q, const Eigen::VectorXf &to_q,
                int steps) {
  std::vector<Eigen::VectorXf> bridge;
  const int dim = std::min(static_cast<int>(from_q.size()),
                           static_cast<int>(to_q.size()));
  if (dim <= 0) {
    return bridge;
  }

  const int clamped_steps = std::max(1, steps);
  bridge.reserve(static_cast<std::size_t>(clamped_steps));
  for (int i = 1; i <= clamped_steps; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(clamped_steps);
    Eigen::VectorXf q = from_q.head(dim) * (1.0f - t) + to_q.head(dim) * t;
    bridge.push_back(q);
  }
  return bridge;
}

class GoalBridgeValidityChecker : public ::robot_sim::planner::StateValidityChecker {
public:
  GoalBridgeValidityChecker(const std::shared_ptr<::kinematics::KinematicChain> &chain,
                            const std::shared_ptr<GNGType> &gng,
                            bool avoid_danger)
      : chain_(chain), gng_(gng), avoid_danger_(avoid_danger) {}

  bool isValid(const Eigen::VectorXd &q) const override {
    if (!chain_ || !gng_) {
      return false;
    }

    std::vector<double> q_vec(q.data(), q.data() + q.size());
    if (!chain_->isWithinLimits(q_vec)) {
      return false;
    }

    const Eigen::VectorXf qf = q.cast<float>();

    int nearest_id = -1;
    float min_dist = std::numeric_limits<float>::max();
    gng_->forEachActiveValid([&](int id, const auto &node) {
      if (node.id == -1 || !node.status.active || !node.status.valid) {
        return;
      }
      if (node.status.is_colliding || (avoid_danger_ && node.status.is_danger)) {
        return;
      }
      const int dim = std::min(static_cast<int>(node.weight_angle.size()),
                               static_cast<int>(qf.size()));
      if (dim <= 0) {
        return;
      }
      const float dist = (node.weight_angle.head(dim) - qf.head(dim)).norm();
      if (dist < min_dist) {
        min_dist = dist;
        nearest_id = id;
      }
    });

    if (nearest_id < 0) {
      return false;
    }

    const auto &node = gng_->nodeAt(nearest_id);
    for (int neighbor_id : gng_->getNeighborsAngle(nearest_id)) {
      if (neighbor_id < 0 ||
          neighbor_id >= static_cast<int>(gng_->getMaxNodeNum())) {
        return false;
      }
      const auto &neighbor = gng_->nodeAt(neighbor_id);
      if (neighbor.id == -1 || !neighbor.status.active || !neighbor.status.valid ||
          neighbor.status.is_colliding ||
          (avoid_danger_ && neighbor.status.is_danger)) {
        return false;
      }
    }

    return node.id != -1 && node.status.active && node.status.valid &&
           !node.status.is_colliding &&
           (!avoid_danger_ || !node.status.is_danger);
  }

private:
  std::shared_ptr<::kinematics::KinematicChain> chain_;
  std::shared_ptr<GNGType> gng_;
  bool avoid_danger_ = true;
};

static inline std::vector<int> collectNearestGoalCandidates(
    const std::shared_ptr<GNGType> &gng, const std::vector<int> &cached_safe_goal_ids,
    bool trial_safe_only, const Eigen::Vector3f &reference_coord,
    int candidate_count) {
  std::vector<int> candidates;
  if (!gng) {
    return candidates;
  }

  struct CandidateDist {
    int id;
    float dist;
    bool operator<(const CandidateDist &other) const { return dist < other.dist; }
  };

  std::vector<CandidateDist> dist_candidates;
  const auto push_candidate = [&](int id, const auto &node) {
    if (id < 0) {
      return;
    }
    if (!node.status.active || !node.status.valid) {
      return;
    }
    if (node.status.is_colliding || node.status.is_danger) {
      return;
    }
    const int dim = std::min(static_cast<int>(node.weight_coord.size()), 3);
    const float dist = (node.weight_coord.head(dim) - reference_coord.head(dim)).norm();
    dist_candidates.push_back({id, dist});
  };

  if (trial_safe_only && !cached_safe_goal_ids.empty()) {
    for (int id : cached_safe_goal_ids) {
      if (id < 0 || id >= static_cast<int>(gng->getMaxNodeNum())) {
        continue;
      }
      push_candidate(id, gng->nodeAt(id));
    }
  } else {
    gng->forEachActiveValid([&](int id, const auto &node) { push_candidate(id, node); });
  }

  if (dist_candidates.empty()) {
    return candidates;
  }

  const int limit = std::min(candidate_count, static_cast<int>(dist_candidates.size()));
  std::partial_sort(dist_candidates.begin(), dist_candidates.begin() + limit,
                    dist_candidates.end());
  candidates.reserve(static_cast<std::size_t>(limit));
  for (int i = 0; i < limit; ++i) {
    candidates.push_back(dist_candidates[static_cast<std::size_t>(i)].id);
  }
  return candidates;
}

static inline std::vector<int> collectNearestStartCandidates(
    const std::shared_ptr<GNGType> &gng, const Eigen::VectorXf &reference_q,
    int candidate_count) {
  std::vector<int> candidates;
  if (!gng) {
    return candidates;
  }

  struct CandidateDist {
    int id;
    float dist;
    bool operator<(const CandidateDist &other) const { return dist < other.dist; }
  };

  std::vector<CandidateDist> dist_candidates;
  gng->forEachActiveValid([&](int id, const auto &node) {
    const int dim = std::min(static_cast<int>(node.weight_angle.size()),
                             static_cast<int>(reference_q.size()));
    if (dim <= 0) {
      return;
    }
    const float dist = (node.weight_angle.head(dim) - reference_q.head(dim)).norm();
    dist_candidates.push_back({id, dist});
  });

  if (dist_candidates.empty()) {
    return candidates;
  }

  const int limit = std::min(candidate_count, static_cast<int>(dist_candidates.size()));
  std::partial_sort(dist_candidates.begin(), dist_candidates.begin() + limit,
                    dist_candidates.end());
  candidates.reserve(static_cast<std::size_t>(limit));
  for (int i = 0; i < limit; ++i) {
    candidates.push_back(dist_candidates[static_cast<std::size_t>(i)].id);
  }
  return candidates;
}

static inline std::pair<int, std::vector<int>> planFromStartCandidates(
    const std::shared_ptr<GNGType> &gng, PlannerType &planner,
    const Eigen::VectorXf &current_q, const std::vector<int> &start_candidates,
    const std::vector<int> &goal_candidates, int &selected_start_id,
    std::unordered_map<int, std::vector<int>> &candidate_path_by_goal,
    std::vector<std::vector<int>> &candidate_paths) {
  selected_start_id = -1;
  candidate_path_by_goal.clear();
  candidate_paths.clear();

  if (!gng || start_candidates.empty() || goal_candidates.empty()) {
    return {-1, {}};
  }

  float best_score = std::numeric_limits<float>::max();
  int best_goal_id = -1;
  std::vector<int> best_path;

  for (int start_id : start_candidates) {
    if (start_id < 0 || start_id >= static_cast<int>(gng->getMaxNodeNum())) {
      continue;
    }
    const auto &start_node = gng->nodeAt(start_id);
    if (start_node.id == -1 || !start_node.status.active || !start_node.status.valid) {
      continue;
    }

    auto [reached_goal_id, node_path] = planner.planToAnyNode(start_id, goal_candidates, *gng);
    if (node_path.empty() || reached_goal_id < 0) {
      continue;
    }

    candidate_path_by_goal.emplace(reached_goal_id, node_path);

    const int dim = std::min(static_cast<int>(start_node.weight_angle.size()),
                             static_cast<int>(current_q.size()));
    const float start_dist =
        (start_node.weight_angle.head(dim) - current_q.head(dim)).norm();
    const float score = static_cast<float>(node_path.size()) + 0.5f * start_dist;
    if (score < best_score) {
      best_score = score;
      best_goal_id = reached_goal_id;
      best_path = node_path;
      selected_start_id = start_id;
    }
  }

  if (best_goal_id < 0) {
    return {-1, {}};
  }

  candidate_path_by_goal[best_goal_id] = best_path;

  for (int goal_id : goal_candidates) {
    if (goal_id == best_goal_id) {
      continue;
    }
    if (goal_id < 0 || goal_id >= static_cast<int>(gng->getMaxNodeNum())) {
      continue;
    }
    const auto &goal_node = gng->nodeAt(goal_id);
    if (goal_node.id == -1 || !goal_node.status.active || !goal_node.status.valid) {
      continue;
    }
    auto [reached_goal_id, path] =
        planner.planToAnyNode(selected_start_id, std::vector<int>{goal_id}, *gng);
    if (reached_goal_id >= 0 && !path.empty()) {
      candidate_path_by_goal.emplace(reached_goal_id, path);
      candidate_paths.push_back(std::move(path));
    }
  }

  return {best_goal_id, best_path};
}

static inline bool buildTrialGoalBridge(
    const std::shared_ptr<::kinematics::KinematicChain> &chain,
    const std::shared_ptr<GNGType> &gng, bool avoid_danger,
    const Eigen::VectorXf &start_q, const Eigen::Vector3f &goal_coord,
    std::vector<Eigen::VectorXf> &out_path) {
  out_path.clear();
  if (!chain || !gng) {
    return false;
  }

  GoalBridgeValidityChecker checker(chain, gng, avoid_danger);
  ::robot_sim::planner::RRTParams params;
  params.step_size = 0.02;
  params.max_iterations = 1200;
  params.max_planning_time_ms = 25.0;
  params.goal_pos_tolerance = 0.003;
  params.max_ik_samples = 6;
  params.max_ik_iterations = 40;

  const Eigen::VectorXd start_qd = start_q.cast<double>();
  const Eigen::Vector3d goal_pos = goal_coord.cast<double>();
  ::robot_sim::planner::IKRRTPlanner planner(*chain, params);
  const auto bridge = planner.plan(start_qd, goal_pos, checker, &params);
  if (bridge.empty()) {
    return false;
  }

  out_path.reserve(bridge.size());
  for (const auto &q : bridge) {
    out_path.push_back(q.cast<float>());
  }
  return true;
}

static inline ais_gng_msgs::msg::TopologicalMap buildPathMessage(
    rclcpp::Node &node, const std::shared_ptr<GNGType> &gng,
    const std::vector<std::vector<int>> &paths) {
  ais_gng_msgs::msg::TopologicalMap msg;
  msg.header.stamp = node.now();
  msg.header.frame_id = "world";
  msg.frame_number = 0;

  if (!gng) {
    return msg;
  }

  std::unordered_map<int, uint16_t> id_to_index;
  for (const auto &path : paths) {
    if (path.empty()) {
      continue;
    }
    for (int node_id : path) {
      if (node_id < 0 || node_id >= static_cast<int>(gng->getMaxNodeNum())) {
        continue;
      }
      const auto &node_ref = gng->nodeAt(node_id);
      if (node_ref.id == -1) {
        continue;
      }
      if (id_to_index.find(node_ref.id) != id_to_index.end()) {
        continue;
      }

      ais_gng_msgs::msg::TopologicalNode out;
      out.id = static_cast<uint16_t>(node_ref.id);
      out.pos.x = node_ref.weight_coord.x();
      out.pos.y = node_ref.weight_coord.y();
      out.pos.z = node_ref.weight_coord.z();
      out.normal.x = node_ref.status.ee_direction.x();
      out.normal.y = node_ref.status.ee_direction.y();
      out.normal.z = node_ref.status.ee_direction.z();
      out.label = pathLabelFromStatus(node_ref.status);
      out.frame = 0;

      const uint16_t published_index = static_cast<uint16_t>(msg.nodes.size());
      id_to_index.emplace(node_ref.id, published_index);
      msg.nodes.push_back(std::move(out));
    }

    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const auto ia = id_to_index.find(path[i]);
      const auto ib = id_to_index.find(path[i + 1]);
      if (ia == id_to_index.end() || ib == id_to_index.end()) {
        continue;
      }
      msg.edges.push_back(ia->second);
      msg.edges.push_back(ib->second);
    }
  }
  return msg;
}

} // namespace robot_sim::planning::topological_map_avoidance
