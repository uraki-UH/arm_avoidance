#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <unordered_map>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_feature_msgs/msg/topological_node_feature.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gng_control_msgs/msg/grasp_candidate_metric.hpp>
#include <gng_control_msgs/msg/grasp_candidate_metric_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "core/common/manipulability_serialization.hpp"

#include "planner/RRT/ik_rrt_planner.hpp"
#include "planner/RRT/rrt_params.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include "planning/gng_dijkstra_planner.hpp"
#include "planning/joint_linf_cost.hpp"
#include "core/kinematics/kinematic_chain.hpp"
#include "core/metrics/manipulability.hpp"
#include "gng/GrowingNeuralGas.hpp"

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

static inline float nanMetric() {
  return std::numeric_limits<float>::quiet_NaN();
}

static inline sensor_msgs::msg::JointState buildJointStateFromQ(
    const Eigen::VectorXf &q, const std::vector<std::string> &joint_names,
    const rclcpp::Time &stamp) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name = joint_names;
  msg.position.resize(joint_names.size(), 0.0);
  msg.velocity.resize(joint_names.size(), 0.0);
  msg.effort.resize(joint_names.size(), 0.0);
  const std::size_t n = std::min<std::size_t>(
      joint_names.size(), static_cast<std::size_t>(q.size()));
  for (std::size_t i = 0; i < n; ++i) {
    msg.position[i] = static_cast<double>(q[static_cast<int>(i)]);
  }
  return msg;
}

template <typename NodeType>
static inline geometry_msgs::msg::Pose buildPoseFromNode(const NodeType &node) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = static_cast<double>(node.weight_coord.x());
  pose.position.y = static_cast<double>(node.weight_coord.y());
  pose.position.z = static_cast<double>(node.weight_coord.z());
  pose.orientation.x = static_cast<double>(node.status.ee_orientation.x());
  pose.orientation.y = static_cast<double>(node.status.ee_orientation.y());
  pose.orientation.z = static_cast<double>(node.status.ee_orientation.z());
  pose.orientation.w = static_cast<double>(node.status.ee_orientation.w());
  return pose;
}

static inline Manipulability::ManipulabilityEllipsoid calculateManipulabilityForQ(
    const std::shared_ptr<::kinematics::KinematicChain> &chain,
    const Eigen::VectorXf &q, bool rotational) {
  Manipulability::ManipulabilityEllipsoid out;
  if (!chain || q.size() == 0) {
    return out;
  }

  std::vector<double> joint_values;
  joint_values.reserve(static_cast<std::size_t>(q.size()));
  for (int i = 0; i < q.size(); ++i) {
    joint_values.push_back(static_cast<double>(q[i]));
  }

  try {
    const Eigen::MatrixXd jacobian =
        chain->calculateJacobianAt(chain->getNumJoints() + 1, joint_values);
    if (jacobian.rows() < 3) {
      return out;
    }
    const Eigen::MatrixXd jacobian_part =
        rotational && jacobian.rows() >= 6 ? jacobian.bottomRows(3) : jacobian.topRows(3);
    out = Manipulability::calculateManipulabilityEllipsoid(jacobian_part);
  } catch (const std::exception &) {
    return out;
  }
  return out;
}

static inline std::vector<float> buildPathManipulability(
    const std::shared_ptr<GNGType> &gng,
    const std::shared_ptr<::kinematics::KinematicChain> &chain,
    const std::vector<int> &path, bool rotational) {
  std::vector<float> values;
  values.reserve(path.size());
  if (!gng) {
    return values;
  }
  for (const int node_id : path) {
    if (node_id < 0 || node_id >= static_cast<int>(gng->getMaxNodeNum())) {
      values.push_back(nanMetric());
      continue;
    }
    const auto &node = gng->nodeAt(node_id);
    if (node.id == -1 || node.weight_angle.size() == 0) {
      values.push_back(nanMetric());
      continue;
    }
    const auto manip = calculateManipulabilityForQ(chain, node.weight_angle, rotational);
    values.push_back(manip.valid ? static_cast<float>(manip.manipulability)
                                 : nanMetric());
  }
  return values;
}

static inline std::pair<float, float> estimatePathEnergyAndDuration(
    const std::shared_ptr<GNGType> &gng, const Eigen::VectorXf &current_q,
    const std::vector<int> &path, double max_joint_velocity) {
  if (!gng || path.empty()) {
    return {0.0f, 0.0f};
  }

  float energy = 0.0f;
  float duration = 0.0f;
  Eigen::VectorXf previous = current_q;
  const double clamped_velocity = std::max(1e-6, max_joint_velocity);

  for (const int node_id : path) {
    if (node_id < 0 || node_id >= static_cast<int>(gng->getMaxNodeNum())) {
      continue;
    }
    const auto &node = gng->nodeAt(node_id);
    if (node.id == -1 || node.weight_angle.size() != previous.size()) {
      continue;
    }

    const Eigen::VectorXf delta = node.weight_angle - previous;
    energy += static_cast<float>(delta.squaredNorm());
    duration += static_cast<float>(delta.cwiseAbs().maxCoeff() / clamped_velocity);
    previous = node.weight_angle;
  }

  return {energy, duration};
}

static inline gng_control_msgs::msg::GraspCandidateMetricArray
buildGraspCandidateMetricArray(
    const rclcpp::Time &stamp, const std::string &frame_id,
    const std::string &robot_name, const std::string &base_frame,
    int selected_goal_node_id, const Eigen::VectorXf &current_q, int start_id,
    const std::vector<int> &goal_candidates,
    const std::unordered_map<int, std::vector<int>> &candidate_path_by_goal,
    const std::shared_ptr<GNGType> &gng,
    const std::shared_ptr<::kinematics::KinematicChain> &chain,
    const std::vector<std::string> &controlled_joint_names,
    double max_joint_velocity) {
  gng_control_msgs::msg::GraspCandidateMetricArray out;
  out.header.stamp = stamp;
  out.header.frame_id = frame_id;
  out.robot_name = robot_name;
  out.base_frame = base_frame;
  out.selected_goal_node_id = selected_goal_node_id;
  out.candidates.reserve(goal_candidates.size());

  if (!gng) {
    return out;
  }

  for (std::size_t i = 0; i < goal_candidates.size(); ++i) {
    const int goal_id = goal_candidates[i];
    gng_control_msgs::msg::GraspCandidateMetric metric;
    metric.candidate_id = static_cast<int32_t>(i);
    metric.goal_node_id = goal_id;
    metric.start_node_id = start_id;
    metric.selected = goal_id == selected_goal_node_id;
    metric.feasible = candidate_path_by_goal.find(goal_id) != candidate_path_by_goal.end();

    metric.position_manipulability = nanMetric();
    metric.rotation_manipulability = nanMetric();
    metric.manipulability_condition_number = nanMetric();
    metric.min_singular_value = nanMetric();
    metric.joint_limit_margin_min = nanMetric();
    metric.joint_limit_margin_mean = nanMetric();
    metric.self_collision_margin = nanMetric();
    metric.environment_collision_margin = nanMetric();
    metric.gripper_width = nanMetric();
    metric.grasp_region_score = nanMetric();
    metric.estimated_energy = nanMetric();
    metric.estimated_duration = nanMetric();

    if (goal_id >= 0 && goal_id < static_cast<int>(gng->getMaxNodeNum())) {
      const auto &node = gng->nodeAt(goal_id);
      if (node.id != -1) {
        metric.end_effector_pose = buildPoseFromNode(node);
        metric.final_joint_state =
            buildJointStateFromQ(node.weight_angle, controlled_joint_names, stamp);
        metric.joint_limit_margin_min = node.status.joint_limit_score;
        metric.joint_limit_margin_mean = node.status.joint_limit_score;

        const auto position_manip =
            node.status.manip_info.valid
                ? node.status.manip_info
                : calculateManipulabilityForQ(chain, node.weight_angle, false);
        const auto rotation_manip =
            calculateManipulabilityForQ(chain, node.weight_angle, true);

        if (position_manip.valid) {
          metric.position_manipulability =
              static_cast<float>(position_manip.manipulability);
          metric.manipulability_condition_number =
              static_cast<float>(position_manip.condition_number);
          metric.min_singular_value =
              static_cast<float>(position_manip.min_singular_value);
          metric.manipulability_singular_values = {
              static_cast<float>(position_manip.singular_values.x()),
              static_cast<float>(position_manip.singular_values.y()),
              static_cast<float>(position_manip.singular_values.z())};
        }
        if (rotation_manip.valid) {
          metric.rotation_manipulability =
              static_cast<float>(rotation_manip.manipulability);
        }

        metric.metric_names = {"dynamic_manipulability"};
        metric.metric_values = {node.status.dynamic_manipulability};
      }
    }

    const auto path_it = candidate_path_by_goal.find(goal_id);
    if (path_it != candidate_path_by_goal.end()) {
      metric.path_node_ids.reserve(path_it->second.size());
      for (const int node_id : path_it->second) {
        metric.path_node_ids.push_back(static_cast<int32_t>(node_id));
      }
      metric.path_position_manipulability =
          buildPathManipulability(gng, chain, path_it->second, false);
      metric.path_rotation_manipulability =
          buildPathManipulability(gng, chain, path_it->second, true);
      const auto [energy, duration] =
          estimatePathEnergyAndDuration(gng, current_q, path_it->second, max_joint_velocity);
      metric.estimated_energy = energy;
      metric.estimated_duration = duration;
    }

    out.candidates.push_back(std::move(metric));
  }

  return out;
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
      if (node.id == -1 || !node.status.active || !node.status.self_collision_free) {
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
      if (neighbor.id == -1 || !neighbor.status.active || !neighbor.status.self_collision_free ||
          neighbor.status.is_colliding ||
          (avoid_danger_ && neighbor.status.is_danger)) {
        return false;
      }
    }

    return node.id != -1 && node.status.active && node.status.self_collision_free &&
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
    bool trial_safe_only, bool allow_danger_goal,
    const Eigen::Vector3f &reference_coord, int candidate_count) {
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
    if (!node.status.active || !node.status.self_collision_free) {
      return;
    }
    if (node.status.is_colliding) {
      return;
    }
    if (!trial_safe_only && node.status.is_danger && !allow_danger_goal) {
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

static inline bool goalCoordinateReached(
    const std::shared_ptr<::kinematics::KinematicChain> &chain,
    bool goal_coord_valid, const Eigen::Vector3f &goal_coord,
    const Eigen::VectorXf &current_q, double waypoint_tolerance) {
  if (!goal_coord_valid || !chain) {
    return false;
  }

  std::vector<double> current_fk_q;
  current_fk_q.reserve(static_cast<std::size_t>(current_q.size()));
  for (int i = 0; i < current_q.size(); ++i) {
    current_fk_q.push_back(static_cast<double>(current_q[i]));
  }

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
  chain->forwardKinematicsAt(current_fk_q, positions, orientations);
  if (positions.empty()) {
    return false;
  }

  const Eigen::Vector3d current_eef_pos = positions.back();
  const Eigen::Vector3d goal_pos = goal_coord.cast<double>();
  const double pos_err = (current_eef_pos - goal_pos).norm();
  return pos_err <= waypoint_tolerance;
}

static inline int pickRandomGoal(
    const std::shared_ptr<GNGType> &gng,
    const std::vector<int> &cached_safe_goal_ids,
    bool trial_safe_only, int start_id, std::mt19937 &rng) {
  std::vector<int> candidates;
  if (trial_safe_only && !cached_safe_goal_ids.empty()) {
    candidates = cached_safe_goal_ids;
  } else if (gng) {
    candidates.reserve(gng->getMaxNodeNum());
    gng->forEachActiveValid([&](int id, const auto &node) {
      (void)node;
      candidates.push_back(id);
    });
  }

  candidates.erase(
      std::remove(candidates.begin(), candidates.end(), start_id),
      candidates.end());
  if (candidates.empty()) {
    return -1;
  }

  std::uniform_int_distribution<std::size_t> dist(0, candidates.size() - 1);
  return candidates[dist(rng)];
}

static inline std::optional<Eigen::Vector3f> selectTrialGoalCoord(
    const std::shared_ptr<GNGType> &gng,
    bool trial_return_home, bool return_home_phase,
    bool trial_safe_only, int start_id,
    const std::vector<int> &cached_safe_goal_ids,
    Eigen::Vector3f &home_coord, bool &home_coord_valid,
    std::mt19937 &rng) {
  if (!gng) {
    return std::nullopt;
  }

  if (trial_return_home && return_home_phase) {
    if (!home_coord_valid) {
      if (start_id < 0 || start_id >= static_cast<int>(gng->getMaxNodeNum())) {
        return std::nullopt;
      }
      const auto &start_node = gng->nodeAt(start_id);
      if (start_node.id == -1) {
        return std::nullopt;
      }
      home_coord = start_node.weight_coord;
      home_coord_valid = true;
    }
    return home_coord;
  }

  const int random_goal_id = pickRandomGoal(
      gng, cached_safe_goal_ids, trial_safe_only, start_id, rng);
  if (random_goal_id < 0 || random_goal_id >= static_cast<int>(gng->getMaxNodeNum())) {
    return std::nullopt;
  }
  const auto &goal_node = gng->nodeAt(random_goal_id);
  if (goal_node.id == -1) {
    return std::nullopt;
  }
  return goal_node.weight_coord;
}

struct WaypointSafetyLookahead {
  int next_node_id = -1;
  int next_next_node_id = -1;
  bool next_unsafe = false;
  bool next_next_unsafe = false;
};

static inline bool isUnsafeNode(
    const std::shared_ptr<GNGType> &gng, int node_id, bool avoid_danger) {
  if (!gng || node_id < 0 || node_id >= static_cast<int>(gng->getMaxNodeNum())) {
    return true;
  }

  const auto &node = gng->nodeAt(node_id);
  return node.id == -1 || !node.status.active || !node.status.self_collision_free ||
         node.status.is_colliding || (avoid_danger && node.status.is_danger);
}

static inline WaypointSafetyLookahead inspectWaypointSafetyLookahead(
    const std::shared_ptr<GNGType> &gng, const std::vector<int> &node_path,
    std::size_t waypoint_index, bool avoid_danger) {
  WaypointSafetyLookahead out;
  const bool has_next = waypoint_index + 1 < node_path.size();
  const bool has_next_next = waypoint_index + 2 < node_path.size();
  out.next_node_id = has_next ? node_path[waypoint_index + 1] : -1;
  out.next_next_node_id = has_next_next ? node_path[waypoint_index + 2] : -1;
  out.next_unsafe = has_next && isUnsafeNode(gng, out.next_node_id, avoid_danger);
  out.next_next_unsafe =
      has_next_next && isUnsafeNode(gng, out.next_next_node_id, avoid_danger);
  return out;
}

static inline std::pair<int, std::vector<int>> planFromStartCandidates(
    const std::shared_ptr<GNGType> &gng, PlannerType &planner,
    const Eigen::VectorXf &current_q, const std::vector<int> &start_candidates,
    const std::vector<int> &goal_candidates, int &selected_start_id,
    std::unordered_map<int, std::vector<int>> &candidate_path_by_goal,
    std::vector<std::vector<int>> &candidate_paths,
    bool allow_danger_goal,
    float goal_rot_manip_weight = 0.0f,
    float goal_joint_limit_weight = 0.0f) {
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
    if (start_node.id == -1 || !start_node.status.active || !start_node.status.self_collision_free) {
      continue;
    }

    auto [reached_goal_id, node_path] =
        planner.planToAnyNode(start_id, goal_candidates, *gng, allow_danger_goal);
    if (node_path.empty() || reached_goal_id < 0) {
      continue;
    }

    candidate_path_by_goal.emplace(reached_goal_id, node_path);

    const int dim = std::min(static_cast<int>(start_node.weight_angle.size()),
                             static_cast<int>(current_q.size()));
    const float start_dist =
        (start_node.weight_angle.head(dim) - current_q.head(dim)).norm();
    float score = static_cast<float>(node_path.size()) + 0.5f * start_dist;

    // Rotational manipulability penalty at goal: high condition number = near wrist singularity.
    // log scale keeps penalty comparable to path length (log(1)=0, log(10)≈2.3, log(100)≈4.6).
    if (goal_rot_manip_weight > 0.0f &&
        reached_goal_id >= 0 && reached_goal_id < static_cast<int>(gng->getMaxNodeNum())) {
      const auto &goal_node = gng->nodeAt(reached_goal_id);
      const auto &rot = goal_node.status.rotational_manip_info;
      if (rot.valid && rot.manipulability > 1e-8) {
        const float cond = std::max(1.0f, static_cast<float>(rot.condition_number));
        score += goal_rot_manip_weight * std::log(cond);
      } else {
        score += goal_rot_manip_weight * std::log(100.0f); // max penalty for invalid/singular
      }
      // Joint limit margin bonus: higher margin = more slack = lower score
      if (goal_joint_limit_weight > 0.0f) {
        const float margin = std::clamp(goal_node.status.joint_limit_score, 0.0f, 1.0f);
        score -= goal_joint_limit_weight * margin;
      }
    }

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
    if (goal_node.id == -1 || !goal_node.status.active || !goal_node.status.self_collision_free) {
      continue;
    }
    auto [reached_goal_id, path] =
        planner.planToAnyNode(selected_start_id, std::vector<int>{goal_id}, *gng,
                              allow_danger_goal);
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
    const std::vector<std::vector<int>> &paths,
    const std::string &frame_id) {
  ais_gng_msgs::msg::TopologicalMap msg;
  msg.header.stamp = node.now();
  msg.header.frame_id = frame_id.empty() ? "world" : frame_id;
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
      ais_gng_feature_msgs::msg::TopologicalNodeFeature feature;
      feature.node_id = static_cast<uint16_t>(node_ref.id);
      feature.is_goal = (!path.empty() && node_ref.id == path.back());
      robot_sim::common::fillManipulabilityFields(feature, node_ref.status.manip_info, feature.is_goal);
      robot_sim::common::fillNodeKinematicsFields(
          feature, node_ref.weight_angle,
          node_ref.weight_coord, node_ref.status.ee_orientation);

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

template <typename GNGType>
static inline ais_gng_msgs::msg::TopologicalMap buildPathMessageWithCurrentPose(
    rclcpp::Node &node, const std::shared_ptr<GNGType> &gng,
    const Eigen::VectorXf &current_q,
    const std::shared_ptr<::kinematics::KinematicChain> &chain,
    const std::vector<std::vector<int>> &paths,
    const std::string &frame_id) {
  auto msg = buildPathMessage(node, gng, paths, frame_id);
  if (!gng || !chain || current_q.size() == 0) {
    return msg;
  }

  std::vector<double> joint_values;
  joint_values.reserve(static_cast<std::size_t>(current_q.size()));
  for (int i = 0; i < current_q.size(); ++i) {
    joint_values.push_back(static_cast<double>(current_q[i]));
  }

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
  try {
    chain->forwardKinematicsAt(joint_values, positions, orientations);
  } catch (const std::exception &) {
    return msg;
  }
  if (positions.empty() || orientations.empty()) {
    return msg;
  }

  constexpr uint16_t kCurrentPoseNodeId = std::numeric_limits<uint16_t>::max();
  const auto virtual_it = std::find_if(
      msg.nodes.begin(), msg.nodes.end(),
      [](const ais_gng_msgs::msg::TopologicalNode &node_msg) {
        return node_msg.id == kCurrentPoseNodeId;
      });
  if (virtual_it == msg.nodes.end()) {
    ais_gng_msgs::msg::TopologicalNode current_node;
    current_node.id = kCurrentPoseNodeId;
    current_node.pos.x = positions.back().x();
    current_node.pos.y = positions.back().y();
    current_node.pos.z = positions.back().z();
    const Eigen::Vector3d forward = orientations.back() * Eigen::Vector3d::UnitZ();
    current_node.normal.x = forward.x();
    current_node.normal.y = forward.y();
    current_node.normal.z = forward.z();
    current_node.label = 1;
    current_node.rho = 0.0f;
    current_node.semantic_label = 0;
    current_node.semantic_reliability = 0.0f;
    current_node.frame = 0;
    current_node.winner_point_count = 0;
    msg.nodes.insert(msg.nodes.begin(), current_node);
    for (std::size_t i = 0; i < msg.edges.size(); ++i) {
      msg.edges[i] = static_cast<uint16_t>(msg.edges[i] + 1);
    }
  }

  for (const auto &path : paths) {
    if (path.empty()) {
      continue;
    }
    const auto first_idx = std::find_if(
        msg.nodes.begin(), msg.nodes.end(),
        [&](const ais_gng_msgs::msg::TopologicalNode &node_msg) {
          return !path.empty() && node_msg.id == static_cast<uint16_t>(path.front());
        });
    if (first_idx == msg.nodes.end()) {
      continue;
    }
    const auto first_index = static_cast<uint16_t>(std::distance(msg.nodes.begin(), first_idx));
    msg.edges.push_back(0);
    msg.edges.push_back(first_index);
  }
  return msg;
}

} // namespace robot_sim::planning::topological_map_avoidance
