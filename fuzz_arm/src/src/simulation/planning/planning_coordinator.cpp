#include "simulation/planning/planning_coordinator.hpp"
#include "control/linear_trajectory_controller.hpp"
#include "gng/GrowingNeuralGas_offline2.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "planner/RRT/ik_rrt_planner.hpp"
#include "planning/advanced_costs.hpp"
#include "planning/gng_dijkstra_planner.hpp"
#include "planning/joint_distance_cost.hpp"
#include "planning/joint_linf_cost.hpp"
#include "planning/workspace_distance_cost.hpp"
#include "simulation/core/simulation_state.hpp"
#include "simulation/planning/ode_state_validity_checker.hpp"
#include "simulation/planning/scenario_manager.hpp"
#include "simulation/reactive_controller.hpp"
#include "simulation/robot/ode/ode_robot_sim.hpp"
#include "simulation/robot/robot_model.hpp"
#include "simulation/safety/influence_management.hpp"
#include "simulation/safety/risk_aware_cost_evaluator.hpp"
#include "simulation/world/dynamic_obstacle_manager.hpp"

#include <cstdlib>

// Redundant includes removed

#include "common/config_manager.hpp"
#include "common/resource_utils.hpp"
#include "simulation/safety/influence_management.hpp"
#include "simulation/world/dynamic_obstacle_manager.hpp"
#include "spatial/dense_spatial_index.hpp"
#include "spatial/sparse_spatial_index.hpp"


namespace robot_sim {
namespace simulation {

PlanningCoordinator::PlanningCoordinator() = default;

PlanningCoordinator::~PlanningCoordinator() = default;

bool PlanningCoordinator::initializeAnalysisSystem(SimulationState &state) {

  // Get Config Singleton
  auto &config = ::common::ConfigManager::Instance();

  // Load Parameters into Config Struct
  AnalysisSystemConfig params;
  params.sensing_resolution =
      config.GetDouble("sensing_resolution", params.sensing_resolution);
  params.spatial_map_resolution =
      config.GetDouble("spatial_map_resolution", params.spatial_map_resolution);
  params.spatial_index_type =
      config.Get("spatial_index_type", params.spatial_index_type);
  params.spatial_correlation_file =
      config.Get("spatial_correlation_file", params.spatial_correlation_file);
  params.obstacle_radius =
      config.GetDouble("obstacle_radius", params.obstacle_radius);

  params.min_x = config.GetDouble("spatial_map_min_x", params.min_x);
  params.min_y = config.GetDouble("spatial_map_min_y", params.min_y);
  params.min_z = config.GetDouble("spatial_map_min_z", params.min_z);
  params.max_x = config.GetDouble("spatial_map_max_x", params.max_x);
  params.max_y = config.GetDouble("spatial_map_max_y", params.max_y);
  params.max_z = config.GetDouble("spatial_map_max_z", params.max_z);

  Eigen::Vector3d world_min(params.min_x, params.min_y, params.min_z);
  Eigen::Vector3d world_max(params.max_x, params.max_y, params.max_z);

  std::vector<double> min_bounds = {params.min_x, params.min_y, params.min_z};

  if (params.spatial_index_type == "dense") {
    state.spatial_index_ptr =
        std::make_shared<robot_sim::analysis::DenseSpatialIndex>(
            params.spatial_map_resolution, world_min, world_max);
  } else {
    state.spatial_index_ptr =
        std::make_shared<robot_sim::analysis::SparseSpatialIndex>(
            params.spatial_map_resolution);
  }

  if (state.spatial_index_ptr->load(
          robot_sim::common::resolvePath(params.spatial_correlation_file))) {

    state.obstacle_manager_ptr =
        std::make_unique<robot_sim::simulation::DynamicObstacleManager>(
            state.spatial_index_ptr, params.sensing_resolution,
            params.spatial_map_resolution, min_bounds);

    // --- Influence Manager Initialization ---
    state.influence_manager_ptr =
        std::make_unique<robot_sim::simulation::InfluenceManager>();

    state.scenario_manager_ptr =
        std::make_shared<robot_sim::simulation::ScenarioManager>(
            state.obstacle_manager_ptr.get());

    // Update demo obstacle radius from config
    state.demo_obstacle_radius = params.obstacle_radius;
    state.scenario_manager_ptr->setObstacleRadius(state.demo_obstacle_radius);

    // AUTO-SETUP: Start in IDLE mode
    state.scenario_manager_ptr->setScenario(
        robot_sim::simulation::ScenarioType::IDLE);

    return true;
  }

  std::cerr << "[Init] Failed to load spatial index." << std::endl;
  return false;
}

bool PlanningCoordinator::initializePlanningSystem(SimulationState &state) {
  auto &config = ::common::ConfigManager::Instance();
  PlanningConfig params;

  // Load Params
  params.rrt_goal_tolerance =
      config.GetDouble("rrt_goal_tolerance", params.rrt_goal_tolerance);
  params.rrt_max_ik_iterations =
      config.GetInt("rrt_max_ik_iterations", params.rrt_max_ik_iterations);
  params.rrt_step_size =
      config.GetDouble("rrt_step_size", params.rrt_step_size);
  params.rrt_max_ik_samples =
      config.GetInt("rrt_max_ik_samples", params.rrt_max_ik_samples);
  params.rrt_max_planning_time_ms = config.GetDouble(
      "rrt_max_planning_time_ms", params.rrt_max_planning_time_ms);
  params.cost_metric = config.Get("planning_cost_metric", params.cost_metric);

  // 1. Initialize Validity Checker if not already
  // (Assuming ValidityChecker is initialized by EnvironmentManager or Main for
  // now,
  //  as it requires many dependencies not yet fully correctly reachable here
  //  without more refactor)

  // 2. Initialize GNG Planner
  state.gng_planner_ptr = std::make_unique<planning::GngDijkstraPlanner<
      Eigen::VectorXf, Eigen::Vector3f,
      GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f>>>();

  // 3. Initialize RRT Planner
  robot_sim::planner::RRTParams rrt_params;
  rrt_params.goal_pos_tolerance = params.rrt_goal_tolerance;
  rrt_params.max_ik_iterations = params.rrt_max_ik_iterations;
  rrt_params.step_size = params.rrt_step_size;
  rrt_params.max_ik_samples = params.rrt_max_ik_samples;
  rrt_params.max_planning_time_ms = params.rrt_max_planning_time_ms;
  state.global_rrt_params = rrt_params;

  if (state.kinematic_chain_ptr) {
    state.rrt_planner_ptr = std::make_unique<robot_sim::planner::IKRRTPlanner>(
        *state.kinematic_chain_ptr, rrt_params);
  } else {
    std::cerr
        << "[Init] Warning: Kinematic Chain not available for RRT Planner."
        << std::endl;
  }

  // 4. Initialize Risk Evaluator
  if (state.safety_state_manager_ptr) {
    auto cost_evaluator =
        std::make_shared<robot_sim::simulation::RiskAwareCostEvaluator<
            Eigen::VectorXf, Eigen::Vector3f>>(state.safety_state_manager_ptr);
    state.gng_planner_ptr->setCostEvaluator(cost_evaluator);
  }

  return true;
}

bool PlanningCoordinator::initializeControlSystem(SimulationState &state) {
  // 1. Initialize Linear Trajectory Controller
  auto &config = ::common::ConfigManager::Instance();
  double speed_scale = config.GetDouble("speed_scale", 1.0);
  state.controller_ptr =
      std::make_unique<::control::LinearTrajectoryController>(speed_scale);

  // 2. Initialize Reactive Controller
  state.reactive_controller_ptr =
      std::make_unique<robot_sim::simulation::ReactiveController>();

  // 3. Configure Controller with Joint Limits
  if (state.kinematic_chain_ptr && state.robot_model_ptr) {
    // Derive target joints from Kinematic Chain
    std::vector<std::string> target_joints;
    int num_joints = state.kinematic_chain_ptr->getNumJoints();
    for (int i = 0; i < num_joints; ++i) {
      // Assuming getJointDOF is available or we check names
      // Based on main.cpp logic:
      if (state.kinematic_chain_ptr->getJointDOF(i) > 0) {
        target_joints.push_back(state.kinematic_chain_ptr->getLinkName(i));
      }
    }

    if (!target_joints.empty()) {
      state.controller_ptr->setTargetJoints(target_joints);

      std::vector<double> max_vels;
      for (const auto &name : target_joints) {
        const auto *joint_props = state.robot_model_ptr->getJoint(name);
        double v_limit = 1.0;
        if (joint_props && joint_props->limits.velocity > 0) {
          v_limit = joint_props->limits.velocity;
        }
        max_vels.push_back(v_limit);
      }
      state.controller_ptr->setMaxVelocities(max_vels);
    } else {
      std::cerr << "[Init] Warning: No movable joints detected for controller."
                << std::endl;
    }
  } else {
    std::cerr << "[Init] Warning: Chain or Model missing for Control Init."
              << std::endl;
  }

  return true;
}

void PlanningCoordinator::planAndExecute(const Eigen::VectorXf &start_q,
                                         const Eigen::VectorXf &goal_q,
                                         SimulationState &state) {
  state.active_path.clear();
  state.is_executing_path = false;

  if (state.getPlanningMethod() == PlanningMethod::GNG_DIJKSTRA) {
    if (!state.gng_ptr || !state.gng_planner_ptr) {
      std::cerr << "[Planning] Error: GNG or Planner not ready." << std::endl;
      return;
    }

    // [Fix] Sync collision avoidance settings to planner
    state.gng_planner_ptr->setAvoidCollisions(state.avoid_collision_mode);
    state.gng_planner_ptr->setStrictGoalCollisionCheck(
        state.strict_goal_collision_check);

    // GNG Dijkstra
    std::vector<Eigen::VectorXf> path;

    if (state.gng_use_multi_target) {
      // --- Multi-Target Planning ---
      // 1. Find candidates near goal
      // Manual nearest neighbor for start (since planner->findNearestNode is
      // private)
      int start_id = -1;
      float min_dist_start = 1e10f;

      // Start Node Search
      state.gng_ptr->forEachActiveValid([&](int i, const auto &n) {
        int dim = std::min((int)n.weight_angle.size(), (int)start_q.size());
        float d = (n.weight_angle.head(dim) - start_q.head(dim)).norm();
        if (d < min_dist_start) {
          min_dist_start = d;
          start_id = i;
        }
      });

      struct NodeDist {
        int id;
        float dist;
      };
      std::vector<NodeDist> candidates;

      // Goal Candidates Search
      state.gng_ptr->forEachActiveValid([&](int i, const auto &n) {
        int dim = std::min((int)n.weight_angle.size(), (int)goal_q.size());
        float d = (n.weight_angle.head(dim) - goal_q.head(dim)).norm();
        candidates.push_back({n.id, d});
      });
      // Sort
      std::sort(
          candidates.begin(), candidates.end(),
          [](const NodeDist &a, const NodeDist &b) { return a.dist < b.dist; });

      // Pick top N
      std::vector<int> goal_ids;
      int count =
          std::min((int)candidates.size(), state.gng_multi_target_count);
      for (int i = 0; i < count; ++i)
        goal_ids.push_back(candidates[i].id);

      // Plan
      if (start_id != -1 && !goal_ids.empty()) {
        auto result = state.gng_planner_ptr->planToAnyNode(start_id, goal_ids,
                                                           *state.gng_ptr);
        int reached_goal = result.first;
        const auto &indices = result.second;

        if (reached_goal != -1 && !indices.empty()) {
          // Convert indices to path
          for (int idx : indices) {
            path.push_back(state.gng_ptr->nodeAt(idx).weight_angle);
          }
          // Append actual goal
          int dim = std::min((int)path.back().size(), (int)goal_q.size());
          if ((path.back().head(dim) - goal_q.head(dim)).norm() > 1e-4) {
            path.push_back(goal_q);
          }
          // reached_goal not printed anymore
        }
      }

    } else {
      // --- Single-Target (Original) ---
      path = state.gng_planner_ptr->plan(start_q, goal_q, *state.gng_ptr);
    }

    if (!path.empty()) {
      for (const auto &q : path) {
        state.active_path.push_back(q.cast<double>());
      }
      state.is_executing_path = true;

      // --- コントローラーへのパス供給と自動モードの有効化 ---
      std::vector<Eigen::VectorXd> path_vec(state.active_path.begin(),
                                            state.active_path.end());
      if (state.controller_ptr) {
        state.controller_ptr->setPath(path_vec);
        state.auto_mode = true;
      }

      // Path found log removed
    } else {
      // Failure log removed
    }

  } else if (state.getPlanningMethod() == PlanningMethod::IK_RRT_CONNECT) {
    // RRT Connect
    if (!state.fk_chain_ptr || !state.rrt_planner_ptr ||
        !state.validity_checker_ptr) {
      std::cerr << "[Planning] Error: RRT components not ready." << std::endl;
      return;
    }

    // Plan: If goal_q is non-zero, it likely means we have a direct joint
    // target. In that case, bypass Cartesian conversion and IK sampling for
    // maximum precision.
    Eigen::VectorXd start_q_d = start_q.cast<double>();
    Eigen::VectorXd goal_q_d = goal_q.cast<double>();
    std::vector<Eigen::VectorXd> path;

    if (goal_q_d.norm() > 1e-6) {
      // Direct joint-to-joint planning
      std::vector<Eigen::VectorXd> goals = {goal_q_d};
      path = state.rrt_planner_ptr->plan(start_q_d, goals,
                                         *state.validity_checker_ptr);
    } else {
      // Cartesian goal planning (fallback or if goal_q is zero)
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          positions;
      std::vector<Eigen::Quaterniond,
                  Eigen::aligned_allocator<Eigen::Quaterniond>>
          orientations;
      state.fk_chain_ptr->forwardKinematicsAt(goal_q_d, positions,
                                              orientations);

      if (!positions.empty()) {
        Eigen::Vector3d target_pos = positions.back();
        path = state.rrt_planner_ptr->plan(start_q_d, target_pos,
                                           *state.validity_checker_ptr);
      }
    }

    if (!path.empty()) {
      for (const auto &q : path) {
        state.active_path.push_back(q);
      }
      state.is_executing_path = true;

      // --- コントローラーへのパス供給と自動モードの有効化 ---
      std::vector<Eigen::VectorXd> path_vec(state.active_path.begin(),
                                            state.active_path.end());
      if (state.controller_ptr) {
        state.controller_ptr->setPath(path_vec);
        state.auto_mode = true;
      }

      // Path found log and stats removed
    }
  } else {
    // Failure log and stats removed
  }
}

void PlanningCoordinator::pickRandomPosture(SimulationState &state) {
  if (!state.robot_hal_ptr || !state.fk_chain_ptr || !state.robot_model_ptr) {
    std::cerr << "[Planning] Error: Robot components not ready." << std::endl;
    return;
  }

  Eigen::VectorXf random_q(state.robot_hal_ptr->getJointPositions().size());
  for (int i = 0; i < random_q.size(); ++i) {
    std::string joint_name = state.robot_hal_ptr->getJointNames()[i];
    const ::simulation::JointProperties *joint_props =
        state.robot_model_ptr->getJoint(joint_name);
    if (joint_props && joint_props->has_limits) {
      random_q[i] = joint_props->limits.lower +
                    (static_cast<float>(rand()) / RAND_MAX) *
                        (joint_props->limits.upper - joint_props->limits.lower);
    } else {
      random_q[i] =
          (static_cast<float>(rand()) / RAND_MAX) * 2.0f * M_PI - M_PI;
    }
  }
  state.global_target_goal_q = random_q;

  int total_dof = state.fk_chain_ptr->getTotalDOF();
  state.fk_chain_ptr->updateKinematics(
      random_q.head(std::min((int)random_q.size(), total_dof)));
  Eigen::Vector3f pos = state.fk_chain_ptr->getEEFPosition().cast<float>();
  state.target_eef_pos = pos.cast<double>();
  state.target_sphere_pos = pos.cast<double>();
  state.target_sphere_quat = state.fk_chain_ptr->getEEFOrientation();
  state.show_target_axes = true;

  // Sync with ScenarioManager and stop any movement
  if (state.scenario_manager_ptr) {
    state.scenario_manager_ptr->setTargetPosition(state.target_sphere_pos);
    state.scenario_manager_ptr->setScenario(
        robot_sim::simulation::ScenarioType::IDLE);
  }
}

void PlanningCoordinator::pickRandomGngNode(SimulationState &state) {
  if (!state.gng_ptr || state.gng_ptr->getMaxNodeNum() == 0) {
    std::cerr << "[Planning] Error: GNG not ready or empty." << std::endl;
    return;
  }

  int node_id = selectRandomActiveNode(state);
  if (node_id < 0) {
    std::cerr << "[Planning] No active nodes found." << std::endl;
    return;
  }

  const auto &node = state.gng_ptr->nodeAt(node_id);
  state.global_target_goal_q = node.weight_angle;
  state.node_cartesian_goal = node.weight_coord.cast<double>();
  state.target_node_id = node_id;

  // Update target sphere position
  state.target_sphere_pos = node.weight_coord.cast<double>();
  if (state.fk_chain_ptr) {
    int total_dof = state.fk_chain_ptr->getTotalDOF();
    state.fk_chain_ptr->updateKinematics(node.weight_angle.head(
        std::min((int)node.weight_angle.size(), total_dof)));
    state.target_sphere_quat = state.fk_chain_ptr->getEEFOrientation();
  }
  state.show_target_axes = true;

  // Sync with ScenarioManager
  if (state.scenario_manager_ptr) {
    state.scenario_manager_ptr->setTargetPosition(state.target_sphere_pos);
    state.scenario_manager_ptr->setScenario(
        robot_sim::simulation::ScenarioType::IDLE);
  }
}

void PlanningCoordinator::switchPlanningMethod(PlanningMethod method,
                                               SimulationState &state) {
  state.current_planning_method = method;
}

void PlanningCoordinator::setCostMetric(
    PlanningMetric metric, SimulationState &state,
    std::shared_ptr<planning::ICostEvaluator<Eigen::VectorXf, Eigen::Vector3f>>
        risk_evaluator) {

  if (!state.gng_planner_ptr) {
    return;
  }

  state.current_metric = metric;

  switch (metric) {
  case PlanningMetric::DISTANCE:
    state.gng_planner_ptr->setCostEvaluator(
        std::make_shared<
            planning::JointDistanceCost<Eigen::VectorXf, Eigen::Vector3f>>());
    break;
  case PlanningMetric::DISTANCE_L_INF:
    state.gng_planner_ptr->setCostEvaluator(
        std::make_shared<
            planning::JointLInfCost<Eigen::VectorXf, Eigen::Vector3f>>());
    break;
  case PlanningMetric::KINEMATIC_MANIPULABILITY:
    state.gng_planner_ptr->setCostEvaluator(
        std::make_shared<planning::KinematicManipulabilityCost<
            Eigen::VectorXf, Eigen::Vector3f>>());
    break;
  case PlanningMetric::DYNAMIC_MANIPULABILITY:
    state.gng_planner_ptr->setCostEvaluator(
        std::make_shared<planning::DynamicManipulabilityCost<
            Eigen::VectorXf, Eigen::Vector3f>>());
    break;
  case PlanningMetric::DIRECTIONAL_MANIPULABILITY:
    state.gng_planner_ptr->setCostEvaluator(
        std::make_shared<planning::DirectionalManipulabilityCost<
            Eigen::VectorXf, Eigen::Vector3f>>());
    break;
  case PlanningMetric::EEF_DISTANCE:
    state.gng_planner_ptr->setCostEvaluator(
        std::make_shared<planning::WorkspaceDistanceCost<Eigen::VectorXf,
                                                         Eigen::Vector3f>>());
    break;
  case PlanningMetric::RISK:
    if (risk_evaluator) {
      state.gng_planner_ptr->setCostEvaluator(risk_evaluator);
    } else {
      std::cerr << "[PlanningCoordinator] Warning: RISK metric selected but no "
                   "evaluator provided. Falling back to DISTANCE."
                << std::endl;
      state.gng_planner_ptr->setCostEvaluator(
          std::make_shared<
              planning::JointDistanceCost<Eigen::VectorXf, Eigen::Vector3f>>());
    }
    break;
  }
}

int PlanningCoordinator::selectRandomActiveNode(const SimulationState &state) {
  if (!state.gng_ptr || state.gng_ptr->getMaxNodeNum() == 0) {
    return -1;
  }

  std::vector<int> active_nodes;
  state.gng_ptr->forEachActive(
      [&](int i, const auto &) { active_nodes.push_back(i); });

  if (active_nodes.empty()) {
    return -1;
  }

  int random_index = rand() % active_nodes.size();
  return active_nodes[random_index];
}

} // namespace simulation
} // namespace robot_sim
