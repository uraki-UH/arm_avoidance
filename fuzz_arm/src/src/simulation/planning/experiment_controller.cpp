#include "simulation/planning/experiment_controller.hpp"
#include "control/linear_trajectory_controller.hpp" // Fix incomplete type
#include "experiment/experiment_runner.hpp"
#include "experiment/gng_planner_wrapper.hpp"
#include "experiment/rrt_planner_wrapper.hpp"
#include "experiment/target_touch_scenario.hpp"

#include "kinematics/kinematic_chain.hpp"
#include "planner/RRT/ik_rrt_planner.hpp"
#include "simulation/world/environment_manager.hpp"
#include "simulation/planning/ode_state_validity_checker.hpp"
#include "simulation/sensing/voxel_grid.hpp"
// #include "planner/gng_dijkstra_planner.hpp"

#include <iostream>

namespace robot_sim {
namespace simulation {

ExperimentController::ExperimentController() : is_active_(false) {}

ExperimentController::~ExperimentController() = default;

void ExperimentController::startExperiment(const std::string &method,
                                           SimulationState &state) {
  if (!state.robot_hal_ptr || !state.controller_ptr || !state.fk_chain_ptr ||
      !state.gng_ptr) {
    std::cerr << "[Experiment] Error: System components not ready."
              << std::endl;
    return;
  }

  // Reset Scenario
  state.current_scenario_ptr =
      std::make_shared<robot_sim::experiment::TargetTouchScenario>();

  // Initialize Runner
  state.experiment_runner_ptr =
      std::make_unique<robot_sim::experiment::ExperimentRunner>(
          *state.robot_hal_ptr, *state.controller_ptr,
          state.fk_chain_ptr.get(), state.state_adapter_ptr.get());
  state.experiment_runner_ptr->setScenario(state.current_scenario_ptr);

  // Set validity checker
  if (!state.validity_checker_ptr) {
    state.validity_checker_ptr =
        std::make_unique<robot_sim::simulation::OdeStateValidityChecker>(
            state.robot_collision_model_ptr.get(),
            state.pc_collision_checker_ptr.get(),
            state.point_cloud_grid_ptr.get(), state.env_manager_ptr.get(),
            state.fk_chain_ptr.get());
  }
  state.experiment_runner_ptr->setValidityChecker(
      state.validity_checker_ptr.get());

  // Prepare Planner Wrappers
  if (method == "GNG") {
    // Use a customized shared_ptr for planner to avoid double-deletion of
    // global unique_ptr Note: This requires state.gng_planner_ptr to be
    // initialized
    if (!state.gng_planner_ptr) {
      std::cerr << "[Experiment] Error: GNG Planner not initialized in state."
                << std::endl;
      return;
    }

    auto gng_planner_ptr = std::shared_ptr<planning::GngDijkstraPlanner<
        Eigen::VectorXf, Eigen::Vector3f,
        GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f>>>(
        state.gng_planner_ptr.get(), [](void *) {});

    state.experiment_runner_ptr->setPlanner(
        std::make_shared<robot_sim::experiment::GNGPlannerWrapper>(
            gng_planner_ptr, *state.gng_ptr, *state.fk_chain_ptr,
            state.state_adapter_ptr.get(), state,
            state.validity_checker_ptr.get()));

    std::cout << "[Experiment] Started GNG Challenge" << std::endl;
    state.current_planning_method = PlanningMethod::GNG_DIJKSTRA;

  } else if (method == "RRT") {
    // Note: This requires RRT functionality linked
    state.experiment_runner_ptr->setPlanner(
        std::make_shared<robot_sim::experiment::RRTPlannerWrapper>(
            *state.fk_chain_ptr, state.validity_checker_ptr.get(),
            state.state_adapter_ptr.get(), state, state.global_rrt_params));

    std::cout << "[Experiment] Started RRT Challenge" << std::endl;
    state.current_planning_method = PlanningMethod::IK_RRT_CONNECT;
  }

  // Disable other autonomous modes to prevent command conflicts
  state.reactive_trailing_mode = false;
  state.auto_mode = false;
  state.experiment_active = true;
  is_active_ = true;

  start_time_ = std::chrono::high_resolution_clock::now();
}

void ExperimentController::resetExperiment(SimulationState &state) {
  state.experiment_active = false;
  is_active_ = false;

  if (state.controller_ptr) {
    state.controller_ptr->stop();
  }

  state.experiment_runner_ptr.reset();
  state.current_scenario_ptr.reset();

  std::cout << "[Experiment] Reset" << std::endl;
}

void ExperimentController::update(double dt, SimulationState &state) {
  if (!is_active_ || !state.experiment_runner_ptr) {
    return;
  }

  // Phase 3: Implement update loop details if needed
  // Currently logic is mostly handled by main loop or internal runner state
  (void)dt;
}

} // namespace simulation
} // namespace robot_sim
