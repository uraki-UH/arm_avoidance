#include "simulation/core/simulation_state.hpp"
#include "control/linear_trajectory_controller.hpp"
#include "experiment/experiment_runner.hpp"
#include "planner/RRT/ik_rrt_planner.hpp"
#include "planning/gng_dijkstra_planner.hpp"
#include "simulation/core/experiment_logger.hpp"
#include "simulation/core/udp_command_bridge.hpp"
#include "simulation/planning/ode_state_validity_checker.hpp"
#include "simulation/planning/scenario_manager.hpp"
#include "simulation/reactive_controller.hpp"
#include "simulation/robot/geometry_management.hpp"
#include "simulation/robot/ode/ode_gng_visualizer.hpp"
#include "simulation/robot/ode/ode_robot_sim.hpp"
#include "simulation/safety/influence_management.hpp"
#include "simulation/sensing/voxel_grid.hpp"
#include "simulation/viz/gui_manager.hpp"
#include "simulation/viz/visualization_manager.hpp"
#include "simulation/world/collision_exclusion_setup.hpp"
#include "simulation/world/dynamic_obstacle_manager.hpp"
#include "simulation/world/sim_obstacle_controller.hpp"
#include "status/gng_status_updater.hpp"
#include "status/graph_topology_analyzer.hpp"
#include "visualization/joint_angle_publisher.hpp"

namespace robot_sim {
namespace simulation {

SimulationState::SimulationState() {
  gui_manager_ptr = std::make_unique<GuiManager>();
  mesh_cache_ptr = std::make_unique<robot_sim::simulation::MeshCache>();
  logger_ptr = std::make_unique<::simulation::ExperimentLogger>();
  replay_manager_ptr = std::make_unique<ReplayManager>();
}

SimulationState::~SimulationState() = default;

void SimulationState::cleanup() {
  robot_hal_ptr.reset();
  gng_ptr.reset();
  fk_chain_ptr.reset();
}

void SimulationState::clearPathVisualizations() {
  active_path.clear();
  current_path_node_ids.clear();
  candidate_paths_viz.clear();
  is_executing_path = false;
  if (gng_viz_ptr) {
    gng_viz_ptr->setTrajectory({});
  }
}

} // namespace simulation
} // namespace robot_sim
