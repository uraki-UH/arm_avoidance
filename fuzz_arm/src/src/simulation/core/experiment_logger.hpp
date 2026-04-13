#pragma once

#include "simulation/world/environment_manager.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <string>
#include <vector>

namespace simulation {

/**
 * @brief Experiment Data Logger
 *
 * Records time-series data for analysis:
 * - Robot Joint State (Positions, Velocities, Torques)
 * - Environment State (Dynamic Obstacle Position, Closest Distance)
 * - GNG/Planner Performance (Graph size, Computation time)
 * - Collisions
 * - Robot Obstacle States
 */
class ExperimentLogger {
public:
  ExperimentLogger();
  ~ExperimentLogger();

  // Initialize logger with base directory and DOF. Creates a timestamped run
  // folder.
  void initialize(int dof, const std::string &base_dir = "experiment_logs");

  // Log a single time step
  void logStep(double timestamp, const Eigen::VectorXf &joint_pos,
               const Eigen::VectorXf &joint_vel,
               const Eigen::VectorXf &joint_effort,
               const Eigen::Vector3d &obstacle_pos,
               double min_distance_to_obstacle, int active_node_count,
               int touched_node_count, double computation_time_ms,
               double env_update_time_ms, double safety_update_time_ms,
               const std::string &scenario_name,
               const std::string &obstacle_mode, bool is_collision);

  // Log collision event
  void logCollision(double timestamp, const std::string &link_name,
                    const std::string &other_name);

  // Log dynamic obstacle state
  void logObstacle(double timestamp, int id, const Eigen::Vector3d &pos,
                   const Eigen::Vector3d &vel, double radius);

  // Log robot obstacle states
  void logRobotObstacles(double timestamp,
                         const std::vector<RobotObstacleState> &states);

  // Close files
  void close();

private:
  std::ofstream log_file_;
  std::ofstream collision_log_file_;
  std::ofstream obstacle_log_file_;
  std::ofstream robot_obstacle_log_file_;

  std::string current_run_dir_;
  bool initialized_ = false;

  void createDirectory(const std::string &path);
};

} // namespace simulation
