#pragma once

namespace robot_sim {
namespace planner {

/**
 * @brief Parameters for RRT-based planners.
 */
struct RRTParams {
  // Basic RRT parameters
  // Basic RRT parameters
  double step_size =
      0.02; // Step size in configuration space (radians) - Reduced for safety
  int max_iterations =
      1000; // Drastically reduced from 10000 for Real-Time performance
  double max_planning_time_ms =
      10.0; // Hard time limit for real-time safety (Increased for reliability)
  double goal_bias =
      0.05; // Probability of sampling the goal directly (for single-tree)
  double connect_max_distance =
      0.5; // Threshold for Connect operation in RRT-Connect

  // Goal reaching parameters
  double goal_pos_tolerance = 0.002; // Cartesian distance threshold (meters)
  double goal_ori_tolerance = 0.1;   // Quaternion distance threshold (radians)

  // IK-RRT specific parameters
  int max_ik_samples = 2;     // Reduced from 10 to prevent IK flooding
  int max_ik_iterations = 20; // Reduced from 100 for faster failure

  // Collision parameters
  double collision_margin =
      0.05; // Safety margin for collision checking (meters)

  // Evaluation / Simulation parameters
  bool simulate_manipulability_overhead =
      false; // If true, calculate manipulability at each tree extension to
             // simulate computational cost
};

} // namespace planner
} // namespace robot_sim
