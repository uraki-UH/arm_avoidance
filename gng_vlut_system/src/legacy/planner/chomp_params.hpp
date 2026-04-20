#pragma once

namespace robot_sim {
namespace planner {

/**
 * @brief Parameters for CHOMP optimizer.
 */
struct ChompParams {
  int num_iterations = 100;
  double learning_rate = 0.05;
  double obstacle_cost_weight = 1.0;
  double smoothness_cost_weight = 1.0;
  double obstacle_threshold = 0.1;
  double min_dist_to_obstacle = 0.5; // New from magic numbers in original code
  double weight = 0.0;               // Placeholder for potential future use
};

} // namespace planner
} // namespace robot_sim
