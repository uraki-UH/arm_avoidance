#pragma once

namespace robot_sim {
namespace planner {

/**
 * @brief Parameters for STOMP optimizer.
 */
struct StompParams {
  int max_iterations = 50; // Original default was 50 in stomp.hpp
  int num_rollouts = 10;
  int num_reused_rollouts = 5;
  double control_cost_weight = 1.0;
  double noise_stddev = 1.0;
  double min_noise_stddev = 0.01;
  double noise_decay = 0.99;
  double temperature = 1.0;
};

} // namespace planner
} // namespace robot_sim
