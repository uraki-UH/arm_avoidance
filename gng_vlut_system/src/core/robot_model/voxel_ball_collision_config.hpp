#pragma once

#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "robot_model/voxel_spherizer.hpp"

namespace simulation {

struct VoxelBallCollisionConfig {
  double voxel_size = 0.0;
  double voxel_padding = 0.0;
  VoxelSphereFitOptions fit_options;
};

inline VoxelBallCollisionConfig declareVoxelBallCollisionConfig(
    rclcpp::Node &node, double default_voxel_size,
    double default_voxel_padding) {
  VoxelBallCollisionConfig config;
  config.voxel_size = node.declare_parameter<double>(
      "collision.voxel_ball.voxel_size", default_voxel_size);
  config.voxel_padding = node.declare_parameter<double>(
      "collision.voxel_ball.voxel_padding", default_voxel_padding);

  config.fit_options.max_spheres = static_cast<std::size_t>(std::max<int64_t>(
      1, node.declare_parameter<int>(
             "collision.voxel_ball.max_spheres",
             static_cast<int>(config.fit_options.max_spheres))));
  config.fit_options.min_points_per_sphere = static_cast<std::size_t>(
      std::max<int64_t>(1, node.declare_parameter<int>(
                               "collision.voxel_ball.min_points_per_sphere",
                               static_cast<int>(
                                   config.fit_options.min_points_per_sphere))));
  config.fit_options.min_gain_ratio = node.declare_parameter<double>(
      "collision.voxel_ball.min_gain_ratio",
      config.fit_options.min_gain_ratio);
  config.fit_options.refine_iterations = static_cast<std::size_t>(
      std::max<int64_t>(0, node.declare_parameter<int>(
                               "collision.voxel_ball.refine_iterations",
                               static_cast<int>(
                                   config.fit_options.refine_iterations))));
  config.fit_options.containment_margin = node.declare_parameter<double>(
      "collision.voxel_ball.containment_margin",
      config.fit_options.containment_margin);
  config.fit_options.verbose = node.declare_parameter<bool>(
      "collision.voxel_ball.verbose", config.fit_options.verbose);
  return config;
}

} // namespace simulation
