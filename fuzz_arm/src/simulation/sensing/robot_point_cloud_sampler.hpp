#pragma once

#include "kinematics/kinematic_chain.hpp"
#include "simulation/robot/robot_model.hpp"
#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

namespace simulation {

/**
 * @brief Utility class to convert a RobotModel into a point cloud for collision
 * detection.
 *
 * It pre-samples points on the surface of robot links (Box, Cylinder, Sphere)
 * and updates their world positions based on current kinematic state.
 */
class RobotPointCloudSampler {
public:
  RobotPointCloudSampler(const RobotModel &model,
                         const kinematics::KinematicChain &chain);

  /**
   * @brief Set the density of sampling.
   * @param points_per_m2 Points per square meter of surface area.
   */
  void setSamplingDensity(double points_per_m2);

  /**
   * @brief Update world positions of the point cloud based on current joint
   * angles.
   * @param fixed_link_info Information about fixed links (child -> {parent,
   * relative_tf}).
   */
  void
  update(const std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
             &fixed_link_info);

  /**
   * @brief Get all sampled points in world coordinates.
   */
  const std::vector<Eigen::Vector3d> &getPoints() const {
    return world_points_;
  }

  /**
   * @brief Get points for a specific link in world coordinates.
   */
  std::vector<Eigen::Vector3d>
  getLinkPoints(const std::string &link_name) const;

private:
  struct LinkPoints {
    std::string link_name;
    std::vector<Eigen::Vector3d>
        local_points; // Local coordinates relative to link origin
  };

  void sampleModel();
  void sampleBox(const Eigen::Vector3d &size,
                 std::vector<Eigen::Vector3d> &points);
  void sampleCylinder(double radius, double length,
                      std::vector<Eigen::Vector3d> &points);
  void sampleSphere(double radius, std::vector<Eigen::Vector3d> &points);

  const RobotModel &model_;
  const kinematics::KinematicChain &chain_;
  double density_ = 1000.0; // points / m^2

  std::vector<LinkPoints> link_point_data_;
  std::vector<Eigen::Vector3d> world_points_;
};

} // namespace simulation
