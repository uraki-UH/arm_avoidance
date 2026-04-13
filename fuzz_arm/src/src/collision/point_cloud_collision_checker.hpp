#pragma once

#include "collision/collision_detector.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "simulation/robot/robot_model.hpp"
#include "simulation/sensing/voxel_grid.hpp"
#include <Eigen/Dense>
#include <vector>

namespace simulation {

/**
 * @brief Performs fast hierarchical collision detection between a robot and a
 * point cloud.
 *
 * Phase 0: Robot-level AABB vs Point Cloud.
 * Phase 1: Link-level AABB vs Point Cloud points.
 * Phase 2: Precise Capsule-Point distance check.
 */
class PointCloudCollisionChecker {
public:
  PointCloudCollisionChecker(const RobotModel &model,
                             const kinematics::KinematicChain &chain);

  /**
   * @brief Check if specify joint angles are in collision with the point cloud.
   * @param joints Current joint angles.
   * @param grid The voxel grid containing the point cloud.
   * @param pc_bounds Pre-calculated AABB of the point cloud.
   * @return true if colliding.
   */
  bool checkCollision(const std::vector<double> &joints, const VoxelGrid &grid,
                      const collision::AABB &pc_bounds,
                      double margin = 0.0) const;

private:
  struct LinkShape {
    std::string name;
    int link_idx; // Index in KinematicChain if available
    collision::Capsule local_capsule;
    collision::AABB local_aabb;
  };

  void initializeShapes();
  collision::AABB computeLinkAABB(const LinkShape &shape,
                                  const Eigen::Isometry3d &tf) const;

  const RobotModel &model_;
  const kinematics::KinematicChain &chain_;
  std::vector<LinkShape> shapes_;
  std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
      fixed_link_info_;
};

} // namespace simulation
