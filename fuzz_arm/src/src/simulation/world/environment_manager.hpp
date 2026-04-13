#pragma once

#include "collision/collision_detector.hpp"
#include "simulation/robot/robot_behavior.hpp"
#include "simulation/robot/robot_model.hpp"
#include "simulation/sensing/robot_point_cloud_sampler.hpp"
#include <memory>
#include <ode/ode.h>
#include <string>
#include <vector>

namespace simulation {

enum class ObstacleType {
  STATIC_BOX,
  STATIC_SPHERE,
  STATIC_CYLINDER,
  DYNAMIC_SPHERE,
  OBSTACLE_ROBOT
};

struct ObstacleConfig {
  ObstacleType type;
  std::string name;
  Eigen::Vector3d pos;
  Eigen::Vector3d size;       // x,y,z or radius,length
  Eigen::Vector3d velocity;   // for moving objects
  std::string urdf_path;      // for robots
  std::string leaf_link_name; // for robots
};

struct RobotObstacleState {
  std::string name;
  std::vector<double> joint_positions; // All joints
  Eigen::Vector3d base_pos;
};

/**
 * @brief Manages a collection of obstacles in the simulation.
 *
 * It handles both ODE geometries for visualization/physics and
 * point-cloud generation for the GNG planner.
 */
class EnvironmentManager {
public:
  EnvironmentManager(dWorldID world, dSpaceID space);
  ~EnvironmentManager();

  /**
   * @brief Load environment configuration from a file.
   * @param filename Path to env_config.txt
   */
  bool load(const std::string &filename);

  /**
   * @brief Update dynamic obstacles for the current time.
   * @param t Current simulation time.
   */
  void update(double t);

  /**
   * @brief Draw obstacles (e.g. wireframe robots).
   */
  void draw(bool show_structure = true, const float *color = nullptr);

  /**
   * @brief Get the combined point cloud from all obstacles.
   */
  const std::vector<Eigen::Vector3d> &getCombinedPointCloud() const;

  /**
   * @brief Get the AABB of the combined point cloud.
   */
  collision::AABB getPointCloudBounds() const;

  /**
   * @brief Get ODE geometries for drawing.
   */
  const std::vector<dGeomID> &getODEGeoms() const { return ode_geoms_; }

  /**
   * @brief Get states of all robot obstacles.
   */
  std::vector<RobotObstacleState> getRobotObstacleStates() const;

  /**
   * @brief Restore all robot obstacle joint states (for Replay).
   * @param states Previously captured states from getRobotObstacleStates().
   */
  void setRobotObstacleStates(const std::vector<RobotObstacleState>& states);

private:
  void clear();
  void addStaticBox(const ObstacleConfig &config);
  void addDynamicSphere(const ObstacleConfig &config);
  void addObstacleRobot(const ObstacleConfig &config);

  dWorldID world_;
  dSpaceID space_;

  std::vector<ObstacleConfig> configs_;
  std::vector<dGeomID> ode_geoms_;

  // For robot obstacles
  struct RobotObstacle {
    std::string name;
    std::unique_ptr<RobotModel> model;
    std::unique_ptr<kinematics::KinematicChain> chain;
    Eigen::Vector3d base_pos;
    std::unique_ptr<RobotPointCloudSampler> sampler;
    std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
        fixed_links;
    std::unique_ptr<IRobotBehavior> behavior;
  };
  std::vector<std::unique_ptr<RobotObstacle>> robot_obstacles_;

  mutable std::vector<Eigen::Vector3d> combined_pc_;
  mutable bool pc_dirty_ = true;
};

} // namespace simulation
