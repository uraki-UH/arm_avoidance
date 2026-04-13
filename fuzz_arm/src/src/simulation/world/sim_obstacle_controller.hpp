/**
 * @file sim_obstacle_controller.hpp
 * @brief Dynamic obstacle control and behavior strategies for simulation.
 */

#pragma once

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <vector>

namespace robot_sim {
namespace simulation {

/**
 * @brief Behavior types for synthetic obstacles in simulation.
 */
enum class ObstacleBehavior {
  PASSIVE,
  AGGRESSIVE, // Hunt when far, Flee when tracked
  EVASIVE,    // Always Flee
  RANDOM,
  TRAJECTORY,
  HYBRID, // Patrol + Aggressive
  BOUNCE,
  LINEAR_RECIPROCATE,
  STATIONARY
};

/**
 * @brief Supported geometric trajectory types.
 */
enum class TrajectoryType { CIRCLE, ELLIPSE, INFINITY_SHAPE, SQUARE };

/**
 * @brief Strategy interface for obstacle movement behaviors.
 */
class IBehaviorStrategy {
public:
  virtual ~IBehaviorStrategy() = default;
  virtual Eigen::Vector3d execute(double dt, const Eigen::Vector3d &current_pos,
                                  const Eigen::Vector3d &robot_eef_pos,
                                  bool is_tracking_scenario) = 0;
  virtual void setSpeed(double /*speed*/) {}
  virtual void setCenter(const Eigen::Vector3d & /*center*/) {}
};

/**
 * @brief Strategy interface for geometric trajectories.
 */
class ITrajectoryStrategy {
public:
  virtual ~ITrajectoryStrategy() = default;
  virtual Eigen::Vector3d getPoint(double angle) = 0;
  virtual void setScale(double scale) = 0;
  virtual void setCenter(const Eigen::Vector3d &center) = 0;
};

/**
 * @brief Controller for simulating dynamic obstacle movements.
 */
class ObstacleController {
public:
  ObstacleController();

  void reset();

  /**
   * @brief Update obstacle position based on selected behavior.
   */
  Eigen::Vector3d update(double dt, const Eigen::Vector3d &current_pos,
                         const Eigen::Vector3d &robot_eef_pos,
                         ObstacleBehavior behavior, bool is_tracking_scenario);

  // Configuration
  void setTrajectoryType(TrajectoryType type);
  void setTrajectorySpeed(double speed);
  double getTrajectorySpeed() const;
  void setTrajectoryScale(double scale);
  Eigen::Vector3d getTrajectoryPoint(double angle) const;

  void setHuntSpeed(double speed);
  void setFleeSpeed(double speed);
  void setPatrolSpeed(double speed);
  void setBounceSpeed(double speed);
  void setPatrolCenter(const Eigen::Vector3d &center);
  Eigen::Vector3d getPatrolCenter() const;

  void setBounds(const Eigen::Vector3d &min, const Eigen::Vector3d &max) {
    bounds_min_ = min;
    bounds_max_ = max;
  }

  Eigen::Vector3d getVelocity() const { return current_velocity_; }

private:
  void registerStrategies();

  double time_elapsed_ = 0.0;
  Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();

  // Selected sub-strategies
  std::map<ObstacleBehavior, std::shared_ptr<IBehaviorStrategy>>
      behavior_strategies_;
  std::map<TrajectoryType, std::shared_ptr<ITrajectoryStrategy>>
      trajectory_strategies_;

  TrajectoryType trajectory_type_ = TrajectoryType::CIRCLE;

  // Cached strategies for parameter updates
  std::shared_ptr<IBehaviorStrategy> patrol_strategy_;
  std::shared_ptr<IBehaviorStrategy> hunt_strategy_;
  std::shared_ptr<IBehaviorStrategy> flee_strategy_;
  std::shared_ptr<IBehaviorStrategy> random_strategy_;
  std::shared_ptr<IBehaviorStrategy> bounce_strategy_;
  std::shared_ptr<IBehaviorStrategy> linear_reciprocate_strategy_;

  // Parameters
  double trajectory_speed_ = 0.5;
  double hunt_speed_default_ = 0.4;
  double flee_speed_default_ = 0.6;
  double patrol_radius_ = 0.2;
  double default_trajectory_scale_ = 0.3;
  Eigen::Vector3d patrol_center_ = Eigen::Vector3d(0.4, 0.0, 0.5);
  Eigen::Vector3d bounds_min_ = Eigen::Vector3d(0.1, -0.5, 0.1);
  Eigen::Vector3d bounds_max_ = Eigen::Vector3d(0.8, 0.5, 0.9);
};

} // namespace simulation
} // namespace robot_sim
