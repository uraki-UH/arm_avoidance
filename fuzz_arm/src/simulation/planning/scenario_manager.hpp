#pragma once

#include "experiment/scenario_definition.hpp"
#include "simulation/world/dynamic_obstacle_manager.hpp"
#include "simulation/world/sim_obstacle_controller.hpp"

#include <Eigen/Dense>
#include <memory>
#include <string>

namespace robot_sim {
namespace simulation {

enum class ScenarioType {
  IDLE,
  AVOIDANCE,  // Scenario 1: Obstacle targets robot
  TRACKING,   // Scenario 2: Robot follows moving target
  NAVIGATION, // Scenario 3: Point A to B (Future work)
  TRAJECTORY, // Scenario 4: Follows a mathematical trajectory
  AVOIDANCE_STATIC // Scenario 5: Stay at a fixed position and evacuate when approached
};

enum class OperationMode {
  SIMULATION, // Synthetic obstacles active
  REAL        // Synthetic obstacles disabled, real sensor input assumed
};

class ScenarioManager {
public:
  ScenarioManager(DynamicObstacleManager *obstacle_mgr);
  // dt : time step(s)

  void update(double dt, const Eigen::Vector3d &current_eef_pos,
              std::vector<int> &collision_counts,
              std::vector<int> &danger_counts, float dilation_radius,
              bool enable_collision = true);

  void setScenario(ScenarioType type);
  void setObstacleBehavior(ObstacleBehavior behavior);

  // Set operation mode (SIMULATION or REAL)
  void setOperationMode(OperationMode mode);

  std::string getScenarioName() const;
  std::string getModeName() const;

  void setObstacleRadius(double radius) { obstacle_radius_ = radius; }
  void setObstacleSpeed(double speed) {
    if (obstacle_controller_) {
      obstacle_controller_->setHuntSpeed(speed * 0.4);   // Base Hunt: 0.4
      obstacle_controller_->setFleeSpeed(speed * 0.6);   // Base Flee: 0.6
      obstacle_controller_->setPatrolSpeed(speed * 0.2); // Base Patrol: 0.2
      obstacle_controller_->setTrajectorySpeed(speed);
      obstacle_controller_->setBounceSpeed(speed);
    }
  }

  // Trajectory Settings
  void setTrajectoryType(TrajectoryType type) {
    if (obstacle_controller_)
      obstacle_controller_->setTrajectoryType(type);
  }
  void setTrajectoryScale(double scale) {
    if (obstacle_controller_)
      obstacle_controller_->setTrajectoryScale(scale);
  }
  void setTrajectorySpeed(double speed) {
    if (obstacle_controller_)
      obstacle_controller_->setTrajectorySpeed(speed);
  }

  Eigen::Vector3d getTrajectoryPoint(double angle) const {
    if (obstacle_controller_)
      return obstacle_controller_->getTrajectoryPoint(angle);
    return Eigen::Vector3d::Zero();
  }
  void setTrajectoryHeight(double h) {
    target_pos_.z() = h;
    if (obstacle_controller_) {
      Eigen::Vector3d center = obstacle_controller_->getPatrolCenter();
      center.z() = h;
      obstacle_controller_->setPatrolCenter(center);
    }
  }
  void setTargetPosition(const Eigen::Vector3d &pos) { target_pos_ = pos; }

  // ステージ制シナリオのサポート
  void loadScenario(const std::string &filepath);
  void startScenario();
  void nextStage();
  bool isScenarioCompleted() const;
  int getCurrentStageIndex() const { return current_stage_idx_; }
  const experiment::ScenarioDefinition &getScenarioDefinition() const {
    return active_scenario_def_;
  }

  // Getters for logging and visualization
  ScenarioType getCurrentScenario() const { return current_scenario_; }
  ObstacleBehavior getObstacleBehavior() const { return obstacle_behavior_; }
  OperationMode getOperationMode() const { return current_mode_; }
  Eigen::Vector3d getTargetPosition() const { return target_pos_; }
  Eigen::Vector3d getObstaclePosition() const { return obstacle_pos_; }
  Eigen::Vector3d getObstacleVelocity() const {
    return obstacle_controller_ ? obstacle_controller_->getVelocity()
                                : Eigen::Vector3d::Zero();
  }

private:
  ScenarioType current_scenario_ = ScenarioType::IDLE;
  ObstacleBehavior obstacle_behavior_ = ObstacleBehavior::PASSIVE;
  OperationMode current_mode_ = OperationMode::SIMULATION;

  DynamicObstacleManager *obstacle_mgr_;
  std::unique_ptr<ObstacleController> obstacle_controller_;

  // State variables
  double time_elapsed_ = 0.0;
  Eigen::Vector3d target_pos_;
  Eigen::Vector3d obstacle_pos_;
  double obstacle_radius_ = 0.05; // Default

  // シナリオの状態
  experiment::ScenarioDefinition active_scenario_def_;
  int current_stage_idx_ = -1;

  void applyStage(int index);

  // Placeholder methods for specific scenario logic
  void updateAvoidanceScenario(double dt, const Eigen::Vector3d &robot_pos);
  void updateTrackingScenario(double dt);
  void updateIdleScenario(double dt);
};

} // namespace simulation
} // namespace robot_sim
