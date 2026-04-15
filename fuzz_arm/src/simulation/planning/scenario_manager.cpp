#include "simulation/planning/scenario_manager.hpp"
#include <iostream>

namespace robot_sim {
namespace simulation {

ScenarioManager::ScenarioManager(DynamicObstacleManager *obstacle_mgr)
    : obstacle_mgr_(obstacle_mgr), time_elapsed_(0.0) {
  // Initialize positions
  target_pos_ = Eigen::Vector3d(0.25, 0.0, 0.35);
  obstacle_pos_ = Eigen::Vector3d(0.4, 0.0, 0.5);

  // Initialize Controller
  obstacle_controller_ = std::make_unique<ObstacleController>();
}

void ScenarioManager::setOperationMode(OperationMode mode) {
  if (current_mode_ != mode) {
    current_mode_ = mode;
    if (mode == OperationMode::REAL) {
      std::cout << "[ScenarioManager] Switched to REAL mode (External Input). "
                   "Synthetic obstacles disabled.\n";
    } else {
      std::cout << "[ScenarioManager] Switched to SIMULATION mode. Synthetic "
                   "obstacles enabled.\n";
      // Reset controller to avoid weird jumps?
      if (obstacle_controller_)
        obstacle_controller_->reset();
    }
  }
}

void ScenarioManager::setScenario(ScenarioType type) {
  if (current_scenario_ != type) {
    current_scenario_ = type; // Reset state
    time_elapsed_ = 0.0;

    // Reset Controller State
    if (obstacle_controller_)
      obstacle_controller_->reset();

    std::string name;
    switch (type) {
    case ScenarioType::IDLE:
      name = "IDLE";
      break;
    case ScenarioType::AVOIDANCE:
      name = "AVOIDANCE";
      break;
    case ScenarioType::TRACKING:
      name = "TRACKING";
      break;
    case ScenarioType::TRAJECTORY:
      name = "TRAJECTORY";
      break;
    case ScenarioType::AVOIDANCE_STATIC:
      name = "AVOIDANCE_STATIC";
      break;
    default:
      name = "UNKNOWN";
    }
    std::cout << "[ScenarioManager] Switched to scenario: " << name
              << std::endl;
  }
}

void ScenarioManager::setObstacleBehavior(ObstacleBehavior behavior) {
  if (obstacle_behavior_ == behavior)
    return;

  obstacle_behavior_ = behavior;
  if (obstacle_controller_)
    obstacle_controller_->reset(); // Reset timers/random

  std::string name;
  switch (behavior) {
  case ObstacleBehavior::PASSIVE:
    name = "PASSIVE";
    break;
  case ObstacleBehavior::AGGRESSIVE:
    name = "AGGRESSIVE (Hunt/Flee)";
    break;
  case ObstacleBehavior::HYBRID:
    name = "HYBRID";
    break;
  case ObstacleBehavior::RANDOM:
    name = "RANDOM";
    break;
  case ObstacleBehavior::EVASIVE:
    name = "EVASIVE";
    break;
  case ObstacleBehavior::TRAJECTORY:
    name = "TRAJECTORY";
    break;
  case ObstacleBehavior::BOUNCE:
    name = "BOUNCE";
    break;
  case ObstacleBehavior::LINEAR_RECIPROCATE:
    name = "LINEAR_RECIPROCATE (Approach)";
    break;
  case ObstacleBehavior::STATIONARY:
    name = "PAUSE (Manual)";
    break;
  }
  std::cout << "[ScenarioManager] Obstacle Behavior set to: " << name
            << std::endl;
}

void ScenarioManager::update(double dt, const Eigen::Vector3d &current_eef_pos,
                             std::vector<int> &collision_counts,
                             std::vector<int> &danger_counts, float dilation_radius,
                             bool enable_collision) {
  time_elapsed_ += dt;

  // Handle REAL Mode (External Input) or Global Collision Disabled
  if (current_mode_ == OperationMode::REAL || !enable_collision) {
    if (obstacle_mgr_ && !obstacle_mgr_->getObstacles().empty()) {
      obstacle_mgr_->clearObstacles(collision_counts, danger_counts);
    }

    // We still update target_pos_ because it might be needed for
    // TRACKING/TRAJECTORY visualization
    if (current_scenario_ == ScenarioType::TRAJECTORY && obstacle_controller_) {
      double angle = time_elapsed_ * obstacle_controller_->getTrajectorySpeed();
      target_pos_ = obstacle_controller_->getTrajectoryPoint(angle);
    }

    // We also update obstacle_pos_ for visualization purposes, even if it has
    // no collision
    if (obstacle_controller_) {
      bool is_tracking = (current_scenario_ == ScenarioType::TRACKING);
      obstacle_pos_ = obstacle_controller_->update(
          dt, obstacle_pos_, current_eef_pos, obstacle_behavior_, is_tracking);
    }

    return;
  }

  // Handle SIMULATION Mode
  if (!obstacle_controller_)
    return;

  bool is_tracking = (current_scenario_ == ScenarioType::TRACKING);

  // 1. Update Target Position independently if in TRAJECTORY scenario
  if (current_scenario_ == ScenarioType::TRAJECTORY && obstacle_controller_) {
    double angle = time_elapsed_ * obstacle_controller_->getTrajectorySpeed();
    target_pos_ = obstacle_controller_->getTrajectoryPoint(angle);
  }

  // 2. Determine effective behavior for the OBSTACLE
  ObstacleBehavior effective_behavior = obstacle_behavior_;

  // Update Position (Removed IDLE guard to allow background movement)
  obstacle_pos_ = obstacle_controller_->update(
      dt, obstacle_pos_, current_eef_pos, effective_behavior, is_tracking);

  // Update Dynamic Obstacle Manager
  if (obstacle_mgr_) {
    obstacle_mgr_->updateObstacleParameters(999, obstacle_pos_,
                                            obstacle_radius_, dilation_radius);
  }
}

void ScenarioManager::loadScenario(const std::string &filepath) {
  active_scenario_def_ = experiment::ScenarioDefinition::loadFromJson(filepath);
  current_stage_idx_ = -1;
}

void ScenarioManager::startScenario() {
  if (active_scenario_def_.stages.empty()) {
    active_scenario_def_ = experiment::ScenarioDefinition::getDefault();
  }
  setScenario(ScenarioType::AVOIDANCE_STATIC);
  applyStage(0);
}

void ScenarioManager::nextStage() {
  if (current_stage_idx_ + 1 < (int)active_scenario_def_.stages.size()) {
    applyStage(current_stage_idx_ + 1);
  } else {
    current_stage_idx_ = (int)active_scenario_def_.stages.size();
    std::cout << "[ScenarioManager] Scenario Completed!" << std::endl;
  }
}

void ScenarioManager::applyStage(int index) {
  if (index < 0 || index >= (int)active_scenario_def_.stages.size())
    return;

  current_stage_idx_ = index;
  const auto &stage = active_scenario_def_.stages[index];

  std::cout << "[ScenarioManager] Applying Stage " << index + 1 << "/"
            << active_scenario_def_.stages.size() << ": " << stage.name
            << " (Target: " << stage.target_pos.transpose() << ")" << std::endl;
  setTargetPosition(stage.target_pos);
  setObstacleBehavior(stage.behavior);
  setObstacleSpeed(stage.obstacle_speed);
  setObstacleRadius(stage.obstacle_radius);

  // Note: time_elapsed_ reset removed to maintain continuous orbit
}

bool ScenarioManager::isScenarioCompleted() const {
  return current_stage_idx_ >= (int)active_scenario_def_.stages.size();
}

std::string ScenarioManager::getScenarioName() const {
  switch (current_scenario_) {
  case ScenarioType::IDLE:
    return "IDLE";
  case ScenarioType::AVOIDANCE:
    return "AVOIDANCE";
  case ScenarioType::TRACKING:
    return "TRACKING";
  case ScenarioType::NAVIGATION:
    return "NAVIGATION";
  case ScenarioType::TRAJECTORY:
    return "TRAJECTORY";
  case ScenarioType::AVOIDANCE_STATIC:
    return "AVOIDANCE_STATIC";
  default:
    return "UNKNOWN";
  }
}

std::string ScenarioManager::getModeName() const {
  if (current_scenario_ == ScenarioType::AVOIDANCE) {
    switch (getObstacleBehavior()) {
    case ObstacleBehavior::PASSIVE:
      return "PASSIVE";
    case ObstacleBehavior::AGGRESSIVE:
      return "AGGRESSIVE";
    case ObstacleBehavior::HYBRID:
      return "HYBRID";
    case ObstacleBehavior::RANDOM:
      return "RANDOM";
    case ObstacleBehavior::EVASIVE:
      return "EVASIVE";
    case ObstacleBehavior::TRAJECTORY:
      return "TRAJECTORY";
    default:
      return "UNKNOWN";
    }
  } else if (current_scenario_ == ScenarioType::TRACKING) {
    switch (getObstacleBehavior()) {
    case ObstacleBehavior::AGGRESSIVE:
      return "EVASIVE (TRACKING)";
    case ObstacleBehavior::EVASIVE:
      return "EVASIVE";
    case ObstacleBehavior::TRAJECTORY:
      return "TRAJECTORY";
    default:
      return "PASSIVE";
    }
  } else if (current_scenario_ == ScenarioType::TRAJECTORY) {
    return "CLOSED_LOOP";
  }
  return "NONE";
}

} // namespace simulation
} // namespace robot_sim
