#pragma once

#include "experiment/i_scenario.hpp"
#include "simulation/planning/scenario_manager.hpp"
#include <deque>
#include <memory>

namespace robot_sim {
namespace experiment {

/**
 * @brief Scenario for continuous trajectory tracking.
 *
 * Manages a moving target (rabbit) that the robot must follow.
 * The high-level path/trajectory is determined by
 * ScenarioManager/ObstacleController.
 */
class TrajectoryTrackingScenario : public IScenario {
public:
  TrajectoryTrackingScenario(simulation::ScenarioManager *manager);
  ~TrajectoryTrackingScenario() override = default;

  void reset() override;
  Eigen::Vector3d getCurrentTargetPos() const override;
  bool onTargetReached() override;
  double getTargetTolerance() const override;
  void update(double dt, const Eigen::Vector3d &current_eef_pos) override;
  std::string getName() const override;
  std::string getProgressString() const override;

private:
  simulation::ScenarioManager *manager_;

  std::deque<Eigen::Vector3d> target_queue_;
  double current_traj_angle_ = 0.0;
  bool initialized_ = false;

  // Parameters
  double angle_step_ = 0.1;
  size_t max_queue_size_ = 63;
  double target_advance_threshold_ = 0.15; // Distance to advance target

  void refillTargetQueue();
};

} // namespace experiment
} // namespace robot_sim
