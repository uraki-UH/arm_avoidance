#include "experiment/scenarios/trajectory_tracking_scenario.hpp"

namespace robot_sim {
namespace experiment {

TrajectoryTrackingScenario::TrajectoryTrackingScenario(
    simulation::ScenarioManager *manager)
    : manager_(manager) {}

void TrajectoryTrackingScenario::reset() {
  target_queue_.clear();
  current_traj_angle_ = 0.0;
  initialized_ = false;
}

Eigen::Vector3d TrajectoryTrackingScenario::getCurrentTargetPos() const {
  if (target_queue_.empty()) {
    // If empty, fallback to something?
    // In reactive module, it returned obstacle pos if queue was empty,
    // or relied on logic to refill.
    // We should ensure update() is called before this.
    return Eigen::Vector3d::Zero();
  }
  return target_queue_.front();
}

bool TrajectoryTrackingScenario::onTargetReached() {
  // Continuous tracking never "completes" in the traditional sense,
  // unless we want to stop after N loops.
  return false;
}

double TrajectoryTrackingScenario::getTargetTolerance() const {
  return 0.05; // 5cm tolerance for "reaching" (though we move target before
               // reach)
}

void TrajectoryTrackingScenario::update(
    double dt, const Eigen::Vector3d &current_eef_pos) {
  (void)dt; // Unused for now, logic is distance based

  if (!manager_)
    return;

  // 1. Refill queue
  const double angle_step = 0.1;
  while (target_queue_.size() < max_queue_size_) {
    target_queue_.push_back(manager_->getTrajectoryPoint(current_traj_angle_));
    current_traj_angle_ += angle_step;
  }

  // 2. Advance target if robot is close
  if (!target_queue_.empty()) {
    Eigen::Vector3d current_target = target_queue_.front();
    double dist = (current_eef_pos - current_target).norm();

    if (dist < target_advance_threshold_) {
      target_queue_.pop_front();
      // Ensure we immediately refill or just wait for next cycle
      if (target_queue_.empty()) {
        target_queue_.push_back(
            manager_->getTrajectoryPoint(current_traj_angle_));
        current_traj_angle_ += angle_step;
      }
    }
  }
}

void TrajectoryTrackingScenario::refillTargetQueue() {
  // Helper used in reset or update if logic gets complex
}

std::string TrajectoryTrackingScenario::getName() const {
  return "TRAJECTORY_TRACKING";
}

std::string TrajectoryTrackingScenario::getProgressString() const {
  return "Angle: " + std::to_string(current_traj_angle_);
}

} // namespace experiment
} // namespace robot_sim
