/**
 * @file sim_obstacle_controller.cpp
 * @brief Implementation of simulation-specific obstacle behaviors.
 */

#include "simulation/world/sim_obstacle_controller.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace robot_sim {
namespace simulation {

// =========================================================
// Concrete Behavior Strategies
// =========================================================

class PatrolBehavior : public IBehaviorStrategy {
public:
  PatrolBehavior(const Eigen::Vector3d &center, double radius)
      : center_(center), radius_(radius) {}

  Eigen::Vector3d execute(double dt, const Eigen::Vector3d &current_pos,
                          const Eigen::Vector3d &, bool) override {
    time_elapsed_ += dt;
    double angle = time_elapsed_ * speed_;
    Eigen::Vector3d target(center_.x() + radius_ * std::cos(angle),
                           center_.y() + radius_ * std::sin(angle),
                           center_.z() + 0.1 * std::sin(angle));

    double step = 0.5 * dt; // approach speed
    Eigen::Vector3d diff = target - current_pos;
    if (diff.norm() > step && diff.norm() > 1e-4)
      return current_pos + diff.normalized() * step;
    return target;
  }

  void setSpeed(double speed) override { speed_ = speed; }
  void setCenter(const Eigen::Vector3d &center) override { center_ = center; }
  Eigen::Vector3d getCenter() const { return center_; }

private:
  Eigen::Vector3d center_;
  double radius_;
  double speed_ = 0.5;
  double time_elapsed_ = 0.0;
};

class HuntBehavior : public IBehaviorStrategy {
public:
  HuntBehavior(double speed) : speed_(speed) {}
  Eigen::Vector3d execute(double dt, const Eigen::Vector3d &current_pos,
                          const Eigen::Vector3d &robot_eef_pos, bool) override {
    Eigen::Vector3d dir = (robot_eef_pos - current_pos).normalized();
    return current_pos + dir * speed_ * dt;
  }
  void setSpeed(double speed) override { speed_ = speed; }

private:
  double speed_;
};

class FleeBehavior : public IBehaviorStrategy {
public:
  FleeBehavior(double speed, const Eigen::Vector3d &min,
               const Eigen::Vector3d &max)
      : speed_(speed), min_(min), max_(max) {}

  Eigen::Vector3d execute(double dt, const Eigen::Vector3d &current_pos,
                          const Eigen::Vector3d &robot_eef_pos, bool) override {
    Eigen::Vector3d dir = (current_pos - robot_eef_pos).normalized();
    for (int i = 0; i < 3; ++i) {
      if (current_pos[i] < min_[i] + 0.1)
        dir[i] += 1.0;
      if (current_pos[i] > max_[i] - 0.1)
        dir[i] -= 1.0;
    }
    dir.normalize();
    return (current_pos + dir * speed_ * dt).cwiseMax(min_).cwiseMin(max_);
  }
  void setSpeed(double speed) override { speed_ = speed; }

private:
  double speed_;
  Eigen::Vector3d min_, max_;
};

class RandomBehavior : public IBehaviorStrategy {
public:
  RandomBehavior(const Eigen::Vector3d &min, const Eigen::Vector3d &max)
      : rng_(std::random_device{}()), min_(min), max_(max) {}

  Eigen::Vector3d execute(double dt, const Eigen::Vector3d &current_pos,
                          const Eigen::Vector3d &, bool) override {
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    Eigen::Vector3d acc(dist(rng_), dist(rng_), dist(rng_));
    velocity_ += acc.normalized() * 5.0 * dt;
    velocity_ -= velocity_ * 1.0 * dt; // Damping
    if (velocity_.norm() > 1.0)
      velocity_.normalize();

    Eigen::Vector3d next = current_pos + velocity_ * dt;
    for (int i = 0; i < 3; ++i) {
      if (next[i] < min_[i] || next[i] > max_[i]) {
        velocity_[i] *= -1.0;
        next[i] = std::max(min_[i], std::min(max_[i], next[i]));
      }
    }
    return next;
  }

private:
  std::mt19937 rng_;
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d min_, max_;
};

class HybridBehavior : public IBehaviorStrategy {
public:
  HybridBehavior(std::shared_ptr<IBehaviorStrategy> passive,
                 std::shared_ptr<IBehaviorStrategy> active)
      : passive_(passive), active_(active) {}

  Eigen::Vector3d execute(double dt, const Eigen::Vector3d &current_pos,
                          const Eigen::Vector3d &robot_eef_pos,
                          bool is_tracking) override {
    timer_ += dt;
    if (is_active_) {
      if (timer_ > 3.0) {
        is_active_ = false;
        timer_ = 0.0;
      }
    } else {
      if (timer_ > 7.0) {
        is_active_ = true;
        timer_ = 0.0;
      }
    }
    return (is_active_ ? active_ : passive_)
        ->execute(dt, current_pos, robot_eef_pos, is_tracking);
  }

private:
  std::shared_ptr<IBehaviorStrategy> passive_, active_;
  double timer_ = 0.0;
  bool is_active_ = false;
};

class BounceBehavior : public IBehaviorStrategy {
public:
  BounceBehavior(double speed, const Eigen::Vector3d &min,
                 const Eigen::Vector3d &max)
      : speed_(speed), min_(min), max_(max) {
    // Initial random direction
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    velocity_ =
        Eigen::Vector3d(dist(rng), dist(rng), dist(rng)).normalized() * speed_;
  }

  Eigen::Vector3d execute(double dt, const Eigen::Vector3d &current_pos,
                          const Eigen::Vector3d &, bool) override {
    Eigen::Vector3d next = current_pos + velocity_ * dt;

    for (int i = 0; i < 3; ++i) {
      if (next[i] < min_[i]) {
        next[i] = min_[i] + (min_[i] - next[i]);
        velocity_[i] *= -1.0;
      } else if (next[i] > max_[i]) {
        next[i] = max_[i] - (next[i] - max_[i]);
        velocity_[i] *= -1.0;
      }
    }
    return next;
  }

  void setSpeed(double speed) override {
    speed_ = speed;
    if (velocity_.norm() > 1e-6) {
      velocity_ = velocity_.normalized() * speed_;
    }
  }

private:
  double speed_;
  Eigen::Vector3d velocity_;
  Eigen::Vector3d min_, max_;
};

class LinearReciprocateBehavior : public IBehaviorStrategy {
public:
  LinearReciprocateBehavior(double speed, const Eigen::Vector3d &min,
                            const Eigen::Vector3d &max)
      : speed_(speed), min_(min), max_(max) {
    direction_ = 1.0;
  }

  Eigen::Vector3d execute(double dt, const Eigen::Vector3d &current_pos,
                          const Eigen::Vector3d &, bool) override {
    Eigen::Vector3d next = current_pos;
    // Move along X-axis
    next.x() += direction_ * speed_ * dt;

    // Maintain Z from center (managed by setCenter or defaults)
    next.z() = center_.z();
    // Keep Y from center
    next.y() = center_.y();

    if (next.x() < min_.x()) {
      next.x() = min_.x();
      direction_ = 1.0;
    } else if (next.x() > max_.x()) {
      next.x() = max_.x();
      direction_ = -1.0;
    }
    return next;
  }

  void setSpeed(double speed) override { speed_ = speed; }
  void setCenter(const Eigen::Vector3d &center) override { center_ = center; }

private:
  double speed_;
  double direction_;
  Eigen::Vector3d min_, max_;
  Eigen::Vector3d center_ = Eigen::Vector3d(0.4, 0.0, 0.3);
};

class StationaryBehavior : public IBehaviorStrategy {
public:
  Eigen::Vector3d execute(double /*dt*/, const Eigen::Vector3d &current_pos,
                          const Eigen::Vector3d &, bool) override {
    return current_pos;
  }
};

// =========================================================
// Trajectory Strategies
// =========================================================

class CircleTrajectory : public ITrajectoryStrategy {
public:
  CircleTrajectory(const Eigen::Vector3d &center, double scale)
      : center_(center), scale_(scale) {}
  Eigen::Vector3d getPoint(double angle) override {
    return center_ + Eigen::Vector3d(scale_ * std::cos(angle),
                                     scale_ * std::sin(angle), 0);
  }
  void setScale(double scale) override { scale_ = scale; }
  void setCenter(const Eigen::Vector3d &center) override { center_ = center; }

private:
  Eigen::Vector3d center_;
  double scale_;
};

class EllipseTrajectory : public ITrajectoryStrategy {
public:
  EllipseTrajectory(const Eigen::Vector3d &center, double scale)
      : center_(center), scale_(scale) {}
  Eigen::Vector3d getPoint(double angle) override {
    return center_ + Eigen::Vector3d(scale_ * std::cos(angle),
                                     scale_ * 0.5 * std::sin(angle), 0);
  }
  void setScale(double scale) override { scale_ = scale; }
  void setCenter(const Eigen::Vector3d &center) override { center_ = center; }

private:
  Eigen::Vector3d center_;
  double scale_;
};

class InfinityTrajectory : public ITrajectoryStrategy {
public:
  InfinityTrajectory(const Eigen::Vector3d &center, double scale)
      : center_(center), scale_(scale) {}
  Eigen::Vector3d getPoint(double angle) override {
    double d = 1 + std::sin(angle) * std::sin(angle);
    return center_ +
           Eigen::Vector3d(scale_ * std::cos(angle) / d,
                           scale_ * std::sin(angle) * std::cos(angle) / d, 0);
  }
  void setScale(double scale) override { scale_ = scale; }
  void setCenter(const Eigen::Vector3d &center) override { center_ = center; }

private:
  Eigen::Vector3d center_;
  double scale_;
};

class SquareTrajectory : public ITrajectoryStrategy {
public:
  SquareTrajectory(const Eigen::Vector3d &center, double scale)
      : center_(center), scale_(scale) {}
  Eigen::Vector3d getPoint(double angle) override {
    double t = std::fmod(angle, 2.0 * M_PI);
    if (t < 0)
      t += 2.0 * M_PI;
    double s = 2.0 * scale_;
    Eigen::Vector3d p = center_;
    if (t < M_PI / 2) {
      p.x() += -scale_ + s * (t / (M_PI / 2));
      p.y() += scale_;
    } else if (t < M_PI) {
      p.x() += scale_;
      p.y() += scale_ - s * ((t - M_PI / 2) / (M_PI / 2));
    } else if (t < 3 * M_PI / 2) {
      p.x() += scale_ - s * ((t - M_PI) / (M_PI / 2));
      p.y() += -scale_;
    } else {
      p.x() += -scale_;
      p.y() += -scale_ + s * ((t - 3 * M_PI / 2) / (M_PI / 2));
    }
    return p;
  }
  void setScale(double scale) override { scale_ = scale; }
  void setCenter(const Eigen::Vector3d &center) override { center_ = center; }

private:
  Eigen::Vector3d center_;
  double scale_;
};

class TrajectoryFollowBehavior : public IBehaviorStrategy {
public:
  TrajectoryFollowBehavior(std::shared_ptr<ITrajectoryStrategy> traj,
                           double speed)
      : traj_(traj), speed_(speed) {}
  Eigen::Vector3d execute(double dt, const Eigen::Vector3d &current_pos,
                          const Eigen::Vector3d &, bool) override {
    time_elapsed_ += dt;
    Eigen::Vector3d target = traj_->getPoint(time_elapsed_ * speed_);
    double step = 1.0 * dt;
    Eigen::Vector3d diff = target - current_pos;
    if (diff.norm() > step && diff.norm() > 1e-4)
      return current_pos + diff.normalized() * step;
    return target;
  }
  void setSpeed(double speed) override { speed_ = speed; }
  double getSpeed() const { return speed_; }

private:
  std::shared_ptr<ITrajectoryStrategy> traj_;
  double speed_;
  double time_elapsed_ = 0.0;
};

// =========================================================
// ObstacleController Implementation
// =========================================================

ObstacleController::ObstacleController() { registerStrategies(); }

void ObstacleController::reset() {
  time_elapsed_ = 0.0;
  current_velocity_ = Eigen::Vector3d::Zero();
}

void ObstacleController::registerStrategies() {
  patrol_strategy_ =
      std::make_shared<PatrolBehavior>(patrol_center_, patrol_radius_);
  behavior_strategies_[ObstacleBehavior::PASSIVE] = patrol_strategy_;

  hunt_strategy_ = std::make_shared<HuntBehavior>(hunt_speed_default_);
  flee_strategy_ = std::make_shared<FleeBehavior>(flee_speed_default_,
                                                  bounds_min_, bounds_max_);

  class SmartAggressive : public IBehaviorStrategy {
    std::shared_ptr<IBehaviorStrategy> h_, f_;

  public:
    SmartAggressive(std::shared_ptr<IBehaviorStrategy> h,
                    std::shared_ptr<IBehaviorStrategy> f)
        : h_(h), f_(f) {}
    Eigen::Vector3d execute(double dt, const Eigen::Vector3d &c,
                            const Eigen::Vector3d &r, bool is_track) override {
      return (is_track ? f_ : h_)->execute(dt, c, r, is_track);
    }
  };
  behavior_strategies_[ObstacleBehavior::AGGRESSIVE] =
      std::make_shared<SmartAggressive>(hunt_strategy_, flee_strategy_);
  behavior_strategies_[ObstacleBehavior::EVASIVE] = flee_strategy_;

  random_strategy_ = std::make_shared<RandomBehavior>(bounds_min_, bounds_max_);
  behavior_strategies_[ObstacleBehavior::RANDOM] = random_strategy_;

  behavior_strategies_[ObstacleBehavior::HYBRID] =
      std::make_shared<HybridBehavior>(
          patrol_strategy_, behavior_strategies_[ObstacleBehavior::AGGRESSIVE]);

  bounce_strategy_ = std::make_shared<BounceBehavior>(
      0.5, bounds_min_, bounds_max_); // Initial speed 0.5
  behavior_strategies_[ObstacleBehavior::BOUNCE] = bounce_strategy_;

  linear_reciprocate_strategy_ = std::make_shared<LinearReciprocateBehavior>(
      0.5, bounds_min_, bounds_max_);
  behavior_strategies_[ObstacleBehavior::LINEAR_RECIPROCATE] =
      linear_reciprocate_strategy_;

  behavior_strategies_[ObstacleBehavior::STATIONARY] =
      std::make_shared<StationaryBehavior>();

  trajectory_strategies_[TrajectoryType::CIRCLE] =
      std::make_shared<CircleTrajectory>(patrol_center_,
                                         default_trajectory_scale_);
  trajectory_strategies_[TrajectoryType::ELLIPSE] =
      std::make_shared<EllipseTrajectory>(patrol_center_,
                                          default_trajectory_scale_);
  trajectory_strategies_[TrajectoryType::INFINITY_SHAPE] =
      std::make_shared<InfinityTrajectory>(patrol_center_,
                                           default_trajectory_scale_);
  trajectory_strategies_[TrajectoryType::SQUARE] =
      std::make_shared<SquareTrajectory>(patrol_center_,
                                         default_trajectory_scale_);

  behavior_strategies_[ObstacleBehavior::TRAJECTORY] =
      std::make_shared<TrajectoryFollowBehavior>(
          trajectory_strategies_[TrajectoryType::CIRCLE], trajectory_speed_);
}

void ObstacleController::setTrajectoryType(TrajectoryType type) {
  trajectory_type_ = type;
  if (trajectory_strategies_.count(type)) {
    behavior_strategies_[ObstacleBehavior::TRAJECTORY] =
        std::make_shared<TrajectoryFollowBehavior>(trajectory_strategies_[type],
                                                   trajectory_speed_);
  }
}

Eigen::Vector3d ObstacleController::update(double dt,
                                           const Eigen::Vector3d &current_pos,
                                           const Eigen::Vector3d &robot_eef_pos,
                                           ObstacleBehavior behavior,
                                           bool is_tracking) {
  time_elapsed_ += dt;
  Eigen::Vector3d next = current_pos;
  auto it = behavior_strategies_.find(behavior);
  if (it != behavior_strategies_.end())
    next = it->second->execute(dt, current_pos, robot_eef_pos, is_tracking);
  if (dt > 1e-6)
    current_velocity_ = (next - current_pos) / dt;
  return next;
}

void ObstacleController::setHuntSpeed(double s) {
  if (hunt_strategy_)
    hunt_strategy_->setSpeed(s);
}
void ObstacleController::setFleeSpeed(double s) {
  if (flee_strategy_)
    flee_strategy_->setSpeed(s);
}
void ObstacleController::setPatrolSpeed(double s) {
  if (patrol_strategy_)
    patrol_strategy_->setSpeed(s);
}
void ObstacleController::setBounceSpeed(double s) {
  if (bounce_strategy_)
    bounce_strategy_->setSpeed(s);
  if (linear_reciprocate_strategy_)
    linear_reciprocate_strategy_->setSpeed(s);
}

void ObstacleController::setPatrolCenter(const Eigen::Vector3d &c) {
  patrol_center_ = c;
  for (auto &pair : behavior_strategies_) {
    if (pair.second)
      pair.second->setCenter(c);
  }
  for (auto &pair : trajectory_strategies_) {
    if (pair.second)
      pair.second->setCenter(c);
  }
}

Eigen::Vector3d ObstacleController::getPatrolCenter() const {
  if (auto p = std::dynamic_pointer_cast<PatrolBehavior>(patrol_strategy_))
    return p->getCenter();
  return Eigen::Vector3d::Zero();
}

void ObstacleController::setTrajectorySpeed(double s) {
  trajectory_speed_ = s;
  if (auto t = std::dynamic_pointer_cast<TrajectoryFollowBehavior>(
          behavior_strategies_[ObstacleBehavior::TRAJECTORY]))
    t->setSpeed(s);
}

double ObstacleController::getTrajectorySpeed() const {
  if (auto t = std::dynamic_pointer_cast<TrajectoryFollowBehavior>(
          behavior_strategies_.at(ObstacleBehavior::TRAJECTORY)))
    return t->getSpeed();
  return 0.0;
}

void ObstacleController::setTrajectoryScale(double s) {
  for (auto &pair : trajectory_strategies_)
    pair.second->setScale(s);
}

Eigen::Vector3d ObstacleController::getTrajectoryPoint(double angle) const {
  auto it = trajectory_strategies_.find(trajectory_type_);
  if (it != trajectory_strategies_.end())
    return it->second->getPoint(angle);
  return Eigen::Vector3d::Zero();
}

} // namespace simulation
} // namespace robot_sim
