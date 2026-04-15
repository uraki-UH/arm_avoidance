#pragma once

#include <Eigen/Dense>
#include <vector>

namespace common {

/**
 * @brief Represents a 3D trajectory in workspace (Cartesian space).
 * Independent of ODE or specific robot details.
 */
class Trajectory {
public:
  Trajectory() = default;

  void clear() { points_.clear(); }

  void addPoint(const Eigen::Vector3d &p) { points_.push_back(p); }

  const std::vector<Eigen::Vector3d> &getPoints() const { return points_; }

  bool empty() const { return points_.empty(); }

  size_t size() const { return points_.size(); }

private:
  std::vector<Eigen::Vector3d> points_;
};

} // namespace common
