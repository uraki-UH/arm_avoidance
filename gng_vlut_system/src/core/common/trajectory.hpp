#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace robot_sim::common {

/**
 * @brief Represents a trajectory in an N-dimensional vector space.
 * Use Dim = 3 for Cartesian positions, or Eigen::Dynamic for joint-space
 * / angle-space sequences with runtime-determined dimensionality.
 */
template <typename Scalar = double, int Dim = Eigen::Dynamic>
class Trajectory {
public:
  Trajectory() = default;

  using VectorType = Eigen::Matrix<Scalar, Dim, 1>;

  void clear() { points_.clear(); }

  void reserve(std::size_t count) { points_.reserve(count); }

  void addPoint(const VectorType &p) { points_.push_back(p); }

  const std::vector<VectorType> &getPoints() const { return points_; }

  bool empty() const { return points_.empty(); }

  std::size_t size() const { return points_.size(); }

private:
  std::vector<VectorType> points_;
};

struct TrajectoryState {
  bool update_requested = true;
  bool valid = false;
  std::vector<Eigen::VectorXf> bridge_path;
  std::size_t bridge_index = 0;
  bool bridge_valid = false;
  std::vector<Eigen::VectorXf> goal_bridge_path;
  std::size_t goal_bridge_index = 0;
  bool goal_bridge_valid = false;
  std::vector<int> node_path;
  std::size_t waypoint_index = 0;
  int goal_id = -1;
  std::vector<int> goal_candidates;

  void clear(bool keep_goal_id = false) {
    valid = false;
    bridge_valid = false;
    bridge_path.clear();
    bridge_index = 0;
    goal_bridge_valid = false;
    goal_bridge_path.clear();
    goal_bridge_index = 0;
    node_path.clear();
    waypoint_index = 0;
    if (!keep_goal_id) {
      goal_id = -1;
    }
  }
};

using Trajectory3d = Trajectory<double, 3>;
using TrajectoryXd = Trajectory<double, Eigen::Dynamic>;

} // namespace robot_sim::common
