#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "robot_model/robot_model.hpp"
#include "kinematics/kinematic_chain.hpp"

namespace robot_sim::reachability {

// URDF関節範囲の低差異サンプリング。未設定のhas_limitsフラグに依存しない上下限
inline double halton(std::uint64_t idx, std::uint32_t base) {
  double value = 0.0;
  double factor = 1.0;
  while (idx > 0) {
    factor /= static_cast<double>(base);
    value += factor * static_cast<double>(idx % base);
    idx /= base;
  }
  return value;
}

inline std::vector<double> make_halton_joint_values(
    const std::vector<std::pair<double, double>> &joint_limits,
    std::uint64_t sample_idx) {
  static constexpr std::array<std::uint32_t, 16> bases{
      2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53};
  if (joint_limits.size() > bases.size()) throw std::invalid_argument("too many sampling dimensions");
  std::vector<double> values;
  values.reserve(joint_limits.size());
  for (std::size_t idx = 0; idx < joint_limits.size(); ++idx) {
    const auto &[min_value, max_value] = joint_limits[idx];
    const double ratio = halton(sample_idx, bases[idx % bases.size()]);
    values.push_back(min_value + ratio * (max_value - min_value));
  }
  return values;
}

inline std::vector<std::pair<double, double>> collect_joint_limits(
    const simulation::RobotModel &model,
    const kinematics::KinematicChain &chain) {
  std::vector<std::pair<double, double>> limits;
  for (int joint_idx = 0; joint_idx < chain.getNumJoints(); ++joint_idx) {
    const int dof = chain.getJointDOF(joint_idx);
    if (dof <= 0) {
      continue;
    }
    const auto *joint = model.getJoint(chain.getJointName(joint_idx));
    const bool has_limits = joint &&
                            std::isfinite(joint->limits.lower) &&
                            std::isfinite(joint->limits.upper) &&
                            joint->limits.lower < joint->limits.upper;
    for (int dof_idx = 0; dof_idx < dof; ++dof_idx) {
      limits.emplace_back(has_limits ? joint->limits.lower : -M_PI,
                          has_limits ? joint->limits.upper : M_PI);
    }
  }
  if (limits.empty() || limits.size() !=
                            static_cast<std::size_t>(chain.getTotalDOF())) {
    throw std::runtime_error("reachability voxel joint limits are invalid");
  }
  return limits;
}

}  // 名前空間robot_sim::reachabilityの終端
