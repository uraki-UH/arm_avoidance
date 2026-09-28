#pragma once

#include <Eigen/Core>
#include <cmath>

namespace motion_smoothing {

// 関節速度制限に基づく全関節共通の変位倍率
inline float calculate_velocity_scale(const Eigen::VectorXf &diff,
                                      float max_joint_velocity, float duration_sec) {
  const float max_step = max_joint_velocity * duration_sec;
  float scale = 1.0f;
  for (Eigen::Index idx = 0; idx < diff.size(); ++idx) {
    const float abs_delta = std::abs(diff[idx]);
    if (abs_delta > 1e-6f) {
      const float required_scale = max_step / abs_delta;
      if (required_scale < scale) {
        scale = required_scale;
      }
    }
  }
  return scale;
}

} // namespace motion_smoothing
