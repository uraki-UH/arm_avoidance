#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry> //quaternion用
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace kinematics {

/**
 * @brief Normalize angle to [-PI, PI].
 */
inline float normalizeAngle(float angle) {
  while (angle > (float)M_PI)
    angle -= (float)(2.0 * M_PI);
  while (angle < (float)-M_PI)
    angle += (float)(2.0 * M_PI);
  return angle;
}

/**
 * @brief Calculate a scaling factor (0.0 to 1.0) to satisfy joint velocity
 * limits.
 */
inline float calculateVelocityScale(const Eigen::VectorXf &diff, float max_vel,
                                    float dt) {
  float max_step = max_vel * dt;
  float scale = 1.0f;
  for (int i = 0; i < diff.size(); ++i) {
    float d_abs = std::abs(diff[i]);
    if (d_abs > 1e-6f) {
      float s_needed = max_step / d_abs;
      if (s_needed < scale)
        scale = s_needed;
    }
  }
  return scale;
}

/**
 * @brief Handle wraparound for joint difference vector.
 */
inline void applyWraparound(Eigen::VectorXf &diff) {
  for (int i = 0; i < diff.size(); ++i) {
    diff[i] = normalizeAngle(diff[i]);
  }
}

} // namespace kinematics
