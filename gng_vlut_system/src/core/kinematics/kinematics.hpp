#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

#include "kinematics/joint.hpp"
#include "kinematics/kinematic_chain.hpp"

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
 * @brief Handle wraparound for joint difference vector.
 */
inline void applyWraparound(Eigen::VectorXf &diff) {
  for (int i = 0; i < diff.size(); ++i) {
    diff[i] = normalizeAngle(diff[i]);
  }
}

} // namespace kinematics
