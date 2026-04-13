#pragma once

#include "planning/cost_evaluator.hpp"
#include <Eigen/Dense>

namespace planning {

/**
 * Basic cost evaluator that calculates Euclidean distance in joint space.
 * Optionally applies a penalty for inactive nodes.
 */
template <typename T_angle, typename T_coord>
class JointDistanceCost : public ICostEvaluator<T_angle, T_coord> {
public:
  JointDistanceCost(float inactive_penalty = 1000.0f)
      : inactive_penalty_(inactive_penalty) {}

  float evaluate(const GNG::NeuronNode<T_angle, T_coord> &u,
                 const GNG::NeuronNode<T_angle, T_coord> &v) override {
    float base_dist = (u.weight_angle - v.weight_angle).norm();

    // Add penalty if the destination node is inactive
    if (!v.status.active) {
      base_dist += inactive_penalty_;
    }

    return base_dist;
  }

  float getNodePenalty(const GNG::NeuronNode<T_angle, T_coord> &node) override {
    return node.status.active ? 0.0f : inactive_penalty_;
  }

private:
  float inactive_penalty_;
};

} // namespace planning
