#pragma once

#include "planning/cost_evaluator.hpp"
#include "safety_engine/runtime/safety_management.hpp"
#include <algorithm>
#include <cmath>
#include <memory>

namespace robot_sim {
namespace simulation {

/**
 * @brief Evaluates path cost by incorporating danger levels from
 * SafetyStateManager.
 * Cost Function:
 * Cost(u, v) = Distance(u, v) * (1.0 + w_risk * max(Danger(u), Danger(v)))
 */
template <typename T_angle, typename T_coord>
class RiskAwareCostEvaluator
    : public planning::ICostEvaluator<T_angle, T_coord> {
public:
  RiskAwareCostEvaluator(std::shared_ptr<SafetyStateManager> safety_manager,
                         float risk_weight = 10.0f)
      : safety_manager_(safety_manager), risk_weight_(risk_weight) {}

  /**
   * @brief Evaluate cost between two nodes incorporating risk.
   */
  float evaluate(const GNG::NeuronNode<T_angle, T_coord> &u,
                 const GNG::NeuronNode<T_angle, T_coord> &v) override {
    // 1. Base Euclidean Distance
    float dist = (u.weight_angle - v.weight_angle).norm();

    // 2. Risk Factor
    // We take the max danger of the two nodes to be conservative.
    float danger_u = safety_manager_->getDangerLevel(u.id);
    float danger_v = safety_manager_->getDangerLevel(v.id);
    float max_danger = std::max(danger_u, danger_v);

    // If danger is 0, factor is 1.0 (pure distance).
    // If danger is 1.0 (max), factor is 1.0 + risk_weight_.
    float risk_factor = 1.0f + risk_weight_ * max_danger;

    return dist * risk_factor;
  }

  /**
   * @brief Get node penalty (optional, for A* heuristics or other usages).
   */
  float getNodePenalty(const GNG::NeuronNode<T_angle, T_coord> &node) override {
    return risk_weight_ * safety_manager_->getDangerLevel(node.id);
  }

  void setRiskWeight(float weight) { risk_weight_ = weight; }

private:
  std::shared_ptr<SafetyStateManager> safety_manager_;
  float risk_weight_;
};

} // namespace simulation
} // namespace robot_sim

