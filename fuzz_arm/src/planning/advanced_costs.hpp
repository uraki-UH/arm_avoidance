#pragma once

#include "planning/cost_evaluator.hpp"
#include "status/manipulability.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace planning {

/**
 * @brief 運動学的（静的）可操作性の逆数に基づくコスト
 */
template <typename T_angle, typename T_coord>
class KinematicManipulabilityCost : public ICostEvaluator<T_angle, T_coord> {
public:
  float evaluate(const GNG::NeuronNode<T_angle, T_coord> &u,
                 const GNG::NeuronNode<T_angle, T_coord> &v) override {
    float distance = (v.weight_angle - u.weight_angle).norm();
    double m = v.status.manip_info.manipulability;
    // 特異姿勢付近でのコスト爆発を抑えるための微小値
    double cost_weight = 1.0 / (m + 1e-4);
    return distance * (float)cost_weight;
  }
};

/**
 * @brief 動的可操作性の逆数に基づくコスト
 */
template <typename T_angle, typename T_coord>
class DynamicManipulabilityCost : public ICostEvaluator<T_angle, T_coord> {
public:
  float evaluate(const GNG::NeuronNode<T_angle, T_coord> &u,
                 const GNG::NeuronNode<T_angle, T_coord> &v) override {
    float distance = (v.weight_angle - u.weight_angle).norm();
    if (!v.status.dynamic_manip_info.valid) {
      return distance * 1000.0f; // 動的パラメータがない場合は高いコスト
    }
    double m = v.status.dynamic_manip_info.manipulability;
    double cost_weight = 1.0 / (m + 1e-4);
    return distance * (float)cost_weight;
  }
};

/**
 * @brief 進行方向に対する可操作性の半径に基づくコスト
 */
template <typename T_angle, typename T_coord>
class DirectionalManipulabilityCost : public ICostEvaluator<T_angle, T_coord> {
public:
  float evaluate(const GNG::NeuronNode<T_angle, T_coord> &u,
                 const GNG::NeuronNode<T_angle, T_coord> &v) override {
    float distance = (v.weight_angle - u.weight_angle).norm();

    if (v.status.joint_positions.empty() || u.status.joint_positions.empty()) {
      return distance;
    }

    // エンドエフェクタ（末端リンク）の移動方向をワークスペースで評価
    Eigen::Vector3f pos_u = u.status.joint_positions.back();
    Eigen::Vector3f pos_v = v.status.joint_positions.back();
    Eigen::Vector3f dir = (pos_v - pos_u).normalized();

    // v の可操作性楕円体における、dir 方向の半径を評価
    double r = Manipulability::evaluateDirectionalManipulability(
        v.status.manip_info, dir.cast<double>());
    double cost_weight = 1.0 / (r + 1e-4);

    return distance * (float)cost_weight;
  }
};

} // namespace planning
