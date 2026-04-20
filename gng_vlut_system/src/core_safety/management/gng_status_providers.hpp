#pragma once

#include "core_safety/gng/node_status.hpp"

#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "kinematics/kinematic_chain.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "description/robot_dynamics.hpp"
#include "description/robot_model.hpp"
#include "status/manipulability.hpp"

namespace GNG {

/**
 * @brief エンドエフェクタの方向と関節位置を付与するプロバイダー
 */
template <typename T_angle, typename T_coord>
class EEDirectionProvider
    : public GrowingNeuralGas<T_angle, T_coord>::IStatusProvider {
  kinematics::KinematicChain *chain_;

public:
  EEDirectionProvider(kinematics::KinematicChain *chain) : chain_(chain) {}

  std::vector<typename GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger>
  getTriggers() const override {
    return {GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger::BATCH_UPDATE};
  }

  void update(NeuronNode<T_angle, T_coord> &node,
              [[maybe_unused]]
              typename GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger
                  trigger) override {
    if (!chain_)
      return;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts;
    std::vector<Eigen::Quaterniond,
                Eigen::aligned_allocator<Eigen::Quaterniond>>
        orientations;
    chain_->forwardKinematicsAt(node.weight_angle, pts, orientations);

    // 手先方向 (末端リンクのX軸)
    node.status.ee_direction =
        (orientations.back() * Eigen::Vector3d::UnitX()).cast<float>();

    // 全関節位置を保存 (プランニング時の方向評価用)
    node.status.joint_positions.clear();
    for (const auto &p : pts) {
      node.status.joint_positions.push_back(p.cast<float>());
    }
  }
};

/**
 * @brief 運動学的可操作性を計算するプロバイダー
 */
template <typename T_angle, typename T_coord>
class ManipulabilityProvider
    : public GrowingNeuralGas<T_angle, T_coord>::IStatusProvider {
  kinematics::KinematicChain *chain_;

public:
  ManipulabilityProvider(kinematics::KinematicChain *chain) : chain_(chain) {}

  std::vector<typename GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger>
  getTriggers() const override {
    return {GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger::BATCH_UPDATE};
  }

  void update(NeuronNode<T_angle, T_coord> &node,
              [[maybe_unused]]
              typename GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger
                  trigger) override {
    if (!chain_)
      return;

    // 1. ヤコビ行列の取得 (位置 3 x N)
    // Calculate at TIP (index = num_joints + 1)
    std::vector<double> joints(node.weight_angle.size());
    for (int i = 0; i < node.weight_angle.size(); ++i)
      joints[i] = node.weight_angle(i);
    Eigen::MatrixXd J =
        chain_->calculateJacobianAt(chain_->getNumJoints() + 1, joints);
    Eigen::MatrixXd Jv = J.topRows(3);

    // 2. 楕円体の計算
    node.status.manip_info = Manipulability::calculateManipulabilityEllipsoid(
        Jv, Manipulability::KINEMATIC);

    // 互換性用フィールドの更新
    node.status.min_singular_value =
        (float)node.status.manip_info.singular_values.minCoeff();
    node.status.combined_score =
        node.status.min_singular_value * node.status.joint_limit_score;
  }
};

/**
 * @brief 動的可操作性を計算するプロバイダー
 */
template <typename T_angle, typename T_coord>
class DynamicManipulabilityProvider
    : public GrowingNeuralGas<T_angle, T_coord>::IStatusProvider {
  kinematics::KinematicChain *chain_;
  simulation::RobotModel *model_;

public:
  DynamicManipulabilityProvider(kinematics::KinematicChain *chain,
                                simulation::RobotModel *model)
      : chain_(chain), model_(model) {}

  std::vector<typename GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger>
  getTriggers() const override {
    return {GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger::BATCH_UPDATE};
  }

  void update(NeuronNode<T_angle, T_coord> &node,
              [[maybe_unused]]
              typename GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger
                  trigger) override {
    if (!chain_ || !model_)
      return;

    std::vector<double> joints(node.weight_angle.size());
    for (int i = 0; i < node.weight_angle.size(); ++i)
      joints[i] = node.weight_angle(i);

    // 1. 質量行列の取得
    auto M_opt =
        dynamics::RobotDynamics::calculateMassMatrix(*chain_, *model_, joints);
    if (!M_opt) {
      node.status.dynamic_manip_info.valid = false;
      return;
    }

    // 2. ヤコビ行列 (位置)
    Eigen::MatrixXd J =
        chain_->calculateJacobianAt(chain_->getNumJoints(), joints);
    Eigen::MatrixXd Jv = J.topRows(3);

    // 3. 動的可操作性楕円体の計算
    node.status.dynamic_manip_info =
        Manipulability::calculateDynamicManipulabilityEllipsoid(Jv, *M_opt);
  }
};

} // namespace GNG
