#pragma once

#include "collision/ode/ode_robot_collision_model.hpp"
#include "gng/GrowingNeuralGas_offline2.hpp" // Use GNG2
#include "kinematics/kinematic_chain.hpp"
#include <Eigen/Dense>

namespace GNG {

/**
 * @brief
 * ODEを利用して自己衝突をチェックし、ノードの有効性を判定するプロバイダー
 */
template <typename T_angle, typename T_coord>
class SelfCollisionProvider
    : public GrowingNeuralGas2<T_angle, T_coord>::IStatusProvider {
public:
  SelfCollisionProvider(simulation::OdeRobotCollisionModel *checker,
                        kinematics::KinematicChain *chain)
      : checker_(checker), chain_(chain) {}

  std::vector<typename GrowingNeuralGas2<T_angle, T_coord>::UpdateTrigger>
  getTriggers() const override {
    // ノードが追加された時、または重み（関節角）が更新された時にチェック
    return {GrowingNeuralGas2<T_angle, T_coord>::UpdateTrigger::NODE_ADDED,
            GrowingNeuralGas2<T_angle, T_coord>::UpdateTrigger::COORD_UPDATED,
            GrowingNeuralGas2<T_angle, T_coord>::UpdateTrigger::BATCH_UPDATE};
  }

  void update(NeuronNode<T_angle, T_coord> &node,
              [[maybe_unused]]
              typename GrowingNeuralGas2<T_angle, T_coord>::UpdateTrigger
                  trigger) override {
    if (!checker_ || !chain_)
      return;

    // 1. 関節角をKinematicChainにセットしてFK計算
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        positions;
    std::vector<Eigen::Quaterniond,
                Eigen::aligned_allocator<Eigen::Quaterniond>>
        orientations;
    // forwardKinematicsAt を使用してノードの姿勢を計算
    chain_->forwardKinematicsAt(node.weight_angle, positions, orientations);

    // 2. 自己衝突チェッカーに姿勢を転送
    checker_->updateBodyPoses(positions, orientations);

    // 3. 衝突チェック実行
    bool is_colliding = checker_->checkCollision();

    // 4. ノードのステータスに反映
    node.status.valid = !is_colliding;

    // 外部デバッグ用にメタデータにも記録（オプション）
    node.status.metadata["self_collision"] = is_colliding ? 1.0f : 0.0f;
  }

private:
  simulation::OdeRobotCollisionModel *checker_;
  kinematics::KinematicChain *chain_;
};

} // namespace GNG
