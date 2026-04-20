#pragma once

#include "core_safety/analysis/iself_collision_checker.hpp"
#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "kinematics/kinematic_chain.hpp"
#include <Eigen/Dense>

namespace GNG {

/**
 * @brief
 * ISelfCollisionCheckerを利用して自己衝突をチェックし、ノードの有効性を判定するプロバイダー
 */
template <typename T_angle, typename T_coord>
class SelfCollisionProvider
    : public GrowingNeuralGas<T_angle, T_coord>::IStatusProvider {
public:
  SelfCollisionProvider(simulation::ISelfCollisionChecker *checker,
                        kinematics::KinematicChain *chain)
      : checker_(checker), chain_(chain) {}

  std::vector<typename GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger>
  getTriggers() const override {
    // ノードが追加された時、または重み（関節角）が更新された時にチェック
    return {GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger::NODE_ADDED,
            GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger::COORD_UPDATED,
            GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger::BATCH_UPDATE};
  }

  void update(NeuronNode<T_angle, T_coord> &node,
              [[maybe_unused]]
              typename GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger
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
  simulation::ISelfCollisionChecker *checker_;
  kinematics::KinematicChain *chain_;
};

} // namespace GNG
