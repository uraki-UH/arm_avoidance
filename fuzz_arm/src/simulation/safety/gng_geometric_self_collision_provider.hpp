#pragma once

#include "collision/geometric_self_collision_checker.hpp"
#include "gng/GrowingNeuralGas_offline.hpp"
#include "kinematics/kinematic_chain.hpp"
#include <Eigen/Dense>

namespace GNG {

/**
 * @brief ODEを使用しない、幾何計算ベースの自己干渉チェックプロバイダー
 */
template <typename T_angle, typename T_coord>
class GeometricSelfCollisionProvider
    : public GrowingNeuralGas2<T_angle, T_coord>::IStatusProvider {
public:
  GeometricSelfCollisionProvider(simulation::ISelfCollisionChecker *checker,
                                 kinematics::KinematicChain *chain)
      : checker_(checker), chain_(chain) {}

  std::vector<typename GrowingNeuralGas2<T_angle, T_coord>::UpdateTrigger>
  getTriggers() const override {
    return {GrowingNeuralGas2<T_angle, T_coord>::UpdateTrigger::NODE_ADDED,
            GrowingNeuralGas2<T_angle, T_coord>::UpdateTrigger::COORD_UPDATED,
            GrowingNeuralGas2<T_angle, T_coord>::UpdateTrigger::BATCH_UPDATE};
  }

  void update(NeuronNode<T_angle, T_coord> &node,
              [[maybe_unused]] typename GrowingNeuralGas2<T_angle, T_coord>::UpdateTrigger trigger) override {
    if (!checker_ || !chain_)
      return;

    // 1. FK計算
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
    chain_->forwardKinematicsAt(node.weight_angle, positions, orientations);

    // 2. 姿勢の更新
    checker_->updateBodyPoses(positions, orientations);

    // 3. 衝突チェック
    bool is_colliding = checker_->checkCollision();

    // 4. 反映
    node.status.valid = !is_colliding;
    node.status.is_colliding = is_colliding;
  }

private:
  simulation::ISelfCollisionChecker *checker_;
  kinematics::KinematicChain *chain_;
};

} // namespace GNG
