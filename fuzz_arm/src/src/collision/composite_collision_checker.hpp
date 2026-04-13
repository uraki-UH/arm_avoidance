#pragma once

#include "collision/environment_collision_checker.hpp"
#include "collision/geometric_self_collision_checker.hpp"
#include "collision/iself_collision_checker.hpp"
#include <memory>

namespace simulation {

/**
 * @brief 複数の CollisionChecker（自己干渉、環境衝突など）を統合して扱うクラス
 * ISelfCollisionChecker インターフェースを実装し、既存の GNG
 * ワークフローに適合させる
 */
class CompositeCollisionChecker : public ISelfCollisionChecker {
public:
  CompositeCollisionChecker() = default;
  ~CompositeCollisionChecker() override = default;

  void setSelfCollisionChecker(
      std::shared_ptr<GeometricSelfCollisionChecker> checker) {
    self_checker_ = checker;
  }

  void setEnvironmentCollisionChecker(
      std::shared_ptr<EnvironmentCollisionChecker> checker) {
    env_checker_ = checker;
  }

  // --- ISelfCollisionChecker Interface ---
  void updateBodyPoses(
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
      const std::vector<Eigen::Quaterniond,
                        Eigen::aligned_allocator<Eigen::Quaterniond>>
          &orientations) override {
    if (self_checker_) {
      self_checker_->updateBodyPoses(positions, orientations);
    }
    // 環境チェッカーは静的なので基本的には update 不要だが、
    // ロボット側の形状データ（collision_objects_）を最新にする必要がある。
  }

  bool checkCollision() override {
    // 1. 自己干渉チェック
    if (self_checker_ && self_checker_->checkCollision()) {
      return true;
    }

    // 2. 環境衝突チェック
    if (env_checker_ && self_checker_) {
      // GeometricSelfCollisionChecker
      // から現在のロボットリンク形状を取得して渡す
      if (env_checker_->checkCollision(self_checker_->getCollisionObjects())) {
        return true;
      }
    }

    return false;
  }

private:
  std::shared_ptr<GeometricSelfCollisionChecker> self_checker_;
  std::shared_ptr<EnvironmentCollisionChecker> env_checker_;
};

} // namespace simulation
