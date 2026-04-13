#pragma once

#include <Eigen/Dense>
#include <vector>

namespace simulation {

/**
 * @brief 自己干渉チェックの共通インターフェース
 * ODE版と幾何計算版(Geometric)の両方を同一視して扱うための基底クラス
 */
class ISelfCollisionChecker {
public:
  virtual ~ISelfCollisionChecker() = default;

  /**
   * @brief ロボットの各リンク/ボディの姿勢を更新する
   * @param positions 各リンクのワールド座標 (KinematicChainの出力順)
   * @param orientations 各リンクのワールド回転 (KinematicChainの出力順)
   */
  virtual void updateBodyPoses(
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
      const std::vector<Eigen::Quaterniond,
                        Eigen::aligned_allocator<Eigen::Quaterniond>>
          &orientations) = 0;

  /**
   * @brief 現在の姿勢で干渉があるかチェックする
   * @return true: 干渉あり, false: 干渉なし
   */
  virtual bool checkCollision() = 0;
};

} // namespace simulation
