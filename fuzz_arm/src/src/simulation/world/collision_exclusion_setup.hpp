#pragma once

#include "collision/geometric_self_collision_checker.hpp"
#include "simulation/robot/ode/ode_collision_manager.hpp"

namespace simulation {

/**
 * @brief デフォルトの衝突除外ペアを設定 (Geometric版)
 */
inline void
setupDefaultCollisionExclusions(GeometricSelfCollisionChecker &checker) {
  // 設置面に固定されたリンク（link_1）と地面（base_link）の衝突を除外
  checker.addCollisionExclusion("link_1", "base_link");
  checker.addCollisionExclusion("link1", "base_link");
  checker.addCollisionExclusion("base_footprint", "base_link");

  // 地面オブジェクトの別名（ODE等と整合をとるため）
  checker.addCollisionExclusion("link_1", "ground");
  checker.addCollisionExclusion("link1", "ground");
  checker.addCollisionExclusion("base_footprint", "ground");
}

/**
 * @brief モデル構成に基づいて隣接リンクを除外設定に追加 (Geometric版)
 */
inline void
setupAdjacentCollisionExclusions(const RobotModel &model,
                                 GeometricSelfCollisionChecker &checker) {
  for (const auto &[name, joint] : model.getJoints()) {
    checker.addCollisionExclusion(joint.parent_link, joint.child_link);
  }
}

/**
 * @brief デフォルトの衝突除外ペアを設定 (ODE版)
 */
inline void setupDefaultCollisionExclusions(CollisionManager &manager) {
  // ODEでは地面は通常 "ground" という名前で登録される
  manager.addCollisionExclusion("link_1", "ground");
  manager.addCollisionExclusion("link1", "ground");
  manager.addCollisionExclusion("link_1", "base_link");
  manager.addCollisionExclusion("link1", "base_link");

  // base_link と地面の干渉を除外 (固定ベースの場合)
  manager.addCollisionExclusion("base_link", "ground");
  manager.addCollisionExclusion("base_footprint", "ground");

  // sc_ground (RRT用) との干渉も除外
  manager.addCollisionExclusion("link_1", "sc_ground");
  manager.addCollisionExclusion("link1", "sc_ground");
  manager.addCollisionExclusion("base_link", "sc_ground");
  manager.addCollisionExclusion("base_footprint", "sc_ground");
  // stand_link も固定ベースの一部とみなして除外
  manager.addCollisionExclusion("stand_link", "ground");
  manager.addCollisionExclusion("stand_link", "sc_ground");
}

/**
 * @brief モデル構成に基づいて隣接リンクを除外設定に追加 (ODE版)
 */
inline void setupAdjacentCollisionExclusions(const RobotModel &model,
                                             CollisionManager &manager) {
  for (const auto &[name, joint] : model.getJoints()) {
    manager.addCollisionExclusion(joint.parent_link, joint.child_link);
  }
}

/**
 * @brief 移動マニピュレータ用の衝突除外ペアを設定（将来の拡張用）
 */
inline void setupMobileBaseCollisionExclusions(
    [[maybe_unused]] GeometricSelfCollisionChecker &checker) {
  // 将来的に必要になったら実装
}

inline void
setupMobileBaseCollisionExclusions([[maybe_unused]] CollisionManager &manager) {
  // 将来的に必要になったら実装
}

} // namespace simulation
