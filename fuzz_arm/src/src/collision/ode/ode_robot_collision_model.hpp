#pragma once

#include "collision/iself_collision_checker.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "simulation/robot/ode/ode_collision_manager.hpp"
#include "simulation/robot/ode/ode_robot_builder.hpp"
#include <Eigen/Dense>
#include <map>
#include <ode/ode.h>
#include <string>
#include <vector>

namespace robot_sim {
namespace simulation {
class MeshCache;
}
} // namespace robot_sim

namespace simulation {

using robot_sim::simulation::MeshCache;

/**
 * @brief ODEを利用してロボットの衝突モデルを管理するクラス。
 * 自己干渉(Self Collision)と環境干渉(Environment Collision)を区別して判定可能。
 */
class OdeRobotCollisionModel : public ISelfCollisionChecker {
public:
  OdeRobotCollisionModel(const RobotModel &model, dWorldID world,
                         dSpaceID space, CollisionManager *collision_manager,
                         const kinematics::KinematicChain &chain,
                         MeshCache *mesh_cache = nullptr,
                         bool use_mesh_collision = true); // Added use_mesh_collision toggle
  ~OdeRobotCollisionModel();

  /**
   * @brief 各リンクのワールド姿勢をODEのボディに反映させる
   * @param positions 各ジョイント/リンクのワールド位置
   * @param orientations 各ジョイント/リンクのワールド回転
   */
  void updateBodyPoses(
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
      const std::vector<Eigen::Quaterniond,
                        Eigen::aligned_allocator<Eigen::Quaterniond>>
          &orientations) override;

  /**
   * @brief 現在の状態で衝突判定(dSpaceCollide)を行い、内部接触情報を更新する。
   * @return なにかしらの衝突があれば true
   */
  bool updateCollisionStatus();

  /**
   * @brief ISelfCollisionChecker インターフェース実装。
   * updateCollisionStatus() を呼び出し、hasSelfCollision() の結果を返す。
   */
  bool checkCollision() override;

  /**
   * @brief ロボット同士の自己干渉があるか (updateCollisionStatus 後に呼ぶ)
   */
  bool hasSelfCollision() const;

  /**
   * @brief ロボットと環境(地面・障害物)との干渉があるか (updateCollisionStatus
   * 後に呼ぶ)
   */
  bool hasEnvironmentCollision() const;

  /**
   * @brief 2つの関節角度の間の直線経路に衝突があるかチェックする
   * @param q1 開始角度
   * @param q2 終了角度
   * @param steps 分割数
   * @return true: 衝突あり, false: 衝突なし
   */
  bool isPathColliding(const Eigen::VectorXf &q1, const Eigen::VectorXf &q2,
                       int steps = 10);

private:
  dWorldID world_;
  dSpaceID space_;
  CollisionManager *collision_manager_;
  std::map<std::string, OdeRobotComponent> components_;
  std::vector<std::string> link_names_order_; // Link order corresponding to
                                              // positions from KinematicChain
  const RobotModel &model_;
};

} // namespace simulation
