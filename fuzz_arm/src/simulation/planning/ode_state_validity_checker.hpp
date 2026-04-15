#pragma once

#include "collision/ode/ode_robot_collision_model.hpp"
#include "collision/point_cloud_collision_checker.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include "simulation/sensing/voxel_grid.hpp"
#include "simulation/world/environment_manager.hpp"
#include <Eigen/Dense>

namespace robot_sim {
namespace simulation {

/**
 * @brief ODE物理エンジンを使用した状態有効性チェッカー
 */
class OdeStateValidityChecker : public planner::StateValidityChecker {
public:
  /**
   * @brief コンストラクタ
   * @param self_checker 自己衝突チェッカー -> RobotCollisionModel
   * @param pc_checker 点群衝突チェッカー
   * @param grid 点群ボクセルグリッド
   * @param env_manager 環境マネージャ
   * @param chain キネマティクスチェーン
   */
  OdeStateValidityChecker(::simulation::OdeRobotCollisionModel *model,
                          ::simulation::PointCloudCollisionChecker *pc_checker,
                          const ::simulation::VoxelGrid *grid,
                          const ::simulation::EnvironmentManager *env_manager,
                          const ::kinematics::KinematicChain *chain);

  bool isValid(const Eigen::VectorXd &q) const override;
  bool isValid(const Eigen::VectorXd &q, double margin) const;

  // Metrics overrides
  void resetMetrics() const override;
  size_t getCheckCount() const override;
  double getTotalCheckTimeUs() const override;

  void setGoalExclusion(const Eigen::Vector3d &pos, double radius) {
    has_exclusion_ = true;
    exclusion_pos_ = pos;
    exclusion_radius_ = radius;
  }
  void clearExclusion() { has_exclusion_ = false; }

private:
  ::simulation::OdeRobotCollisionModel *collision_model_;
  ::simulation::PointCloudCollisionChecker *pc_checker_;
  const ::simulation::VoxelGrid *grid_;
  const ::simulation::EnvironmentManager *env_manager_;
  const ::kinematics::KinematicChain *chain_;

  bool has_exclusion_ = false;
  Eigen::Vector3d exclusion_pos_ = Eigen::Vector3d::Zero();
  double exclusion_radius_ = 0.05;

  // Mutable metrics
  mutable size_t check_count_ = 0;
  mutable double total_check_time_us_ = 0.0;
};

} // namespace simulation
} // namespace robot_sim
