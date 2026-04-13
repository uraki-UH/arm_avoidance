#pragma once

// #include "simulation/core/simulation_state.hpp"  // Temporarily disabled
// (Phase 1)
#include <Eigen/Dense>
#include <vector>

namespace robot_sim {
namespace simulation {

// Forward declarations
class SimulationState;
struct CandidatePathVisualization;

/**
 * @brief 可視化を管理するクラス
 *
 * GNGノード、エッジ、ロボット、障害物、マニピュラビリティ楕円体などの
 * 描画ロジックを一元管理します。
 */
class VisualizationManager {
public:
  VisualizationManager() = default;
  ~VisualizationManager() = default;

  /**
   * @brief ワイヤーフレームボックスを描画
   * @param center ボックスの中心位置
   * @param size ボックスのサイズ
   */
  void drawWireframeBox(const Eigen::Vector3f &center,
                        const Eigen::Vector3f &size);

  /**
   * @brief マニピュラビリティ楕円体を描画
   */
  static void drawEllipsoid(const Eigen::Vector3d &pos,
                            const Eigen::Vector3d &singular_values,
                            const Eigen::Matrix3d &principal_directions,
                            float r, float g, float b, float alpha);

  /**
   * @brief マニピュラビリティ可視化を描画
   * @param state シミュレーション状態
   */
  void drawManipulabilityViz(const SimulationState &state);

  /**
   * @brief ターゲット球を描画
   */
  void drawTargetSphere(const Eigen::Vector3d &pos,
                        const Eigen::Quaterniond &quat, double radius,
                        bool show_axes);

  /**
   * @brief 候補パスを描画
   */
  void drawCandidatePaths(const std::vector<CandidatePathVisualization> &paths);

  /**
   * @brief 障害物ボクセルを描画
   */
  void drawObstacleVoxels(const SimulationState &state);

  /**
   * @brief 障害物ボクセルを描画（オーバーロード）
   */
  void drawObstacleVoxels(const std::vector<long> &occupied_voxels,
                          float voxel_size);

  /**
   * @brief デンジャーフィールドを描画
   */
  /**
   * @brief デンジャーフィールドを描画
   */
  void drawDangerField(const SimulationState &state);

  /**
   * @brief 汎用パス描画 (Cartesian coordinates)
   */
  void drawPath(const std::vector<Eigen::Vector3d> &points, float r, float g,
                float b);

  /**
   * @brief 汎用球体描画
   */
  void drawSphere(const Eigen::Vector3d &center, float radius, float r, float g,
                  float b, float alpha = 1.0f);

  /**
   * @brief 影響ノードを描画
   */
  void drawInfluenceNodes(const SimulationState &state);

private:
  int debug_counter_ = 0;
};

} // namespace simulation
} // namespace robot_sim
