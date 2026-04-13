#pragma once

#include "planning/cost_evaluator.hpp"
#include "simulation/core/simulation_state.hpp"
#include <Eigen/Dense>

namespace robot_sim {
namespace simulation {

/**
 * @brief プランニングを調整するクラス
 *
 * プランニング手法の切り替え、経路計画の実行、プランナーの初期化を管理します。
 */
// Forward declarations
namespace common {
class ConfigReader;
}

/**
 * @brief プランニングを調整するクラス
 *
 * プランニング手法の切り替え、経路計画の実行、プランナーの初期化を管理します。
 */
class PlanningCoordinator {
public:
  PlanningCoordinator();
  ~PlanningCoordinator();

  /**
   * @brief High-Speed Dynamic Obstacle Systemのパラメータ設定
   */
  struct AnalysisSystemConfig {
    double sensing_resolution = 0.05;
    double spatial_map_resolution = 0.008;
    std::string spatial_index_type = "sparse";
    std::string spatial_correlation_file = "gng_spatial_correlation.bin";
    double obstacle_radius = 0.04;

    // Bounds
    double min_x = -2.0, min_y = -2.0, min_z = -1.0;
    double max_x = 2.0, max_y = 2.0, max_z = 2.0;
  };

  /**
   * @brief 初期化メソッド群
   */
  bool initializeAnalysisSystem(SimulationState &state);
  bool initializePlanningSystem(SimulationState &state);

  /**
   * @brief Control Systemの初期化 (Controller, ReactiveController)
   */
  bool initializeControlSystem(SimulationState &state);

  struct PlanningConfig {
    // RRT Params
    double rrt_goal_tolerance = 0.008;
    int rrt_max_ik_iterations = 10000;
    double rrt_step_size = 0.1;
    int rrt_max_ik_samples = 25;
    double rrt_max_planning_time_ms = 10.0;

    // Metric / Evaluator
    std::string cost_metric = "distance"; // distance, risk
  };

  /**
   * @brief 経路計画を実行し、コントローラーに設定
   * @param start_q 開始姿勢
   * @param goal_q 目標姿勢
   * @param state シミュレーション状態
   */
  void planAndExecute(const Eigen::VectorXf &start_q,
                      const Eigen::VectorXf &goal_q, SimulationState &state);

  /**
   * @brief ランダムな姿勢を選択
   * @param state シミュレーション状態
   */
  void pickRandomPosture(SimulationState &state);

  /**
   * @brief ランダムなGNGノードを選択
   * @param state シミュレーション状態
   */
  void pickRandomGngNode(SimulationState &state);

  /**
   * @brief プランニング手法を切り替え
   * @param method プランニング手法
   * @param state シミュレーション状態
   */
  void switchPlanningMethod(PlanningMethod method, SimulationState &state);

  /**
   * @brief コストメトリックと評価器を設定
   * @param metric 選択されたコストメトリック
   * @param state シミュレーション状態
   * @param risk_evaluator RISKメトリック選択時に使用する評価器（オプション）
   */
  void
  setCostMetric(PlanningMetric metric, SimulationState &state,
                std::shared_ptr<
                    planning::ICostEvaluator<Eigen::VectorXf, Eigen::Vector3f>>
                    risk_evaluator = nullptr);

private:
  int selectRandomActiveNode(const SimulationState &state);
};

} // namespace simulation
} // namespace robot_sim
