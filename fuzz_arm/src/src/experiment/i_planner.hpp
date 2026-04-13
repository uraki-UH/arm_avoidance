#pragma once
#include <Eigen/Dense>
#include <vector>

namespace robot_sim {
namespace experiment {

/**
 * @brief 実験用プランナーインターフェース
 */
class IPlanner {
public:
  struct PlanStats {
    double planning_time_ms = 0.0;
    size_t node_count = 0;
    size_t collision_check_count = 0;
    double total_check_time_us = 0.0;
    double avg_check_time_us = 0.0;
    bool success = false;
  };

  virtual ~IPlanner() = default;

  /**
   * @brief 計画実行
   * @param start_q 開始関節角
   * @param target_pos 目標デカルト座標
   * @return 関節軌道 (vector of q). 失敗時は空。
   */
  virtual std::vector<Eigen::VectorXd>
  plan(const Eigen::VectorXd &start_q, const Eigen::Vector3d &target_pos) = 0;

  /**
   * @brief プランナーの名称取得 (ログ用)
   */
  virtual std::string getName() const = 0;

  /**
   * @brief 可操作度計算の計算負荷をデータとして取得するフェーズ
   * @param state 計測対象の姿勢
   * @return 計算にかかった時間 (ms)
   */
  virtual double
  measureManipulabilityOverhead(const Eigen::VectorXd &state) = 0;

  /**
   * @brief 直近の計画統計情報を取得
   */
  virtual PlanStats getLastPlanStats() const = 0;

  /**
   * Check if the remaining path is still valid (e.g. not colliding).
   * @param current_q Current joint state.
   * @param progress_index Current progress index within the current path.
   */
  virtual bool isPathValid(const Eigen::VectorXd &current_q,
                           size_t progress_index) const = 0;
};

} // namespace experiment
} // namespace robot_sim
