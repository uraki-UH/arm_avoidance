#pragma once

#include "simulation/core/simulation_state.hpp"
#include <chrono>
#include <string>

namespace robot_sim {
namespace simulation {

/**
 * @brief 実験を管理するクラス
 *
 * 実験の開始・停止・リセット、シナリオ管理、ロギングを一元管理します。
 */
class ExperimentController {
public:
  ExperimentController();
  ~ExperimentController();

  /**
   * @brief 実験を開始
   * @param method プランニング手法 ("GNG" or "RRT")
   * @param state シミュレーション状態
   */
  void startExperiment(const std::string &method, SimulationState &state);

  /**
   * @brief 実験をリセット
   * @param state シミュレーション状態
   */
  void resetExperiment(SimulationState &state);

  /**
   * @brief 実験を更新（毎フレーム呼び出し）
   * @param dt デルタタイム
   * @param state シミュレーション状態
   */
  void update(double dt, SimulationState &state);

  /**
   * @brief 実験が実行中か確認
   */
  bool isExperimentActive() const { return is_active_; }

private:
  bool is_active_ = false;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

} // namespace simulation
} // namespace robot_sim
