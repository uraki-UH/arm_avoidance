#pragma once

#include <memory>

namespace robot_sim {
namespace simulation {

// Forward declarations
class SimulationState;
class ReactiveTrackingModule;

/**
 * @brief リアクティブ制御を管理するクラス
 *
 * リアクティブ回避、動的障害物への対応、予測フィールドの管理を行います。
 * 内部的にReactiveTrackingModuleを使用して複雑な追跡ロジックを実装します。
 */
class ReactiveController {
public:
  ReactiveController();
  ~ReactiveController();

  // コピー禁止
  ReactiveController(const ReactiveController &) = delete;
  ReactiveController &operator=(const ReactiveController &) = delete;

  /**
   * @brief リアクティブ制御を更新
   * @param dt デルタタイム
   * @param state シミュレーション状態
   */
  void update(double dt, SimulationState &state);

  /**
   * @brief 動的障害物の予測を処理
   * @param state シミュレーション状態
   */
  void handleDynamicObstaclePrediction(SimulationState &state);

  /**
   * @brief 追跡モジュールの可視化を描画
   * @param state シミュレーション状態
   */
  void draw(const SimulationState &state) const;

  /**
   * @brief リアクティブモードを設定
   * @param enabled 有効/無効
   */
  void setReactiveMode(bool enabled) { reactive_mode_enabled_ = enabled; }

  /**
   * @brief リアクティブモードが有効か確認
   */
  bool isReactiveMode() const { return reactive_mode_enabled_; }

private:
  bool reactive_mode_enabled_ = false;
  std::unique_ptr<ReactiveTrackingModule> tracking_module_;
};

} // namespace simulation
} // namespace robot_sim
