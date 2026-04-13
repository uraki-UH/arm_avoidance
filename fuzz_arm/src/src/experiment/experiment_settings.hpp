#pragma once

namespace robot_sim {
namespace experiment {

/**
 * @brief 実験の動作設定をまとめた構造体
 * シナリオや手法に応じて設定を切り替えるために使用する
 */
struct ExperimentSettings {
  // 衝突監視を行うか (RRT連続追従実験などではfalseにして高速化する)
  bool enable_collision_monitoring = true;

  // 経路の先頭を間引く数 (RRTの初期位置グラつき対策用: 通常2程度)
  int path_pruning_count = 0;

  // 目標到達判定の閾値 [m]
  // (ConfigManagerのデフォルト値より優先される場合はセット)
  double target_tolerance = 0.02;

  // 経路追従の速度スケール
  // (ConfigManagerのデフォルト値より優先される場合はセット)
  double speed_scale = 1.0;

  // 経路を先読みしてバッファリングするか (RRT連続追従用)
  bool use_stockpiling = false;
};

} // namespace experiment
} // namespace robot_sim
