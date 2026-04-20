#pragma once

#include "planning/cost_evaluator.hpp"
#include <Eigen/Dense>

namespace planning {

/**
 * ワークスペース（デカルト座標系）でのユークリッド距離に基づくコスト評価クラス。
 * ノードの weight_coord（通常はエンドエフェクタの位置）を使用する。
 */
template <typename T_angle, typename T_coord>
class WorkspaceDistanceCost : public ICostEvaluator<T_angle, T_coord> {
public:
  /**
   * @param inactive_penalty
   * 非アクティブノードに適用するペナルティ値（デフォルト: 1000.0f）
   */
  WorkspaceDistanceCost(float inactive_penalty = 1000.0f)
      : inactive_penalty_(inactive_penalty) {}

  /**
   * ノードuからノードvへの移動コストを計算する。
   * @param u ソースノード
   * @param v デスティネーションノード
   * @return 移動コスト。非アクティブノードにはペナルティが追加される。
   */
  float evaluate(const GNG::NeuronNode<T_angle, T_coord> &u,
                 const GNG::NeuronNode<T_angle, T_coord> &v) override {
    float base_dist = (u.weight_coord - v.weight_coord).norm();

    // 探訪ノードが非アクティブの場合、ペナルティを追加
    if (!v.status.active) {
      base_dist += inactive_penalty_;
    }

    return base_dist;
  }

private:
  float inactive_penalty_; // 非アクティブノードに適用するペナルティ値
};

} // namespace planning