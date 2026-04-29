#pragma once

#include "safety_engine/gng/GrowingNeuralGas.hpp"
#include <Eigen/Dense>

namespace planning {

/**
 * ノード間、または特定のノードでのコストやリスクを評価するためのインターフェース。
 * パスプランナーが最適なトラジェクトリを決定するために使用される。
 */
template <typename T_angle, typename T_coord> class ICostEvaluator {
public:
  virtual ~ICostEvaluator() = default;

  /**
   * ノードuからノードvへの移動コストを評価する。
   * @param u ソースノード
   * @param v デスティネーションノード
   * @return 移動コスト。値が高いほど移動が難しい、またはリスクが高いことを意味する。
   */
  virtual float evaluate(const GNG::NeuronNode<T_angle, T_coord> &u,
                         const GNG::NeuronNode<T_angle, T_coord> &v) = 0;

  /**
   * 特定のノードにいることによるペナルティやリスクを評価する。
   * 'アクティブ' ステータスや障害物への近さをコストとして組み込むのに便利。
   */
  virtual float getNodePenalty(const GNG::NeuronNode<T_angle, T_coord> &node [[maybe_unused]]) {
    return 0.0f;
  }
};

} // namespace planning