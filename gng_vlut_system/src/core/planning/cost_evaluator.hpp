#pragma once

#include "gng/GrowingNeuralGas.hpp"
#include <Eigen/Dense>
#include <optional>

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
   * @return 移動コスト。
   */
  virtual float evaluate(const GNG::NeuronNode<T_angle, T_coord> &u,
                         const GNG::NeuronNode<T_angle, T_coord> &v) = 0;

  // 活性ノード間で安全ラベルに依存しない固定エッジコスト。未対応の評価器は事前計算対象外
  virtual std::optional<float> static_edge_cost(
      const GNG::NeuronNode<T_angle, T_coord> &,
      const GNG::NeuronNode<T_angle, T_coord> &) const { return std::nullopt; }

  /**
   * 特定のノードにいることによるペナルティやリスクを評価する。
   * 'アクティブ' ステータスや障害物への近さをコストとして組み込むのに便利。
   */
  virtual float getNodePenalty(const GNG::NeuronNode<T_angle, T_coord> &node [[maybe_unused]]) {
    return 0.0f;
  }
};

} // namespace planning
