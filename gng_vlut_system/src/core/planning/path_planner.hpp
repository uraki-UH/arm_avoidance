#pragma once

#include "safety_engine/gng/GrowingNeuralGas.hpp" // For GNG::GrowingNeuralGas and GNG::NeuronNode
#include "planning/cost_evaluator.hpp" // For ICostEvaluator
#include <memory>                      // For std::shared_ptr
#include <vector>

namespace planning {

template <typename T_angle, typename T_coord, typename T_GNG>
class IPathPlanner {
public:
  virtual ~IPathPlanner() = default;

  /**
   * コスト評価クラスを設定する。
   * @param evaluator コスト評価に使用するICostEvaluatorの共有ポインタ。
   */
  virtual void setCostEvaluator(
      std::shared_ptr<ICostEvaluator<T_angle, T_coord>> evaluator) = 0;

  /**
   * スタートとゴールのジョイント構成間のパスを計画する。
   * @param start スタートのジョイント値。
   * @param goal ゴールのジョイント値。
   * @param gng パス計画に使用するGNGグラフ。
   * @return ジョイント構成のシーケンス（パス）。パスが見つからない場合は空。
   */
  virtual std::vector<T_angle> plan(const T_angle &start, const T_angle &goal,
                                    const T_GNG &gng) = 0;

  /**
   * 特定のノードID間のパスを計画する。
   * このメソッドは内部的な使用を意図しており、グラフのノードIDに基づいている。
   * @param start_id スタートノードのID。
   * @param goal_id ゴールノードのID。
   * @param gng パス計画に使用するGNGグラフ。
   * @return ノードID의シーケンス（パス）。パスが見つからない場合は空。
   */
  virtual std::vector<int> planNodeIndices(int start_id, int goal_id,
                                           const T_GNG &gng) = 0;
};

} // namespace planning