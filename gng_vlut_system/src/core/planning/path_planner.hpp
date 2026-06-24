#pragma once

#include "gng/GrowingNeuralGas.hpp" // For GNG::GrowingNeuralGas and GNG::NeuronNode
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

  //スタートとゴールの姿勢を入力して経路を返す
  virtual std::vector<T_angle> plan(const T_angle &start, const T_angle &goal,
                                    const T_GNG &gng) = 0;

//ノードのスタートとゴールのIDを入力して経路を返す
  virtual std::vector<int> planNodeIndices(int start_id, int goal_id,
                                           const T_GNG &gng) = 0;
};

} // namespace planning
