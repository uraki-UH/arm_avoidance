#pragma once

namespace GNG {
namespace Analysis {

/**
 * @brief GNGノード解析・分類のためのインターフェース
 *
 * @tparam GNGType 処理対象のGNGクラス型
 */
template <typename GNGType> class INodeClassifier {
public:
  virtual ~INodeClassifier() = default;

  /**
   * @brief GNGの全ノードを解析し、Statusを更新する
   * @param gng 解析対象のGNGインスタンス（参照渡し）
   */
  virtual void classify(GNGType &gng) = 0;
};

} // namespace Analysis
} // namespace GNG
