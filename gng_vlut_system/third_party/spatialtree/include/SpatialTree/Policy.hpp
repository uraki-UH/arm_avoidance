#ifndef SPATIAL_TREE_POLICY_HPP
#define SPATIAL_TREE_POLICY_HPP

namespace SpatialTree {

/**
 * @brief セルマージンなしポリシー(trueの場合)
 * 静的なデータセットの場合に最適
 */
struct NoHysteresis {
  static constexpr bool enabled = false;
  static constexpr bool use_upward_traversal = false;
};

/**
 * @brief 適応的マージンポリシー（マージンベースの更新抑制）。
 * GNG の学習など、ノードが頻繁に微小移動する場合など、セル境界で振動的な更新が発生しそうな場合に有効
 */
struct AdaptiveHysteresis {
  static constexpr bool enabled = true;
  static constexpr bool use_upward_traversal = true; 
};

} // namespace SpatialTree

#endif // SPATIAL_TREE_POLICY_HPP
