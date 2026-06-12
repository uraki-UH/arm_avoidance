#ifndef SPATIAL_TREE_GNG_GNG_POLICY_HPP
#define SPATIAL_TREE_GNG_GNG_POLICY_HPP

#include <SpatialTree/Policy.hpp>

namespace SpatialTree {

/**
 * @brief GNGの更新量に応じた適応的マージン +
 * 近傍ノード更新の際のセル更新判定のスキップ (trueの場合)
 */
struct LazyNeighborUpdate : public AdaptiveHysteresis {
  static constexpr bool lazy_neighbors = true;
};

/**
 * @brief マージンなし（第一勝者ノードは常に厳密にセル更新） +
 * 近傍ノード更新の際のセル更新判定のスキップ(trueの場合)
 */
struct NoHysteresisLazyNeighbor : public NoHysteresis {
  static constexpr bool lazy_neighbors = true;
};

} // namespace SpatialTree

#endif // SPATIAL_TREE_GNG_GNG_POLICY_HPP
