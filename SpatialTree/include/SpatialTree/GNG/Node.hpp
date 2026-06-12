#ifndef SPATIAL_TREE_NODE_HPP
#define SPATIAL_TREE_NODE_HPP

#include <SpatialTree/Config.hpp>
#include <unordered_map>
#include <vector>

namespace SpatialTree {

/**
 * @brief GNG のノード構造体（N次元対応）。
 */
template <typename Scalar, int Dim = 2> struct Node {
  int id = -1;
  Point<Scalar, Dim> position;

  // GNG 固有のパラメータ
  Scalar error = 0;    // 累積誤差
  bool active = false; // ノードが活性化（使用中）かどうか

  std::unordered_map<Node *, int> neighbors; // 隣接ノード -> エッジの年齢

  // 空間インデックス用ハンドル
  void *spatial_handle = nullptr;
  int index_in_cell = -1;
  int index_in_active_list = -1;
  int error_heap_index = -1;
  int utility_heap_index = -1;

  Node() = default;
  Node(int id, const Point<Scalar, Dim> &pos)
      : id(id), position(pos), active(true) {}
};

} // namespace SpatialTree

#endif // SPATIAL_TREE_NODE_HPP
