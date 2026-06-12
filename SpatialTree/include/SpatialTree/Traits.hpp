#ifndef SPATIAL_TREE_TRAITS_HPP
#define SPATIAL_TREE_TRAITS_HPP

#include "Config.hpp"

namespace SpatialTree {

template <typename T, typename Scalar, int Dim> struct SpatialTraits {
  static const Point<Scalar, Dim> &getPosition(const T *obj) {
    return obj->position;
  }

  static void setPosition(T *obj, const Point<Scalar, Dim> &pos) {
    obj->position = pos;
  }

 
  //自分自身が所属しているツリー内のセルのアドレスを取得
  // これにより、削除や更新が O(1) 程度で実行
  static const void *getHandle(const T *obj) { return obj->spatial_handle; }

  static void setHandle(T *obj, const void *handle) {
    obj->spatial_handle = const_cast<void *>(handle);
  }

  static int getIndex(const T *obj) { return obj->index_in_cell; }
  static void setIndex(T *obj, int index) { obj->index_in_cell = index; }
};

} // namespace SpatialTree

#endif // SPATIAL_TREE_TRAITS_HPP
