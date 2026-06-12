#ifndef SPATIAL_TREE_HPP
#define SPATIAL_TREE_HPP

#include "Policy.hpp"
#include "Traits.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <queue>
#include <vector>

namespace SpatialTree {
template <typename T, typename Scalar = DefaultScalar, int Dim = 2>
struct SearchResult {
  T *element;
  const void *cell_handle;
  Scalar distance_sq;
};

/**
 * @brief N次元バウンディングボックス。
 */
template <typename Scalar, int Dim> struct BoundingBox {
  Point<Scalar, Dim> center;       // 中心座標
  Point<Scalar, Dim> half_extents; // 各軸方向の長さの半分

  bool contains(const Point<Scalar, Dim> &p) const {
    for (int i = 0; i < Dim; ++i) {
      if (p[i] < center[i] - half_extents[i] ||
          p[i] > center[i] + half_extents[i])
        return false;
    }
    return true;
  }

  bool contains_with_margin(const Point<Scalar, Dim> &p, Scalar margin) const {
    for (int i = 0; i < Dim; ++i) {
      if (p[i] < center[i] - half_extents[i] - margin ||
          p[i] > center[i] + half_extents[i] + margin)
        return false;
    }
    return true;
  }

  Scalar squared_distance_to(const Point<Scalar, Dim> &p) const {
    Scalar dist_sq = 0;
    for (int i = 0; i < Dim; ++i) {
      Scalar d =
          std::max<Scalar>(0, std::abs(p[i] - center[i]) - half_extents[i]);
      dist_sq += d * d;
    }
    return dist_sq;
  }
};

/**
 * @brief 空間計算の動作パラメータ。
 */
template <typename Scalar> struct SpatialTreeParams {
  int max_nodes_per_cell = 20;
  int min_nodes_for_merge = 4;
  Scalar min_cell_size = 0.01;
  int max_depth = 32; // 安全上の上限（通常は min_cell_size が優先される）
};

/**
 * @brief 汎用 N次元アダプティブ空間分割ツリー。
 */
template <typename T, typename Scalar = DefaultScalar, int Dim = 2,
          typename Traits = SpatialTraits<T, Scalar, Dim>,
          typename HysteresisPolicy = NoHysteresis>
class AdaptiveTree {
public:
  static constexpr int ChildCount = (1 << Dim);

  struct Cell {
    BoundingBox<Scalar, Dim> bounds;
    std::vector<T *> elements;
    std::unique_ptr<Cell[]> children_block;
    Cell *parent = nullptr;
    bool is_subdivided = false;
    const SpatialTreeParams<Scalar> *params;
    int depth = 0;

    // セル統合の判定を高速化するために、セル内とその子孫に含まれる要素数を保持
    int subtree_element_count = 0; // 自分とその子孫に含まれる合計要素数 (セル統合時の判定の高速化に使用)
    int subdivided_children_count = 0; // すぐ下の子のうち、分割されているものの数()

    Cell() : params(nullptr) {}

    Cell(const BoundingBox<Scalar, Dim> &b, const SpatialTreeParams<Scalar> *p,
         Cell *prnt = nullptr, int d = 0)
        : bounds(b), parent(prnt), params(p), depth(d) {
      elements.reserve(p->max_nodes_per_cell);
    }

    void reset(const BoundingBox<Scalar, Dim> &b,
               const SpatialTreeParams<Scalar> *p, Cell *prnt = nullptr,
               int d = 0) {
      bounds = b;
      params = p;
      parent = prnt;
      depth = d;
      elements.clear();
      elements.reserve(p->max_nodes_per_cell);
      is_subdivided = false;
      subtree_element_count = 0;
      subdivided_children_count = 0;
      children_block.reset();
    }

    Cell* add(T *element) {
      subtree_element_count++;
      if (is_subdivided) {
        return children_block[getChildIndex(Traits::getPosition(element))].add(element);
      } else {
        Traits::setIndex(element, static_cast<int>(elements.size()));
        elements.push_back(element);
        Traits::setHandle(element, this);
        if (shouldSubdivide()) {
          subdivide();
          // After subdivision, the element is in one of the children.
          return static_cast<Cell *>(const_cast<void *>(Traits::getHandle(element)));
        }
        return this;
      }
    }

    void removeDirect(T *element, bool update_counts_upward = true) {
      int idx = Traits::getIndex(element);
      bool found = false;
      if (idx >= 0 && idx < static_cast<int>(elements.size()) &&
          elements[idx] == element) {
        if (idx != static_cast<int>(elements.size()) - 1) {
          elements[idx] = elements.back();
          Traits::setIndex(elements[idx], idx);
        }
        elements.pop_back();
        found = true;
      } else {
        auto it = std::find(elements.begin(), elements.end(), element);
        if (it != elements.end()) {
          elements.erase(it);
          found = true;
        }
      }

      if (found) {
        Traits::setHandle(element, nullptr);
        Traits::setIndex(element, -1);
        if (update_counts_upward) {
          Cell *curr = this;
          while (curr) {
            curr->subtree_element_count--;
            if (curr->shouldMerge())
              curr->merge();
            curr = curr->parent;
          }
        } else {
          // Only update local count if upward update is deferred
          subtree_element_count--;
        }
      }
    }

    bool removeRecursive(T *element) {
      if (is_subdivided) {
        bool removed =
            children_block[getChildIndex(Traits::getPosition(element))]
                .removeRecursive(element);
        return removed;
      } else {
        auto it = std::find(elements.begin(), elements.end(), element);
        if (it != elements.end()) {
          removeDirect(element);
          return true;
        }
        return false;
      }
    }

    bool shouldSubdivide() const {
      return elements.size() >
                 static_cast<size_t>(params->max_nodes_per_cell) &&
             (bounds.half_extents[0] * 2 > params->min_cell_size);
    }

    bool shouldMerge() const {
      if (!is_subdivided || subdivided_children_count > 0)
        return false;
      return subtree_element_count <= params->min_nodes_for_merge;
    }

    void subdivide() {
      if (depth >= params->max_depth)
        return;
      if (params->min_cell_size > 0 &&
          bounds.half_extents[0] * 2 <= params->min_cell_size)
        return;

      is_subdivided = true;
      if (parent)
        parent->subdivided_children_count++;
      Point<Scalar, Dim> new_half = bounds.half_extents * 0.5;

      children_block = std::make_unique<Cell[]>(ChildCount);
      for (int i = 0; i < ChildCount; ++i) {
        Point<Scalar, Dim> new_center;
        for (int d = 0; d < Dim; ++d) {
          new_center[d] =
              bounds.center[d] + ((i & (1 << d)) ? new_half[d] : -new_half[d]);
        }
        children_block[i].reset(BoundingBox<Scalar, Dim>{new_center, new_half},
                                params, this, depth + 1);
      }

      std::vector<T *> old_elements = std::move(elements);
      elements.clear();
      elements.shrink_to_fit();

      for (auto *el : old_elements) {
        children_block[getChildIndex(Traits::getPosition(el))].add(el);
      }
    }

    void merge() {
      elements.clear();
      elements.reserve(subtree_element_count);

      for (int i = 0; i < ChildCount; ++i) {
        for (auto *el : children_block[i].elements) {
          Traits::setHandle(el, this);
          Traits::setIndex(el, static_cast<int>(elements.size()));
          elements.push_back(el);
        }
      }

      is_subdivided = false;
      if (parent)
        parent->subdivided_children_count--;
      children_block.reset();
    }

    int getChildIndex(const Point<Scalar, Dim> &p) const {
      if constexpr (Dim == 2) {
        return (int(p[0] >= bounds.center[0])) |
               (int(p[1] >= bounds.center[1]) << 1);
      } else if constexpr (Dim == 3) {
        return (int(p[0] >= bounds.center[0])) |
               (int(p[1] >= bounds.center[1]) << 1) |
               (int(p[2] >= bounds.center[2]) << 2);
      } else {
        int idx = 0;
        for (int d = 0; d < Dim; ++d) {
          idx |= (int(p[d] >= bounds.center[d]) << d);
        }
        return idx;
      }
    }

    Cell* findContainingAncestor(const Point<Scalar, Dim>& p) {
      Cell* curr = this;
      while (curr && !curr->bounds.contains(p)) {
        curr = curr->parent;
      }
      return curr;
    }
    struct SearchState {
      const Point<Scalar, Dim> &p;
      int n;
      std::pair<Scalar, std::pair<T *, const void *>> *results;
      int found_count;
      Scalar worst_dist_sq;
      Scalar search_margin;
      Scalar worst_dist_threshold; // (sqrt(worst_dist_sq) + search_margin)^2

      void update_threshold() {
        if (search_margin <= 0) {
          worst_dist_threshold = worst_dist_sq;
        } else {
          Scalar d = std::sqrt(worst_dist_sq) + search_margin;
          worst_dist_threshold = d * d;
        }
      }

      void add_match(Scalar d_sq, T *el, const void *cell) {
        if (found_count < n) {
          results[found_count] = {d_sq, {el, cell}};
          found_count++;
          if (found_count == n) {
            std::make_heap(results, results + n);
            worst_dist_sq = results[0].first;
            update_threshold();
          }
        } else if (d_sq < worst_dist_sq) {
          std::pop_heap(results, results + n);
          results[n - 1] = {d_sq, {el, cell}};
          std::push_heap(results, results + n);
          worst_dist_sq = results[0].first;
          update_threshold();
        }
      }

      void add_match_small(Scalar d_sq, T *el, const void *cell) {
        if (found_count < n) {
          results[found_count] = {d_sq, {el, cell}};
          found_count++;
          if (found_count == 2 && n == 2) {
            if (results[0].first < results[1].first)
              std::swap(results[0], results[1]);
            worst_dist_sq = results[0].first;
            update_threshold();
          }
        } else if (d_sq < worst_dist_sq) {
          if (n == 2) {
            if (d_sq < results[1].first) {
              results[0] = results[1];
              results[1] = {d_sq, {el, cell}};
            } else {
              results[0] = {d_sq, {el, cell}};
            }
            worst_dist_sq = results[0].first;
            update_threshold();
          } else {
            add_match(d_sq, el, cell);
          }
        }
      }
    };

    void findNBest(SearchState &state) const {
      if (is_subdivided) {
        if (state.n == 2 && Dim == 2) {
          int first = getChildIndex(state.p);
          children_block[first].findNBest(state);

          // 2D: 残り3つの兄弟を算術的距離で枝刈り
          Scalar dx = std::abs(state.p[0] - bounds.center[0]);
          Scalar dy = std::abs(state.p[1] - bounds.center[1]);
          int ax = (dx < dy) ? 0 : 1;
          int ay = 1 - ax;
          Scalar d2[2] = {dx * dx, dy * dy};

          const int order[3] = {first ^ (1 << ax), first ^ (1 << ay),
                                first ^ 3};
          const Scalar costs[3] = {d2[ax], d2[ay], d2[0] + d2[1]};

          for (int i = 0; i < 3; ++i) {
            if (state.found_count >= state.n) {
              if (costs[i] > state.worst_dist_threshold) continue;
            }
            children_block[order[i]].findNBest(state);
          }
        } else {
          // 一般パス (n > 2 または Dim >= 3)
          std::array<std::pair<Scalar, int>, ChildCount> dists;
          for (int i = 0; i < ChildCount; ++i) {
            dists[i] = {children_block[i].bounds.squared_distance_to(state.p),
                        i};
          }
          std::sort(dists.begin(), dists.end());
          for (const auto &d_entry : dists) {
            if (state.found_count >= state.n) {
              if (d_entry.first > state.worst_dist_threshold) break;
            }
            children_block[d_entry.second].findNBest(state);
          }
        }
      } else {
        for (auto *el : elements) {
          Scalar d_sq = (Traits::getPosition(el) - state.p).squaredNorm();

          if (state.found_count < state.n || d_sq < state.worst_dist_sq) {
            if (state.n <= 4)
              state.add_match_small(d_sq, el, this);
            else
              state.add_match(d_sq, el, this);
          }
        }
      }
    }

    template <typename Func>
    void visitCells(Func visitor, int depth = 0) const {
      visitor(*this, depth);
      if (is_subdivided) {
        for (int i = 0; i < ChildCount; ++i) {
          children_block[i].visitCells(visitor, depth + 1);
        }
      }
    }
  };

  AdaptiveTree(const BoundingBox<Scalar, Dim> &world_bounds,
               const SpatialTreeParams<Scalar> &params)
      : params_(params) {
    root_ = std::make_unique<Cell>(world_bounds, &params_);
  }

  AdaptiveTree(const Point<Scalar, Dim> &full_extents,
               const SpatialTreeParams<Scalar> &params)
      : params_(params) {
    root_ = std::make_unique<Cell>(
        BoundingBox<Scalar, Dim>{Point<Scalar, Dim>::Zero(),
                                 full_extents * 0.5},
        &params_);
  }

  void* add(T *element) {
    total_elements_++;
    return root_->add(element);
  }

  void remove(T *element) {
    Cell *cell =
        static_cast<Cell *>(const_cast<void *>(Traits::getHandle(element)));
    if (cell) {
      this->removeByHandle(element, cell);
    } else {
      if (root_->removeRecursive(element))
        total_elements_--;
    }
  }

  void removeByHandle(T *element, Cell *cell) {
    if (!cell)
      return;
    // Consolidation: removeDirect handles both the vector remove and the upward count/merge loop.
    cell->removeDirect(element, true);
    total_elements_--;
  }

  void* updatePosition(T *element, const Point<Scalar, Dim> &next_pos, Scalar margin = 0) {
    Cell *old_cell = static_cast<Cell *>(const_cast<void *>(Traits::getHandle(element)));

    if (old_cell) {
      // Fast path: Intra-cell movement
      bool stays_in_cell = false;
      if constexpr (HysteresisPolicy::enabled) {
        stays_in_cell = old_cell->bounds.contains_with_margin(next_pos, margin);
      } else {
        stays_in_cell = old_cell->bounds.contains(next_pos);
      }

      if (stays_in_cell) {
        Traits::setPosition(element, next_pos);
        return old_cell;
      }

      // LCA Path: Climb up until we find an ancestor that contains the new position
      Cell* lca = old_cell->findContainingAncestor(next_pos);
      
      // If no ancestor contains it (out of tree), we must use root
      if (!lca) {
          this->removeByHandle(element, old_cell);
          Traits::setPosition(element, next_pos);
          return this->add(element);
      }

      // We found a Lowest Common Ancestor (LCA).
      // 1. Remove from old leaf but stop count updates at LCA
      old_cell->removeDirect(element, false); // No upward update by default
      Cell* c_up = old_cell->parent;
      while (c_up && c_up != lca) {
          c_up->subtree_element_count--;
          if (c_up->shouldMerge()) c_up->merge();
          c_up = c_up->parent;
      }

      // 2. Insert into new path below LCA
      // Offset the increment that lca->add will immediately do.
      // Net change for LCA and above will be zero.
      lca->subtree_element_count--;
      Traits::setPosition(element, next_pos);
      return lca->add(element);
    }

    // Default: Add as new
    Traits::setPosition(element, next_pos);
    return root_->add(element);
  }

  /**
   * @brief N個の最近傍を検索（動的確保なしバージョン）。
   */
  template <std::size_t MaxN>
  int findNBest(const Point<Scalar, Dim> &p, int n,
                std::array<SearchResult<T, Scalar, Dim>, MaxN> &results,
                Scalar search_margin = 0) const {
    if (n <= 0) return 0;
    int n_to_find = std::min<int>(n, MaxN);

    std::pair<Scalar, std::pair<T *, const void *>> nearest_buffer[MaxN];
    typename Cell::SearchState state{p, n_to_find, nearest_buffer, 0,
                                     std::numeric_limits<Scalar>::max(),
                                     search_margin,
                                     std::numeric_limits<Scalar>::max()};
    state.update_threshold();

    root_->findNBest(state);

    int count = std::min<int>(n, state.found_count);
    if (count == 2) {
      if (nearest_buffer[0].first > nearest_buffer[1].first) {
        std::swap(nearest_buffer[0], nearest_buffer[1]);
      }
    } else if (count > 2) {
      std::sort(nearest_buffer, nearest_buffer + count);
    }

    for (int i = 0; i < count; ++i) {
      results[i] = {nearest_buffer[i].second.first, nearest_buffer[i].second.second, nearest_buffer[i].first};
    }
    return count;
  }

  // 後方互換性のための実装 (std::vector 版も距離を返すようにアップグレード)
  std::vector<SearchResult<T, Scalar, Dim>>
  findNBest(const Point<Scalar, Dim> &p, int n, Scalar search_margin = 0) const {
    if (n <= 0) return {};
    
    // 64個以下の場合は高速なスタックバッファを使用
    if (n <= 64) {
      std::array<SearchResult<T, Scalar, Dim>, 64> buffer;
      int count = findNBest<64>(p, n, buffer, search_margin);
      std::vector<SearchResult<T, Scalar, Dim>> res(count);
      for (int i = 0; i < count; ++i) res[i] = buffer[i];
      return res;
    } else {
      std::vector<std::pair<Scalar, std::pair<T *, const void *>>> heap_buffer(n);
      typename Cell::SearchState state{p, n, heap_buffer.data(), 0,
                                       std::numeric_limits<Scalar>::max(),
                                       search_margin,
                                       std::numeric_limits<Scalar>::max()};
      state.update_threshold();
      root_->findNBest(state);
      
      int count = std::min<int>(n, state.found_count);
      if (count > 2) {
          std::sort(heap_buffer.begin(), heap_buffer.begin() + count);
      } else if (count == 2) {
          if (heap_buffer[0].first > heap_buffer[1].first) std::swap(heap_buffer[0], heap_buffer[1]);
      }
      
      std::vector<SearchResult<T, Scalar, Dim>> res;
      res.reserve(count);
      for (int i = 0; i < count; ++i) {
        res.push_back({heap_buffer[i].second.first, heap_buffer[i].second.second, heap_buffer[i].first});
      }
      return res;
    }
  }

  int getTotalNodes() const { return total_elements_; }

  template <typename Func> void visitCells(Func visitor) const {
    root_->visitCells(visitor, 0);
  }

protected:
private:
  SpatialTreeParams<Scalar> params_;
  std::unique_ptr<Cell> root_;
  int total_elements_ = 0;
};

} // namespace SpatialTree

#endif // SPATIAL_TREE_HPP
