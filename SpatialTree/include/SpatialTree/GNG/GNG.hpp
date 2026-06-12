#ifndef SPATIAL_TREE_GNG_HPP
#define SPATIAL_TREE_GNG_HPP

#include <SpatialTree/GNG/GNGPolicy.hpp>
#include <SpatialTree/GNG/Node.hpp>
#include <SpatialTree/SpatialTree.hpp>
#include <functional>
#include <random>

#include <limits>
#include <queue>
#include <type_traits>

namespace SpatialTree {

// Treeの適用方法のポリシーを定義
template <typename T> struct has_lazy_neighbors_trait {
private:
  template <typename U>
  static auto test(int) -> std::integral_constant<bool, U::lazy_neighbors>;
  template <typename U> static auto test(...) -> std::false_type;

public:
  static constexpr bool value = decltype(test<T>(0))::value;
};

template <typename T> constexpr bool has_lazy_neighbors() {
  return has_lazy_neighbors_trait<T>::value;
}

/**
 * @brief GNG の学習パラメータ。
 */
template <typename Scalar> struct GNGParams {
  int lambda = 100;
  Scalar alpha = 0.5;
  Scalar beta = 0.995;
  Scalar eps_w = 0.05;
  Scalar eps_n = 0.0006;
  int max_age = 50;
  int max_nodes = 1000;
  Scalar n_best_candidates = 2; // 実装の都合上64以下にすること
  // いまのところ4が早そう

  // マージン関連
  Scalar lpf_alpha = 0.1;
  Scalar hysteresis_margin_factor = 0.15;
};

/**
 * @brief N次元対応 Growing Neural Gas クラス。
 */
template <typename Scalar = DefaultScalar, int Dim = 2,
          typename HysteresisPolicy = AdaptiveHysteresis,
          bool UseMinCellSize = true>
class GrowingNeuralGas {
public:
  using PointT = Point<Scalar, Dim>;
  using NodeT = Node<Scalar, Dim>;
  using TraitsT = SpatialTraits<NodeT, Scalar, Dim>;
  using TreeT = AdaptiveTree<NodeT, Scalar, Dim, TraitsT, HysteresisPolicy>;

  GrowingNeuralGas(const PointT &full_extents,
                   const GNGParams<Scalar> &g_params,
                   const SpatialTreeParams<Scalar> &s_params)
      : g_params_(g_params), tree_(full_extents, s_params) {

    node_pool_.reserve(g_params.max_nodes);
    for (int i = 0; i < g_params.max_nodes; ++i) {
      node_pool_.push_back(std::make_unique<NodeT>());
      node_pool_.back()->id = i;
      free_node_indices_.push_back(i);
    }
  }

  void train_step(const PointT &sample) {
    std::array<SearchResult<NodeT, Scalar, Dim>, 64>
        nearest_buffer; // Stack buffer for N-best search
    Scalar margin = 0;
    if constexpr (HysteresisPolicy::enabled) {
      margin = g_params_.hysteresis_margin_factor * global_filtered_error_;
    }

    int n_to_find = static_cast<int>(g_params_.n_best_candidates);
    int found_count = tree_.findNBest(sample, n_to_find, nearest_buffer, margin);

    if (found_count < 2) {
      if (active_nodes_ < g_params_.max_nodes)
        addNode(sample);
      return;
    }

    NodeT *s1 = nearest_buffer[0].element;
    NodeT *s2 = nearest_buffer[1].element;

    Scalar d1_sq = nearest_buffer[0].distance_sq;
    Scalar d1 = std::sqrt(d1_sq);
    s1->error += d1 / global_error_multiplier_;
    upHeapError(s1->error_heap_index);

    // セルマージン有効時のみグローバル累積誤差を更新
    if constexpr (HysteresisPolicy::enabled) {
      global_filtered_error_ =
          (1.0 - g_params_.lpf_alpha) * global_filtered_error_ +
          g_params_.lpf_alpha * d1;
    }

    PointT next_s1 = s1->position + g_params_.eps_w * (sample - s1->position);
    margin = g_params_.hysteresis_margin_factor * global_filtered_error_;
    tree_.updatePosition(s1, next_s1, margin);

    s1->neighbors[s2] = 0;
    s2->neighbors[s1] = 0;

    std::vector<NodeT *> to_remove;
    for (auto it = s1->neighbors.begin(); it != s1->neighbors.end();) {
      NodeT *nbr = it->first;
      it->second++;

      if (it->second > g_params_.max_age) {
        nbr->neighbors.erase(s1);
        it = s1->neighbors.erase(it);
        if (nbr->neighbors.empty())
          to_remove.push_back(nbr);
      } else {
        PointT next_nbr =
            nbr->position + g_params_.eps_n * (sample - nbr->position);

        if constexpr (has_lazy_neighbors<HysteresisPolicy>()) {
          TraitsT::setPosition(nbr, next_nbr);
        } else {
          tree_.updatePosition(nbr, next_nbr, margin);
        }
        ++it;
      }
    }

    for (auto *n : to_remove)
      removeNode(n);
    if (s1->neighbors.empty())
      removeNode(s1);

    if (++step_count_ % g_params_.lambda == 0) {
      add_distributed_node();
    }
    decay_errors();
  }

  const std::vector<NodeT *> &getActiveNodes() const {
    return active_node_ptrs_;
  }
  int getNodesCount() const { return active_nodes_; }
  const TreeT &getTree() const { return tree_; }

  NodeT *addNode(const PointT &pos) {
    if (!free_node_indices_.empty()) {
      int idx = free_node_indices_.back();
      free_node_indices_.pop_back();
      auto &n = node_pool_[idx];
      n->active = true;
      n->position = pos;
      n->error = 0;
      n->neighbors.clear();
      tree_.add(n.get());
      n->index_in_active_list = static_cast<int>(active_node_ptrs_.size());
      active_node_ptrs_.push_back(n.get());
      active_nodes_++;

      n->error_heap_index = static_cast<int>(error_heap_.size());
      error_heap_.push_back(n.get());
      upHeapError(n->error_heap_index);
      return n.get();
    }
    return nullptr;
  }

  void removeNode(NodeT *node) {
    if (!node || !node->active)
      return;
    tree_.remove(node);
    node->active = false;

    int idx = node->index_in_active_list;
    if (idx >= 0 && idx < static_cast<int>(active_node_ptrs_.size())) {
      if (idx != static_cast<int>(active_node_ptrs_.size()) - 1) {
        active_node_ptrs_[idx] = active_node_ptrs_.back();
        active_node_ptrs_[idx]->index_in_active_list = idx;
      }
      active_node_ptrs_.pop_back();
    }
    node->index_in_active_list = -1;

    int h_idx = node->error_heap_index;
    if (h_idx >= 0 && h_idx < static_cast<int>(error_heap_.size())) {
      if (h_idx != static_cast<int>(error_heap_.size()) - 1) {
        error_heap_[h_idx] = error_heap_.back();
        error_heap_[h_idx]->error_heap_index = h_idx;
        upHeapError(h_idx);
        downHeapError(h_idx);
      }
      error_heap_.pop_back();
    }
    node->error_heap_index = -1;

    free_node_indices_.push_back(node->id);
    active_nodes_--;
  }

  void add_distributed_node() {
    if (error_heap_.empty())
      return;
    NodeT *q = error_heap_.front();
    if (!q)
      return;

    NodeT *f = nullptr;
    Scalar max_f_err = -1;
    for (auto &pair : q->neighbors) {
      if (pair.first->error > max_f_err) {
        max_f_err = pair.first->error;
        f = pair.first;
      }
    }
    if (!f)
      return;

    PointT mid = static_cast<Scalar>(0.5) * (q->position + f->position);
    NodeT *r = addNode(mid);
    if (r) {
      q->neighbors.erase(f);
      f->neighbors.erase(q);
      q->neighbors[r] = 0;
      r->neighbors[q] = 0;
      f->neighbors[r] = 0;
      r->neighbors[f] = 0;

      q->error *= g_params_.alpha;
      f->error *= g_params_.alpha;
      r->error = q->error;

      downHeapError(q->error_heap_index);
      downHeapError(f->error_heap_index);
      upHeapError(r->error_heap_index);
    }
  }

  void decay_errors() {
    global_error_multiplier_ *= g_params_.beta;

    if (global_error_multiplier_ < 1e-15) {
      for (auto *n : active_node_ptrs_) {
        n->error *= global_error_multiplier_;
      }
      global_error_multiplier_ = 1.0;
    }
  }

  void upHeapError(int idx) {
    while (idx > 0) {
      int p = (idx - 1) / 2;
      if (error_heap_[idx]->error <= error_heap_[p]->error)
        break;
      std::swap(error_heap_[idx], error_heap_[p]);
      error_heap_[idx]->error_heap_index = idx;
      error_heap_[p]->error_heap_index = p;
      idx = p;
    }
  }

  void downHeapError(int idx) {
    while (true) {
      int l = idx * 2 + 1;
      int r = idx * 2 + 2;
      int largest = idx;
      if (l < static_cast<int>(error_heap_.size()) &&
          error_heap_[l]->error > error_heap_[largest]->error)
        largest = l;
      if (r < static_cast<int>(error_heap_.size()) &&
          error_heap_[r]->error > error_heap_[largest]->error)
        largest = r;
      if (largest == idx)
        break;
      std::swap(error_heap_[idx], error_heap_[largest]);
      error_heap_[idx]->error_heap_index = idx;
      error_heap_[largest]->error_heap_index = largest;
      idx = largest;
    }
  }

private:
  GNGParams<Scalar> g_params_;
  TreeT tree_;
  std::vector<std::unique_ptr<NodeT>> node_pool_;
  std::vector<NodeT *> active_node_ptrs_;
  int active_nodes_ = 0;
  int step_count_ = 0;
  std::vector<int> free_node_indices_;
  Scalar global_error_multiplier_ = 1.0; // 誤差の減衰を効率的に行うための乗数。
  Scalar global_filtered_error_ =
      0; // セルマージン用の第一勝者に対するグローバル累積誤差(使わないなら処理を無効化した方がよい)
  std::vector<NodeT *> error_heap_;
};

} // namespace SpatialTree

#endif // SPATIAL_TREE_GNG_HPP
