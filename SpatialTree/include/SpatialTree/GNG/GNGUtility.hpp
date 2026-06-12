#ifndef SPATIAL_TREE_GNG_UTILITY_HPP
#define SPATIAL_TREE_GNG_UTILITY_HPP

// 動的変化対応できているかの検証のためで雑な実装なのでミスあるかもなので、参考程度

#include <SpatialTree/SpatialTree.hpp>
#include <SpatialTree/GNG/GNG.hpp>
#include <functional>
#include <random>
#include <type_traits>
#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>

namespace SpatialTree {

/**
 * @brief GNG-U 向けのノード構造体。Utility変数を追加。
 */
template <typename Scalar, int Dim = 2>
struct UtilityNode {
    int id = -1;
    Point<Scalar, Dim> position;
    
    Scalar error = 0;
    Scalar utility = 0; // GNG-U の貢献度
    bool active = false;
    
    std::unordered_map<UtilityNode*, int> neighbors;
    void* spatial_handle = nullptr;
    int index_in_cell = -1;
    int index_in_active_list = -1;
    int error_heap_index = -1;
    int utility_heap_index = -1;
    
    UtilityNode() = default;
    UtilityNode(int id, const Point<Scalar, Dim>& pos) : id(id), position(pos), active(true) {}
};

/**
 * @brief GNG-U 学習パラメータ。
 */
template <typename Scalar>
struct GNGUtilityParams : public GNGParams<Scalar> {
    Scalar utility_k = 10.0; // 削除閾値 (max_error / min_utility > k なら削除)
    Scalar utility_beta = 0.995; // 貢献の衰退率。
    Scalar utility_inheritance = 0.5; // 新規ノードが親から引き継ぐ貢献度の割合。
};

/**
 * @brief Utility-based Growing Neural Gas (GNG-U).
 * 古い・寄与度の低いノードを動的に削除し、変化する環境に高速に適応
 */
template <typename Scalar = DefaultScalar, int Dim = 2,
          typename HysteresisPolicy = AdaptiveHysteresis,
          bool UseMinCellSize = true>
class GrowingNeuralGasUtility {
public:
    using PointT = Point<Scalar, Dim>;
    using NodeT = UtilityNode<Scalar, Dim>;
    using TraitsT = SpatialTraits<NodeT, Scalar, Dim>;
    using TreeT = AdaptiveTree<NodeT, Scalar, Dim, TraitsT, HysteresisPolicy>;


    GrowingNeuralGasUtility(const PointT &full_extents,
                            const GNGUtilityParams<Scalar> &g_params,
                            const SpatialTreeParams<Scalar> &s_params)
        : g_params_(g_params), tree_(full_extents, s_params) {

        node_pool_.reserve(g_params.max_nodes + 1); // +1 incase we need to add then remove
        for (int i = 0; i < g_params.max_nodes + 1; ++i) {
            node_pool_.push_back(std::make_unique<NodeT>());
            node_pool_.back()->id = i;
            free_node_indices_.push_back(i);
        }
    }

    void train_step(const PointT &sample) {
        std::array<SearchResult<NodeT, Scalar, Dim>, 64> nearest_buffer;
        Scalar margin = 0;
        if constexpr (HysteresisPolicy::enabled) {
            margin = g_params_.hysteresis_margin_factor * global_filtered_error_;
        }
        int n_to_find = static_cast<int>(g_params_.n_best_candidates);
        int found_count = tree_.findNBest(sample, n_to_find, nearest_buffer, margin);

        if (found_count < 2) {
            if (active_nodes_ < g_params_.max_nodes) addNode(sample);
            return;
        }

        NodeT *s1 = nearest_buffer[0].element;
        NodeT *s2 = nearest_buffer[1].element;

        Scalar d1_sq = nearest_buffer[0].distance_sq;
        Scalar d1 = std::sqrt(d1_sq);
        Scalar d1_sq_relative = d1_sq / global_error_multiplier_;
        s1->error += d1_sq_relative;
        upHeapError(s1->error_heap_index);

        // Utility: s2が代わりに選ばれていた際の誤差削減量
        if (s2) {
            Scalar d2_sq = nearest_buffer[1].distance_sq;
            s1->utility += (d2_sq - d1_sq) / global_utility_multiplier_;
            upHeapUtility(s1->utility_heap_index);
            downHeapUtility(s1->utility_heap_index);
        }

        if constexpr (HysteresisPolicy::enabled) {
            global_filtered_error_ = (1.0 - g_params_.lpf_alpha) * global_filtered_error_ + g_params_.lpf_alpha * d1;
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
                if (nbr->neighbors.empty()) to_remove.push_back(nbr);
            } else {
                PointT next_nbr = nbr->position + g_params_.eps_n * (sample - nbr->position);
                if constexpr (has_lazy_neighbors<HysteresisPolicy>()) {
                    TraitsT::setPosition(nbr, next_nbr);
                } else {
                    tree_.updatePosition(nbr, next_nbr, margin);
                }
                ++it;
            }
        }

        for (auto *n : to_remove) removeNode(n);
        if (s1->neighbors.empty()) removeNode(s1);

        if (++step_count_ % g_params_.lambda == 0) {
            add_distributed_node();
        }
        decay_errors_and_utility();
    }

    const std::vector<NodeT *> &getActiveNodes() const {
        return active_node_ptrs_;
    }
    int getNodesCount() const { return active_nodes_; }
    const TreeT& getTree() const { return tree_; }

    NodeT *addNode(const PointT &pos) {
        // GNG-U: 定員時は最もUtilityが低いノードを削除して場所を空ける
        if (active_nodes_ >= g_params_.max_nodes) {
            NodeT* worst_u = findLowestUtilityNode();
            if (worst_u) removeNode(worst_u);
        }

        if (!free_node_indices_.empty()) {
            int idx = free_node_indices_.back();
            free_node_indices_.pop_back();
            auto& n = node_pool_[idx];
            n->active = true;
            n->position = pos;
            n->error = 0;
            n->utility = 0;
            n->neighbors.clear();
            tree_.add(n.get());
            n->index_in_active_list = static_cast<int>(active_node_ptrs_.size());
            active_node_ptrs_.push_back(n.get());
            active_nodes_++;

            // Heaps
            n->error_heap_index = static_cast<int>(error_heap_.size());
            error_heap_.push_back(n.get());
            upHeapError(n->error_heap_index);

            n->utility_heap_index = static_cast<int>(utility_heap_.size());
            utility_heap_.push_back(n.get());
            upHeapUtility(n->utility_heap_index);

            return n.get();
        }
        return nullptr;
    }

    void removeNode(NodeT *node) {
        if (!node || !node->active) return;
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

        // Manual error heap removal
        int eh_idx = node->error_heap_index;
        if (eh_idx >= 0 && eh_idx < static_cast<int>(error_heap_.size())) {
            if (eh_idx != static_cast<int>(error_heap_.size()) - 1) {
                error_heap_[eh_idx] = error_heap_.back();
                error_heap_[eh_idx]->error_heap_index = eh_idx;
                upHeapError(eh_idx);
                downHeapError(eh_idx);
            }
            error_heap_.pop_back();
        }
        node->error_heap_index = -1;

        // Manual utility heap removal
        int uh_idx = node->utility_heap_index;
        if (uh_idx >= 0 && uh_idx < static_cast<int>(utility_heap_.size())) {
            if (uh_idx != static_cast<int>(utility_heap_.size()) - 1) {
                utility_heap_[uh_idx] = utility_heap_.back();
                utility_heap_[uh_idx]->utility_heap_index = uh_idx;
                upHeapUtility(uh_idx);
                downHeapUtility(uh_idx);
            }
            utility_heap_.pop_back();
        }
        node->utility_heap_index = -1;

        free_node_indices_.push_back(node->id);
        active_nodes_--;

        // Remove from all neighbors
        for (auto& pair : node->neighbors) {
            pair.first->neighbors.erase(node);
        }
        node->neighbors.clear();
    }

    void add_distributed_node() {
        if (error_heap_.empty()) return;
        NodeT* max_e = error_heap_.front();

        NodeT* min_u = findLowestUtilityNode();

        Scalar max_err_actual = (max_e ? max_e->error * global_error_multiplier_ : 0);
        Scalar min_util_actual = (min_u ? min_u->utility * global_utility_multiplier_ : 0);

        if (active_nodes_ >= g_params_.max_nodes && min_u && max_e && (min_util_actual == 0.0 || max_err_actual / min_util_actual > g_params_.utility_k)) {
            removeNode(min_u);
            if (min_u == max_e) return;
        }

        if (!max_e) return;

        NodeT *f = nullptr;
        Scalar max_f_err = -1;
        for (auto &pair : max_e->neighbors) {
            if (pair.first->error > max_f_err) {
                max_f_err = pair.first->error;
                f = pair.first;
            }
        }
        
        if (!f) return;

        PointT mid = static_cast<Scalar>(0.5) * (max_e->position + f->position);
        NodeT *r = addNode(mid);
        if (r) {
            max_e->neighbors.erase(f);
            f->neighbors.erase(max_e);
            max_e->neighbors[r] = 0;
            r->neighbors[max_e] = 0;
            f->neighbors[r] = 0;
            r->neighbors[f] = 0;

            max_e->error *= g_params_.alpha;
            f->error *= g_params_.alpha;
            r->error = max_e->error;
            downHeapError(max_e->error_heap_index);
            downHeapError(f->error_heap_index);
            upHeapError(r->error_heap_index);

            // GNG-U: 新規ノードの初期貢献度を親から引き継ぐ（比率調整可能）
            r->utility = g_params_.utility_inheritance * (max_e->utility + f->utility) * static_cast<Scalar>(0.5);
            upHeapUtility(r->utility_heap_index);
            downHeapUtility(r->utility_heap_index);
        }
    }

    void decay_errors_and_utility() {
        global_error_multiplier_ *= g_params_.beta;
        global_utility_multiplier_ *= g_params_.utility_beta;

        if (global_error_multiplier_ < 1e-15 || global_utility_multiplier_ < 1e-15) {
            for (auto* n : active_node_ptrs_) {
                n->error *= global_error_multiplier_;
                n->utility *= global_utility_multiplier_;
            }
            global_error_multiplier_ = 1.0;
            global_utility_multiplier_ = 1.0;
        }
    }

    NodeT* findLowestUtilityNode() {
        if (utility_heap_.empty()) return nullptr;
        return utility_heap_.front();
    }

    void upHeapError(int idx) {
        while (idx > 0) {
            int p = (idx - 1) / 2;
            if (error_heap_[idx]->error <= error_heap_[p]->error) break;
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
            if (l < static_cast<int>(error_heap_.size()) && error_heap_[l]->error > error_heap_[largest]->error) largest = l;
            if (r < static_cast<int>(error_heap_.size()) && error_heap_[r]->error > error_heap_[largest]->error) largest = r;
            if (largest == idx) break;
            std::swap(error_heap_[idx], error_heap_[largest]);
            error_heap_[idx]->error_heap_index = idx;
            error_heap_[largest]->error_heap_index = largest;
            idx = largest;
        }
    }

    void upHeapUtility(int idx) {
        while (idx > 0) {
            int p = (idx - 1) / 2;
            if (utility_heap_[idx]->utility >= utility_heap_[p]->utility) break;
            std::swap(utility_heap_[idx], utility_heap_[p]);
            utility_heap_[idx]->utility_heap_index = idx;
            utility_heap_[p]->utility_heap_index = p;
            idx = p;
        }
    }

    void downHeapUtility(int idx) {
        while (true) {
            int l = idx * 2 + 1;
            int r = idx * 2 + 2;
            int smallest = idx;
            if (l < static_cast<int>(utility_heap_.size()) && utility_heap_[l]->utility < utility_heap_[smallest]->utility) smallest = l;
            if (r < static_cast<int>(utility_heap_.size()) && utility_heap_[r]->utility < utility_heap_[smallest]->utility) smallest = r;
            if (smallest == idx) break;
            std::swap(utility_heap_[idx], utility_heap_[smallest]);
            utility_heap_[idx]->utility_heap_index = idx;
            utility_heap_[smallest]->utility_heap_index = smallest;
            idx = smallest;
        }
    }

private:
    GNGUtilityParams<Scalar> g_params_;
    TreeT tree_;
    std::vector<std::unique_ptr<NodeT>> node_pool_;
    std::vector<NodeT *> active_node_ptrs_;
    int active_nodes_ = 0;
    int step_count_ = 0;
    std::vector<int> free_node_indices_;
    Scalar global_filtered_error_ = 0;
    Scalar global_error_multiplier_ = 1.0;
    Scalar global_utility_multiplier_ = 1.0;

    std::vector<NodeT*> error_heap_;
    std::vector<NodeT*> utility_heap_;
};

} // namespace SpatialTree

#endif // SPATIAL_TREE_GNG_UTILITY_HPP
