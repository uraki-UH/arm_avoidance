#include "nearest_node_index.hpp"

#include <SpatialTree/MovingBSPTree.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace GNG {

struct nearest_node_index::impl {
  virtual ~impl() = default;
  virtual void clear() = 0;
  virtual void set_point(int node_id, const float *values) = 0;
  virtual void remove_point(int node_id) = 0;
  virtual std::size_t size() const = 0;
  virtual bool query(const float *values, int num_candidates,
                     const std::function<float(int)> &calc_dist,
                     std::vector<std::pair<float, int>> &result) const = 0;
  virtual std::string check_invariants() const = 0;

  template <int num_dims> struct fixed_impl;
};

template <int num_dims>
struct nearest_node_index::impl::fixed_impl final : impl {
  using point_type = SpatialTree::Point<double, num_dims>;

  struct entry {
    point_type position;
    int node_id = -1;
    void *spatial_handle = nullptr;
    int idx_in_cell = -1;
    bool has_finite_values = false;
  };

  // SpatialTree の Traits 契約に対応する外部 API 名。
  struct traits {
    static const point_type &getPosition(const entry *value) {
      return value->position;
    }
    static void setPosition(entry *value, const point_type &point) {
      value->position = point;
    }
    static const void *getHandle(const entry *value) {
      return value->spatial_handle;
    }
    static void setHandle(entry *value, const void *handle) {
      value->spatial_handle = const_cast<void *>(handle);
    }
    static int getIndex(const entry *value) { return value->idx_in_cell; }
    static void setIndex(entry *value, int idx) { value->idx_in_cell = idx; }
  };

  using tree_type = SpatialTree::MovingBSPTree<
      entry, double, num_dims, traits, SpatialTree::NoHysteresis>;

  std::unordered_map<int, std::unique_ptr<entry>> entries;
  std::unique_ptr<tree_type> tree;
  std::size_t num_invalid_points = 0;
  double max_abs_coord = 0.0;

  fixed_impl() { clear(); }

  void clear() override {
    // 木の解放を先行させる、要素ハンドルの寿命順序。
    tree.reset();
    entries.clear();
    num_invalid_points = 0;
    max_abs_coord = 0.0;
    SpatialTree::MovingBSPParams<double> params;
    params.max_depth = 64;
    params.approx_eps = 0.0;
    tree = std::make_unique<tree_type>(params);
  }

  static bool read_point(const float *values, point_type &point) {
    if (!values)
      return false;
    for (int idx = 0; idx < num_dims; ++idx) {
      if (!std::isfinite(values[idx]))
        return false;
      point[idx] = static_cast<double>(values[idx]);
    }
    return true;
  }

  void set_point(int node_id, const float *values) override {
    point_type point;
    const bool has_finite_values = read_point(values, point);
    auto found = entries.find(node_id);
    if (found == entries.end()) {
      auto value = std::make_unique<entry>();
      value->node_id = node_id;
      found = entries.emplace(node_id, std::move(value)).first;
      ++num_invalid_points;
    }
    entry &value = *found->second;
    if (!has_finite_values) {
      if (value.has_finite_values) {
        tree->remove(&value);
        value.has_finite_values = false;
        ++num_invalid_points;
      }
      return;
    }
    for (int idx = 0; idx < num_dims; ++idx)
      max_abs_coord = std::max(max_abs_coord, std::abs(point[idx]));
    if (value.has_finite_values) {
      tree->updatePosition(&value, point);
    } else {
      value.position = point;
      tree->add(&value);
      value.has_finite_values = true;
      --num_invalid_points;
    }
  }

  void remove_point(int node_id) override {
    const auto found = entries.find(node_id);
    if (found == entries.end())
      return;
    if (found->second->has_finite_values)
      tree->remove(found->second.get());
    else
      --num_invalid_points;
    entries.erase(found);
  }

  std::size_t size() const override { return entries.size(); }

  bool query(const float *values, int num_candidates,
             const std::function<float(int)> &calc_dist,
             std::vector<std::pair<float, int>> &result) const override {
    result.clear();
    point_type point;
    if (num_invalid_points != 0 || !read_point(values, point) || !calc_dist)
      return false;
    if (num_candidates <= 0 || entries.empty())
      return true;
    if (entries.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
      return false;
    const std::size_t num_output =
        std::min(entries.size(), static_cast<std::size_t>(num_candidates));
    const auto append_dist = [&](int node_id) {
      const float dist = calc_dist(node_id);
      if (!std::isfinite(dist) || dist < 0.0f)
        return false;
      result.emplace_back(dist, node_id);
      return true;
    };
    const auto select_output = [&]() {
      std::partial_sort(result.begin(), result.begin() + num_output, result.end());
    };

    double max_query_coord = max_abs_coord;
    for (int idx = 0; idx < num_dims; ++idx)
      max_query_coord = std::max(max_query_coord, std::abs(point[idx]));

    // 深さ 64 の木の距離・領域更新の double 丸め誤差に対する絶対幅。
    // 座標履歴の包絡と次元に基づく過大評価。大きな尺度差では全件への切替。
    const double tree_error =
        4096.0 * static_cast<double>(num_dims + 1) *
        std::numeric_limits<double>::epsilon() * max_query_coord * max_query_coord;
    // float 減算・積・任意の二乗和順序の誤差幅と、極小値の丸め・FTZ 対策。
    const double relative_error =
        16.0 * static_cast<double>(num_dims + 1) *
        static_cast<double>(std::numeric_limits<float>::epsilon());
    const double absolute_error =
        16.0 * static_cast<double>(num_dims + 1) *
        static_cast<double>(std::numeric_limits<float>::min());

    std::size_t num_fetch = std::min(
        entries.size(), num_output + std::max<std::size_t>(num_output, 4));
    for (;;) {
      result.clear();
      if (num_fetch == entries.size()) {
        // 同距離群や曖昧な丸め境界の確定。木の全件挿入整列の二次コスト回避。
        result.reserve(entries.size());
        for (const auto &item : entries) {
          if (!append_dist(item.first)) {
            result.clear();
            return false;
          }
        }
        select_output();
        result.resize(num_output);
        return true;
      }

      const auto found = tree->findNBest(point, static_cast<int>(num_fetch));
      if (found.size() != num_fetch)
        return false;
      result.reserve(num_fetch);
      for (const auto &item : found) {
        if (!append_dist(item.element->node_id)) {
          result.clear();
          return false;
        }
      }
      select_output();
      const double min_unseen_dist =
          std::max(0.0, found.back().distance_sq - tree_error) *
              (1.0 - relative_error) -
          absolute_error;
      // 未取得候補と距離同値の場合も、ID 順確定まで探索件数の倍増。
      if (min_unseen_dist > static_cast<double>(result[num_output - 1].first)) {
        result.resize(num_output);
        return true;
      }
      num_fetch = std::min(entries.size(), num_fetch * 2);
    }
  }

  std::string check_invariants() const override {
    const std::string tree_error = tree->checkInvariants();
    if (!tree_error.empty())
      return tree_error;
    std::size_t num_finite = 0;
    for (const auto &item : entries) {
      const entry &value = *item.second;
      if (value.node_id != item.first)
        return "entry ID mismatch";
      if (value.has_finite_values) {
        ++num_finite;
        if (!value.spatial_handle || value.idx_in_cell < 0)
          return "finite entry handle mismatch";
      } else if (value.spatial_handle || value.idx_in_cell != -1) {
        return "invalid entry handle mismatch";
      }
    }
    if (num_finite + num_invalid_points != entries.size() ||
        num_finite != static_cast<std::size_t>(tree->getTotalNodes()))
      return "entry count mismatch";
    return {};
  }
};

nearest_node_index::nearest_node_index(int num_dims) {
  switch (num_dims) {
  case 3:
    impl_ = std::make_unique<impl::fixed_impl<3>>();
    break;
  case 7:
    impl_ = std::make_unique<impl::fixed_impl<7>>();
    break;
  case 14:
    impl_ = std::make_unique<impl::fixed_impl<14>>();
    break;
  default:
    break;
  }
}

nearest_node_index::~nearest_node_index() = default;
bool nearest_node_index::is_supported() const { return static_cast<bool>(impl_); }
void nearest_node_index::clear() {
  if (impl_)
    impl_->clear();
}
void nearest_node_index::set_point(int node_id, const float *values) {
  if (impl_)
    impl_->set_point(node_id, values);
}
void nearest_node_index::remove_point(int node_id) {
  if (impl_)
    impl_->remove_point(node_id);
}
std::size_t nearest_node_index::size() const { return impl_ ? impl_->size() : 0; }
bool nearest_node_index::query(
    const float *values, int num_candidates,
    const std::function<float(int)> &calc_dist,
    std::vector<std::pair<float, int>> &result) const {
  result.clear();
  return impl_ && impl_->query(values, num_candidates, calc_dist, result);
}
std::string nearest_node_index::check_invariants() const {
  return impl_ ? impl_->check_invariants() : std::string{};
}

} // namespace GNG
