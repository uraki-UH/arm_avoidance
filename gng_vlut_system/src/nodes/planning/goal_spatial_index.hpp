#pragma once

#include <bsp3d/bsp3d.hpp>
#include <deque>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <tf2/LinearMath/Transform.h>

namespace robot_sim::planning {

// map座標系の差分索引。TF・ID・ラベル変更では空間索引の更新なし
class goal_spatial_index {
  using point = bsp3d::point3<double>;
  struct entry {
    point position;
    std::size_t idx;
    void *spatial_handle = nullptr;
    int index_in_cell = -1;
    bool is_indexed = false;
  };
  using tree = bsp3d::Index<entry, double>;
  using map_type = ais_gng_msgs::msg::TopologicalMap;

public:
  bool update(map_type::ConstSharedPtr map) {
    if (map == map_) return false;
    if (!map) {
      tree_.reset();
      points_.clear();
      map_.reset();
      return true;
    }
    const auto old_num = points_.size();
    const auto num = map->nodes.size();
    bool has_changes = !map_ || old_num != num;
    // deque内の要素アドレスを保持。件数変更時も残存要素のハンドルが有効。
    while (points_.size() > num) {
      auto &value = points_.back();
      if (value.is_indexed) tree_->remove(&value);
      points_.pop_back();
    }
    while (points_.size() < num) {
      const auto idx = points_.size();
      points_.push_back({point{}, idx});
    }
    for (std::size_t idx = 0; idx < num; ++idx) {
      const auto &p = map->nodes[idx].pos;
      const point next{p.x, p.y, p.z};
      auto &value = points_[idx];
      bool has_same_position = idx < old_num;
      for (int axis = 0; axis < 3; ++axis)
        has_same_position = has_same_position && (value.position[axis] == next[axis] ||
            (std::isnan(value.position[axis]) && std::isnan(next[axis])));
      if (has_same_position) continue;
      has_changes = true;
      const bool is_finite = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
      if (value.is_indexed && is_finite) {
        tree_->updatePosition(&value, next);
      } else {
        if (value.is_indexed) tree_->remove(&value);
        value.position = next;
        value.is_indexed = is_finite;
        if (is_finite) {
          if (!tree_) tree_ = std::make_unique<tree>();
          tree_->add(&value);
        }
      }
    }
    map_ = std::move(map);
    return has_changes;
  }

  bool has_map(const map_type *map) const { return map && map == map_.get(); }

  std::vector<std::size_t> query_cell(const std::array<double, 3> &key, double size,
      const tf2::Vector3 &origin, const tf2::Transform &map_to_reach) const {
    std::vector<std::size_t> result;
    if (!tree_) return result;
    const auto reach_to_map = map_to_reach.inverse();
    const tf2::Vector3 reach_center(origin.x() + (key[0] + 0.5) * size,
        origin.y() + (key[1] + 0.5) * size, origin.z() + (key[2] + 0.5) * size);
    const auto center = reach_to_map * reach_center;
    point min_point, max_point;
    // 逆変換後の回転セルを包含する箱。浮動小数点境界の取りこぼし防止
    double scale = 1.0;
    for (int axis = 0; axis < 3; ++axis)
      scale = std::max({scale, std::abs(center[axis]), std::abs(reach_center[axis]),
          std::abs(reach_to_map.getOrigin()[axis]), std::abs(origin[axis])});
    const double roundoff = 64 * std::numeric_limits<double>::epsilon() * scale;
    for (int axis = 0; axis < 3; ++axis) {
      const auto &row = reach_to_map.getBasis()[axis];
      const double half = 0.5 * size * (std::abs(row[0]) + std::abs(row[1]) + std::abs(row[2])) + roundoff;
      min_point[axis] = center[axis] - half;
      max_point[axis] = center[axis] + half;
    }
    // 異常スケール時の保守的な全件候補化。最終判定は従来のセル所属式
    bool has_finite_bounds = true;
    for (int axis = 0; axis < 3; ++axis)
      has_finite_bounds = has_finite_bounds && std::isfinite(min_point[axis]) && std::isfinite(max_point[axis]);
    if (!has_finite_bounds) {
      for (const auto &value : points_) if (value.is_indexed) result.push_back(value.idx);
    } else {
      tree_->query_aabb(min_point, max_point, [&](const entry *value) { result.push_back(value->idx); });
    }
    return result;
  }

private:
  map_type::ConstSharedPtr map_;
  std::deque<entry> points_;
  std::unique_ptr<tree> tree_;
};

}  // 目標選択の差分空間索引
