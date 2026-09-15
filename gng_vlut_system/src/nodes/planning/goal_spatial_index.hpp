#pragma once

#include <SpatialTree/SpatialTree.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <tf2/LinearMath/Transform.h>

namespace robot_sim::planning {

// map座標系の静的索引。TF・ID・ラベル変更では位置索引の再構築なし
class goal_spatial_index {
  using point = SpatialTree::Point<double, 3>;
  struct entry {
    point position;
    std::size_t idx;
  };
  // 外部SpatialTreeのTraits APIへの適合。静的用途のセルハンドル保持なし
  struct traits {
    static const point &getPosition(const entry *value) { return value->position; }
    static void setHandle(entry *, const void *) {}
    static void setIndex(entry *, int) {}
    static const void *getHandle(const entry *) { return nullptr; }
  };
  using tree = SpatialTree::AdaptiveTree<entry, double, 3, traits>;
  using map_type = ais_gng_msgs::msg::TopologicalMap;

public:
  bool update(map_type::ConstSharedPtr map) {
    if (map == map_) return false;
    const bool has_same_positions = map && map_ && map->nodes.size() == map_->nodes.size() &&
        std::equal(map->nodes.begin(), map->nodes.end(), map_->nodes.begin(), [](const auto &a, const auto &b) {
          const auto equal = [](float x, float y) { return x == y || (std::isnan(x) && std::isnan(y)); };
          return equal(a.pos.x, b.pos.x) && equal(a.pos.y, b.pos.y) && equal(a.pos.z, b.pos.z);
        });
    map_ = std::move(map);
    if (has_same_positions) return false;
    tree_.reset();
    points_.clear();
    if (!map_) return true;
    points_.reserve(map_->nodes.size());
    for (std::size_t idx = 0; idx < map_->nodes.size(); ++idx) {
      const auto &p = map_->nodes[idx].pos;
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
        points_.push_back({point{p.x, p.y, p.z}, idx});
    }
    if (points_.empty()) return true;
    point min_point = points_.front().position, max_point = min_point;
    for (const auto &value : points_) for (int axis = 0; axis < 3; ++axis) {
      min_point[axis] = std::min(min_point[axis], value.position[axis]);
      max_point[axis] = std::max(max_point[axis], value.position[axis]);
    }
    const point span = max_point - min_point;
    const double half = std::max({span[0], span[1], span[2], 0.01}) * 0.5 + 0.01;
    tree_ = std::make_unique<tree>(SpatialTree::BoundingBox<double, 3>{
        (min_point + max_point) * 0.5, point{half, half, half}}, SpatialTree::SpatialTreeParams<double>{});
    for (auto &value : points_) tree_->add(&value);
    return true;
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
      for (const auto &value : points_) result.push_back(value.idx);
    } else {
      tree_->query_aabb(min_point, max_point, [&](const entry *value) { result.push_back(value->idx); });
    }
    return result;
  }

private:
  map_type::ConstSharedPtr map_;
  std::vector<entry> points_;
  std::unique_ptr<tree> tree_;
};

}  // 目標選択の静的空間索引
