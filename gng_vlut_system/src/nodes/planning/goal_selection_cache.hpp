#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_feature_msgs/msg/topological_node_feature_array.hpp>

namespace robot_sim::planning {

// 不変の受信メッセージに対応するID参照表。選択周期内の全体走査削減。
class goal_selection_cache {
  using map_type = ais_gng_msgs::msg::TopologicalMap;
  using features_type = ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray;
  using feature_type = ais_gng_feature_msgs::msg::TopologicalNodeFeature;
  static constexpr std::size_t no_idx = std::numeric_limits<std::size_t>::max();
  static constexpr std::size_t num_ids = std::size_t{1} << 16;

public:
  bool update_map(map_type::ConstSharedPtr map) {
    if (map == map_) return false;
    map_ = std::move(map);
    std::fill(first_by_id_.begin(), first_by_id_.end(), no_idx);
    next_by_idx_.resize(map_ ? map_->nodes.size() : 0);
    // 同一IDの全要素を保持。非有限座標・衝突ラベルも出力段階では従来どおり保持。
    for (std::size_t idx = next_by_idx_.size(); idx > 0;) {
      --idx;
      const auto id = map_->nodes[idx].id;
      next_by_idx_[idx] = first_by_id_[id];
      first_by_id_[id] = idx;
    }
    return true;
  }

  bool update_features(features_type::ConstSharedPtr features) {
    if (features == features_) return false;
    features_ = std::move(features);
    std::fill(features_by_id_.begin(), features_by_id_.end(), nullptr);
    if (features_)
      for (const auto &feature : features_->features)
        features_by_id_[feature.node_id] = &feature;
    return true;
  }

  bool has_map(const map_type *map) const { return map && map == map_.get(); }
  bool has_features(const features_type *features) const { return features == features_.get(); }
  const feature_type *find_feature(uint16_t id) const { return features_by_id_[id]; }

  void append_nodes(const std::vector<int32_t> &ids, map_type &output) const {
    std::vector<std::size_t> indices;
    for (const auto id : ids) {
      if (id < 0 || static_cast<std::size_t>(id) >= num_ids) continue;
      for (auto idx = first_by_id_[id]; idx != no_idx; idx = next_by_idx_[idx])
        indices.push_back(idx);
    }
    // 選択IDの昇順とは独立した、元mapの配列順の維持。
    std::sort(indices.begin(), indices.end());
    output.nodes.reserve(output.nodes.size() + indices.size());
    for (const auto idx : indices) output.nodes.push_back(map_->nodes[idx]);
  }

private:
  map_type::ConstSharedPtr map_;
  features_type::ConstSharedPtr features_;
  std::vector<std::size_t> first_by_id_ = std::vector<std::size_t>(num_ids, no_idx);
  std::vector<std::size_t> next_by_idx_;
  std::vector<const feature_type *> features_by_id_ = std::vector<const feature_type *>(num_ids, nullptr);
};

}  // 目標選択の受信データ参照表
