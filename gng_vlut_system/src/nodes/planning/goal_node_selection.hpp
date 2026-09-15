#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <unordered_map>
#include <vector>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_feature_msgs/msg/topological_node_feature_array.hpp>
#include <gng_control_msgs/msg/grasp_candidate_array.hpp>
#include <tf2/LinearMath/Transform.h>
#include "goal_spatial_index.hpp"

namespace robot_sim::planning {

struct goal_selection_options {
  std::size_t num_candidates = 8;
  bool allow_collision = false;
  double orientation_weight = 0.25;
  double manipulability_weight = 0.25;
};

using goal_transform_lookup = std::function<std::optional<tf2::Transform>(
    const std::string &, const std::string &)>;

struct goal_selection_result {
  ais_gng_msgs::msg::TopologicalMap map;
  std::vector<int32_t> ids;
};

// 到達セル内の計画GNGだけを対象とした選択。入力map・候補・評価値の変更なし。
inline goal_selection_result select_goal_nodes(
    const ais_gng_msgs::msg::TopologicalMap *map,
    const gng_control_msgs::msg::GraspCandidateArray &source,
    const ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray *features,
    const goal_selection_options &options, const goal_transform_lookup &lookup,
    const goal_spatial_index *spatial_index = nullptr) {
  goal_selection_result result;
  if (!map) return result;
  result.map.header = map->header;
  result.map.frame_number = map->frame_number;
  const auto &origin = source.voxel_origin;
  const auto size = source.voxel_size;
  if (!std::isfinite(size) || size <= 0.0 || !std::isfinite(origin.x) ||
      !std::isfinite(origin.y) || !std::isfinite(origin.z)) return result;
  if (std::none_of(source.candidates.begin(), source.candidates.end(), [](const auto &candidate) {
        return candidate.state == gng_control_msgs::msg::GraspCandidate::INSIDE;
      })) return result;

  // 同一周期内の座標系ペア単位のTF取得。失敗時は旧目標の失効。
  std::map<std::pair<std::string, std::string>, std::optional<tf2::Transform>> transforms;
  const auto resolve = [&](const std::string &target, const std::string &frame) {
    if (target.empty() || frame.empty()) return std::optional<tf2::Transform>{};
    if (target == frame) return std::optional<tf2::Transform>{tf2::Transform::getIdentity()};
    const auto key = std::make_pair(target, frame);
    auto found = transforms.find(key);
    if (found == transforms.end()) found = transforms.emplace(key, lookup(target, frame)).first;
    return found->second;
  };
  const auto map_to_reach = resolve(source.evaluation_header.frame_id, map->header.frame_id);
  const auto source_to_reach = resolve(source.evaluation_header.frame_id, source.header.frame_id);
  const auto source_to_map = resolve(map->header.frame_id, source.header.frame_id);
  if (!map_to_reach || !source_to_reach || !source_to_map) return result;

  using cell = std::array<double, 3>;
  const auto voxel_cell = [&](const tf2::Vector3 &point) -> std::optional<cell> {
    const cell key{std::floor((point.x() - origin.x) / size),
                   std::floor((point.y() - origin.y) / size),
                   std::floor((point.z() - origin.z) / size)};
    if (!std::all_of(key.begin(), key.end(), [](double value) { return std::isfinite(value); }))
      return std::nullopt;
    return key;
  };
  const auto position = [](const auto &point) { return tf2::Vector3(point.x, point.y, point.z); };
  const auto normalize = [](const tf2::Vector3 &vec) {
    const auto norm = vec.length();
    return norm <= 1e-9 ? tf2::Vector3(0, 0, 0) : vec / norm;
  };
  std::map<cell, std::vector<std::size_t>> goal_cells;
  const bool has_spatial_index = spatial_index && spatial_index->has_map(map);
  if (!has_spatial_index) {
    for (std::size_t idx = 0; idx < map->nodes.size(); ++idx) {
      const auto key = voxel_cell(*map_to_reach * position(map->nodes[idx].pos));
      if (key) goal_cells[*key].push_back(idx);
    }
  }
  std::unordered_map<uint16_t, const ais_gng_feature_msgs::msg::TopologicalNodeFeature *> node_features;
  if (features && options.manipulability_weight > 0.0) {
    for (const auto &feature : features->features) node_features[feature.node_id] = &feature;
  }

  std::set<int32_t> selected_ids;
  for (const auto &candidate : source.candidates) {
    if (candidate.state != gng_control_msgs::msg::GraspCandidate::INSIDE) continue;
    const auto &quat = candidate.pose.orientation;
    const std::array<double, 4> components{quat.x, quat.y, quat.z, quat.w};
    if (!std::all_of(components.begin(), components.end(), [](double value) { return std::isfinite(value); }) ||
        std::hypot(std::hypot(quat.x, quat.y), std::hypot(quat.z, quat.w)) <= 1e-12) continue;
    const auto target_position = position(candidate.pose.position);
    const auto key = voxel_cell(*source_to_reach * target_position);
    if (!key) continue;
    if (has_spatial_index && !goal_cells.count(*key)) {
      auto &indices = goal_cells[*key];
      for (const auto idx : spatial_index->query_cell(*key, size, position(origin), *map_to_reach)) {
        // 回転セルの外接箱だけでは採用せず、従来の半開区間の所属を再確認
        if (voxel_cell(*map_to_reach * position(map->nodes[idx].pos)) == key) indices.push_back(idx);
      }
    }
    const auto found = goal_cells.find(*key);
    if (found == goal_cells.end()) continue;
    const auto target = *source_to_map * target_position;
    const auto rotation = source_to_map->getRotation() * tf2::Quaternion(quat.x, quat.y, quat.z, quat.w);
    const auto target_dir = normalize(tf2::Vector3(
        2.0 * (rotation.x() * rotation.z() + rotation.y() * rotation.w()),
        2.0 * (rotation.y() * rotation.z() - rotation.x() * rotation.w()),
        1.0 - 2.0 * (rotation.x() * rotation.x() + rotation.y() * rotation.y())));
    std::vector<std::pair<double, std::size_t>> scored;
    // 候補ごとの全ノード走査・中間mapコピーなし。同点は元mapの配列順。
    for (const auto idx : found->second) {
      const auto &node = map->nodes[idx];
      if (!options.allow_collision && node.label == ais_gng_msgs::msg::TopologicalMap::WALL) continue;
      auto score = (position(node.pos) - target).length();
      if (options.orientation_weight != 0.0) {
        const auto node_dir = normalize(position(node.normal));
        if (!node_dir.fuzzyZero()) score += options.orientation_weight * (1.0 - std::abs(node_dir.dot(target_dir)));
      }
      const auto feature = node_features.find(node.id);
      if (feature != node_features.end()) {
        const auto &value = *feature->second;
        const auto condition = value.manip_valid && std::isfinite(value.manip_condition_number)
            ? std::max(1.0, static_cast<double>(value.manip_condition_number)) : 100.0;
        score += options.manipulability_weight * std::log(condition);
      }
      scored.emplace_back(score, idx);
    }
    const auto num = std::min(options.num_candidates, scored.size());
    std::partial_sort(scored.begin(), scored.begin() + num, scored.end());
    for (std::size_t idx = 0; idx < num; ++idx) selected_ids.insert(map->nodes[scored[idx].second].id);
  }
  result.ids.assign(selected_ids.begin(), selected_ids.end());
  for (const auto &node : map->nodes) {
    if (selected_ids.count(node.id)) result.map.nodes.push_back(node);
  }
  return result;
}

}  // 計画目標選択の名前空間
