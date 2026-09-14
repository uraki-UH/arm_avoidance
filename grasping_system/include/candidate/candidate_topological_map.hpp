#pragma once

#include <ais_gng_msgs/msg/topological_map.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace grasping_system::candidate
{

// 元GNGの候補内誘導部分グラフ。平面・付属の重複除外と、添字基準のエッジ抽出
inline ais_gng_msgs::msg::TopologicalMap extract_candidate_graph(
  const ais_gng_msgs::msg::TopologicalMap &source,
  const std::vector<std::uint32_t> &plane_indices,
  const std::vector<std::uint32_t> &attached_indices)
{
  ais_gng_msgs::msg::TopologicalMap output;
  output.header = source.header;
  output.frame_number = source.frame_number;
  std::vector<int> local_indices(source.nodes.size(), -1);
  for (const auto *indices : {&plane_indices, &attached_indices}) {
    for (const auto idx : *indices) {
      if (idx >= source.nodes.size() || local_indices[idx] >= 0) continue;
      const auto &point = source.nodes[idx].pos;
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      if (output.nodes.size() > std::numeric_limits<std::uint16_t>::max()) {
        throw std::overflow_error("candidate graph node id exhausted");
      }
      local_indices[idx] = static_cast<int>(output.nodes.size());
      auto node = source.nodes[idx];
      node.id = static_cast<std::uint16_t>(output.nodes.size());
      output.nodes.push_back(std::move(node));
    }
  }
  for (std::size_t idx = 0; idx + 1 < source.edges.size(); idx += 2) {
    const auto first = source.edges[idx], second = source.edges[idx + 1];
    if (first >= local_indices.size() || second >= local_indices.size() ||
      local_indices[first] < 0 || local_indices[second] < 0) continue;
    output.edges.push_back(static_cast<std::uint16_t>(local_indices[first]));
    output.edges.push_back(static_cast<std::uint16_t>(local_indices[second]));
  }
  return output;
}

// 候補ごとの独立した所属。重複候補の共有ノードも別IDとし、候補間接続を防止
inline void append_candidate_graph(
  ais_gng_msgs::msg::TopologicalMap &output,
  const ais_gng_msgs::msg::TopologicalMap &snapshot, std::uint32_t candidate_id)
{
  if (snapshot.nodes.empty()) return;
  const auto offset = output.nodes.size();
  if (offset + snapshot.nodes.size() >
    static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max()) + 1U) {
    throw std::overflow_error("candidate graph node id exhausted");
  }
  ais_gng_msgs::msg::TopologicalCluster cluster;
  cluster.id = candidate_id;
  cluster.frame = snapshot.frame_number;
  cluster.quat.w = 1.0;
  auto min_pos = snapshot.nodes.front().pos;
  auto max_pos = min_pos;
  for (auto node : snapshot.nodes) {
    min_pos.x = std::min(min_pos.x, node.pos.x); max_pos.x = std::max(max_pos.x, node.pos.x);
    min_pos.y = std::min(min_pos.y, node.pos.y); max_pos.y = std::max(max_pos.y, node.pos.y);
    min_pos.z = std::min(min_pos.z, node.pos.z); max_pos.z = std::max(max_pos.z, node.pos.z);
    node.id = static_cast<std::uint16_t>(output.nodes.size());
    cluster.nodes.push_back(node.id);
    output.nodes.push_back(std::move(node));
  }
  cluster.pos.x = (min_pos.x + max_pos.x) * 0.5F;
  cluster.pos.y = (min_pos.y + max_pos.y) * 0.5F;
  cluster.pos.z = (min_pos.z + max_pos.z) * 0.5F;
  cluster.scale.x = max_pos.x - min_pos.x;
  cluster.scale.y = max_pos.y - min_pos.y;
  cluster.scale.z = max_pos.z - min_pos.z;
  for (const auto idx : snapshot.edges) {
    output.edges.push_back(static_cast<std::uint16_t>(offset + idx));
  }
  output.clusters.push_back(std::move(cluster));
}

}  // 把持候補のグラフ変換
