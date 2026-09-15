#pragma once

#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_set>
#include <vector>

namespace nonplane_graph {

// 同一フレームの所属添字からの表示用グラフ構築。元ID・属性と実エッジだけの保持
inline std::optional<ais_gng_msgs::msg::TopologicalMap> build(
    const std_msgs::msg::UInt32MultiArray& components,
    const ais_gng_msgs::msg::TopologicalMap& source,
    const ais_gng_msgs::msg::PlaneClusterArray* planes)
{
    const auto& data = components.data;
    if (data.size() < 2 || data[0] != source.frame_number ||
        data[1] > (data.size() - 2) / 2 || source.edges.size() % 2 != 0) return std::nullopt;
    if (data[1] && (!planes || planes->frame_number != source.frame_number ||
        planes->header.frame_id != source.header.frame_id)) return std::nullopt;
    ais_gng_msgs::msg::TopologicalMap result;
    result.header = source.header;
    result.frame_number = source.frame_number;
    std::vector<int> owner(source.nodes.size(), -1), remap(source.nodes.size(), -1);
    std::unordered_set<uint32_t> component_ids;
    std::size_t cursor = 2;
    for (uint32_t component_idx = 0; component_idx < data[1]; ++component_idx) {
        if (data.size() - cursor < 2) return std::nullopt;
        const auto id = data[cursor++], num_nodes = data[cursor++];
        if (id == std::numeric_limits<uint32_t>::max() || !component_ids.insert(id).second ||
            num_nodes > data.size() - cursor) return std::nullopt;
        ais_gng_msgs::msg::TopologicalCluster cluster;
        cluster.id = id;
        cluster.frame = source.frame_number;
        cluster.quat.w = 1;
        float min_x = INFINITY, min_y = INFINITY, min_z = INFINITY;
        float max_x = -INFINITY, max_y = -INFINITY, max_z = -INFINITY;
        for (uint32_t idx = 0; idx < num_nodes; ++idx) {
            const auto node_idx = data[cursor++];
            if (node_idx >= source.nodes.size() || owner[node_idx] != -1) return std::nullopt;
            owner[node_idx] = static_cast<int>(result.clusters.size());
            const auto& node = source.nodes[node_idx];
            if (!std::isfinite(node.pos.x) || !std::isfinite(node.pos.y) || !std::isfinite(node.pos.z)) return std::nullopt;
            cluster.nodes.push_back(node.id);
            min_x = std::min(min_x, node.pos.x); max_x = std::max(max_x, node.pos.x);
            min_y = std::min(min_y, node.pos.y); max_y = std::max(max_y, node.pos.y);
            min_z = std::min(min_z, node.pos.z); max_z = std::max(max_z, node.pos.z);
        }
        if (num_nodes == 0) continue;
        cluster.pos.x = (min_x + max_x) / 2; cluster.scale.x = max_x - min_x;
        cluster.pos.y = (min_y + max_y) / 2; cluster.scale.y = max_y - min_y;
        cluster.pos.z = (min_z + max_z) / 2; cluster.scale.z = max_z - min_z;
        result.clusters.push_back(std::move(cluster));
    }
    if (cursor != data.size()) return std::nullopt;
    std::vector<bool> is_plane(source.nodes.size(), false), is_selected(source.nodes.size(), false);
    if (planes && planes->frame_number == source.frame_number && planes->header.frame_id == source.header.frame_id) {
        for (const auto& plane : planes->clusters) for (auto idx : plane.node_indices) {
            if (idx >= source.nodes.size() || owner[idx] != -1) return std::nullopt;
            is_plane[idx] = true;
        }
    }
    for (std::size_t idx = 0; idx < owner.size(); ++idx) is_selected[idx] = owner[idx] != -1;
    std::vector<uint16_t> kept_edges;
    for (std::size_t idx = 0; idx < source.edges.size(); idx += 2) {
        const auto from = source.edges[idx], to = source.edges[idx + 1];
        if (from >= owner.size() || to >= owner.size()) return std::nullopt;
        const bool is_internal = owner[from] != -1 && owner[from] == owner[to];
        const bool is_attachment = (owner[from] != -1 && is_plane[to]) || (owner[to] != -1 && is_plane[from]);
        if (!is_internal && !is_attachment) continue;
        is_selected[from] = is_selected[to] = true;
        kept_edges.push_back(from); kept_edges.push_back(to);
    }
    std::unordered_set<uint16_t> node_ids;
    for (std::size_t idx = 0; idx < source.nodes.size(); ++idx) {
        if (!is_selected[idx]) continue;
        const auto& node = source.nodes[idx];
        if (result.nodes.size() > std::numeric_limits<uint16_t>::max() || !node_ids.insert(node.id).second ||
            !std::isfinite(node.pos.x) || !std::isfinite(node.pos.y) || !std::isfinite(node.pos.z)) return std::nullopt;
        remap[idx] = static_cast<int>(result.nodes.size());
        result.nodes.push_back(node);
        // 平面側端点は成分の所属・Bbox対象外。ラベル・法線等の元属性は保持
        result.nodes.back().nonplane_component_id = owner[idx] < 0 ? std::numeric_limits<uint32_t>::max() :
            result.clusters[owner[idx]].id;
    }
    result.edges.reserve(kept_edges.size());
    for (auto idx : kept_edges) result.edges.push_back(static_cast<uint16_t>(remap[idx]));
    return result;
}
}  // 名前空間nonplane_graph
