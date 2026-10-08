#pragma once
#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <optional>
#include <cmath>
#include <unordered_set>
#include <vector>

namespace plane_graph {
// 同一フレームの平面所属と観測済みエッジからの表示グラフ。面の補間・穴埋めなし。
inline std::optional<ais_gng_msgs::msg::TopologicalMap> build(
    const ais_gng_msgs::msg::PlaneClusterArray& planes,
    const ais_gng_msgs::msg::TopologicalMap& source)
{
    if (planes.frame_number != source.frame_number || planes.header.frame_id != source.header.frame_id ||
        source.edges.size() % 2) return std::nullopt;
    ais_gng_msgs::msg::TopologicalMap result;
    result.header = source.header;
    result.frame_number = source.frame_number;
    std::vector<int> owner(source.nodes.size(), -1), remap(source.nodes.size(), -1);
    std::unordered_set<uint32_t> plane_ids;
    for (const auto& plane : planes.clusters) {
        if (!plane_ids.insert(plane.id).second) return std::nullopt;
        ais_gng_msgs::msg::TopologicalCluster cluster;
        cluster.id = plane.id;
        cluster.pos = plane.centroid;
        cluster.quat.w = 1;
        for (auto idx : plane.node_indices) {
            if (idx >= source.nodes.size() || owner[idx] != -1) return std::nullopt;
            owner[idx] = static_cast<int>(result.clusters.size());
            cluster.nodes.push_back(source.nodes[idx].id);
        }
        result.clusters.push_back(std::move(cluster));
    }
    std::unordered_set<uint16_t> node_ids;
    for (std::size_t idx = 0; idx < owner.size(); ++idx) {
        if (owner[idx] == -1) continue;
        const auto& node = source.nodes[idx];
        if (result.nodes.size() >= 65536 || !node_ids.insert(node.id).second ||
            !std::isfinite(node.pos.x) || !std::isfinite(node.pos.y) || !std::isfinite(node.pos.z)) return std::nullopt;
        remap[idx] = static_cast<int>(result.nodes.size());
        result.nodes.push_back(source.nodes[idx]);
    }
    for (std::size_t idx = 0; idx < source.edges.size(); idx += 2) {
        const auto from = source.edges[idx], to = source.edges[idx + 1];
        if (from >= owner.size() || to >= owner.size()) return std::nullopt;
        if (owner[from] == -1 || owner[from] != owner[to]) continue;
        result.edges.push_back(static_cast<uint16_t>(remap[from]));
        result.edges.push_back(static_cast<uint16_t>(remap[to]));
    }
    return result;
}
}  // 名前空間plane_graph
