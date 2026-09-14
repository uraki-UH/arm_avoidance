#pragma once

#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Transform.h>

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace graph_inspection {
using json = nlohmann::json;

inline std::array<double, 3> point(const json& value) {
    std::array<double, 3> result{value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
    for (const auto coordinate : result) {
        if (!std::isfinite(coordinate)) throw std::invalid_argument("Nonfinite position");
    }
    return result;
}

inline std::array<double, 3> marker_point(const json& marker, const json& value) {
    const auto p = point(value);
    const auto origin = point(marker.value("pos", json::array({0, 0, 0})));
    const auto q = marker.value("quat", json::array({0, 0, 0, 1}));
    tf2::Quaternion rotation(q.at(0).get<double>(), q.at(1).get<double>(),
        q.at(2).get<double>(), q.at(3).get<double>());
    if (!std::isfinite(rotation.length2()) || rotation.length2() < 1e-12) {
        throw std::invalid_argument("Invalid marker orientation");
    }
    const auto transformed = tf2::Transform(rotation.normalized(),
        tf2::Vector3(origin[0], origin[1], origin[2])) * tf2::Vector3(p[0], p[1], p[2]);
    return {transformed.x(), transformed.y(), transformed.z()};
}

// クリック時の受信データからの読取専用切り出し。ROS購読・TF・元データの変更なし。
inline json snapshot(const json& params) {
    const bool enable_bounds_only = params.value("enable_bounds_only", false);
    auto selection = params.at("selection");
    auto kind = selection.at("kind").get<std::string>();
    auto id = selection.at("id").get<std::int64_t>();
    auto graph = params.value("graph", json::object());
    std::string title;
    json node_color = nullptr;
    double node_diameter = 0;
    if (kind == "marker") {
        const auto& markers = params.at("marker_array").at("markers");
        const auto ns = selection.at("ns").get<std::string>();
        const json* selected = nullptr;
        for (const auto& marker : markers) {
            if (marker.at("id") == id && marker.at("ns") == ns && marker.value("action", 0) < 2) {
                selected = &marker;
                break;
            }
        }
        if (!selected || selected->value("type", "") != "sphere_list") {
            throw std::invalid_argument("Selected node marker is unavailable");
        }
        graph = {{"nodes", json::array()}, {"edges", json::array()}, {"clusters", json::array()},
            {"frameId", selected->value("frameId", "")}, {"timestamp", 0}};
        std::map<std::array<double, 3>, std::size_t> indices;
        const bool is_grasp_part = ns == "grasp_plane" || ns == "grasp_nonplane";
        for (const auto& part : markers) {
            const auto part_ns = part.value("ns", "");
            // 既存把持候補の平面・付属非平面は同一候補IDの別部品。一般Markerの同IDとは区別。
            const bool is_same_part = part_ns == ns || (is_grasp_part &&
                (part_ns == "grasp_plane" || part_ns == "grasp_nonplane"));
            if (!is_same_part || part.at("id") != id || part.value("action", 0) >= 2 ||
                part.value("type", "") != "sphere_list") continue;
            if (part.value("frameId", "") != graph["frameId"]) {
                throw std::invalid_argument("Candidate parts use different frames");
            }
            if (graph["nodes"].size() + part.at("points").size() > 200000) {
                throw std::invalid_argument("Oversized candidate");
            }
            for (const auto& value : part.at("points")) {
                const auto p = marker_point(part, value);
                const auto idx = graph["nodes"].size();
                if (!indices.emplace(p, idx).second) continue;
                graph["nodes"].push_back({{"id", idx}, {"x", p[0]}, {"y", p[1]}, {"z", p[2]},
                    {"nx", 0}, {"ny", 0}, {"nz", 0}, {"label", 0}, {"age", 0}});
            }
        }
        // 明示された線分のうち、両端とも選択ノードに属するエッジのみ。
        for (const auto& marker : markers) {
            if (enable_bounds_only) break;
            if (marker.value("type", "") != "line_list" || marker.value("action", 0) >= 2 ||
                marker.value("frameId", "") != graph["frameId"]) continue;
            const auto& points = marker.at("points");
            for (std::size_t idx = 0; idx + 1 < points.size(); idx += 2) {
                const auto a = indices.find(marker_point(marker, points[idx]));
                const auto b = indices.find(marker_point(marker, points[idx + 1]));
                if (a != indices.end() && b != indices.end()) {
                    graph["edges"].push_back(a->second);
                    graph["edges"].push_back(b->second);
                }
            }
        }
        node_color = selected->value("color", json::array({0.3, 0.9, 0.7, 1}));
        for (const auto& value : selected->value("scale", json::array())) {
            const auto size = value.get<double>();
            if (std::isfinite(size) && size > 0) { node_diameter = size; break; }
        }
        if (node_diameter == 0) node_diameter = 0.02;
        title = (is_grasp_part ? "grasp candidate" : ns) + " #" + std::to_string(id);
    }

    const auto& nodes = graph.at("nodes");
    const auto& edges = graph.at("edges");
    if (!nodes.is_array() || nodes.empty() || nodes.size() > 200000 ||
        !edges.is_array() || edges.size() > 2000000 || edges.size() % 2 != 0) {
        throw std::invalid_argument("Invalid or oversized inspection graph");
    }
    const auto clusters = graph.value("clusters", json::array());
    if (kind == "node") {
        const json* node = nullptr;
        for (std::size_t idx = 0; idx < nodes.size(); ++idx) {
            if (nodes[idx].value("id", static_cast<std::int64_t>(idx)) == id) node = &nodes[idx];
        }
        if (!node) throw std::invalid_argument("Selected node disappeared");
        std::vector<std::int64_t> owners;
        for (const auto& cluster : clusters) {
            for (const auto& member : cluster.at("nodeIds")) {
                if (member == id) { owners.push_back(cluster.at("id")); break; }
            }
        }
        if (owners.size() > 1) throw std::invalid_argument("Multiple candidate memberships: select a cluster box");
        if (!owners.empty()) { kind = "cluster"; id = owners.front(); }
        else if (node->contains("nonplaneComponentId") && node->at("nonplaneComponentId").is_number_integer() &&
            node->at("nonplaneComponentId").get<std::int64_t>() >= 0 &&
            node->at("nonplaneComponentId").get<std::int64_t>() < 0xFFFFFFFFLL) {
            kind = "component"; id = node->at("nonplaneComponentId");
        }
        selection = {{"kind", kind}, {"id", id}};
    }
    std::set<std::int64_t> member_ids;
    json selected_clusters = json::array();
    if (kind == "cluster") {
        for (const auto& cluster : clusters) {
            if (cluster.at("id") != id) continue;
            selected_clusters.push_back(cluster);
            for (const auto& member : cluster.at("nodeIds")) member_ids.insert(member.get<std::int64_t>());
        }
    } else if (kind != "node" && kind != "component" && kind != "marker") {
        throw std::invalid_argument("Unsupported inspection selection");
    }

    json selected_nodes = json::array(), selected_edges = json::array();
    std::vector<std::int64_t> remap(nodes.size(), -1);
    std::array<double, 3> min_position, max_position;
    min_position.fill(std::numeric_limits<double>::infinity());
    max_position.fill(-std::numeric_limits<double>::infinity());
    std::size_t num_selected = 0;
    for (std::size_t idx = 0; idx < nodes.size(); ++idx) {
        const auto& node = nodes[idx];
        const auto node_id = node.value("id", static_cast<std::int64_t>(idx));
        const bool is_selected = kind == "marker" || (kind == "cluster" && member_ids.count(node_id)) ||
            (kind == "node" && node_id == id) ||
            (kind == "component" && node.value("nonplaneComponentId", -1LL) == id);
        if (!is_selected) continue;
        const auto p = point(json::array({node.at("x"), node.at("y"), node.at("z")}));
        for (std::size_t axis = 0; axis < 3; ++axis) {
            min_position[axis] = std::min(min_position[axis], p[axis]);
            max_position[axis] = std::max(max_position[axis], p[axis]);
        }
        remap[idx] = static_cast<std::int64_t>(num_selected++);
        if (!enable_bounds_only) selected_nodes.push_back(node);
    }
    if (num_selected == 0) throw std::invalid_argument("Selected candidate disappeared");
    json result = {{"source_id", params.at("source_id")}, {"selection", selection},
        {"min_position", min_position}, {"max_position", max_position},
        {"frame_id", graph.value("frameId", "")}, {"node_diameter", node_diameter}};
    // ホバー用途は境界情報のみ。ノード列・エッジ列の返送とエッジ再対応なし。
    if (enable_bounds_only) return result;
    // 所属はノードID、edgesは元配列添字。切り出し先の添字への再対応。
    for (std::size_t idx = 0; idx < edges.size(); idx += 2) {
        const auto a = edges[idx].get<std::size_t>(), b = edges[idx + 1].get<std::size_t>();
        if (a >= remap.size() || b >= remap.size()) throw std::invalid_argument("Invalid edge index");
        if (remap[a] >= 0 && remap[b] >= 0) {
            selected_edges.push_back(remap[a]); selected_edges.push_back(remap[b]);
        }
    }
    if (title.empty()) title = kind + " #" + std::to_string(id);
    graph["nodes"] = std::move(selected_nodes);
    graph["edges"] = std::move(selected_edges);
    graph["clusters"] = std::move(selected_clusters);
    result["title"] = title;
    result["graph"] = std::move(graph);
    result["node_color"] = node_color;
    return result;
}
}  // graph_inspection名前空間
