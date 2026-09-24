#pragma once

#include "ais_gng/boundary_attention.hpp"
#include "ais_gng/topological_plane/nonplane_component_extractor.hpp"

namespace fuzzrobo::nonplane_attention {

// 非平面連結成分の点数による重点対象の選別。元グラフ・出力成分の変更なし。
inline std::vector<boundary_attention::point> select_anchors(
        const ais_gng_msgs::msg::TopologicalMap &map,
        const topological_plane::nonplane::extraction_result &components,
        std::size_t min_component_nodes) {
    std::vector<boundary_attention::point> anchors;
    for (const auto &component : components.components) {
        if (component.node_indices.size() < min_component_nodes) {continue;}
        const auto begin = anchors.size();
        for (const auto idx : component.node_indices) {
            if (idx >= map.nodes.size()) {continue;}
            const auto &pos = map.nodes[idx].pos;
            const boundary_attention::point point{pos.x, pos.y, pos.z};
            if (std::all_of(point.begin(), point.end(), [](float value) {return std::isfinite(value);})) {
                anchors.push_back(point);
            }
        }
        if (anchors.size() - begin < min_component_nodes) {anchors.resize(begin);}
    }
    return anchors;
}

}  // 名前空間fuzzrobo::nonplane_attention
