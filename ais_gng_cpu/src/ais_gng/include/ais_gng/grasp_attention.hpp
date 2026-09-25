#pragma once

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <fuzzrobo/libgng/api.h>
#if allow_external_sampler_build
#include <fuzzrobo/libgng/voxel_framework.hpp>
#endif
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace fuzzrobo::grasp_attention {

// クラスタごとのAABBと余白。別候補の結合・未所属ノードの混入なし。
class regions {
    using point = std::array<float, 3>;
    using bounds = gng_sampling_box;
    std::vector<bounds> boxes_;
public:
    const std::vector<bounds> &boxes() const {return boxes_;}
    bool empty() const {return boxes_.empty();}
    // セルと候補領域の関係。0: 非交差、1: 部分交差、2: 単一領域内への包含。
    int cell_relation(const double *min_pos, const double *max_pos) const {
        int relation = 0;
        for (const auto &box : boxes_) {
            bool has_intersection = true, is_contained = true;
            for (uint32_t dim = 0; dim < 3; ++dim) {
                has_intersection &= max_pos[dim] >= box.min_pos[dim] && min_pos[dim] <= box.max_pos[dim];
                is_contained &= min_pos[dim] >= box.min_pos[dim] && max_pos[dim] <= box.max_pos[dim];
            }
            if (is_contained) {return 2;}
            if (has_intersection) {relation = 1;}
        }
        return relation;
    }
    void assign(const ais_gng_msgs::msg::TopologicalMap &map,
        const std::vector<point> &positions, double margin) {
        boxes_.clear();
        if (!std::isfinite(margin) || margin < 0 || positions.size() != map.nodes.size() ||
            map.nodes.empty() || map.clusters.empty()) {return;}
        uint16_t max_id = 0;
        for (const auto &node : map.nodes) {max_id = std::max(max_id, node.id);}
        std::vector<int> node_by_id(static_cast<std::size_t>(max_id) + 1, -1);
        for (std::size_t idx = 0; idx < map.nodes.size(); ++idx) {
            auto &entry = node_by_id[map.nodes[idx].id];
            if (entry != -1) {return;}
            entry = static_cast<int>(idx);
        }
        for (const auto &cluster : map.clusters) {
            if (cluster.nodes.empty()) {continue;}
            const auto inf = std::numeric_limits<double>::infinity();
            bounds box{{inf, inf, inf}, {-inf, -inf, -inf}};
            bool is_valid = true;
            for (auto id : cluster.nodes) {
                if (id >= node_by_id.size() || node_by_id[id] < 0) {is_valid = false; break;}
                const auto &position = positions[node_by_id[id]];
                for (int axis = 0; axis < 3; ++axis) {
                    if (!std::isfinite(position[axis])) {is_valid = false; break;}
                    box.min_pos[axis] = std::min(box.min_pos[axis], static_cast<double>(position[axis]));
                    box.max_pos[axis] = std::max(box.max_pos[axis], static_cast<double>(position[axis]));
                }
                if (!is_valid) {break;}
            }
            if (!is_valid) {continue;}
            for (int axis = 0; axis < 3; ++axis) {
                box.min_pos[axis] -= margin;
                box.max_pos[axis] += margin;
            }
            boxes_.push_back(box);
        }
    }
    bool is_inside(const float *p) const {
        if (!p || !std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {return false;}
        for (const auto &box : boxes_) {
            if (p[0] >= box.min_pos[0] && p[0] <= box.max_pos[0] &&
                p[1] >= box.min_pos[1] && p[1] <= box.max_pos[1] &&
                p[2] >= box.min_pos[2] && p[2] <= box.max_pos[2]) {return true;}
        }
        return false;
    }
    std::vector<uint32_t> select(const float *points, uint32_t num_points) const {
        std::vector<uint32_t> ids;
        if (!points || boxes_.empty()) {return ids;}
        for (uint32_t idx = 0; idx < num_points; ++idx) {
            if (is_inside(points + 3 * static_cast<std::size_t>(idx))) {ids.push_back(idx);}
        }
        return ids;
    }
};

#if allow_external_sampler_build
// 社内開発用の規則生成。製品版は領域データだけを組込みAPIへ入力。
struct sampling_policy {
    const regions &region_set;
    gng_sampling_score baseline(const gng_sampling_cell &cell) const {
        const auto relation = region_set.cell_relation(cell.min_pos, cell.max_pos);
        return {relation ? 1.0 : 0.0, static_cast<uint8_t>(relation == 1)};
    }
    const gng_sampling_cell &collect(const gng_sampling_cell &cell) const {return cell;}
    gng_sampling_score evaluate(const gng_sampling_cell &cell) const {return baseline(cell);}
};

inline gng_sampling_rule sampling_rule(uint32_t id, double ratio, const regions &region_set) {
    gng_sampling_rule rule;
    rule.id = id; rule.ratio = ratio; rule.data = &region_set;
    rule.cell_score = [](const gng_sampling_cell &cell, const void *data) {
        sampling_policy policy{*static_cast<const regions *>(data)};
        voxel_framework::pipeline<voxel_framework::configured_features<>, sampling_policy> pipeline;
        return pipeline.evaluate(cell, policy);
    };
    rule.point_score = [](const float *point, const void *data) {
        return static_cast<const regions *>(data)->is_inside(point) ? 1.0 : 0.0;
    };
    return rule;
}
#endif
}  // 名前空間fuzzrobo::grasp_attention
