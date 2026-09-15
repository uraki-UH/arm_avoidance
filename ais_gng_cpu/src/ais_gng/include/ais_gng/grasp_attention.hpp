#pragma once

#include <ais_gng_msgs/msg/topological_map.hpp>
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
    struct bounds {
        std::array<double, 3> min_position, max_position;
    };
    std::vector<bounds> boxes_;
public:
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
                    box.min_position[axis] = std::min(box.min_position[axis], static_cast<double>(position[axis]));
                    box.max_position[axis] = std::max(box.max_position[axis], static_cast<double>(position[axis]));
                }
                if (!is_valid) {break;}
            }
            if (!is_valid) {continue;}
            for (int axis = 0; axis < 3; ++axis) {
                box.min_position[axis] -= margin;
                box.max_position[axis] += margin;
            }
            boxes_.push_back(box);
        }
    }
    bool is_inside(const float *p) const {
        if (!p || !std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {return false;}
        for (const auto &box : boxes_) {
            if (p[0] >= box.min_position[0] && p[0] <= box.max_position[0] &&
                p[1] >= box.min_position[1] && p[1] <= box.max_position[1] &&
                p[2] >= box.min_position[2] && p[2] <= box.max_position[2]) {return true;}
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
}  // 名前空間fuzzrobo::grasp_attention
