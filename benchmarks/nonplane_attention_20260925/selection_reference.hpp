#pragma once

#include "ais_gng/boundary_attention.hpp"

namespace selection_reference {
using point = fuzzrobo::boundary_attention::point;

// 2026-09-25最適化前の検索方式。速度・候補重み照合専用、本番利用なし。
class nearest_boundary {
    std::vector<point> nodes;

    void build(std::size_t begin, std::size_t end, std::size_t axis) {
        if (begin >= end) {return;}
        const auto mid = begin + (end - begin) / 2;
        std::nth_element(nodes.begin() + begin, nodes.begin() + mid, nodes.begin() + end,
            [axis](const auto &a, const auto &b) {return a[axis] < b[axis];});
        build(begin, mid, (axis + 1) % 3);
        build(mid + 1, end, (axis + 1) % 3);
    }

    void search(const float *p, std::size_t begin, std::size_t end, std::size_t axis,
            double &min_dist_sq, bool &has_neighbor) const {
        if (begin >= end) {return;}
        const auto mid = begin + (end - begin) / 2;
        double dist_sq = 0;
        for (std::size_t dim = 0; dim < 3; ++dim) {
            const double diff = static_cast<double>(p[dim]) - nodes[mid][dim];
            dist_sq += diff * diff;
        }
        if (dist_sq <= min_dist_sq) {min_dist_sq = dist_sq; has_neighbor = true;}
        const double split = static_cast<double>(p[axis]) - nodes[mid][axis];
        const auto next = (axis + 1) % 3;
        if (split < 0) {
            search(p, begin, mid, next, min_dist_sq, has_neighbor);
            if (split * split <= min_dist_sq) {search(p, mid + 1, end, next, min_dist_sq, has_neighbor);}
        } else {
            search(p, mid + 1, end, next, min_dist_sq, has_neighbor);
            if (split * split <= min_dist_sq) {search(p, begin, mid, next, min_dist_sq, has_neighbor);}
        }
    }

public:
    explicit nearest_boundary(const std::vector<point> &anchors) {
        for (const auto &anchor : anchors) {
            if (std::all_of(anchor.begin(), anchor.end(), [](float value) {return std::isfinite(value);})) {
                nodes.push_back(anchor);
            }
        }
        build(0, nodes.size(), 0);
    }

    float weight(const float *p, double radius_sq) const {
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {return 0;}
        double min_dist_sq = radius_sq;
        bool has_neighbor = false;
        search(p, 0, nodes.size(), 0, min_dist_sq, has_neighbor);
        return has_neighbor ? static_cast<float>(std::exp(-4.5 * min_dist_sq / radius_sq)) : 0;
    }
};

inline std::vector<float> make_weights(const float *points, uint32_t num_points,
        const std::vector<point> &anchors, double radius) {
    std::vector<float> weights(num_points, 0);
    if (!points || anchors.empty() || !std::isfinite(radius) || radius <= 0) {return weights;}
    const double radius_sq = radius * radius;
    if (!std::isfinite(radius_sq) || radius_sq <= 0) {return weights;}
    const nearest_boundary tree(anchors);
    for (uint32_t idx = 0; idx < num_points; ++idx) {
        weights[idx] = tree.weight(points + 3 * static_cast<std::size_t>(idx), radius_sq);
    }
    return weights;
}
}  // 名前空間selection_reference
