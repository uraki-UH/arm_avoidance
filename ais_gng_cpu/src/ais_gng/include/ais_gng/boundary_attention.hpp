#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace fuzzrobo::boundary_attention {

using point = std::array<float, 3>;

// 時刻逆行・同一入力再処理・座標系変更・受信停止時の旧境界の除外。
inline bool can_reuse(const std::string &old_frame, double old_sec,
        const std::string &frame, double sec, double elapsed_sec, double timeout_sec) {
    return !frame.empty() && frame == old_frame && std::isfinite(old_sec) &&
        std::isfinite(sec) && std::isfinite(elapsed_sec) && old_sec > 0 && sec > old_sec &&
        sec - old_sec <= timeout_sec && elapsed_sec >= 0 && elapsed_sec <= timeout_sec;
}

// 有限な対象点の平衡kd-tree。包囲箱による末端の枝刈りと連続走査。
class nearest_boundary {
    struct branch {
        point min_pos;
        point max_pos;
        std::size_t begin, end;
        std::size_t left = 0, right = 0;
        std::size_t axis = 0;
        float split = 0;
    };
    std::vector<point> nodes;
    std::vector<branch> branches;

    std::size_t build(std::size_t begin, std::size_t end) {
        const auto idx = branches.size();
        branch current{nodes[begin], nodes[begin], begin, end};
        for (auto pos = begin + 1; pos < end; ++pos) {
            for (std::size_t dim = 0; dim < 3; ++dim) {
                current.min_pos[dim] = std::min(current.min_pos[dim], nodes[pos][dim]);
                current.max_pos[dim] = std::max(current.max_pos[dim], nodes[pos][dim]);
            }
        }
        branches.push_back(current);
        // 末端サイズは探索結果に影響しない実装上の定数。
        if (end - begin <= 16) {return idx;}
        std::size_t axis = 0;
        for (std::size_t dim = 1; dim < 3; ++dim) {
            if (static_cast<double>(current.max_pos[dim]) - current.min_pos[dim] >
                static_cast<double>(current.max_pos[axis]) - current.min_pos[axis]) {axis = dim;}
        }
        const auto mid = begin + (end - begin) / 2;
        std::nth_element(nodes.begin() + begin, nodes.begin() + mid, nodes.begin() + end,
            [axis](const auto &a, const auto &b) {return a[axis] < b[axis];});
        const auto split = nodes[mid][axis];
        const auto left = build(begin, mid);
        const auto right = build(mid, end);
        branches[idx].left = left;
        branches[idx].right = right;
        branches[idx].axis = axis;
        branches[idx].split = split;
        return idx;
    }

    static double box_dist_sq(const float *p, const branch &current) {
        double result = 0;
        for (std::size_t dim = 0; dim < 3; ++dim) {
            const double diff = std::max({static_cast<double>(current.min_pos[dim]) - p[dim],
                static_cast<double>(p[dim]) - current.max_pos[dim], 0.0});
            result += diff * diff;
        }
        return result;
    }

    void search(const float *p, std::size_t idx,
            double &min_dist_sq, bool &has_neighbor) const {
        const auto &current = branches[idx];
        if (current.left == 0) {
            if (box_dist_sq(p, current) > min_dist_sq) {return;}
            for (auto pos = current.begin; pos < current.end; ++pos) {
                double dist_sq = 0;
                for (std::size_t dim = 0; dim < 3; ++dim) {
                    const double diff = static_cast<double>(p[dim]) - nodes[pos][dim];
                    dist_sq += diff * diff;
                }
                if (dist_sq <= min_dist_sq) {min_dist_sq = dist_sq; has_neighbor = true;}
            }
            return;
        }
        const double split = static_cast<double>(p[current.axis]) - current.split;
        const auto near_idx = split < 0 ? current.left : current.right;
        const auto far_idx = split < 0 ? current.right : current.left;
        search(p, near_idx, min_dist_sq, has_neighbor);
        if (split * split <= min_dist_sq) {search(p, far_idx, min_dist_sq, has_neighbor);}
    }

public:
    explicit nearest_boundary(const std::vector<point> &anchors) {
        nodes.reserve(anchors.size());
        for (const auto &anchor : anchors) {
            if (std::all_of(anchor.begin(), anchor.end(), [](float value) {return std::isfinite(value);})) {
                nodes.push_back(anchor);
            }
        }
        branches.reserve(nodes.size());
        if (!nodes.empty()) {build(0, nodes.size());}
    }

    float weight(const float *p, double radius_sq) const {
        if (branches.empty() || !std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]) ||
            box_dist_sq(p, branches[0]) > radius_sq) {return 0;}
        double min_dist_sq = radius_sq;
        bool has_neighbor = false;
        search(p, 0, min_dist_sq, has_neighbor);
        return has_neighbor ? static_cast<float>(std::exp(-4.5 * min_dist_sq / radius_sq)) : 0;
    }
};

// 最近傍境界から半径内のガウス重み。標準偏差は半径の1/3、境界重複での増幅なし。
inline std::vector<float> make_weights(const float *points, uint32_t num_points,
        const std::vector<point> &anchors, double radius) {
    std::vector<float> weights(num_points, 0);
    if (!points || anchors.empty() || !std::isfinite(radius) || radius <= 0) {return weights;}
    const double radius_sq = radius * radius;
    if (!std::isfinite(radius_sq) || radius_sq <= 0) {return weights;}
    const nearest_boundary tree(anchors);
    for (uint32_t idx = 0; idx < num_points; ++idx) {
        const auto *p = points + 3 * static_cast<std::size_t>(idx);
        weights[idx] = tree.weight(p, radius_sq);
    }
    return weights;
}

struct mixture {
    std::vector<uint32_t> ids;
    std::vector<float> weights;
    float ratio = 0;
};

// 把持・境界・非平面の重点枠の混合。対象なしの配分は通常学習へ返却。
inline mixture mix(const std::vector<uint32_t> &grasp_ids, double grasp_ratio,
        const std::vector<float> &boundary_weights, double boundary_ratio,
        const std::vector<float> &nonplane_weights = {}, double nonplane_ratio = 0) {
    mixture result;
    double boundary_sum = 0;
    for (float weight : boundary_weights) {boundary_sum += weight;}
    double nonplane_sum = 0;
    for (float weight : nonplane_weights) {nonplane_sum += weight;}
    const double grasp_share = grasp_ids.empty() ? 0 : grasp_ratio;
    const double boundary_share = boundary_sum > 0 ? boundary_ratio : 0;
    const double nonplane_share = nonplane_sum > 0 ? nonplane_ratio : 0;
    result.ratio = static_cast<float>(grasp_share + boundary_share + nonplane_share);
    if (result.ratio <= 0) {return result;}
    std::vector<float> weights(std::max(boundary_weights.size(), nonplane_weights.size()), 0);
    for (const auto idx : grasp_ids) {
        if (idx < weights.size()) {weights[idx] += static_cast<float>(grasp_share / grasp_ids.size());}
    }
    for (std::size_t idx = 0; idx < weights.size(); ++idx) {
        if (boundary_sum > 0 && idx < boundary_weights.size()) {
            weights[idx] += static_cast<float>(boundary_share * boundary_weights[idx] / boundary_sum);
        }
        if (nonplane_sum > 0 && idx < nonplane_weights.size()) {
            weights[idx] += static_cast<float>(nonplane_share * nonplane_weights[idx] / nonplane_sum);
        }
        if (weights[idx] > 0) {result.ids.push_back(idx); result.weights.push_back(weights[idx]);}
    }
    return result;
}
}  // namespace fuzzrobo::boundary_attention
