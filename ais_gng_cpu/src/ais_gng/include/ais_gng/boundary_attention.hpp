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

// 境界点だけの平衡kd-tree。入力点の全組合せ走査を回避。
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

// 把持候補の一様枠と境界の距離重み枠の混合。対象なしの配分は通常学習へ返却。
inline mixture mix(const std::vector<uint32_t> &grasp_ids, double grasp_ratio,
        const std::vector<float> &boundary_weights, double boundary_ratio) {
    mixture result;
    double boundary_sum = 0;
    for (float weight : boundary_weights) {boundary_sum += weight;}
    const double grasp_share = grasp_ids.empty() ? 0 : grasp_ratio;
    const double boundary_share = boundary_sum > 0 ? boundary_ratio : 0;
    result.ratio = static_cast<float>(grasp_share + boundary_share);
    if (result.ratio <= 0) {return result;}
    std::vector<float> weights(boundary_weights.size(), 0);
    for (const auto idx : grasp_ids) {
        if (idx < weights.size()) {weights[idx] += static_cast<float>(grasp_share / grasp_ids.size());}
    }
    for (std::size_t idx = 0; idx < weights.size(); ++idx) {
        if (boundary_sum > 0) {weights[idx] += static_cast<float>(boundary_share * boundary_weights[idx] / boundary_sum);}
        if (weights[idx] > 0) {result.ids.push_back(idx); result.weights.push_back(weights[idx]);}
    }
    return result;
}
}  // namespace fuzzrobo::boundary_attention
