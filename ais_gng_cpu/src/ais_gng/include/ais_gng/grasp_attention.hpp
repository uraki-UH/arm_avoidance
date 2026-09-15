#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace fuzzrobo::grasp_attention {

// 候補中心だけの平衡kd-tree。入力点群のボクセル再登録・点ごとのメモリ確保なし。
class regions {
    using point = std::array<float, 3>;
    std::vector<point> centers_;
    double radius2_ = 0;
    void partition(std::size_t begin, std::size_t end, int axis) {
        if (begin == end) {return;}
        const auto mid = begin + (end - begin) / 2;
        std::nth_element(centers_.begin() + begin, centers_.begin() + mid, centers_.begin() + end,
            [axis](const auto &a, const auto &b) {return a[axis] < b[axis];});
        partition(begin, mid, (axis + 1) % 3);
        partition(mid + 1, end, (axis + 1) % 3);
    }
    bool is_near(const float *p, std::size_t begin, std::size_t end, int axis) const {
        if (begin == end) {return false;}
        const auto mid = begin + (end - begin) / 2;
        const auto &center = centers_[mid];
        double dist2 = 0;
        for (int dim = 0; dim < 3; ++dim) {
            const double delta = static_cast<double>(p[dim]) - center[dim];
            dist2 += delta * delta;
        }
        if (dist2 <= radius2_) {return true;}
        const double delta = static_cast<double>(p[axis]) - center[axis];
        const int next_axis = (axis + 1) % 3;
        if (delta < 0) {
            return is_near(p, begin, mid, next_axis) ||
                (delta * delta <= radius2_ && is_near(p, mid + 1, end, next_axis));
        }
        return is_near(p, mid + 1, end, next_axis) ||
            (delta * delta <= radius2_ && is_near(p, begin, mid, next_axis));
    }
public:
    void assign(std::vector<point> centers, double radius) {
        centers_.clear();
        radius2_ = radius * radius;
        if (!std::isfinite(radius) || radius <= 0) {return;}
        centers.erase(std::remove_if(centers.begin(), centers.end(), [](const auto &p) {
            return !std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]);
        }), centers.end());
        std::sort(centers.begin(), centers.end());
        centers.erase(std::unique(centers.begin(), centers.end()), centers.end());
        centers_ = std::move(centers);
        partition(0, centers_.size(), 0);
    }
    bool contains(const float *p) const {
        return p && std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
            is_near(p, 0, centers_.size(), 0);
    }
    std::vector<uint32_t> select(const float *points, uint32_t num_points) const {
        std::vector<uint32_t> ids;
        if (!points || centers_.empty()) {return ids;}
        for (uint32_t idx = 0; idx < num_points; ++idx) {
            if (contains(points + 3 * static_cast<std::size_t>(idx))) {ids.push_back(idx);}
        }
        return ids;
    }
};
}  // 名前空間fuzzrobo::grasp_attention
