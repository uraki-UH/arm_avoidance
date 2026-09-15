#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace fuzzrobo::grasp_attention {

// 候補中心だけの空間索引。入力点群のボクセル再登録なし。
class regions {
    using point = std::array<float, 3>;
    using cell = std::array<int64_t, 3>;
    struct cell_hash {
        std::size_t operator()(const cell &value) const {
            std::size_t hash = 0;
            for (auto axis : value) {hash ^= std::hash<int64_t>{}(axis) + 0x9e3779b9 + (hash << 6) + (hash >> 2);}
            return hash;
        }
    };
    std::unordered_map<cell, std::vector<point>, cell_hash> cells_;
    double radius_ = 0;
    bool get_cell(const float *p, cell &key) const {
        for (int axis = 0; axis < 3; ++axis) {
            const double value = std::floor(p[axis] / radius_);
            if (!std::isfinite(value) || std::abs(value) > 1e12) {return false;}
            key[axis] = static_cast<int64_t>(value);
        }
        return true;
    }
public:
    void assign(std::vector<point> centers, double radius) {
        cells_.clear();
        radius_ = radius;
        if (!std::isfinite(radius) || radius <= 0) {return;}
        centers.erase(std::remove_if(centers.begin(), centers.end(), [](const auto &p) {
            return !std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]);
        }), centers.end());
        std::sort(centers.begin(), centers.end());
        centers.erase(std::unique(centers.begin(), centers.end()), centers.end());
        for (const auto &p : centers) {
            cell key;
            if (get_cell(p.data(), key)) {cells_[key].push_back(p);}
        }
    }
    bool contains(const float *p) const {
        if (cells_.empty()) {return false;}
        cell key;
        if (!get_cell(p, key)) {return false;}
        for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z) {
            const auto it = cells_.find({key[0] + x, key[1] + y, key[2] + z});
            if (it == cells_.end()) {continue;}
            for (const auto &center : it->second) {
                double dist2 = 0;
                for (int axis = 0; axis < 3; ++axis) {
                    const double delta = static_cast<double>(p[axis]) - center[axis];
                    dist2 += delta * delta;
                }
                if (dist2 <= radius_ * radius_) {return true;}
            }
        }
        return false;
    }
    std::vector<uint32_t> select(const float *points, uint32_t num_points) const {
        std::vector<uint32_t> ids;
        if (!points || cells_.empty()) {return ids;}
        for (uint32_t idx = 0; idx < num_points; ++idx) {
            if (contains(points + 3 * static_cast<std::size_t>(idx))) {ids.push_back(idx);}
        }
        return ids;
    }
};
}  // 名前空間fuzzrobo::grasp_attention
