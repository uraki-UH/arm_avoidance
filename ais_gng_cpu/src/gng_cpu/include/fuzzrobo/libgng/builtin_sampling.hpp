#pragma once

#include <fuzzrobo/libgng/api.h>
#include <fuzzrobo/libgng/voxel_framework.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <limits>

namespace fuzzrobo::boundary_attention {

using point = std::array<float, 3>;

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
    // セルAABBと境界点の半径近傍の交差。元点走査前の枝刈り。
    bool intersects(const double *min_pos, const double *max_pos, double radius_sq, std::size_t idx = 0) const {
        if (branches.empty()) {return false;}
        const auto &current = branches[idx];
        double box_gap_sq = 0;
        for (uint32_t dim = 0; dim < 3; ++dim) {
            const double gap = std::max({0.0, current.min_pos[dim] - max_pos[dim], min_pos[dim] - current.max_pos[dim]});
            box_gap_sq += gap * gap;
        }
        if (box_gap_sq > radius_sq) {return false;}
        if (current.left != 0) {
            return intersects(min_pos, max_pos, radius_sq, current.left) ||
                intersects(min_pos, max_pos, radius_sq, current.right);
        }
        for (auto pos = current.begin; pos < current.end; ++pos) {
            double dist_sq = 0;
            for (uint32_t dim = 0; dim < 3; ++dim) {
                const double gap = std::max({0.0, nodes[pos][dim] - max_pos[dim], min_pos[dim] - nodes[pos][dim]});
                dist_sq += gap * gap;
            }
            if (dist_sq <= radius_sq) {return true;}
        }
        return false;
    }
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

    nearest_boundary(const Vec3 *anchors, uint32_t num) {
        nodes.reserve(num);
        for (uint32_t idx = 0; idx < num; ++idx) {
            const auto &p = anchors[idx];
            nodes.push_back({p.x, p.y, p.z});
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

struct sampling_data {
    nearest_boundary tree;
    double radius_sq;
    sampling_data(const std::vector<point> &points, double radius)
        : tree(points), radius_sq(radius * radius) {}
    sampling_data(const Vec3 *points, uint32_t num, double radius)
        : tree(points, num), radius_sq(radius * radius) {}
};

struct sampling_policy {
    const sampling_data &boundary;
    gng_sampling_score baseline(const gng_sampling_cell &cell) const {
        return {boundary.tree.intersects(cell.min_pos, cell.max_pos, boundary.radius_sq) ? 1.0 : 0.0, 1};
    }
    const gng_sampling_cell &collect(const gng_sampling_cell &cell) const {return cell;}
    gng_sampling_score evaluate(const gng_sampling_cell &cell) const {return baseline(cell);}
};

inline gng_sampling_rule sampling_rule(uint32_t id, double ratio, const sampling_data &boundary) {
    gng_sampling_rule rule;
    rule.id = id; rule.ratio = ratio; rule.data = &boundary;
    rule.cell_score = [](const gng_sampling_cell &cell, const void *data) {
        sampling_policy policy{*static_cast<const sampling_data *>(data)};
        voxel_framework::pipeline<voxel_framework::configured_features<>, sampling_policy> pipeline;
        return pipeline.evaluate(cell, policy);
    };
    rule.point_score = [](const float *point, const void *data) {
        const auto &value = *static_cast<const sampling_data *>(data);
        return static_cast<double>(value.tree.weight(point, value.radius_sq));
    };
    return rule;
}

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

// 把持・境界の重点枠の混合。対象なしの配分は通常学習へ返却。
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
        if (boundary_sum > 0 && idx < boundary_weights.size()) {
            weights[idx] += static_cast<float>(boundary_share * boundary_weights[idx] / boundary_sum);
        }
        if (weights[idx] > 0) {result.ids.push_back(idx); result.weights.push_back(weights[idx]);}
    }
    return result;
}
}  // 名前空間fuzzrobo::boundary_attention

namespace fuzzrobo::builtin_sampling {

// 粗いセルの件数・座標和だけの保持。XYZ複製、追加ソート、全点近傍探索なし。
class tracking_cells {
public:
    using key = std::array<int32_t, 3>;
    struct key_hash {
        std::size_t operator()(const key &value) const {
            std::size_t result = 0;
            for (auto item : value) {result ^= std::hash<int32_t>{}(item) + 0x9e3779b9U + (result << 6U) + (result >> 2U);}
            return result;
        }
    };
    struct cell {
        key position;
        uint32_t num_points = 0, num_nodes = 0, num_nonplane = 0;
        double point_sum[3]{}, node_sum[3]{};
        double weight = 0;
    };
    static constexpr uint32_t mixed_cell = UINT32_MAX;
    gng_tracking_sampling_input config;
    std::vector<cell> cells;
    std::vector<uint32_t> fine_cells;
    std::vector<uint32_t> nonplane_generations;

    static bool is_valid(const gng_tracking_sampling_input &value) {
        if (value.mode == gng_tracking_sampling_mode::nearest_nonplane) {
            return std::isfinite(value.ratio) && value.ratio >= 0 && value.ratio < 1;
        }
        if (value.mode != gng_tracking_sampling_mode::coarse) {return false;}
        return std::isfinite(value.ratio) && value.ratio >= 0 && value.ratio < 1 &&
            std::isfinite(value.cell_size) && value.cell_size > 0 &&
            value.min_points > 0 && value.min_nonplane_nodes > 0 &&
            std::isfinite(value.max_points_per_node_th) && value.max_points_per_node_th > 0 &&
            std::isfinite(value.min_centroid_dist_ratio_th) && value.min_centroid_dist_ratio_th >= 0 &&
            std::isfinite(value.max_centroid_dist_ratio) &&
            value.max_centroid_dist_ratio > value.min_centroid_dist_ratio_th;
    }
    void reset(const gng_tracking_sampling_input &value) {
        config = value; config.nonplane_nodes = nullptr;
        lookup.clear(); cells.clear(); fine_cells.clear();
    }
    void add_node(const float *point, bool is_nonplane) {
        const auto idx = locate(point);
        if (idx == mixed_cell) {return;}
        auto &value = cells[idx]; ++value.num_nodes;
        value.num_nonplane += is_nonplane;
        for (uint32_t dim = 0; dim < 3; ++dim) {value.node_sum[dim] += point[dim];}
    }
    uint32_t add_point(const float *point) {
        const auto idx = locate(point);
        if (idx == mixed_cell) {return idx;}
        auto &value = cells[idx]; ++value.num_points;
        for (uint32_t dim = 0; dim < 3; ++dim) {value.point_sum[dim] += point[dim];}
        return idx;
    }
    void evaluate() {
        for (auto &value : cells) {
            if (value.num_points < config.min_points) {continue;}
            const double deficit = std::max(0., 1. - value.num_nodes *
                config.max_points_per_node_th / value.num_points);
            double centroid_score = 0;
            if (value.num_nodes) {
                double dist_sq = 0;
                for (uint32_t dim = 0; dim < 3; ++dim) {
                    const double delta = (value.point_sum[dim] / value.num_points -
                        value.node_sum[dim] / value.num_nodes) / config.cell_size;
                    dist_sq += delta * delta;
                }
                centroid_score = std::clamp((std::sqrt(dist_sq) - config.min_centroid_dist_ratio_th) /
                    (config.max_centroid_dist_ratio - config.min_centroid_dist_ratio_th), 0., 1.);
            }
            const double error = std::max(deficit, centroid_score);
            if (error == 0) {continue;}
            // 自セルと26隣接セルの最大支持。散発ノードの寄せ集めによる誤重点化の抑制。
            uint32_t support = 0;
            for (int z = -1; z <= 1; ++z) for (int y = -1; y <= 1; ++y) for (int x = -1; x <= 1; ++x) {
                const key neighbor{value.position[0]+x, value.position[1]+y, value.position[2]+z};
                const auto found = lookup.find(neighbor);
                if (found != lookup.end()) {support = std::max(support, cells[found->second].num_nonplane);}
            }
            if (support < config.min_nonplane_nodes) {continue;}
            // セル質量は有界。共通抽選側の点数乗算を相殺し、点数の二重強調を回避。
            const double evidence = double(value.num_points) / (double(value.num_points) + config.min_points);
            const double nonplane_support = double(support) / (double(support) + config.min_nonplane_nodes);
            value.weight = evidence * nonplane_support * error / value.num_points;
        }
    }
    gng_sampling_rule sampling_rule() const {
        gng_sampling_rule result;
        result.id = 3; result.ratio = config.ratio; result.data = this;
        result.enable_cell_bounds = 0; result.enable_nearest = 0;
        if (config.mode == gng_tracking_sampling_mode::nearest_nonplane) {
            result.enable_nearest = 1;
            // 既存のセル照合結果と世代付き所属表だけの参照。質量はセル内点数に比例。
            result.cell_score = [](const gng_sampling_cell &input, const void *data) {
                const auto &self = *static_cast<const tracking_cells *>(data);
                const bool is_nonplane = input.node_id < self.nonplane_generations.size() &&
                    self.nonplane_generations[input.node_id] != UINT32_MAX &&
                    self.nonplane_generations[input.node_id] == input.node_frame;
                return gng_sampling_score{input.num_points && is_nonplane ? 1. : 0., 0};
            };
            return result;
        }
        result.cell_score = [](const gng_sampling_cell &input, const void *data) {
            const auto &self = *static_cast<const tracking_cells *>(data);
            if (input.idx >= self.fine_cells.size()) {return gng_sampling_score{};}
            const auto idx = self.fine_cells[input.idx];
            // 粗い境界を跨ぐ入力voxelだけ元点評価。それ以外は既存セル区間を直接抽選。
            return idx == mixed_cell ? gng_sampling_score{1, 1} : gng_sampling_score{self.cells[idx].weight, 0};
        };
        result.point_score = [](const float *point, const void *data) {
            const auto &self = *static_cast<const tracking_cells *>(data);
            key position;
            if (!self.to_key(point, position)) {return 0.;}
            const auto found = self.lookup.find(position);
            return found == self.lookup.end() ? 0. : self.cells[found->second].weight;
        };
        return result;
    }
private:
    std::unordered_map<key, uint32_t, key_hash> lookup;
    bool to_key(const float *point, key &position) const {
        for (uint32_t dim = 0; dim < 3; ++dim) {
            const double value = std::floor(double(point[dim]) / config.cell_size);
            // 26隣接の加算余白を含む座標範囲。
            if (!std::isfinite(value) || value <= INT32_MIN || value >= INT32_MAX) {return false;}
            position[dim] = static_cast<int32_t>(value);
        }
        return true;
    }
    uint32_t locate(const float *point) {
        key position;
        if (!to_key(point, position)) {return mixed_cell;}
        const auto found = lookup.try_emplace(position, cells.size());
        if (found.second) {cells.push_back(cell{position});}
        return found.first->second;
    }
};

// 製品ライブラリが所有する、一入力分の組込み評価データ。
struct state {
    tracking_cells tracking;
    std::vector<gng_sampling_box> boxes;
    std::unique_ptr<boundary_attention::sampling_data> boundary;
    void reset() {boxes.clear(); boundary.reset();}
    gng_sampling_score baseline(const gng_sampling_cell &cell) const {
        int relation = 0;
        for (const auto &box : boxes) {
            bool has_intersection = true, is_contained = true;
            for (uint32_t dim = 0; dim < 3; ++dim) {
                has_intersection &= cell.max_pos[dim] >= box.min_pos[dim] && cell.min_pos[dim] <= box.max_pos[dim];
                is_contained &= cell.min_pos[dim] >= box.min_pos[dim] && cell.max_pos[dim] <= box.max_pos[dim];
            }
            if (is_contained) {return {1, 0};}
            if (has_intersection) {relation = 1;}
        }
        return {relation ? 1.0 : 0.0, static_cast<uint8_t>(relation == 1)};
    }
    const gng_sampling_cell &collect(const gng_sampling_cell &cell) const {return cell;}
    gng_sampling_score evaluate(const gng_sampling_cell &cell) const {return baseline(cell);}
    double point_score(const float *p) const {
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {return 0;}
        for (const auto &box : boxes) {
            if (p[0] >= box.min_pos[0] && p[0] <= box.max_pos[0] &&
                p[1] >= box.min_pos[1] && p[1] <= box.max_pos[1] &&
                p[2] >= box.min_pos[2] && p[2] <= box.max_pos[2]) {return 1;}
        }
        return 0;
    }
    gng_sampling_rule grasp_rule(double ratio) const {
        gng_sampling_rule rule;
        rule.id = 1; rule.ratio = ratio; rule.data = this;
        rule.cell_score = [](const gng_sampling_cell &cell, const void *data) {
            const auto &policy = *static_cast<const state *>(data);
            voxel_framework::pipeline<voxel_framework::configured_features<>, const state> pipeline;
            return pipeline.evaluate(cell, policy);
        };
        rule.point_score = [](const float *p, const void *data) {return static_cast<const state *>(data)->point_score(p);};
        return rule;
    }
};
}  // 名前空間fuzzrobo::builtin_sampling
