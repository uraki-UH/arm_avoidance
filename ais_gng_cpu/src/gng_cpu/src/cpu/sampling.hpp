#pragma once
#include <fuzzrobo/libgng/api.h>
#include "voxel_grid.hpp"
#include "../utils/node.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace gng_sampling {
enum class source { regular, unknown, priority };

// 通常・unknown・追加条件で共用する固定学習回数の配分。
inline source select_source(int iter, int &regular_iter, double priority_ratio,
        int num_unknown, int unknown_rate) {
    if (priority_ratio > 0 && static_cast<int>((iter + 1) * priority_ratio) > static_cast<int>(iter * priority_ratio)) {
        return source::priority;
    }
    if (num_unknown == 0 || ++regular_iter == unknown_rate) {
        regular_iter = 0;
        return source::regular;
    }
    return source::unknown;
}

struct match {uint32_t id = UINT32_MAX; float dist_sq = 0;};
struct entry {uint32_t rule_idx, cell_idx, raw_idx; double mass;};

// 共有セル走査・疎な候補・一つの累積分布。無指定時はセル走査なし。
class frame_sampler {
public:
    std::vector<gng_sampling_rule> rules;
    std::vector<match> matches;
    std::vector<entry> entries;
    gng_sampling_stats stats;
    double ratio = 0;

    void reset_input() {
        rules.clear(); matches.clear(); entries.clear(); cumulative.clear(); debug_ids.clear(); rule_ids.clear();
        stats = {}; ratio = 0;
    }

    void build(const VoxelGrid *grid, const std::vector<Vec3f> *points,
            std::vector<Node> &nodes, const std::vector<uint32_t> &legacy_ids,
            const std::vector<float> &legacy_weights, double legacy_ratio) {
        entries.clear(); cumulative.clear(); debug_ids.clear(); stats = {}; ratio = 0;
        sums.assign(rules.size() + 1, 0);
        scores.resize(rules.size());
        if (!rules.empty() && grid && points) {
            const bool enable_counts = grid->enable_voxel_downsampling &&
                std::any_of(rules.begin(), rules.end(), [](const auto &rule) {return rule.enable_node_counts;});
            if (enable_counts) {
                node_counts.assign(grid->filtered_pcl_num, 0);
                const auto begin = grid->voxel_range.begin(), end = begin + grid->filtered_pcl_num;
                for (auto &node : nodes) {
                    if (node.id == NODE_NOID) {continue;}
                    const auto key = grid->voxel_config->getIndex(node.pos);
                    const auto found = std::lower_bound(begin, end, key, [&](const auto &range, uint32_t value) {
                        return grid->voxel_index[range.start].voxel_index < value;
                    });
                    if (found != end && grid->voxel_index[found->start].voxel_index == key) {++node_counts[found - begin];}
                }
            }
            try {
                for (uint32_t idx = 0; idx < grid->filtered_pcl_num; ++idx) {
                    const auto &range = grid->voxel_range[idx];
                    gng_sampling_cell cell;
                    cell.idx = idx; cell.num_points = range.end - range.start;
                    cell.has_volume = grid->enable_voxel_downsampling;
                    cell.has_node_count = enable_counts;
                    if (enable_counts) {cell.num_nodes = node_counts[idx];}
                    if (idx < matches.size()) {
                        const auto &nearest = matches[idx];
                        cell.nearest_dist_sq = nearest.dist_sq;
                        if (nearest.id < nodes.size() && nodes[nearest.id].id != NODE_NOID) {
                            cell.node_id = nearest.id;
                            cell.node_frame = nodes[nearest.id].frame;
                            cell.node_label = nodes[nearest.id].label;
                        }
                    }
                    if (grid->enable_voxel_downsampling) {
                        const auto &config = *grid->voxel_config;
                        const auto key = grid->voxel_index[range.start].voxel_index;
                        const uint32_t coords[] = {key % config.max[0], key / config.max[0] % config.max[1], key / config.maxXY};
                        const double origin[] = {config.x_min, config.y_min, config.z_min};
                        for (uint32_t dim = 0; dim < 3; ++dim) {
                            const double min_pos = origin[dim] + coords[dim] * static_cast<double>(config.unit);
                            const double max_pos = min_pos + config.unit;
                            // floatのセル量子化に対する保守的な丸め余白。
                            const double slack = 8 * std::numeric_limits<float>::epsilon() *
                                std::max({1.0, std::abs(origin[dim]), std::abs(min_pos), std::abs(max_pos)});
                            cell.min_pos[dim] = min_pos - slack; cell.max_pos[dim] = max_pos + slack;
                        }
                    } else {
                        const auto &point = (*points)[grid->voxel_index[range.start].raw_index];
                        for (uint32_t dim = 0; dim < 3; ++dim) {cell.min_pos[dim] = cell.max_pos[dim] = point.p[dim];}
                    }
                    ++stats.num_cells;
                    point_rules.clear();
                    for (uint32_t rule_idx = 0; rule_idx < rules.size(); ++rule_idx) {
                        const auto &rule = rules[rule_idx];
                        const auto score = rule.cell_score(cell, rule.data);
                        ++stats.num_cell_evaluations;
                        if (!is_valid_weight(score.weight) || (score.enable_point_weights && !rule.point_score)) {
                            stats.has_invalid_score = 1; break;
                        }
                        scores[rule_idx] = score;
                        if (score.weight == 0) {continue;}
                        if (score.enable_point_weights) {point_rules.push_back(rule_idx);}
                        else {append(rule_idx, idx, UINT32_MAX, score.weight * cell.num_points);}
                    }
                    if (stats.has_invalid_score) {break;}
                    if (point_rules.empty()) {continue;}
                    // 関係するセル内だけの元点参照。複数条件でもXYZの取得は共用。
                    for (uint32_t pos = range.start; pos < range.end; ++pos) {
                        const auto raw_idx = grid->voxel_index[pos].raw_index;
                        const auto *point = (*points)[raw_idx].p;
                        for (const auto rule_idx : point_rules) {
                            const auto &score = scores[rule_idx];
                            const auto &rule = rules[rule_idx];
                            ++stats.num_point_evaluations;
                            const double weight = rule.point_score(point, rule.data);
                            if (!is_valid_weight(weight)) {stats.has_invalid_score = 1; break;}
                            append(rule_idx, idx, raw_idx, score.weight * weight);
                        }
                        if (stats.has_invalid_score) {break;}
                    }
                    if (stats.has_invalid_score) {break;}
                }
            } catch (...) {stats.has_invalid_score = 1;}
        }
        if (stats.has_invalid_score) {entries.clear(); std::fill(sums.begin(), sums.end(), 0);}
        // 既存の添字APIも同じ分布へ接続。入力の再ソートなし。
        const auto legacy_idx = static_cast<uint32_t>(rules.size());
        for (uint32_t idx = 0; points && idx < legacy_ids.size(); ++idx) {
            append(legacy_idx, UINT32_MAX, legacy_ids[idx],
                legacy_weights.size() == legacy_ids.size() ? legacy_weights[idx] : 1);
        }
        double sum = 0;
        for (uint32_t idx = 0; idx < sums.size(); ++idx) {
            if (sums[idx] > 0) {ratio += idx == legacy_idx ? legacy_ratio : rules[idx].ratio;}
        }
        for (const auto &item : entries) {
            const auto share = item.rule_idx == legacy_idx ? legacy_ratio : rules[item.rule_idx].ratio;
            sum += item.mass / sums[item.rule_idx] * share;
            cumulative.push_back(sum);
        }
        stats.num_entries = entries.size();
        if (!std::isfinite(sum) || !std::isfinite(ratio) || ratio >= 1 || sum <= 0) {
            if (!entries.empty()) {stats.has_invalid_score = 1;}
            entries.clear(); cumulative.clear(); ratio = 0;
        }
    }

    uint32_t sample(std::mt19937 &random, const VoxelGrid *grid) {
        const auto value = std::generate_canonical<double, 53>(random) * cumulative.back();
        const auto pos = std::upper_bound(cumulative.begin(), cumulative.end(), value) - cumulative.begin();
        const auto &item = entries[std::min<std::size_t>(pos, entries.size() - 1)];
        ++stats.num_priority_samples;
        if (item.raw_idx != UINT32_MAX) {return item.raw_idx;}
        const auto &range = grid->voxel_range[item.cell_idx];
        return grid->voxel_index[std::uniform_int_distribution<uint32_t>(range.start, range.end - 1)(random)].raw_index;
    }

    const std::vector<uint32_t> &points_for(uint32_t rule_id, const VoxelGrid &grid) {
        debug_ids.clear();
        for (const auto &item : entries) {
            if (item.rule_idx >= rule_ids.size() || rule_ids[item.rule_idx] != rule_id) {continue;}
            if (item.raw_idx != UINT32_MAX) {debug_ids.push_back(item.raw_idx);}
            else {
                const auto &range = grid.voxel_range[item.cell_idx];
                for (uint32_t pos = range.start; pos < range.end; ++pos) {debug_ids.push_back(grid.voxel_index[pos].raw_index);}
            }
        }
        std::sort(debug_ids.begin(), debug_ids.end());
        return debug_ids;
    }

    void finish_input() {
        rule_ids.clear();
        for (const auto &rule : rules) {rule_ids.push_back(rule.id);}
        rules.clear();
    }

private:
    std::vector<double> sums, cumulative;
    std::vector<gng_sampling_score> scores;
    std::vector<uint32_t> node_counts, debug_ids, rule_ids, point_rules;
    static bool is_valid_weight(double weight) {return std::isfinite(weight) && weight >= 0;}
    void append(uint32_t rule_idx, uint32_t cell_idx, uint32_t raw_idx, double mass) {
        if (!is_valid_weight(mass) || !std::isfinite(sums[rule_idx] + mass)) {stats.has_invalid_score = 1; return;}
        if (mass == 0) {return;}
        entries.push_back({rule_idx, cell_idx, raw_idx, mass}); sums[rule_idx] += mass;
    }
};
}  // 名前空間gng_sampling
