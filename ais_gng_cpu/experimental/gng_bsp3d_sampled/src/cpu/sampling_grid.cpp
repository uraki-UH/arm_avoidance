#include "sampling_grid.hpp"
#include <numeric>

size_t sampling_grid::cell_hash::operator()(const cell_key &key) const {
    const auto mix = [](uint64_t value) {
        value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
        value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
        return value ^ (value >> 31);
    };
    return mix(static_cast<uint64_t>(key.x)) ^
        mix(static_cast<uint64_t>(key.y) + 0x9e3779b97f4a7c15ULL) ^
        mix(static_cast<uint64_t>(key.z) + 0x3c6ef372fe94f82aULL);
}

sampling_grid::cell_key sampling_grid::key_at(const Vec3f &point, double offset) const {
    return {static_cast<int64_t>(std::floor((static_cast<double>(point.p[0]) + offset) / cell_size)),
        static_cast<int64_t>(std::floor((static_cast<double>(point.p[1]) + offset) / cell_size)),
        static_cast<int64_t>(std::floor((static_cast<double>(point.p[2]) + offset) / cell_size))};
}

float sampling_grid::box_dist2(const Vec3f &point, const cell &entry) {
    float result = 0;
    for (int axis = 0; axis < 3; ++axis) {
        const float diff = std::max({entry.min_pos.p[axis] - point.p[axis],
            point.p[axis] - entry.max_pos.p[axis], 0.0f});
        result += diff * diff;
    }
    return result;
}

void sampling_grid::build(const VoxelGrid &voxels, const vector<Vec3f> &points, double size) {
    input_points = &points;
    cell_size = size;
    lookup.clear();
    cells.clear();
    const uint32_t num_points = voxels.voxel_index_num;
    point_cells.resize(num_points);
    point_ids.resize(num_points);
    for (uint32_t idx = 0; idx < num_points; ++idx) {
        const auto raw_idx = voxels.voxel_index[idx].raw_index;
        const auto &point = points[raw_idx];
        const auto [entry, is_new] = lookup.try_emplace(key_at(point), cells.size());
        if (is_new) {
            cell value;
            value.min_pos = value.max_pos = point;
            cells.push_back(value);
        }
        point_cells[idx] = entry->second;
        auto &bucket = cells[entry->second];
        ++bucket.num;
        for (int axis = 0; axis < 3; ++axis) {
            bucket.min_pos.p[axis] = std::min(bucket.min_pos.p[axis], point.p[axis]);
            bucket.max_pos.p[axis] = std::max(bucket.max_pos.p[axis], point.p[axis]);
        }
    }
    uint32_t offset = 0;
    for (auto &entry : cells) {
        entry.begin = entry.cursor = offset;
        offset += entry.num;
        entry.end = offset;
    }
    for (uint32_t idx = 0; idx < num_points; ++idx) {
        point_ids[cells[point_cells[idx]].cursor++] = voxels.voxel_index[idx].raw_index;
    }
}

bool sampling_grid::has_point_near(const Vec3f &position, float max_dist2) const {
    if (!input_points || max_dist2 <= 0) {return false;}
    const double radius = std::sqrt(static_cast<double>(max_dist2));
    const auto min_key = key_at(position, -radius);
    const auto max_key = key_at(position, radius);
    for (int64_t z = min_key.z; z <= max_key.z; ++z) {
        for (int64_t y = min_key.y; y <= max_key.y; ++y) {
            for (int64_t x = min_key.x; x <= max_key.x; ++x) {
                const auto found = lookup.find({x, y, z});
                if (found == lookup.end()) {continue;}
                const auto &entry = cells[found->second];
                if (box_dist2(position, entry) >= max_dist2) {continue;}
                for (uint32_t idx = entry.begin; idx < entry.end; ++idx) {
                    const auto &point = (*input_points)[point_ids[idx]];
                    const float x = position.p[0] - point.p[0];
                    const float y = position.p[1] - point.p[1];
                    const float z = position.p[2] - point.p[2];
                    if (x * x + y * y + z * z < max_dist2) {return true;}
                }
            }
        }
    }
    return false;
}

void sampling_grid::mark_attention(const Vec3f &position, float max_dist2, uint8_t label) {
    if (max_dist2 <= 0) {return;}
    const double radius = std::sqrt(static_cast<double>(max_dist2));
    const auto min_key = key_at(position, -radius);
    const auto max_key = key_at(position, radius);
    for (int64_t z = min_key.z; z <= max_key.z; ++z) {
        for (int64_t y = min_key.y; y <= max_key.y; ++y) {
            for (int64_t x = min_key.x; x <= max_key.x; ++x) {
                const auto found = lookup.find({x, y, z});
                if (found == lookup.end()) {continue;}
                auto &entry = cells[found->second];
                // 実点群の境界箱との重なり。重点対象の取りこぼしを避ける候補判定。
                if (box_dist2(position, entry) < max_dist2) {entry.label |= label;}
            }
        }
    }
}

void sampling_grid::prepare(CUGNG &graph, const VoxelGrid &voxels, const vector<Vec3f> &points,
    vector<uint8_t> &labels, float min_cell_size, uint32_t max_probe_num,
    vector<uint32_t> &attention_raw_ids) {
    const auto &config = graph.gng_config;
    // 観測・重点判定で訪れるセル数の抑制。入力点の削除・平均化なし。
    const double max_radius = std::sqrt(static_cast<double>(std::max(config.s1_reset_range2, config.ds_range_max2)));
    build(voxels, points, std::max(static_cast<double>(min_cell_size), 2.0 * max_radius));
    attention_raw_ids.clear();
    if (cells.empty()) {return;}
    for (auto &node : graph.nodes) {
        if (node.id == NODE_NOID) {continue;}
        // 学習への採用と独立した、実測点による全ノードの寿命維持。
        if (has_point_near(node.pos, config.s1_reset_range2)) {
            node.age_s1 = 0;
            ++graph.sampling_statistics.num_observed_nodes;
        }
        if (node.clusted_label == HUMAN) {
            mark_attention(node.pos, config.ds_range_max2, 0b111);
        } else if (node.label == UNKNOWN_OBJECT) {
            mark_attention(node.pos, config.ds_range_max2, 0b011);
        }
    }
    for (const auto &entry : cells) {
        if (entry.label == 0) {continue;}
        for (uint32_t idx = entry.begin; idx < entry.end; ++idx) {
            const auto raw_idx = point_ids[idx];
            labels[raw_idx] = entry.label;
            attention_raw_ids.push_back(raw_idx);
        }
    }
    graph.sampling_statistics.num_attention_candidates = attention_raw_ids.size();

    // 入力voxel列を等分した全域探索。区間内の巡回と、YAML解像度に沿う処理枠の配分。
    const uint32_t num_probe = std::min(max_probe_num, voxels.filtered_pcl_num);
    vector<uint32_t> probe_order(num_probe);
    std::iota(probe_order.begin(), probe_order.end(), 0U);
    std::mt19937 random(graph.frame_number);
    // ノード上限に先に到達する入力層の偏り防止。treeの子セル順とは独立。
    std::shuffle(probe_order.begin(), probe_order.end(), random);
    for (const uint32_t idx : probe_order) {
        const uint32_t begin = static_cast<uint64_t>(idx) * voxels.filtered_pcl_num / num_probe;
        const uint32_t end = static_cast<uint64_t>(idx + 1) * voxels.filtered_pcl_num / num_probe;
        auto point = voxels.filtered_pcl[begin + graph.frame_number % (end - begin)];
        Node_d winners;
        const bool is_in_vigilance = graph.getMinGrid(point, winners);
        if (!is_in_vigilance) {
            const auto node_idx = graph.add_node(point);
            // 次の全点走査に依存しない新規ノードの接続。探索枠削減時の即時孤立削除の防止。
            if (node_idx != NODE_NOID) {
                if (winners.id1 != NODE_NOID) {graph.connect(node_idx, winners.id1);}
                if (winners.id2 != NODE_NOID) {graph.connect(node_idx, winners.id2);}
            }
        }
        if (winners.id1 != NODE_NOID && winners.id2 != NODE_NOID) {graph.connect(winners.id1, winners.id2);}
        ++graph.sampling_statistics.num_probe_points;
    }
}
