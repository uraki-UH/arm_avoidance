#include "cpu/gng.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <set>

namespace {
// 既存unknown枠を無効にした場合の全体学習と、入力置換・実行後の既定復帰。
bool check_unknown_attention_switch() {
    GNG gng;
    gng.param.config.point_cloud_num = 16;
    gng.param.config.node_grid = 1;
    gng.param.config.x_min = gng.param.config.y_min = gng.param.config.z_min = -5;
    gng.param.config.x_max = gng.param.config.y_max = gng.param.config.z_max = 5;
    gng.param.node.num_max = 8;
    gng.param.node.learning_num = 4;
    gng.param.node.unknown_learning_rate = 100;
    gng.param.node.eta_s1 = gng.param.node.eta_s2 = 0;
    for (auto &value : gng.param.node.vigilance2) {value = 100;}
    if (gng.init("") != SUCCESS) {return false;}
    std::vector<Vec3f> input{Vec3f(0.1f, 0, 0)}, attention{Vec3f(2.1f, 0, 0)};
    auto first = Vec3f(0, 0, 0), second = Vec3f(2, 0, 0), third = Vec3f(3, 0, 0);
    const auto first_id = gng.n1.add_node(first);
    const auto second_id = gng.n1.add_node(second);
    gng.n1.add_node(third);
    gng.n1.setTrainingEventCapture(true);
    for (const bool enable_unknown : {true, false}) {
        gng.n1.enable_unknown_attention = enable_unknown;
        gng.n1.learn(input, 1, attention, 1);
        uint32_t num = 0;
        const auto *events = gng.n1.getTrainingEvents(&num);
        if (num != 4) {return false;}
        for (uint32_t idx = 0; idx < num; ++idx) {
            if (events[idx].winner_node_id != (enable_unknown ? second_id : first_id)) {return false;}
        }
    }
    LiDAR_Config config;
    config.point_step = 12;
    const float points[] = {0.1f, 0, 0, 2.1f, 0, 0};
    gng.n1.enable_unknown_attention = false;
    gng.setPointCloud(reinterpret_cast<const uint8_t *>(points), 2, &config);
    if (!gng.n1.enable_unknown_attention) {return false;}
    gng.n1.enable_unknown_attention = false;
    gng.exec();
    return gng.n1.enable_unknown_attention;
}

uint32_t lookup_span_raw_idx(const GNG &gng, uint32_t idx) {
    // 学習側と同じ64点索引による展開。基準は別途作成した従来XYZ候補列。
    const auto &spans = gng.observation_attention_spans;
    const auto &blocks = gng.observation_attention_blocks;
    const auto block = idx / 64;
    const auto begin = spans.begin() + blocks.at(block);
    const auto end = spans.begin() + std::min<std::size_t>(
        static_cast<std::size_t>(blocks.at(block + 1)) + 1, spans.size());
    const auto span = idx < begin->end ? begin : std::upper_bound(begin + 1, end, idx,
        [](uint32_t value, const auto &entry) {return value < entry.end;});
    if (span == end || idx < span->begin || idx >= span->end) {return UINT32_MAX;}
    return gng.vg.voxel_index.at(span->source_begin + idx - span->begin).raw_index;
}

bool check_learn_point(GNG &gng, uint32_t raw_idx, uint32_t source_idx, bool enable_spans) {
    auto &core = gng.n1;
    auto point = gng.map.input_pcl[raw_idx];
    Node_d nearest;
    core.getMinGrid(point, nearest);
    if (nearest.id1 == NODE_NOID) {return false;}
    const auto origin = core.nodes[nearest.id1].pos;
    const Vec3f residual(point.p[0] - origin.p[0], point.p[1] - origin.p[1], point.p[2] - origin.p[2]);
    std::vector<Vec3f> input{point}, unused_attention;
    std::vector<uint32_t> raw_ids{raw_idx};
    std::vector<observation_attention_span> spans{{0, 1, source_idx}};
    std::vector<uint32_t> blocks{0, 1};
    core.learn(input, 1, unused_attention, 1, nullptr, nullptr,
        enable_spans ? nullptr : &raw_ids, &gng.map.input_pcl, &gng.vg,
        enable_spans ? &spans : nullptr, enable_spans ? &blocks : nullptr);
    uint32_t num = 0;
    const auto *events = core.getTrainingEvents(&num);
    return num == 1 && events && events[0].winner_node_id == nearest.id1 &&
        std::memcmp(&events[0].residual.x, residual.p, sizeof(residual.p)) == 0;
}

bool check_case(uint32_t point_num, float voxel_unit, bool has_angle_table, bool has_candidates) {
    GNG gng;
    gng.param.config.point_cloud_num = std::max(1U, point_num);
    gng.param.config.voxel_grid_unit = voxel_unit;
    gng.param.config.node_grid = 1;
    gng.param.config.x_min = gng.param.config.y_min = gng.param.config.z_min = -5;
    gng.param.config.x_max = gng.param.config.y_max = gng.param.config.z_max = 5;
    gng.param.node.num_max = 4;
    gng.param.node.learning_num = 1;
    gng.param.node.unknown_learning_rate = 100;
    gng.param.node.eta_s1 = gng.param.node.eta_s2 = 0;
    gng.param.node.ds_range_max2 = 1;
    for (auto &value : gng.param.node.vigilance2) {value = 100;}
    if (gng.init("") != SUCCESS) {return false;}
    for (uint32_t idx = 0; idx < 4; ++idx) {
        auto point = Vec3f(static_cast<float>(idx * 2) - 3, 0, 0);
        const auto id = gng.n1.add_node(point);
        if (id == NODE_NOID) {return false;}
        auto &node = gng.n1.nodes[id];
        node.label = has_candidates && idx != 2 ? UNKNOWN_OBJECT : SAFE_TERRAIN;
        node.clusted_label = has_candidates && idx == 1 ? HUMAN : SAFE_TERRAIN;
    }
    gng.n1.setTrainingEventCapture(true);
    gng_observation::ray_angles angles{};
    gng.n1.observation_angle_table = has_angle_table ? &angles : nullptr;
    gng.input_pcl_num = point_num;
    for (uint32_t idx = 0; idx < point_num; ++idx) {
        // 異なるセル・大小の区間・範囲外点を含む入力。元番号ごとに異なるXYZ。
        const auto group = (idx * 7 + idx / 101) % 4;
        gng.map.input_pcl[idx] = Vec3f(static_cast<float>(group * 2) - 3 + 0.0001f * (idx % 97),
            0.005f * (idx % 83), 0.001f * (idx % 67));
        if (idx % 17 == 0) {gng.map.input_pcl[idx].p[0] = 100;}
    }
    gng.vg.applyFilter(gng.map.input_pcl, point_num, gng.map.inpcl_labels);
    const auto filtered_labels = gng.map.inpcl_labels;
    gng.attention();
    const bool enable_expected_spans = point_num > 65536 &&
        (ATTENTION_LOOKUP_VARIANT == 2 || has_angle_table);
    if (gng.enable_observation_attention_compact != enable_expected_spans) {return false;}
    if (!has_candidates && gng.attention_pcl_num != 0) {return false;}

    std::vector<uint32_t> expected_raw_ids, expected_source_ids;
    std::vector<Vec3f> expected_points;
    auto expected_labels = filtered_labels;
    for (uint32_t idx = 0; idx < gng.vg.filtered_pcl_num; ++idx) {
        if (gng.voxel_labels[idx] == 0) {continue;}
        for (uint32_t source_idx = gng.vg.voxel_range[idx].start;
             source_idx < gng.vg.voxel_range[idx].end; ++source_idx) {
            const auto raw_idx = gng.vg.voxel_index[source_idx].raw_index;
            expected_raw_ids.push_back(raw_idx);
            expected_source_ids.push_back(source_idx);
            expected_points.push_back(gng.map.input_pcl[raw_idx]);
            expected_labels[raw_idx] = gng.voxel_labels[idx];
        }
    }
    if (expected_raw_ids.size() != static_cast<uint32_t>(gng.attention_pcl_num) ||
        expected_labels != gng.map.inpcl_labels) {return false;}
    for (uint32_t idx = 0; idx < point_num; idx += 17) {
        if (gng.map.inpcl_labels[idx] != 0) {return false;}
    }
    for (uint32_t idx = 0; idx < expected_raw_ids.size(); ++idx) {
        const bool has_raw_lookup = ATTENTION_LOOKUP_VARIANT != 0 || has_angle_table;
        const auto raw_idx = enable_expected_spans ? lookup_span_raw_idx(gng, idx)
            : has_raw_lookup ? gng.observation_attention_raw_ids.at(idx) : expected_raw_ids[idx];
        if (raw_idx != expected_raw_ids[idx]) {return false;}
        const auto &point = has_raw_lookup ? gng.map.input_pcl[raw_idx] : gng.attention_pcl[idx];
        if (std::memcmp(point.p, expected_points[idx].p, sizeof(point.p)) != 0) {return false;}
    }
    if (enable_expected_spans) {
        if (gng.observation_attention_blocks.size() != (expected_raw_ids.size() + 63) / 64 + 1 ||
            gng.observation_attention_blocks.back() != gng.observation_attention_spans.size()) {return false;}
    }

    // 区間端と64点ブロック境界から選んだ、実学習関数の座標参照検証。
    std::set<uint32_t> selected_ids{0, 1, 62, 63, 64, 65};
    if (!expected_raw_ids.empty()) {
        selected_ids.insert(expected_raw_ids.size() - 1);
        selected_ids.insert(expected_raw_ids.size() / 2);
    }
    const auto &spans = gng.observation_attention_spans;
    for (uint32_t idx = 0; idx < std::min<std::size_t>(16, spans.size()); ++idx) {
        selected_ids.insert(spans[idx].begin);
        selected_ids.insert(spans[idx].end - 1);
    }
    for (const auto idx : selected_ids) {
        if (idx >= expected_raw_ids.size()) {continue;}
        if (!check_learn_point(gng, expected_raw_ids[idx], expected_source_ids[idx], false) ||
            !check_learn_point(gng, expected_raw_ids[idx], expected_source_ids[idx], true)) {return false;}
    }
    return true;
}
}

int main() {
    if (!check_unknown_attention_switch()) {
        std::cerr << "unknown重点枠の切替・復帰の不整合\n";
        return 1;
    }
    uint32_t case_num = 0;
    for (const uint32_t point_num : {0U, 63U, 64U, 65U, 65535U, 65536U, 65537U}) {
        for (const auto voxel_unit : {0.f, 0.1f}) {
            for (const auto has_angle_table : {false, true}) {
                for (const auto has_candidates : {false, true}) {
                    if (!check_case(point_num, voxel_unit, has_angle_table, has_candidates)) {
                        std::cerr << "attention_lookup_mismatch " << point_num << ' ' << voxel_unit << ' '
                                  << has_angle_table << ' ' << has_candidates << '\n';
                        return 1;
                    }
                    ++case_num;
                }
            }
        }
    }
    std::cout << "attention_lookup_test=passed cases=" << case_num << '\n';
}
