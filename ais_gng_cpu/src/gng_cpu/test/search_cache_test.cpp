#include "cpu/cugng.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <numeric>
#include <random>

namespace {
bool has_same_float(float first, float second) {
    return std::memcmp(&first, &second, sizeof(float)) == 0;
}

bool has_same_graph(const CUGNG &first, const CUGNG &second) {
    if (first.node_num != second.node_num || first.frame_number != second.frame_number ||
        first.nodes.size() != second.nodes.size() || first.grid != second.grid ||
        first.grid_node_num != second.grid_node_num ||
        first.grid_page_offsets != second.grid_page_offsets || first.edge_count != second.edge_count) {
        return false;
    }
    for (std::size_t idx = 0; idx < first.nodes.size(); ++idx) {
        const auto &a = first.nodes[idx];
        const auto &b = second.nodes[idx];
        if (a.id != b.id) {return false;}
        if (a.id == NODE_NOID) {continue;}
        if (std::memcmp(a.pos.p, b.pos.p, sizeof(a.pos.p)) != 0 ||
            a.label != b.label || a.clusted_label != b.clusted_label ||
            a.age_s1 != b.age_s1 || a.frame != b.frame || a.edge_num != b.edge_num ||
            a.grid_i != b.grid_i || a.grid_vec_i != b.grid_vec_i ||
            !has_same_float(a.eta_s1, b.eta_s1) || !has_same_float(a.eta_s2, b.eta_s2) ||
            !std::equal(a.edges, a.edges + a.edge_num, b.edges)) {return false;}
    }
    return true;
}

void reference_down_sampling(CUGNG &gng, std::vector<Vec3f> &points,
                            std::vector<uint8_t> &labels) {
    // バッチ外のNode直接参照による基準処理。追加・接続・乱数順の維持。
    std::vector<uint32_t> order(points.size());
    std::iota(order.begin(), order.end(), 0U);
    std::mt19937 random(gng.frame_number);
    std::shuffle(order.begin(), order.end(), random);
    for (const auto idx : order) {
        Node_d nearest;
        if (!gng.getDownSamplingGrid(points[idx], labels[idx], nearest)) {gng.add_node(points[idx]);}
        if (nearest.id1 != NODE_NOID) {
            if (nearest.id2 != NODE_NOID) {gng.connect(nearest.id1, nearest.id2);}
        }
    }
}
}

int main() {
    CUGNG actual, expected;
    NodeConfig config{};
    config.eta_s1 = 0.13f;
    config.eta_s2 = 0.023f;
    config.learning_num = 29;
    config.unknown_learning_rate = 2;
    config.s1_reset_range2 = 0.25f;
    config.ds_range_max2 = 2.3f;
    for (uint32_t idx = 0; idx < 4; ++idx) {config.vigilance2[idx] = 0.025f * (idx + 1);}
    EdgeConfig edge{};
    edge.age_max = 11;
    OtherConfig input{};
    input.node_grid = input.voxel_grid_unit = 1;
    input.x_min = input.y_min = input.z_min = -10;
    input.x_max = input.y_max = input.z_max = 10;
    std::mt19937 random(20260924);
    std::uniform_real_distribution<float> coordinate(-3.f, 3.f);
    for (const int max_nodes : {128, 31, 257}) {
        config.num_max = max_nodes;
        actual.clear();
        expected.clear();
        if (!actual.init(&config, &edge, &input) || !expected.init(&config, &edge, &input)) {return 1;}
        std::vector<Vec3f> points;
        for (uint32_t idx = 0; idx < 300; ++idx) {
            points.emplace_back(coordinate(random), coordinate(random), coordinate(random));
        }
        // 同一距離の順位と、セル境界の候補順の確認。
        points.emplace_back(0.5f, 0.f, 0.f);
        points.emplace_back(-0.5f, 0.f, 0.f);
        points.emplace_back(0.f, 0.f, 0.f);
        std::vector<uint8_t> labels_actual(points.size()), labels_expected(points.size());
        for (uint32_t frame = 0; frame < 12; ++frame) {
            for (uint32_t idx = 0; idx < actual.nodes.size(); ++idx) {
                auto &a = actual.nodes[idx];
                auto &b = expected.nodes[idx];
                if (a.id == NODE_NOID) {continue;}
                a.age_s1 = b.age_s1 = 100 + idx;
                a.label = b.label = (idx + frame) % 4;
                a.clusted_label = b.clusted_label = idx % 5 == 0 ? HUMAN : UNKNOWN_OBJECT;
                if ((idx + frame) % 11 == 0) {
                    actual.delete_node(idx);
                    expected.delete_node(idx);
                } else if ((idx + frame) % 7 == 0) {
                    auto point = Vec3f(coordinate(random), coordinate(random), coordinate(random));
                    actual.move_node(a, point);
                    expected.move_node(b, point);
                }
            }
            actual.getDownSampling(points, points.size(), labels_actual);
            reference_down_sampling(expected, points, labels_expected);
            if (labels_actual != labels_expected || !has_same_graph(actual, expected)) {
                std::cerr << "down_sampling_mismatch " << max_nodes << ' ' << frame << '\n';
                return 2;
            }
            // 単一候補の繰返し学習による、同一セル内・セル間移動の逐次反映。
            std::vector<Vec3f> learn_points{points[(frame * 23) % points.size()]};
            std::vector<Vec3f> attention;
            actual.learn(learn_points, 1, attention, 0);
            ++expected.frame_number;
            for (int idx = 0; idx < config.learning_num; ++idx) {expected.learn_normal(learn_points[0]);}
            if (!has_same_graph(actual, expected)) {
                std::cerr << "learn_mismatch " << max_nodes << ' ' << frame << '\n';
                return 4;
            }
            // バッチ終了後の直接更新と、単発探索時の即時参照。
            for (auto &node : actual.nodes) {
                if (node.id != NODE_NOID) {node.label = UNKNOWN_OBJECT; node.clusted_label = HUMAN;}
            }
            for (auto &node : expected.nodes) {
                if (node.id != NODE_NOID) {node.label = UNKNOWN_OBJECT; node.clusted_label = HUMAN;}
            }
            Node_d nearest_actual, nearest_expected;
            uint8_t label_actual = 0, label_expected = 0;
            const auto has_actual = actual.getDownSamplingGrid(points[0], label_actual, nearest_actual);
            const auto has_expected = expected.getDownSamplingGrid(points[0], label_expected, nearest_expected);
            if (has_actual != has_expected || label_actual != label_expected ||
                std::memcmp(&nearest_actual, &nearest_expected, sizeof(Node_d)) != 0) {return 5;}
        }
        // 空入力による早期終了後の、バッチ状態の復帰。
        std::vector<Vec3f> empty_points;
        actual.learn(empty_points, 0, empty_points, 0);
        ++expected.frame_number;
        if (!has_same_graph(actual, expected)) {return 6;}
    }
    std::cout << "search_cache_test=passed\n";
}
