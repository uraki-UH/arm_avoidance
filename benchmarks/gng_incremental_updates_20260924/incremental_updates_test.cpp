#include "cpu/cugng.hpp"
#include "edge_dense_reference/cugng.hpp"

#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>

namespace {

void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}

bool is_same_key(const GngNodeKey &first, const GngNodeKey &second) {
    return first.id == second.id && first.frame == second.frame;
}

void compare(CUGNG &actual, dense_cugng &expected) {
    require(actual.node_num == expected.node_num, "node_num");
    require(actual.nodes.size() == expected.nodes.size(), "nodes_size");
    require(actual.edge_count.size() <= 1 + actual.nodes.size() * NODE_MAX_EDGE / 2, "pool_bound");
    for (size_t idx = 0; idx < actual.nodes.size(); ++idx) {
        const auto &node = actual.nodes[idx];
        const auto &reference = expected.nodes[idx];
        require(node.id == reference.id, "node_id");
        if (node.id == NODE_NOID) {continue;}
        require(node.edge_num == reference.edge_num, "edge_num");
        require(node.frame == reference.frame && node.age_s1 == reference.age_s1, "node_age");
        require(std::memcmp(&node.eta_s1, &reference.eta_s1, sizeof(float)) == 0 &&
            std::memcmp(&node.eta_s2, &reference.eta_s2, sizeof(float)) == 0, "eta");
        require(node.static_node == reference.static_node &&
            node.clustered_flag == reference.clustered_flag, "flags");
        require(std::memcmp(node.pos.p, reference.pos.p, sizeof(node.pos.p)) == 0, "node_position");
        require(std::memcmp(node.edges, reference.edges, sizeof(uint32_t) * node.edge_num) == 0,
                "adjacency_order");
        for (size_t other_idx = 0; other_idx < actual.nodes.size(); ++other_idx) {
            const auto actual_idx = actual.getEdgeIndex(idx, other_idx);
            const auto expected_idx = expected.getEdgeIndex(idx, other_idx);
            require(actual.edge_count[actual_idx] == expected.edge_count[expected_idx], "edge_age");
        }
        for (uint32_t slot_idx = 0; slot_idx < node.edge_num; ++slot_idx) {
            const auto other_idx = node.edges[slot_idx];
            const auto actual_idx = actual.getEdgeIndex(idx, other_idx);
            const auto expected_idx = expected.getEdgeIndex(idx, other_idx);
            require(actual_idx != 0, "active_edge_id");
            require(actual_idx == actual.getEdgeIndex(other_idx, idx), "reciprocal_id");
            require(actual.edge_slots[idx][slot_idx] == actual_idx, "slot_id");
            require(std::memcmp(&actual.edge_distance[actual_idx], &expected.edge_distance[expected_idx],
                                sizeof(float)) == 0, "edge_dist");
        }
    }
    const auto *actual_delta = actual.getMapDelta();
    const auto *expected_delta = expected.getMapDelta();
    require((actual_delta != nullptr) == (expected_delta != nullptr), "delta_enabled");
    if (actual_delta) {
        require(actual_delta->node_delta_count == expected_delta->node_delta_count, "node_delta_num");
        require(actual_delta->edge_delta_count == expected_delta->edge_delta_count, "edge_delta_num");
        for (uint32_t idx = 0; idx < actual_delta->node_delta_count; ++idx) {
            const auto &first = actual_delta->node_deltas[idx];
            const auto &second = expected_delta->node_deltas[idx];
            require(is_same_key(first.key, second.key) && first.operation == second.operation,
                    "node_delta_order");
        }
        for (uint32_t idx = 0; idx < actual_delta->edge_delta_count; ++idx) {
            const auto &first = actual_delta->edge_deltas[idx];
            const auto &second = expected_delta->edge_deltas[idx];
            require(is_same_key(first.first, second.first) && is_same_key(first.second, second.second) &&
                    first.operation == second.operation, "edge_delta_order");
        }
    }
}

}  // 名前空間

int main() {
    try {
        Param param;
        param.init();
        param.config.node_grid = 2.f;
        param.config.voxel_grid_unit = 0.1f;
        param.config.x_min = param.config.y_min = param.config.z_min = -10;
        param.config.x_max = param.config.y_max = param.config.z_max = 10;
        param.node.eta_s1 = 0.05f;
        param.node.eta_s2 = 0.005f;
        param.node.learning_num = 8;
        CUGNG actual;
        dense_cugng expected;
        std::mt19937 random(20260924);
        uint32_t compared_num = 0;
        const auto check = [&]() {
            actual.calc_edge_distanceXY();
            expected.calc_edge_distanceXY();
            compare(actual, expected);
            actual.end_update_frame();
            ++compared_num;
        };
        const auto add = [&](Vec3f point) {
            require(actual.add_node(point) == expected.add_node(point), "added_id");
        };
        for (const uint32_t max_age : {0U, 1U, 100U, 254U, 255U, 256U}) {
            // 64ビット境界と末尾の余りを含む容量変更、clearと再初期化の両経路。
            param.node.num_max = max_age % 2 ? 65 : 129;
            param.edge.age_max = max_age;
            param.node.eta_decay_rate = max_age % 2 ? 0.99f : 1.f;
            if (max_age == 100 || max_age == 255) {actual.clear(); expected.clear();}
            actual.setMapDeltaCapture(false);
            expected.setMapDeltaCapture(false);
            require(actual.init(&param.node, &param.edge, &param.config), "actual_init");
            require(expected.init(&param.node, &param.edge, &param.config), "reference_init");
            actual.setMapDeltaCapture(true);
            expected.setMapDeltaCapture(true);
            for (int idx = 0; idx < param.node.num_max; ++idx) {
                add(Vec3f((idx % 8 - 4) * 0.8f, (idx / 8 - 4) * 0.8f, 0));
            }
            // 自己接続、重複接続、次数上限到達後の接続、逆順からの寿命リセット。
            for (uint32_t idx = 0; idx < 13; ++idx) {
                actual.connect(0, idx);
                expected.connect(0, idx);
                actual.connect(idx, 0);
                expected.connect(idx, 0);
                check();
            }
            // 寿命の巻戻り後に既存版が生成する重複隣接と、共有寿命の維持。
            actual.disconnect_all(1);
            expected.disconnect_all(1);
            actual.connect(1, 2);
            expected.connect(1, 2);
            actual.edge_count[actual.getEdgeIndex(1, 2)] = 0;
            expected.edge_count[expected.getEdgeIndex(1, 2)] = 0;
            actual.connect(1, 2);
            expected.connect(1, 2);
            check();
            actual.disconnect(1, 2);
            expected.disconnect(1, 2);
            check();
            actual.disconnect_all(1);
            expected.disconnect_all(1);
            check();

            for (uint32_t iter = 0; iter < 600; ++iter) {
                actual.finishMapDeltaFrame();
                expected.finishMapDeltaFrame();
                actual.beginMapDeltaFrame();
                expected.beginMapDeltaFrame();
                actual.frame_number = expected.frame_number = iter + 1;
                // 同期再利用・孤立候補・距離更新の全組合せ。既存孤立ノードも同期時に収集。
                actual.begin_update_frame(iter & 1U, iter & 2U, iter & 4U);
                std::vector<Vec3f> points;
                for (uint32_t idx = 0; idx < 12; ++idx) {
                    points.emplace_back((int(random() % 60) - 30) * 0.1f,
                        (int(random() % 60) - 30) * 0.1f, 0);
                }
                std::vector<uint8_t> first_labels(points.size()), second_labels(points.size());
                std::vector<Voxel> first_mapping(points.size()), second_mapping(points.size());
                uint32_t first_num = 0, second_num = 0;
                actual.getDownSampling(points, points.size(), first_labels, first_mapping, first_num);
                expected.getDownSampling(points, points.size(), second_labels, second_mapping, second_num);
                require(first_labels == second_labels && first_num == second_num &&
                    std::memcmp(first_mapping.data(), second_mapping.data(), sizeof(Voxel) * first_num) == 0,
                    "matching");
                const uint32_t first_idx = random() % param.node.num_max;
                const uint32_t second_idx = random() % param.node.num_max;
                const bool has_first = actual.nodes[first_idx].id != NODE_NOID;
                const bool has_second = actual.nodes[second_idx].id != NODE_NOID;
                const uint32_t operation = random() % 9;
                if (operation <= 1 && has_first && has_second) {
                    actual.connect(first_idx, second_idx);
                    expected.connect(first_idx, second_idx);
                } else if (operation == 2) {
                    actual.disconnect(first_idx, second_idx);
                    expected.disconnect(first_idx, second_idx);
                } else if (operation == 3) {
                    actual.disconnect_all(first_idx);
                    expected.disconnect_all(first_idx);
                } else if (operation == 4) {
                    actual.delete_node(first_idx);
                    expected.delete_node(first_idx);
                } else if (operation == 5) {
                    add(Vec3f((int(random() % 60) - 30) * 0.1f,
                              (int(random() % 60) - 30) * 0.1f, 0));
                } else if (operation == 6 && has_first) {
                    auto point = actual.nodes[first_idx].pos;
                    point.p[0] += 0.03f;
                    actual.move_node(actual.nodes[first_idx], point);
                    expected.move_node(expected.nodes[first_idx], point);
                } else if (operation == 7 && has_first) {
                    // 学習本体による寿命加算、接続、隣接ノード移動の同時照合。
                    auto point = actual.nodes[first_idx].pos;
                    point.p[0] += 0.01f;
                    if (actual.nodes[first_idx].edge_num) {
                        const auto neighbor_idx = actual.nodes[first_idx].edges[0];
                        const uint8_t age = iter % 2 ? 255 : max_age;
                        actual.edge_count[actual.getEdgeIndex(first_idx, neighbor_idx)] = age;
                        expected.edge_count[expected.getEdgeIndex(first_idx, neighbor_idx)] = age;
                    }
                    actual.learn_normal(point);
                    expected.learn_normal(point);
                } else if (operation == 8) {
                    actual.check_edge_distance();
                    expected.check_edge_distance();
                }
                // 寿命切れによる他ノードの孤立と、ID順・最少残存数の確認。
                actual.check_age();
                expected.check_age();
                actual.check_delete_no_edge_and_decay_eta();
                expected.check_delete_no_edge_and_decay_eta();
                check();
                // フレーム外の位置更新を含む次フレームでの全走査への復帰。
                if (iter % 11 == 0 && actual.nodes[0].id != NODE_NOID) {
                    auto point = actual.nodes[0].pos; point.p[0] += 0.001f;
                    actual.move_node(actual.nodes[0], point);
                    expected.move_node(expected.nodes[0], point);
                }
            }
        }
        // 外部で単独利用されるNodeの初期化・隣接構造の不変性。
        Node standalone;
        auto point = Vec3f(1, 2, 3);
        standalone.init(9, 0.1f, 0.01f, point);
        require(standalone.id == 9 && standalone.edge_num == 0, "standalone_node");
        require(std::memcmp(point.p, standalone.pos.p, sizeof(point.p)) == 0, "standalone_position");
        std::cout << "incremental_updates_test=passed comparisons=" << compared_num << '\n';
    } catch (const std::exception &error) {
        std::cerr << "incremental_updates_test=failed " << error.what() << '\n';
        return 1;
    }
}
