#include "../src/cpu/cugng.hpp"

#include <iostream>
#include <stdexcept>

namespace {
void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}

void check_nearest(CUGNG &graph, Vec3f point) {
    std::vector<float> expected;
    for (const auto &node : graph.nodes) {
        if (node.id != NODE_NOID) {expected.push_back(point.squaredNorm(node.pos));}
    }
    std::sort(expected.begin(), expected.end());
    Node_d winners;
    graph.getMinGrid(point, winners);
    const uint32_t ids[]{winners.id1, winners.id2};
    const float dist2[]{winners.id1_d2, winners.id2_d2};
    for (size_t rank = 0; rank < 2; ++rank) {
        if (rank >= expected.size()) {
            require(ids[rank] == NODE_NOID && dist2[rank] == FLT_MAX, "候補不足時の無効値");
            continue;
        }
        require(ids[rank] < graph.nodes.size() && graph.nodes[ids[rank]].id == ids[rank], "有効ノードの選択");
        const float max_dist_error_th = 1e-5f * std::max(1.0f, expected[rank]);
        require(std::abs(dist2[rank] - expected[rank]) <= max_dist_error_th, "全ノード走査との最近傍距離一致");
        require(std::abs(point.squaredNorm(graph.nodes[ids[rank]].pos) - expected[rank]) <= max_dist_error_th,
                "選択ノード座標と距離の一致");
    }
    require(winners.id1 == NODE_NOID || winners.id1 != winners.id2, "異なる2ノードの選択");
}
}

int main() {
    try {
        Param params;
        params.node.num_max = 512;
        params.config.x_min = params.config.y_min = params.config.z_min = -20;
        params.config.x_max = params.config.y_max = params.config.z_max = 20;
        CUGNG graph;
        require(graph.init(&params.node, &params.edge, &params.config), "初期化");
        Vec3f point(-10, -8, -6);
        check_nearest(graph, point);
        uint8_t label = 255;
        Node_d winners;
        require(!graph.getDownSamplingGrid(point, label, winners) && label == 0, "空の木の判定");
        // 従来27セルの探索範囲外にある最近傍の検証。
        for (int idx = 0; idx < 3; ++idx) {
            Vec3f position(10 + idx * 2, 8 + idx * 2, 6 + idx * 2);
            require(graph.add_node(position) != NODE_NOID, "遠方ノードの追加");
            check_nearest(graph, point);
        }

        // 3番目以降のノードを含まない寿命・ラベル・警戒領域の判定。
        point = Vec3f(9, 8, 6);
        graph.gng_config.s1_reset_range2 = graph.gng_config.ds_range_max2 = 100;
        graph.gng_config.vigilance2[SAFE_TERRAIN] = 0.25f;
        graph.gng_config.vigilance2[UNKNOWN_OBJECT] = 100;
        for (auto &node : graph.nodes) {
            node.age_s1 = 7;
            node.label = node.clusted_label = SAFE_TERRAIN;
        }
        graph.nodes[2].label = UNKNOWN_OBJECT;
        graph.nodes[2].clusted_label = HUMAN;
        require(!graph.getDownSamplingGrid(point, label, winners), "3番目の警戒領域を判定に不使用");
        require(label == 0 && graph.nodes[0].age_s1 == 0 && graph.nodes[1].age_s1 == 0 &&
                graph.nodes[2].age_s1 == 7, "最近傍2ノードだけの寿命・ラベル判定");
        graph.nodes[0].label = UNKNOWN_OBJECT;
        require(graph.getDownSamplingGrid(point, label, winners) && label == 3, "最近傍の警戒領域と未知物体ラベル");
        graph.nodes[1].clusted_label = HUMAN;
        graph.getDownSamplingGrid(point, label, winners);
        require(label == 7, "第2近傍のヒトラベル");
        graph.gng_config.s1_reset_range2 = graph.gng_config.ds_range_max2 = 0.25f;
        graph.nodes[0].age_s1 = graph.nodes[1].age_s1 = 7;
        graph.getDownSamplingGrid(point, label, winners);
        require(label == 0 && graph.nodes[0].age_s1 == 7 && graph.nodes[1].age_s1 == 7,
                "距離パラメータによる寿命・ラベル判定");
        graph.getMinGrid(point, winners);
        require(graph.nodes[0].age_s1 == 7 && graph.nodes[1].age_s1 == 7, "学習用検索による寿命変更なし");

        graph.clear();
        require(graph.init(&params.node, &params.edge, &params.config), "再初期化");
        std::mt19937 random(20260923);
        std::uniform_real_distribution<float> distribution(-19, 19);
        const auto random_point = [&] {return Vec3f(distribution(random), distribution(random), distribution(random));};
        for (int idx = 0; idx < 256; ++idx) {
            auto position = random_point();
            require(graph.add_node(position) != NODE_NOID, "分割を伴うノード追加");
        }
        // セル境界・根領域外・追加・移動・削除・ID再利用を含む全走査との照合。
        for (const float coordinate : {-30.0f, -20.0f, 0.0f, 20.0f, 30.0f}) {
            check_nearest(graph, Vec3f(coordinate, coordinate, coordinate));
        }
        for (int iter = 0; iter < 2000; ++iter) {
            check_nearest(graph, random_point());
            const uint32_t node_idx = random() % 256;
            if (iter % 3 == 0) {
                auto position = random_point();
                graph.move_node(graph.nodes[node_idx], position);
            }
            if (iter % 7 == 0) {
                graph.delete_node(node_idx);
                check_nearest(graph, random_point());
                auto position = random_point();
                require(graph.add_node(position) == node_idx, "削除IDの再利用");
            }
        }
        // 根領域外の異なる方向、および削除によるセル統合後の照合。
        for (int iter = 0; iter < 200; ++iter) {
            auto query = random_point();
            for (int axis = 0; axis < 3; ++axis) {query.p[axis] *= 2;}
            check_nearest(graph, query);
        }
        for (uint32_t node_idx = 2; node_idx < 256; ++node_idx) {
            graph.delete_node(node_idx);
            if (node_idx % 16 == 0) {check_nearest(graph, random_point());}
        }
        require(graph.node_num == 2, "セル統合を伴う削除");
        for (int idx = 0; idx < 128; ++idx) {
            auto position = random_point();
            require(graph.add_node(position) != NODE_NOID, "統合後の再分割");
            check_nearest(graph, random_point());
        }
        // 同距離候補の距離一致。候補間のID順序への非依存。
        graph.clear();
        require(graph.init(&params.node, &params.edge, &params.config), "同距離検証の初期化");
        for (const float coordinate : {-1.0f, 1.0f}) {
            Vec3f position(coordinate, 0, 0);
            graph.add_node(position);
        }
        check_nearest(graph, Vec3f(0, 0, 0));
        graph.clear();
        std::cout << "最近傍・境界・寿命・ラベル・動的索引の検証成功\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
