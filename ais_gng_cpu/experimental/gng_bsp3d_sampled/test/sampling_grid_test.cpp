#include "../src/cpu/sampling_grid.hpp"
#include <iostream>
#include <stdexcept>

void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}

int main() {
    try {
        Param params;
        params.node.num_max = 128;
        params.node.learning_num = 0;
        params.node.s1_reset_range2 = 0.01f;
        params.node.ds_range_max2 = 0.01f;
        params.config.point_cloud_num = 1024;
        params.config.voxel_grid_unit = 0.5f;
        params.config.x_min = params.config.y_min = params.config.z_min = -2;
        params.config.x_max = params.config.y_max = params.config.z_max = 2;
        CUGNG graph;
        require(graph.init(&params.node, &params.edge, &params.config), "初期化");
        vector<Vec3f> points{{0.01f, 0.01f, 0.01f}, {0.49f, 0.49f, 0.49f},
            {-0.001f, -0.001f, -0.001f}, {0.49f, 0.01f, 1.0f}, {0.01f, 0.49f, 1.0f}};
        vector<uint8_t> labels(params.config.point_cloud_num);
        VoxelGrid voxels;
        voxels.init(&graph.voxel_config, &params.config);
        voxels.applyFilter(points, points.size(), labels);
        for (const auto &position : vector<Vec3f>{{0.48f, 0.49f, 0.49f}, {0.49f, 0.48f, 0.49f},
            {0.49f, 0.49f, 0.48f}, {0.9f, 0.49f, 0.49f}, {0.25f, 0.25f, 1.0f}}) {
            auto point = position;
            graph.add_node(point);
        }
        for (auto &node : graph.nodes) {
            if (node.id == NODE_NOID) {continue;}
            node.age_s1 = 7;
            node.label = UNKNOWN_OBJECT;
        }
        sampling_grid index;
        vector<uint32_t> attention_ids;
        index.prepare(graph, voxels, points, labels, 0.5f, 0, attention_ids);
        // 同じ観測点の周囲にある3ノード。2近傍への制限なし・学習回数0。
        require(graph.nodes[0].age_s1 == 0 && graph.nodes[1].age_s1 == 0 && graph.nodes[2].age_s1 == 0,
            "未学習・第3近傍も実測点で寿命維持");
        require(graph.nodes[3].age_s1 == 7 && graph.nodes[4].age_s1 == 7,
            "実測点のない場所の寿命維持なし");
        require(index.has_point_near(Vec3f(0.001f, 0.001f, 0.001f), 0.0001f), "負座標セル境界をまたぐ観測");
        require(std::find(attention_ids.begin(), attention_ids.end(), 1U) != attention_ids.end(),
            "重心から離れた元点も重点候補に保持");
        require(graph.sampling_statistics.num_nearest_queries == 0, "観測確認・候補選別のノードtree検索なし");
        graph.sampling_statistics = {};
        index.prepare(graph, voxels, points, labels, 0.5f, 2, attention_ids);
        require(graph.sampling_statistics.num_probe_points == 2 && graph.sampling_statistics.num_nearest_queries == 2,
            "全域探索の回数制限");
        points.clear();
        voxels.applyFilter(points, 0, labels);
        index.prepare(graph, voxels, points, labels, 0.5f, 2, attention_ids);
        require(index.num_cells() == 0 && attention_ids.empty(), "空入力時の候補と索引の消去");
        // 探索で追加したノードの即時接続。後続の学習なしでも孤立しない構成。
        require(graph.init(&params.node, &params.edge, &params.config), "接続検証の再初期化");
        points = {{-1, 0, 0}, {0, 0, 0}, {1, 0, 0}};
        voxels.applyFilter(points, points.size(), labels);
        index.prepare(graph, voxels, points, labels, 0.5f, 3, attention_ids);
        require(graph.node_num == 3 && graph.nodes[0].edge_num > 0 &&
            graph.nodes[1].edge_num > 0 && graph.nodes[2].edge_num > 0, "新規ノードの接続維持");

        // 観測判定の独立した全点走査との照合。セルの境界・空隙を含む配置。
        std::mt19937 random(20260923);
        std::uniform_real_distribution<float> uniform(-1.9f, 1.9f);
        points.clear();
        for (uint32_t idx = 0; idx < 1000; ++idx) {
            points.emplace_back(uniform(random), uniform(random), uniform(random));
        }
        voxels.applyFilter(points, points.size(), labels);
        index.prepare(graph, voxels, points, labels, 0.5f, 0, attention_ids);
        for (uint32_t idx = 0; idx < 300; ++idx) {
            Vec3f query(uniform(random), uniform(random), uniform(random));
            bool has_expected = false;
            for (const auto &point : points) {has_expected |= query.squaredNorm(point) < 0.01f;}
            require(index.has_point_near(query, 0.01f) == has_expected, "元点全走査との観測判定一致");
        }
        // 重複点数による重点枠の独占防止。両voxelの元点への均等な選択機会。
        params.node.learning_num = 1000;
        params.node.eta_s1 = params.node.eta_s2 = 0;
        params.config.point_cloud_num = 2048;
        require(graph.init(&params.node, &params.edge, &params.config), "重点配分検証の再初期化");
        points.assign(1000, Vec3f(0, 0, 0));
        points.emplace_back(1, 0, 0);
        labels.resize(params.config.point_cloud_num);
        voxels.init(&graph.voxel_config, &params.config);
        voxels.applyFilter(points, points.size(), labels);
        require(voxels.filtered_pcl_num == 2, "重複点と離れた点の2voxel");
        for (auto point : vector<Vec3f>{{0,0,0},{1,0,0}}) {
            const auto node_idx = graph.add_node(point);
            graph.nodes[node_idx].label = UNKNOWN_OBJECT;
        }
        graph.attention_voxel_ids = {0,1};
        vector<uint32_t> raw_ids(points.size());
        for (uint32_t idx = 0; idx < raw_ids.size(); ++idx) {raw_ids[idx] = idx;}
        vector<Vec3f> unused_attention;
        graph.learn(voxels.filtered_pcl, 2, unused_attention, 2, nullptr, nullptr, &raw_ids, &points, &voxels);
        require(graph.sampling_statistics.num_zero_samples > 350 && graph.sampling_statistics.num_zero_samples < 650,
            "点数比1000対1に依存しない重点学習配分");
        require(graph.sampling_statistics.num_attention_hits == 800, "重点学習枠と取得済み最近傍の再利用");
        std::cout << "元点保持・観測寿命・重点候補・探索回数・即時接続・重点配分の検証成功\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
