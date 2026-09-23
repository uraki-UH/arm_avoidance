#include <fuzzrobo/libgng/api.h>
#include <fuzzrobo/libgng/sampling_api.h>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <iostream>

void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}

int main() {
    try {
        require(!gng_setParameter("input.voxel_grid_unit", 0, 0.000001f), "入力ボクセル設定の拒否");
        require(!gng_setParameter("node.grid", 0, 0.000001f), "ノードグリッド設定の拒否");
        require(!gng_setParameter("sampling.max_probe_num", 0, 100), "事前探索枠設定の拒否");
        require(gng_setParameter("node.num_max", 0, 128), "ノード数設定");
        require(gng_setParameter("input.point_cloud_num", 0, 100), "入力上限設定");
        require(gng_setParameter("node.learning_num", 0, 4000), "学習回数設定");
        for (const auto *name : {"input.x_min", "input.y_min", "input.z_min"}) {
            require(gng_setParameter(name, 0, -1), "入力範囲の下端設定");
        }
        for (const auto *name : {"input.x_max", "input.y_max", "input.z_max"}) {
            require(gng_setParameter(name, 0, 1), "入力範囲の上端設定");
        }
        require(gng_init() == SUCCESS, "初期化");
        const std::array<Vec3, 9> points{{{0,0,0},{0,0,0},{0.01f,0,0},{-1,0,0},{1,0,0},
            {-0.5f,0.5f,0},{2,0,0},{std::numeric_limits<float>::quiet_NaN(),0,0},
            {0,std::numeric_limits<float>::infinity(),0}}};
        LiDAR_Config config;
        config.point_step = sizeof(Vec3);
        gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size(), &config);
        gng_exec();
        const auto &statistics = *gng_get_minimal_statistics();
        require(statistics.num_input_points == 6, "重複点・境界点・近接点を含む全元点の保持");
        require(statistics.num_nearest_queries == 4000, "最近傍検索は学習枠だけ");
        require(statistics.num_zero_samples > 1000, "重複原点も入力比率どおり選択");
        require(statistics.voxel_ms == 0 && statistics.attention_ms == 0 &&
            statistics.cluster_ms == 0 && statistics.num_probe_points == 0, "ボクセル・事前照合・クラスタ処理なし");
        uint32_t num_labels = 0;
        const auto *labels = gng_getDownSampling(&num_labels);
        require(num_labels == points.size(), "元点番号の保持");
        for (uint32_t idx = 0; idx < num_labels; ++idx) {
            require(labels[idx] == (idx < 6 ? 1 : 0), "YAML範囲と有限座標だけの選別");
        }
        const auto graph = gng_getTopologicalMap();
        require(graph.node_num > 1 && graph.edge_num > 0 && graph.cluster_num == 0, "非空グラフとクラスタ停止");
        for (uint32_t idx = 0; idx < graph.node_num; ++idx) {
            const auto &p = graph.nodes[idx].pos;
            require(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
                p.x >= -1 && p.x <= 1 && p.y >= -1 && p.y <= 1 && p.z >= -1 && p.z <= 1, "有限・範囲内のグラフ");
        }
        gng_setPointCloud(nullptr, 0, &config);
        gng_exec();
        require(gng_get_minimal_statistics()->num_nearest_queries == 0 &&
            gng_get_minimal_statistics()->num_input_points == 0, "空入力で旧点群を再学習しないこと");
        gng_setParameter("node.learning_num", 0, 0);
        gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size(), &config);
        gng_exec();
        require(gng_get_minimal_statistics()->num_nearest_queries == 0, "学習0回で隠れた全点照合なし");
        std::cout << "元点保持・検索回数・空入力・設定範囲の検証成功\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
