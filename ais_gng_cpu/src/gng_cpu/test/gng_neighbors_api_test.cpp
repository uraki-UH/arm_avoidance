#include <fuzzrobo/libgng/api.h>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}

int main() {
    require(gng_get_node_num_neighbors(65535) == UINT32_MAX, "無効ノードの返値不一致");
    require(gng_setParameter("node.num_max", 0, 1024), "ノード数設定の失敗");
    require(gng_setParameter("node.learning_num", 0, 4000), "学習回数設定の失敗");
    require(gng_setParameter("input.point_cloud_num", 0, 2000), "入力容量設定の失敗");
    require(gng_init() == SUCCESS, "初期化の失敗");
    LiDAR_Config config{};
    config.point_step = 3 * sizeof(float);
    std::vector<float> points;
    for (int row = 0; row < 30; ++row) {
        for (int column = 0; column < 30; ++column) {
            points.insert(points.end(), {1.0f + 0.04f * column, -0.6f + 0.04f * row, 0.2f});
        }
    }
    uint32_t max_observed_neighbors = 0;
    for (int iter = 0; iter < 20; ++iter) {
        gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size() / 3, &config);
        gng_exec();
        const auto map = gng_getTopologicalMap();
        require(map.node_num > 0, "学習ノードなし");
        std::vector<uint32_t> expected(map.node_num, 0);
        for (uint32_t idx = 0; idx + 1 < map.edge_num; idx += 2) {
            require(map.edges[idx] < map.node_num && map.edges[idx + 1] < map.node_num, "出力エッジ添字の不正値");
            ++expected[map.edges[idx]];
            ++expected[map.edges[idx + 1]];
        }
        for (uint32_t idx = 0; idx < map.node_num; ++idx) {
            const auto num_neighbors = gng_get_node_num_neighbors(map.nodes[idx].id);
            require(num_neighbors == expected[idx], "既存次数と出力エッジの不一致");
            if (num_neighbors > max_observed_neighbors) {max_observed_neighbors = num_neighbors;}
        }
    }
    require(max_observed_neighbors > 4, "次数4の両側を含まない検証データ");
    require(gng_get_node_num_neighbors(65535) == UINT32_MAX, "無効ノードの誤認識");
    std::cout << "隣接数API: 学習20フレームと出力グラフの次数一致\n";
}
