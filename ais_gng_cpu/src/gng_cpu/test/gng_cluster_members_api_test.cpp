#include <fuzzrobo/libgng/api.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}
}

int main() {
    require(gng_setParameter("node.num_max", 0, 512), "ノード容量設定の失敗");
    require(gng_setParameter("node.learning_num", 0, 0), "学習回数設定の失敗");
    require(gng_setParameter("node.grid", 0, 0.5f), "探索グリッド設定の失敗");
    require(gng_setParameter("edge.num_max", 0, 5120), "エッジ容量設定の失敗");
    require(gng_setParameter("cluster.node_num_min", 0, 3), "クラスタ構成数設定の失敗");
    require(gng_setParameter("cluster.plane.volume", 0, 1000), "平面クラスタ面積設定の失敗");
    require(gng_setParameter("input.point_cloud_num", 0, 1024), "入力容量設定の失敗");
    require(gng_setParameter("input.voxel_grid_unit", 0, 0.05f), "入力ボクセル設定の失敗");
    for (uint32_t idx = 0; idx < 4; ++idx) {
        require(gng_setParameter("node.interval", idx, 0.04f), "ノード間隔設定の失敗");
    }
    require(gng_init() == SUCCESS, "初期化の失敗");

    // 離れた二つの立体点群。乱数学習なしのクラスタ所属情報の確認。
    std::vector<Vec3> points;
    for (const float offset : {2.0f, 6.0f}) {
        for (int z_idx = 0; z_idx < 10; ++z_idx) {
            for (int y_idx = 0; y_idx < 4; ++y_idx) {
                for (int x_idx = 0; x_idx < 7; ++x_idx) {
                    points.push_back({offset + 0.12f * x_idx,
                        -0.18f + 0.12f * y_idx, 0.2f + 0.12f * z_idx});
                }
            }
        }
    }
    LiDAR_Config config{};
    config.point_step = sizeof(Vec3);
    uint32_t max_clusters = 0;
    uint32_t max_members = 0;
    for (int iter = 0; iter < 12; ++iter) {
        gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size(), &config);
        gng_exec();
        const auto map = gng_getTopologicalMap();
        std::vector<bool> has_owner(map.node_num, false);
        for (uint32_t idx = 0; idx < map.cluster_num; ++idx) {
            const auto &cluster = map.clusters[idx];
            require(cluster.node_num > 0, "所属ノード数が0の出力クラスタ");
            require(cluster.nodes != nullptr, "所属ノード配列の欠落");
            max_members = std::max(max_members, cluster.node_num);
            for (uint32_t member_idx = 0; member_idx < cluster.node_num; ++member_idx) {
                const auto node_idx = cluster.nodes[member_idx];
                require(node_idx < map.node_num, "所属ノード添字の範囲外");
                require(!has_owner[node_idx], "所属ノードの重複またはクラスタ配列の重複");
                has_owner[node_idx] = true;
            }
        }
        max_clusters = std::max(max_clusters, map.cluster_num);
    }
    require(max_clusters >= 2, "複数クラスタの未生成");
    require(max_members >= 30, "分類器の入力数を満たすクラスタの未生成");
    std::cout << "クラスタ所属API: 12フレーム、最大クラスタ数=" << max_clusters
              << "、最大所属ノード数=" << max_members << '\n';
}
