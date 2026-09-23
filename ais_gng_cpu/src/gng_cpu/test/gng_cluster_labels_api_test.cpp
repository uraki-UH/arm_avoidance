#include <fuzzrobo/libgng/api.h>

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}
}

int main(int argc, char **argv) {
    require(argc == 2, "人または車のテスト指定の欠落");
    const uint8_t label = std::string(argv[1]) == "human" ? HUMAN : CAR;
    const uint8_t other_label = label == HUMAN ? CAR : HUMAN;
    auto set_param = [](const char *name, float value) {
        require(gng_setParameter(name, 0, value), "パラメータ設定の失敗");
    };
    set_param("node.num_max", 512);
    set_param("node.learning_num", 0);
    set_param("node.grid", 0.5f);
    set_param("edge.num_max", 5120);
    set_param("cluster.node_num_min", 3);
    set_param("cluster.plane.volume", 1000);
    set_param("input.point_cloud_num", 1024);
    set_param("input.voxel_grid_unit", 0.05f);
    set_param("label.fuzzy.unknown", 0.05f);
    set_param("label.fuzzy.lpf_time_constant", 0);
    set_param("cluster.human.confirmation_age", 3);
    set_param("cluster.car.confirmation_age", 3);
    set_param("cluster.human.hysteresis_age", 2);
    set_param("cluster.car.hysteresis_age", 2);
    for (uint32_t idx = 0; idx < 4; ++idx) {
        require(gng_setParameter("node.interval", idx, 0.04f), "ノード間隔設定の失敗");
    }
    require(gng_init() == SUCCESS, "初期化の失敗");
    std::vector<Vec3> points;
    for (int z_idx = 0; z_idx < 10; ++z_idx) {
        for (int y_idx = 0; y_idx < 4; ++y_idx) {
            for (int x_idx = 0; x_idx < 7; ++x_idx) {
                points.push_back({2.0f + 0.12f * x_idx,
                    -0.18f + 0.12f * y_idx, 0.2f + 0.12f * z_idx});
            }
        }
    }
    LiDAR_Config config{};
    config.point_step = sizeof(Vec3);
    auto advance = [&]() {
        gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size(), &config);
        gng_exec();
    };
    for (int iter = 0; iter < 12; ++iter) {advance();}
    const auto initial_map = gng_getTopologicalMap();
    require(initial_map.cluster_num > 0, "テスト用クラスタの未生成");
    const auto id = initial_map.clusters[0].id;
    const auto birth_frame = initial_map.clusters[0].frame;
    auto get_cluster = [&]() {
        const auto map = gng_getTopologicalMap();
        for (uint32_t idx = 0; idx < map.cluster_num; ++idx) {
            if (map.clusters[idx].id == id && map.clusters[idx].frame == birth_frame) {
                return map.clusters[idx];
            }
        }
        throw std::runtime_error("テスト中のクラスタ追跡切れ");
    };
    auto feedback = [&](uint8_t inferred_label, uint32_t extra_age = 0) {
        const auto map = gng_getTopologicalMap();
        const uint32_t age = map.frame_number - get_cluster().frame + extra_age;
        gng_setInferredClusterLabels(&id, &age, &inferred_label, 1);
    };
    auto expect_unconfirmed = [&]() {
        const auto cluster = get_cluster();
        require(cluster.label != HUMAN && cluster.label != CAR, "確認前または期限切れの誤確定");
    };
    expect_unconfirmed();

    // 現在より古い個体を指す年齢の拒否。
    for (int iter = 0; iter < 4; ++iter) {feedback(label, 1); advance();}
    expect_unconfirmed();

    // 同一フレーム内の重複結果は確認回数への二重加算なし。
    for (int iter = 1; iter <= 3; ++iter) {
        for (int duplicate = 0; duplicate < 4; ++duplicate) {feedback(label);}
        advance();
        if (iter < 3) {expect_unconfirmed();}
    }
    require(get_cluster().label == label, "連続推論後のラベル未確定");
    for (int iter = 0; iter < 4; ++iter) {
        feedback(label);
        advance();
        require(get_cluster().label == label, "連続推論中のラベル消失");
    }

    // 推論途絶時の2フレーム保持と期限切れ後の失効。
    for (int iter = 0; iter < 2; ++iter) {
        feedback(UNKNOWN_OBJECT);
        advance();
        require(get_cluster().label == label, "保持期間内のラベル消失");
    }
    advance();
    expect_unconfirmed();
    for (int iter = 0; iter < 4; ++iter) {advance(); expect_unconfirmed();}

    // 期限切れ後の古い確認回数の再利用防止。
    for (int iter = 1; iter <= 3; ++iter) {
        feedback(label);
        advance();
        if (iter < 3) {expect_unconfirmed();}
    }
    require(get_cluster().label == label, "再確認後のラベル未確定");

    // 人と車の確認回数の混在防止。
    for (int iter = 1; iter <= 3; ++iter) {
        feedback(other_label);
        advance();
        if (iter < 3) {expect_unconfirmed();}
    }
    require(get_cluster().label == other_label, "別クラスの確認後のラベル未確定");

    // 保持設定0でも次フレームでの推論反映は有効。
    set_param("cluster.human.hysteresis_age", 0);
    set_param("cluster.car.hysteresis_age", 0);
    feedback(other_label);
    advance();
    require(get_cluster().label == other_label, "保持設定0での推論結果消失");
    advance();
    expect_unconfirmed();
    std::cout << argv[1] << ": 確定・保持・失効・再確認・クラス切替の成功\n";
}
