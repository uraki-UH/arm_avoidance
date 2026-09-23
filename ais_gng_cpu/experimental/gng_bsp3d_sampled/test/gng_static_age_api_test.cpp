#include <fuzzrobo/libgng/api.h>

#include <array>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

int main(int argc, char **argv) {
    if (argc < 2 || argc > 4) {return 1;}
    const int expected_age = std::atoi(argv[1]);
    const auto set_parameter = [](const char *name, float value) {
        return gng_setParameter(name, 0, value) != 0;
    };
    // 不正な寿命の拒否。既定値・直前値の保持確認は後続の削除時刻で実施。
    const auto can_reject_invalid_age = [&]() {
        for (const float value : std::array<float, 6>{
            0, -1, 1.5f, std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::quiet_NaN(),
            static_cast<float>(std::numeric_limits<int>::max())}) {
            if (set_parameter("node.static.s1_age_max", value)) {return false;}
        }
        return true;
    };
    if (argc >= 3 && !set_parameter("node.static.s1_age_max", std::atoi(argv[2]))) {return 2;}
    if (!can_reject_invalid_age()) {return 3;}
    if (!set_parameter("node.num_max", 128) || !set_parameter("node.learning_num", 0) ||
        !set_parameter("node.grid", 0.5f) || !set_parameter("node.static.age_min", 1) ||
        !set_parameter("input.point_cloud_num", 128) ||
        !set_parameter("input.voxel_grid_unit", 0.05f)) {return 4;}
    for (const char *name : {"input.x_min", "input.y_min", "input.z_min"}) {
        if (!set_parameter(name, -5)) {return 5;}
    }
    for (const char *name : {"input.x_max", "input.y_max", "input.z_max"}) {
        if (!set_parameter(name, 5)) {return 6;}
    }
    for (uint32_t idx = 0; idx < 4; ++idx) {
        if (!gng_setParameter("node.interval", idx, 0.05f) ||
            !gng_setParameter("node.s1_age_max", idx, 1000) ||
            !gng_setParameter("node.clusted_s1_age", idx, 1000)) {return 7;}
    }
    if (gng_init() != SUCCESS) {return 8;}
    std::vector<Vec3> near_points, far_points;
    for (const float x : {-0.3f, 0.3f}) {
        for (const float y : {-0.3f, 0.3f}) {
            for (const float z : {-0.3f, 0.3f}) {
                near_points.push_back({x, y, z});
                far_points.push_back({x + 3, y, z});
            }
        }
    }
    LiDAR_Config config;
    config.point_step = sizeof(Vec3);
    const auto update = [&](const std::vector<Vec3> &points) {
        gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size(), &config);
        gng_exec();
        const auto map = gng_getTopologicalMap();
        uint32_t num_near = 0;
        for (uint32_t idx = 0; idx < map.node_num; ++idx) {
            num_near += map.nodes[idx].pos.x < 1;
        }
        return num_near;
    };
    // 学習なし・同一点群の観測による長期記憶への昇格と寿命リセット。
    for (int iter = 0; iter < 5; ++iter) {update(near_points);}
    if (argc == 4 && !set_parameter("node.static.s1_age_max", std::atoi(argv[3]))) {return 9;}
    if (!can_reject_invalid_age()) {return 10;}
    for (int iter = 0; iter <= expected_age; ++iter) {
        if (update(near_points) == 0) {return 11;}
    }
    // 遠方だけの観測へ切替。通常寿命1000とは独立した長期記憶寿命の確認。
    for (int missed_frames = 1; missed_frames <= expected_age; ++missed_frames) {
        const auto num_near = update(far_points);
        if ((num_near == 0) != (missed_frames == expected_age)) {
            std::cerr << "age=" << expected_age << " missed_frames=" << missed_frames
                      << " remaining=" << num_near << '\n';
            return 12;
        }
    }
    std::cout << "static_age=" << expected_age << " passed\n";
    return 0;
}
