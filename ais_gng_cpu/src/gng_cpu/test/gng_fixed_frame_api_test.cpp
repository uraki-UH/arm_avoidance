#include <fuzzrobo/libgng/api.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

int main() {
    const auto set_parameter = [](const char *name, float value) {
        return gng_setParameter(name, 0, value) != 0;
    };
    if (!set_parameter("node.num_max", 128) || !set_parameter("node.learning_num", 0) ||
        !set_parameter("node.grid", .5f) || !set_parameter("node.static.age_min", -1) ||
        !set_parameter("input.point_cloud_num", 128) ||
        !set_parameter("input.voxel_grid_unit", .05f) ||
        !set_parameter("input.local_coordinates", 0)) {return 1;}
    for (const char *name : {"input.x_min", "input.y_min", "input.z_min"}) {
        if (!set_parameter(name, -5)) {return 2;}
    }
    for (const char *name : {"input.x_max", "input.y_max", "input.z_max"}) {
        if (!set_parameter(name, 5)) {return 3;}
    }
    for (uint32_t idx = 0; idx < 4; ++idx) {
        if (!gng_setParameter("node.interval", idx, .05f)) {return 4;}
    }
    if (gng_init() != SUCCESS) {return 5;}
    std::vector<Vec3> points;
    for (float x : {.8f, 1.2f}) {
        for (float y : {.2f, .6f}) {
            for (float z : {.5f, .9f}) {points.push_back({x, y, z});}
        }
    }
    LiDAR_Config config;
    config.point_step = sizeof(Vec3);
    const auto submit = [&]() {
        gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size(), &config);
    };
    for (int iter = 0; iter < 2; ++iter) {submit(); gng_exec();}
    const auto initial = gng_getTopologicalMap();
    const auto initial_frame = initial.frame_number;
    std::map<uint16_t, Vec3> saved;
    for (uint32_t idx = 0; idx < initial.node_num; ++idx) {
        saved.emplace(initial.nodes[idx].id, initial.nodes[idx].pos);
    }
    if (saved.empty()) {return 6;}
    float max_node_dist = 0;
    // 学習なしの入力TF更新。既存ノードの座標・ID・学習フレームの不変条件
    const auto verify_nodes = [&]() {
        const auto current = gng_getTopologicalMap();
        if (current.frame_number != initial_frame || current.node_num != saved.size()) {return false;}
        for (uint32_t idx = 0; idx < current.node_num; ++idx) {
            const auto &node = current.nodes[idx];
            const auto found = saved.find(node.id);
            if (found == saved.end()) {return false;}
            const auto &before = found->second;
            const float dist = std::sqrt(std::pow(node.pos.x-before.x, 2) +
                std::pow(node.pos.y-before.y, 2) + std::pow(node.pos.z-before.z, 2));
            max_node_dist = std::max(max_node_dist, dist);
        }
        return max_node_dist < 1e-6f;
    };
    // yaw・pitch・rollの90度回転、平行移動、元姿勢への復帰
    const float half = std::sqrt(.5f);
    const Quaternion rotations[] = {{0, 0, half, half}, {0, half, 0, half},
                                    {half, 0, 0, half}, {0, 0, 0, 1}};
    for (const auto &rotation : rotations) {
        config.quat = rotation;
        config.pos = {.3f, -.2f, .1f};
        submit();
        if (!verify_nodes()) {
            std::cerr << "max_node_dist_m=" << max_node_dist << '\n';
            return 7;
        }
    }
    // 新規入力点群のTF適用は維持。z軸90度回転と並進の独立検証
    config.quat = rotations[0];
    submit();
    uint32_t num_points = 0;
    const auto transformed = gng_getAffineTransformedInputPointCloud(&num_points);
    if (num_points != points.size() || !transformed) {return 8;}
    for (uint32_t idx = 0; idx < num_points; ++idx) {
        const auto &point = points[idx];
        if (std::abs(transformed[3*idx] - (-point.y+.3f)) > 1e-6f ||
            std::abs(transformed[3*idx+1] - (point.x-.2f)) > 1e-6f ||
            std::abs(transformed[3*idx+2] - (point.z+.1f)) > 1e-6f) {return 9;}
    }
    config.quat = {0, 0, 0, 1};
    config.pos = {0, 0, 0};
    submit();
    if (!verify_nodes()) {return 10;}
    std::cout << "num_nodes=" << saved.size() << " max_node_dist_m=" << max_node_dist << '\n';
    return 0;
}
