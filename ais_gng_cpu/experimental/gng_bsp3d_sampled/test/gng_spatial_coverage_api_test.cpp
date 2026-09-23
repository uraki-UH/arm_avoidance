#include <fuzzrobo/libgng/api.h>

#include <cstdint>
#include <iostream>
#include <vector>

int main() {
    // ノード上限を超える二層の点群。入力順とボクセル順はいずれも下層優先。
    gng_setParameter("node.num_max", 0, 512);
    gng_setParameter("node.learning_num", 0, 0);
    gng_setParameter("node.grid", 0, 0.5f);
    gng_setParameter("input.point_cloud_num", 0, 8192);
    gng_setParameter("input.voxel_grid_unit", 0, 0.05f);
    for (uint32_t idx = 0; idx < 4; ++idx) {
        gng_setParameter("node.interval", idx, 0.05f);
    }
    if (gng_init() != SUCCESS) {return 1;}
    std::vector<Vec3> points;
    for (const float z : {-1.0f, 1.0f}) {
        for (uint32_t y_idx = 0; y_idx < 64; ++y_idx) {
            for (uint32_t x_idx = 0; x_idx < 64; ++x_idx) {
                points.push_back({-2.52f + 0.08f * x_idx, -2.52f + 0.08f * y_idx, z});
            }
        }
    }
    LiDAR_Config config;
    config.point_step = sizeof(Vec3);
    gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size(), &config);
    gng_exec();
    const auto map = gng_getTopologicalMap();
    uint32_t num_lower = 0, num_upper = 0;
    for (uint32_t idx = 0; idx < map.node_num; ++idx) {
        num_lower += map.nodes[idx].pos.z < -0.5f;
        num_upper += map.nodes[idx].pos.z > 0.5f;
    }
    std::cout << "nodes=" << map.node_num << " lower=" << num_lower
              << " upper=" << num_upper << '\n';
    // 容量制約下でも両層に残るノードの確認。乱数学習なしの初期生成経路。
    return map.node_num <= 512 && num_lower >= 100 && num_upper >= 100 ? 0 : 2;
}
