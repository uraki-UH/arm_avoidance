#include <fuzzrobo/libgng/api.h>
#include <array>
#include <iostream>

int main() {
    if (!gng_setParameter("node.grid", 0, 0.5f) ||
        !gng_setParameter("input.voxel_grid_unit", 0, 0) ||
        gng_setParameter("input.voxel_grid_unit", 0, -1) ||
        !gng_setParameter("node.num_max", 0, 128) || gng_init() != SUCCESS) {return 1;}
    if (gng_setParameter("node.num_max", 0, 256) ||
        gng_setParameter("input.x_max", 0, 100)) {return 2;}
    const std::array<Vec3, 8> points{{
        {-0.3f,-0.3f,0}, {-0.3f,0.3f,0}, {0.3f,-0.3f,0}, {0.3f,0.3f,0},
        {-0.2f,-0.2f,0.1f}, {-0.2f,0.2f,0.1f}, {0.2f,-0.2f,0.1f}, {0.2f,0.2f,0.1f}}};
    LiDAR_Config config;
    config.point_step = sizeof(Vec3);
    gng_setTrainingEventCapture(1);
    gng_setTrainingEventMaxWinnerRank(1);
    // 初期化後の変更が実際の学習回数とイベント確保量へ反映されることの確認。
    for (const int num : {0, 12, 6000, 0}) {
        if (!gng_setParameter("node.learning_num", 0, num)) {return 3;}
        gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size(), &config);
        gng_exec();
        uint32_t num_events = 0;
        gng_getTrainingEvents(&num_events);
        std::cout << "learning=" << num << " events=" << num_events << '\n';
        if (num_events != static_cast<uint32_t>(num)) {return 4;}
    }
    return 0;
}
