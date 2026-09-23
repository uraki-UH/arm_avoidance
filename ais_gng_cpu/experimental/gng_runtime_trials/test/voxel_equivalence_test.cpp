#include "../src/cpu/input_voxels.hpp"
#include <random>
#include <stdexcept>
int main() {
    OtherConfig params{};
    params.x_min = params.y_min = -200; params.z_min = -5;
    params.x_max = params.y_max = 200; params.z_max = 10;
    params.point_cloud_num = 200000;
    std::mt19937 random(74);
    std::uniform_real_distribution<float> xy(-200,200), z(-5,10);
    vector<Vec3f> raw;
    for (uint32_t idx = 0; idx < 150000; ++idx) {
        raw.emplace_back(xy(random), xy(random), z(random));
        if (idx % 7 == 0) {raw.back() = Vec3f(0,0,0);}
    }
    vector<uint8_t> labels(raw.size());
    for (const float unit : {0.1f,0.5f}) {
        params.voxel_grid_unit = unit;
        input_voxels voxels;
        if (!voxels.init(params)) {throw std::runtime_error("入力ボクセル初期化失敗");}
        voxels.prepare(raw, raw.size(), labels);
        // 元の二段階実装と同じソート済み点列による参照重心。
        uint32_t start = 0, point_idx = 0;
        while (start < voxels.entries.size()) {
            uint32_t end = start + 1;
            while (end < voxels.entries.size() &&
                voxels.entries[end].cell_idx == voxels.entries[start].cell_idx) {++end;}
            Vec3f expected(0,0,0);
            for (uint32_t idx = start; idx < end; ++idx) {
                for (int axis = 0; axis < 3; ++axis) {expected.p[axis] += raw[voxels.entries[idx].raw_idx].p[axis];}
            }
            const float weight = 1.0f / (end - start);
            for (int axis = 0; axis < 3; ++axis) {
                expected.p[axis] = std::clamp(expected.p[axis] * weight, voxels.config.min_pos.p[axis], voxels.config.max_pos.p[axis]);
                if (expected.p[axis] != voxels.points[point_idx].p[axis]) {throw std::runtime_error("重心の計算順序・値の不一致");}
            }
            ++point_idx; start = end;
        }
        if (point_idx != voxels.points.size()) {throw std::runtime_error("占有セル数の不一致");}
    }
}
