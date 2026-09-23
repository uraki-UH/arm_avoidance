#include "../src/cpu/input_voxels.hpp"
#include <stdexcept>
#include <limits>
void require(bool is_valid) {if (!is_valid) {throw std::runtime_error("重心・境界・無効入力の検証失敗");}}
int main() {
    OtherConfig params{};
    params.x_min = params.y_min = params.z_min = -1;
    params.x_max = params.y_max = params.z_max = 1;
    params.voxel_grid_unit = 0.5f; params.point_cloud_num = 20;
    input_voxels voxels;
    require(voxels.init(params));
    vector<Vec3f> raw{{0,0,0},{0.3f,0,0},{1,1,1},{-1,-1,-1},
        {std::numeric_limits<float>::quiet_NaN(),0,0},{0,0,2}};
    vector<uint8_t> labels(raw.size());
    voxels.prepare(raw, raw.size(), labels);
    require(voxels.num_input_points == 4 && voxels.points.size() == 3);
    require(labels == vector<uint8_t>({1,1,1,1,0,0}));
    require(voxels.points[0].p[0] == -1 && std::abs(voxels.points[1].p[0] - 0.15f) < 1e-6f &&
        voxels.points[2].p[2] == 1);
    voxels.prepare(raw, 0, labels);
    require(voxels.points.empty() && voxels.num_input_points == 0);
    params.voxel_grid_unit = 1e-9f;
    require(!voxels.init(params));
}
