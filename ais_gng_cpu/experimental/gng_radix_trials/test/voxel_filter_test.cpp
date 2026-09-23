#include "cpu/voxel_grid.hpp"
#include <cmath>
#include <iostream>

int main() {
    GridConfig grid{};
    grid.x_min = grid.y_min = grid.z_min = -2;
    grid.x_max = grid.y_max = grid.z_max = 2;
    grid.unit = 1;
    if (!grid.init(grid)) {return 1;}
    OtherConfig config{};
    config.point_cloud_num = 8;
    config.voxel_grid_unit = 0;
    VoxelGrid filter;
    filter.init(&grid, &config);
    std::vector<Vec3f> points{{0.1f,0,0}, {0.2f,0,0}, {1.1f,0,0}, {3,0,0}};
    std::vector<uint8_t> labels(4);
    filter.applyFilter(points, points.size(), labels);
    // ゼロ指定時の重複セル内点の保持、元番号対応、YAML範囲の維持。
    if (filter.filtered_pcl_num != 3 || labels[3] != 0) {return 2;}
    for (uint32_t idx = 0; idx < 3; ++idx) {
        if (filter.voxel_index[idx].raw_index != idx ||
            filter.voxel_range[idx].end != idx + 1 ||
            filter.filtered_pcl[idx][0] != points[idx][0]) {return 3;}
    }
    config.voxel_grid_unit = 1;
    filter.init(&grid, &config);
    filter.applyFilter(points, points.size(), labels);
    // 有効時の平均化と最後のセルの保持。
    if (filter.filtered_pcl_num != 2 ||
        std::abs(filter.filtered_pcl[0][0] - 0.15f) > 1e-6f ||
        std::abs(filter.filtered_pcl[1][0] - 1.1f) > 1e-6f) {return 4;}
    points = {{3,0,0}};
    filter.applyFilter(points, points.size(), labels);
    if (filter.filtered_pcl_num != 0 || filter.voxel_index_num != 0) {return 5;}
    points = {{0.1f,0,0}};
    filter.applyFilter(points, points.size(), labels);
    if (filter.filtered_pcl_num != 1) {return 6;}
    filter.applyFilter(points, 0, labels);
    if (filter.filtered_pcl_num != 0 || filter.voxel_index_num != 0) {return 7;}
    std::cout << "voxel_filter_test=passed\n";
    return 0;
}
