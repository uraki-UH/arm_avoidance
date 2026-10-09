#include "cpu/voxel_grid.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

void test_registered_input() {
    const char *stage = "格子初期化";
    const auto require = [&](bool is_valid) {if (!is_valid) {throw std::runtime_error(stage);}};
    GridConfig grid{};
    grid.x_min = grid.y_min = grid.z_min = -2;
    grid.x_max = grid.y_max = grid.z_max = 2;
    grid.unit = .1f; require(grid.init(grid));
    OtherConfig config{}; config.point_cloud_num = 10003; config.voxel_grid_unit = .1f;
    VoxelGrid normal, registered; normal.init(&grid, &config); registered.init(&grid, &config);
    std::mt19937 random(41); std::uniform_real_distribution<float> coord(-2.2f, 2.2f);
    std::vector<Vec3f> points;
    std::vector<std::uint64_t> entries;
    for (uint32_t idx = 0; idx < config.point_cloud_num; ++idx) {
        points.emplace_back(coord(random), coord(random), coord(random));
        if (idx < 3) {points.back() = Vec3f(idx == 0 ? -2.f : 2.f, idx == 2 ? 2.f : 0.f, idx == 2 ? 2.f : 0.f);}
        const auto cell_idx = grid.getIndex(points.back());
        if (cell_idx < grid.maxXYZ) {entries.push_back((uint64_t{cell_idx} << 32) | idx);}
    }
    std::sort(entries.begin(), entries.end());
    std::vector<uint8_t> normal_labels(points.size()), registered_labels(points.size());
    fuzzrobo::builtin_sampling::tracking_cells normal_tracking, registered_tracking;
    gng_tracking_sampling_input settings; settings.ratio = .25;
    normal_tracking.reset(settings); registered_tracking.reset(settings);
    normal.tracking = &normal_tracking;
    normal.applyFilter(points, points.size(), normal_labels);
    stage = "登録配列の受理";
    require(registered.set_registered_input(entries.data(), entries.size(), points.size(), registered_labels));
    // 呼出し元配列の借用なし。入力置換前の再実行でも登録結果の保持
    const auto saved_entries = entries; entries.assign(entries.size(), UINT64_MAX);
    registered.tracking = &registered_tracking;
    registered.applyFilter(points, points.size(), registered_labels);
    stage = "代表点数・元点数の一致";
    require(normal.filtered_pcl_num == registered.filtered_pcl_num && normal.voxel_index_num == registered.voxel_index_num);
    stage = "範囲内ラベルの一致"; require(normal_labels == registered_labels);
    stage = "セル番号・元番号の一致";
    for (uint32_t idx = 0; idx < normal.voxel_index_num; ++idx) {
        require(normal.voxel_index[idx].voxel_index == registered.voxel_index[idx].voxel_index);
        require(normal.voxel_index[idx].raw_index == registered.voxel_index[idx].raw_index);
    }
    stage = "セル範囲・代表点の一致";
    for (uint32_t idx = 0; idx < normal.filtered_pcl_num; ++idx) {
        require(normal.voxel_range[idx].start == registered.voxel_range[idx].start);
        require(normal.voxel_range[idx].end == registered.voxel_range[idx].end);
        for (uint32_t axis = 0; axis < 3; ++axis) {require(normal.filtered_pcl[idx][axis] == registered.filtered_pcl[idx][axis]);}
    }
    stage = "追従用細セル参照の一致";
    require(normal_tracking.fine_cells == registered_tracking.fine_cells);
    stage = "追従用粗セル集計の一致";
    require(normal_tracking.cells.size() == registered_tracking.cells.size());
    for (std::size_t idx = 0; idx < normal_tracking.cells.size(); ++idx) {
        require(normal_tracking.cells[idx].num_points == registered_tracking.cells[idx].num_points);
        for (uint32_t axis = 0; axis < 3; ++axis) {
            require(normal_tracking.cells[idx].point_sum[axis] == registered_tracking.cells[idx].point_sum[axis]);
        }
    }
    stage = "再実行・不正入力・復帰";
    // 学習等で上書きされた元点ラベルの再初期化、範囲外点ラベルの消去
    std::fill(registered_labels.begin(), registered_labels.end(), 7);
    registered.applyFilter(points, points.size(), registered_labels);
    require(normal_labels == registered_labels);
    require(!registered.set_registered_input(nullptr, 1, points.size(), registered_labels));
    registered.applyFilter(points, points.size(), registered_labels);
    require(registered.filtered_pcl_num == 0);
    require(registered.set_registered_input(nullptr, 0, points.size(), registered_labels));
    registered.applyFilter(points, points.size(), registered_labels);
    require(registered.filtered_pcl_num == 0);
    require(registered.set_registered_input(saved_entries.data(), saved_entries.size(), points.size(), registered_labels));
    registered.applyFilter(points, points.size(), registered_labels);
    require(normal_labels == registered_labels);
}

int main() {
    test_registered_input();
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
    // 有効時の代表元点と最後のセルの保持。平均座標ではないことの確認。
    if (filter.filtered_pcl_num != 2 ||
        filter.filtered_pcl[0][0] != points[0][0] ||
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
