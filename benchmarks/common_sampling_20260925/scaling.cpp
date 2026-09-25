#include "cpu/sampling.hpp"
#include <chrono>
#include <fstream>
#include <iostream>

// 固定実入力におけるセル条件数と準備コスト。学習・ROS配信を含まない測定。
int main(int argc, char **argv) {
    if (argc != 2) {return 2;}
    std::ifstream input(argv[1], std::ios::binary);
    uint32_t num_anchors = 0, num_points = 0;
    input.read(reinterpret_cast<char *>(&num_anchors), 4);
    input.read(reinterpret_cast<char *>(&num_points), 4);
    if (!input || num_anchors > 65534 || num_points > 1000000) {return 2;}
    input.seekg(num_anchors * 3 * sizeof(float), std::ios::cur);
    std::vector<Vec3f> points(num_points);
    for (auto &point : points) {input.read(reinterpret_cast<char *>(point.p), 12);}
    if (!input) {return 2;}
    GridConfig config{};
    config.unit = .5; config.x_min = config.y_min = -200; config.x_max = config.y_max = 200;
    config.z_min = -20; config.z_max = 80;
    if (!config.init(config)) {return 2;}
    OtherConfig other{}; other.point_cloud_num = num_points; other.voxel_grid_unit = .5;
    VoxelGrid grid; grid.init(&config, &other);
    std::vector<uint8_t> labels(num_points);
    grid.applyFilter(points, num_points, labels);
    std::vector<Node> nodes;
    for (const uint32_t num_rules : {0, 1, 4, 16}) {
        gng_sampling::frame_sampler sampler;
        for (uint32_t idx = 0; idx < num_rules; ++idx) {
            gng_sampling_rule rule;
            rule.id = idx; rule.ratio = .5 / num_rules;
            rule.cell_score = [](const gng_sampling_cell &cell, const void *) {
                return gng_sampling_score{cell.num_points >= 3 ? 1.0 : 0.0, 0};
            };
            sampler.rules.push_back(rule);
        }
        std::vector<double> times;
        for (uint32_t iter = 0; iter < 12; ++iter) {
            const auto start = std::chrono::steady_clock::now();
            sampler.build(&grid, &points, nodes, {}, {}, 0);
            const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            if (iter >= 3) {times.push_back(ms);}
        }
        std::sort(times.begin(), times.end());
        std::cout << "rules=" << num_rules << " points=" << num_points << " cells=" << grid.filtered_pcl_num
            << " prepare_ms=" << times[times.size()/2] << " point_evaluations=" << sampler.stats.num_point_evaluations
            << " entries=" << sampler.stats.num_entries << '\n';
    }
}
