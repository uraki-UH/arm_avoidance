#include "cpu/sampling.hpp"
#include <chrono>
#include <iostream>
#include <random>

// 同一入力・同一ノードによる、集計と抽選準備だけの追加コスト。
int main() {
    using clock = std::chrono::steady_clock;
    std::mt19937 random(20260926);
    std::uniform_real_distribution<float> xy(-10, 10), height(.1f, 1.8f);
    std::vector<Vec3f> points;
    for (uint32_t idx = 0; idx < 100000; ++idx) {
        points.push_back(idx < 95000 ? Vec3f(xy(random),xy(random),0) :
            Vec3f(1.2f+.25f*std::cos(idx),.23f+.25f*std::sin(idx),height(random)));
    }
    GridConfig bounds{}; bounds.x_min = bounds.y_min = -12; bounds.z_min = -2;
    bounds.x_max = bounds.y_max = 12; bounds.z_max = 4;
    OtherConfig options{}; options.point_cloud_num = points.size();
    std::vector<Node> nodes(4000);
    for (uint32_t idx = 0; idx < nodes.size(); ++idx) {
        nodes[idx].id = idx; nodes[idx].pos = points[idx < 3800 ? idx*25 : 95000+(idx-3800)*25];
        if (idx >= 3800) {nodes[idx].pos.p[0] -= .12f;}
    }
    for (double input_size : {.1, .5}) for (double coarse_size : {0., .5, 1.}) {
        bounds.unit = input_size; options.voxel_grid_unit = input_size;
        if (!bounds.init(bounds)) {return 1;}
        VoxelGrid grid; grid.init(&bounds, &options);
        std::vector<uint8_t> labels(points.size());
        fuzzrobo::builtin_sampling::tracking_cells cells;
        gng_tracking_sampling_input settings; settings.ratio = .25;
        settings.cell_size = coarse_size ? coarse_size : .5;
        gng_sampling::frame_sampler sampler;
        double setup_ms = 0, voxel_ms = 0, selection_ms = 0;
        uint32_t candidates = 0, point_evaluations = 0;
        for (uint32_t iter = 0; iter < 120; ++iter) {
            const auto begin = clock::now();
            sampler.reset_input();
            if (coarse_size) {
                cells.reset(settings);
                for (const auto &node : nodes) {cells.add_node(node.pos.p, node.id >= 3800);}
                grid.tracking = &cells;
                sampler.rules = {cells.sampling_rule()};
            }
            const auto setup_end = clock::now();
            grid.applyFilter(points, points.size(), labels);
            const auto voxel_end = clock::now();
            sampler.build(&grid, &points, nodes, {}, {}, 0);
            const auto end = clock::now();
            if (iter >= 20) {
                setup_ms += std::chrono::duration<double,std::milli>(setup_end-begin).count();
                voxel_ms += std::chrono::duration<double,std::milli>(voxel_end-setup_end).count();
                selection_ms += std::chrono::duration<double,std::milli>(end-voxel_end).count();
            }
            candidates = sampler.stats.num_entries;
            point_evaluations = sampler.stats.num_point_evaluations;
        }
        std::cout << "input_size=" << input_size << " coarse_size=" << coarse_size
            << " setup_ms=" << setup_ms/100 << " voxel_ms=" << voxel_ms/100
            << " selection_ms=" << selection_ms/100 << " total_ms=" << (setup_ms+voxel_ms+selection_ms)/100
            << " entries=" << candidates << " point_evaluations=" << point_evaluations << '\n';
    }
}
