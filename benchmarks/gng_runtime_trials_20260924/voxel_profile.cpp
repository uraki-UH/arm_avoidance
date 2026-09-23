#include "../../ais_gng_cpu/experimental/gng_runtime_trials/src/cpu/input_voxels.hpp"
#include <fstream>
#include <array>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
    if (argc != 2) {return 2;}
    std::ifstream input(argv[1], std::ios::binary);
    uint32_t num_points;
    input.read(reinterpret_cast<char *>(&num_points), sizeof(num_points));
    vector<Vec3f> raw(num_points);
    input.read(reinterpret_cast<char *>(raw.data()), sizeof(Vec3f) * raw.size());
    if (!input || sizeof(Vec3f) != 12) {throw std::runtime_error("入力点群形式の不一致");}
    OtherConfig params{};
    params.x_min = params.y_min = -200; params.z_min = -5;
    params.x_max = params.y_max = 200; params.z_max = 10;
    params.point_cloud_num = num_points;
    for (const auto unit : {0.1f,0.5f}) {
        params.voxel_grid_unit = unit;
        input_voxels reference, profiled;
        reference.init(params); profiled.init(params);
        vector<uint8_t> expected_labels(num_points), labels(num_points);
        reference.prepare(raw, num_points, expected_labels);
        double duration[4]{};
        for (int iter = 0; iter < 35; ++iter) {
            const auto start = std::chrono::steady_clock::now();
            auto &entries = profiled.entries;
            auto &ranges = profiled.ranges;
            auto &points = profiled.points;
            entries.clear(); ranges.clear(); points.clear();
            std::fill(labels.begin(), labels.end(), 0);
            for (uint32_t idx = 0; idx < num_points; ++idx) {
                const auto cell_idx = profiled.config.get_cell_idx(raw[idx]);
                if (cell_idx == UINT32_MAX) {continue;}
                entries.push_back({cell_idx,idx}); labels[idx] = 1;
            }
            const auto after_cells = std::chrono::steady_clock::now();
            boost::sort::spreadsort::integer_sort(entries.begin(),entries.end(),
                [](const input_voxels::entry &value,unsigned offset) {return value.cell_idx >> offset;});
            const auto after_sort = std::chrono::steady_clock::now();
            uint32_t start_idx = 0;
            for (uint32_t idx = 1; idx < entries.size(); ++idx) {
                if (entries[idx].cell_idx == entries[start_idx].cell_idx) {continue;}
                ranges.push_back({start_idx,idx}); start_idx = idx;
            }
            ranges.push_back({start_idx,static_cast<uint32_t>(entries.size())});
            const auto after_ranges = std::chrono::steady_clock::now();
            for (const auto &range : ranges) {
                Vec3f sum(0,0,0);
                for (uint32_t idx = range.start; idx < range.end; ++idx) {
                    const auto &point = raw[entries[idx].raw_idx];
                    for (int axis = 0; axis < 3; ++axis) {sum.p[axis] += point.p[axis];}
                }
                const float weight = 1.0f / (range.end - range.start);
                for (int axis = 0; axis < 3; ++axis) {
                    sum.p[axis] = std::clamp(sum.p[axis] * weight, profiled.config.min_pos.p[axis], profiled.config.max_pos.p[axis]);
                }
                points.push_back(sum);
            }
            const auto end = std::chrono::steady_clock::now();
            if (iter >= 5) {
                const auto stamps = std::array{start,after_cells,after_sort,after_ranges,end};
                for (int idx = 0; idx < 4; ++idx) {
                    duration[idx] += std::chrono::duration<double,std::milli>(stamps[idx+1]-stamps[idx]).count()/30;
                }
            }
            if (points.size() != reference.points.size() || labels != expected_labels ||
                std::memcmp(points.data(), reference.points.data(), points.size()*sizeof(Vec3f)) != 0) {
                throw std::runtime_error("計時対象と元実装の重心不一致");
            }
        }
        std::cout << unit << " cells_ms=" << duration[0] << " sort_ms=" << duration[1]
            << " ranges_ms=" << duration[2] << " centroids_ms=" << duration[3] << '\n';
    }
}
