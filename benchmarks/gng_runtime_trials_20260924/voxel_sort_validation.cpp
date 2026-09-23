#include "../../ais_gng_cpu/experimental/gng_runtime_trials/src/cpu/input_voxels.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
int main(int argc, char **argv) {
    if (argc != 2) {return 2;}
    std::ifstream input(argv[1], std::ios::binary);
    uint32_t num_points;
    input.read(reinterpret_cast<char *>(&num_points), sizeof(num_points));
    vector<Vec3f> raw(num_points);
    input.read(reinterpret_cast<char *>(raw.data()), sizeof(Vec3f)*raw.size());
    if (!input || sizeof(Vec3f) != 12) {return 3;}
    OtherConfig params{};
    params.x_min = params.y_min = -200; params.z_min = -5;
    params.x_max = params.y_max = 200; params.z_max = 10;
    params.point_cloud_num = num_points;
    for (const auto unit : {0.1f,0.5f}) {
        params.voxel_grid_unit = unit;
        input_voxels candidate; candidate.init(params);
        vector<uint8_t> labels(num_points), seen(num_points,0);
        candidate.prepare(raw,num_points,labels);
        vector<input_voxels::entry> original;
        for (uint32_t idx = 0; idx < num_points; ++idx) {
            const auto cell_idx = candidate.config.get_cell_idx(raw[idx]);
            if (cell_idx != UINT32_MAX) {original.push_back({cell_idx,idx});}
        }
        boost::sort::spreadsort::integer_sort(original.begin(),original.end(),
            [](const input_voxels::entry &value,unsigned offset) {return value.cell_idx >> offset;});
        if (original.size() != candidate.entries.size()) {throw std::runtime_error("有効入力点数の不一致");}
        for (uint32_t idx = 0; idx < original.size(); ++idx) {
            const auto &value = candidate.entries[idx];
            if (value.cell_idx != original[idx].cell_idx || value.raw_idx >= num_points ||
                seen[value.raw_idx]++ || value.cell_idx != candidate.config.get_cell_idx(raw[value.raw_idx])) {
                throw std::runtime_error("セル番号・点の欠落・重複の検証失敗");
            }
        }
        if (seen != labels) {throw std::runtime_error("有効点集合の不一致");}
        uint32_t start = 0, point_idx = 0, num_changed = 0;
        double max_error = 0;
        while (start < original.size()) {
            uint32_t end = start+1;
            while (end < original.size() && original[end].cell_idx == original[start].cell_idx) {++end;}
            float sum[3]{};
            for (uint32_t idx = start; idx < end; ++idx) {
                for (int axis = 0; axis < 3; ++axis) {sum[axis] += raw[original[idx].raw_idx].p[axis];}
            }
            bool has_change = false;
            const float weight = 1.0f/(end-start);
            for (int axis = 0; axis < 3; ++axis) {
                const auto expected = std::clamp(sum[axis]*weight,candidate.config.min_pos.p[axis],candidate.config.max_pos.p[axis]);
                const auto actual = candidate.points[point_idx].p[axis];
                has_change |= expected != actual;
                max_error = std::max(max_error,std::abs(static_cast<double>(expected)-actual));
            }
            num_changed += has_change; ++point_idx; start=end;
        }
        if (point_idx != candidate.points.size()) {throw std::runtime_error("占有セル数の不一致");}
        std::cout << unit << " input_points=" << original.size() << " cells=" << point_idx
            << " changed_centroids=" << num_changed << " max_coordinate_error_m=" << max_error << '\n';
    }
}
