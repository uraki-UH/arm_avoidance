#pragma once
#include "cell_grid.hpp"
#include <boost/sort/spreadsort/spreadsort.hpp>

// 元CPU版と同じセル番号の整数ソート・連続区間・重心の入力整理。
struct input_voxels {
    struct entry {
        uint32_t cell_idx, raw_idx;
        bool operator<(const entry &other) const {return cell_idx < other.cell_idx;}
    };
    struct point_range {uint32_t start, end;};
    cell_grid config;
    vector<entry> entries;
    vector<point_range> ranges;
    vector<Vec3f> points;
    uint32_t num_input_points = 0;
    bool init(const OtherConfig &params) {
        entries.reserve(params.point_cloud_num);
        ranges.reserve(params.point_cloud_num);
        points.reserve(params.point_cloud_num);
        return config.init(Vec3f(params.x_min, params.y_min, params.z_min),
            Vec3f(params.x_max, params.y_max, params.z_max), params.voxel_grid_unit);
    }
    void prepare(const vector<Vec3f> &raw_points, uint32_t num_points, vector<uint8_t> &labels) {
        entries.clear(); ranges.clear(); points.clear();
        std::fill(labels.begin(), labels.begin() + num_points, 0);
        for (uint32_t idx = 0; idx < num_points; ++idx) {
            const auto cell_idx = config.get_cell_idx(raw_points[idx]);
            if (cell_idx == UINT32_MAX) {continue;}
            entries.push_back({cell_idx, idx});
            labels[idx] = 1;
        }
        num_input_points = entries.size();
        if (entries.empty()) {return;}
        boost::sort::spreadsort::integer_sort(entries.begin(), entries.end(),
            [](const entry &value, unsigned offset) {return value.cell_idx >> offset;});
        uint32_t start = 0;
        for (uint32_t idx = 1; idx < entries.size(); ++idx) {
            if (entries[idx].cell_idx == entries[start].cell_idx) {continue;}
            ranges.push_back({start, idx}); start = idx;
        }
        ranges.push_back({start, static_cast<uint32_t>(entries.size())});
        for (const auto &range : ranges) {
            Vec3f sum(0, 0, 0);
            for (uint32_t idx = range.start; idx < range.end; ++idx) {
                const auto &point = raw_points[entries[idx].raw_idx];
                for (int axis = 0; axis < 3; ++axis) {sum.p[axis] += point.p[axis];}
            }
            const float weight = 1.0f / (range.end - range.start);
            for (int axis = 0; axis < 3; ++axis) {
                sum.p[axis] = std::clamp(sum.p[axis] * weight, config.min_pos.p[axis], config.max_pos.p[axis]);
            }
            points.push_back(sum);
        }
    }
};
