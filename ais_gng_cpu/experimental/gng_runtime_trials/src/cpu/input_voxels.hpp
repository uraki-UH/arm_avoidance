#pragma once
#include "cell_grid.hpp"
#include <array>
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
#ifdef GNG_RADIX_VOXELS
    vector<entry> sort_buffer;
#endif
    vector<point_range> ranges;
    vector<Vec3f> points;
    uint32_t num_input_points = 0;
    bool init(const OtherConfig &params) {
        entries.reserve(params.point_cloud_num);
#ifdef GNG_RADIX_VOXELS
        sort_buffer.reserve(params.point_cloud_num);
#endif
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
#ifdef GNG_RADIX_VOXELS
        // 32bitセル番号の8bit単位の安定基数ソート。点の省略なし。
        sort_buffer.resize(entries.size());
        for (unsigned shift = 0; shift < 32; shift += 8) {
            std::array<uint32_t, 256> counts{};
            for (const auto &value : entries) {++counts[(value.cell_idx >> shift) & 255];}
            uint32_t offset = 0;
            for (auto &count : counts) {const auto num = count; count = offset; offset += num;}
            for (const auto &value : entries) {
                sort_buffer[counts[(value.cell_idx >> shift) & 255]++] = value;
            }
            entries.swap(sort_buffer);
        }
#else
        boost::sort::spreadsort::integer_sort(entries.begin(), entries.end(),
            [](const entry &value, unsigned offset) {return value.cell_idx >> offset;});
#endif
#ifdef GNG_FUSE_VOXEL_REDUCTION
        // ソート済み順序を維持した区間検出と重心計算の単一走査。
        uint32_t start = 0;
        while (start < entries.size()) {
            const auto cell_idx = entries[start].cell_idx;
            uint32_t end = start;
            float sum[3]{};
            do {
                const auto &point = raw_points[entries[end].raw_idx];
                for (int axis = 0; axis < 3; ++axis) {sum[axis] += point.p[axis];}
                ++end;
            } while (end < entries.size() && entries[end].cell_idx == cell_idx);
            const float weight = 1.0f / (end - start);
            for (int axis = 0; axis < 3; ++axis) {
                sum[axis] = std::clamp(sum[axis] * weight, config.min_pos.p[axis], config.max_pos.p[axis]);
            }
            points.emplace_back(sum[0], sum[1], sum[2]);
            start = end;
        }
#else
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
#endif
    }
};
