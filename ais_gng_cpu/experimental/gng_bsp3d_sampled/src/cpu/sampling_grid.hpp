#pragma once

#include "cugng.hpp"
#include <unordered_map>

// 元点を捨てない入力索引。ノードtreeとは独立した観測・重点候補の整理。
class sampling_grid {
 public:
    void prepare(CUGNG &graph, const VoxelGrid &voxels, const vector<Vec3f> &points,
        vector<uint8_t> &labels, float min_cell_size, uint32_t max_probe_num,
        vector<uint32_t> &attention_raw_ids);
    bool has_point_near(const Vec3f &position, float max_dist2) const;
    size_t num_cells() const {return cells.size();}

 private:
    struct cell_key {
        int64_t x, y, z;
        bool operator==(const cell_key &) const = default;
    };
    struct cell_hash {
        size_t operator()(const cell_key &key) const;
    };
    struct cell {
        Vec3f min_pos, max_pos;
        uint32_t begin = 0, end = 0, num = 0, cursor = 0;
        uint8_t label = 0;
    };
    std::unordered_map<cell_key, uint32_t, cell_hash> lookup;
    vector<cell> cells;
    vector<uint32_t> point_cells, point_ids;
    const vector<Vec3f> *input_points = nullptr;
    double cell_size = 0.5;

    cell_key key_at(const Vec3f &point, double offset = 0) const;
    static float box_dist2(const Vec3f &point, const cell &entry);
    void build(const VoxelGrid &voxels, const vector<Vec3f> &points, double size);
    void mark_attention(const Vec3f &position, float max_dist2, uint8_t label);
};
