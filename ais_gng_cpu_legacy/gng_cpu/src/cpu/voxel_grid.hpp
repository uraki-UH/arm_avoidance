#pragma once
#include "../utils/param.hpp"

struct Voxel{
    uint32_t voxel_index;
    uint32_t raw_index;
    Voxel() = default;
    Voxel(uint32_t _voxel_index, uint32_t _raw_index) : voxel_index(_voxel_index), raw_index(_raw_index) {}
    bool operator<(const Voxel &v) const { return (voxel_index < v.voxel_index); }
};

struct VoxelRange{
    uint32_t start;
    uint32_t end;
    VoxelRange() = default;
    VoxelRange(uint32_t _start, uint32_t _end) : start(_start), end(_end) {}
};

uint32_t voxel_rightshift_func(const Voxel &x, const unsigned offset);

class VoxelGrid {
    public:
        VoxelGrid();
        ~VoxelGrid();
        GridConfig *voxel_config;
        vector<Voxel> voxel_index;
        vector<VoxelRange> voxel_range;
        vector<Vec3f> filtered_pcl;  // フィルタリング後の点群
        uint32_t filtered_pcl_num;
        uint32_t voxel_index_num;
        float *sorted_inpcl;

        void init(GridConfig *_grid_config, OtherConfig *_other_config);
        void applyFilter(float *input_pcl, uint32_t inpcl_num, uint8_t *labels);

        const uint32_t ykey5[4] = {_YK_KEY5_1, _YK_KEY5_2, _YK_KEY5_3, _YK_KEY5_4};
        const uint32_t fkey5[4] = {_FILE_KEY5_1, _FILE_KEY5_2, _FILE_KEY5_3, _FILE_KEY5_4};
};