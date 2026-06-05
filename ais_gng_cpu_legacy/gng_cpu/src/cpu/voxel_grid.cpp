#include "voxel_grid.hpp"

#include <boost/sort/spreadsort/spreadsort.hpp>


uint32_t voxel_rightshift_func(const Voxel &x, const unsigned offset) {
    return x.voxel_index >> offset;
}

VoxelGrid::VoxelGrid(){

}
VoxelGrid::~VoxelGrid() { 
    free(sorted_inpcl);
}

void VoxelGrid::init(GridConfig *_grid_config, OtherConfig *_other_config) { 
    voxel_config = _grid_config;
    voxel_index.resize(_other_config->point_cloud_num);
    voxel_range.resize(_other_config->point_cloud_num);
    filtered_pcl.resize(_other_config->point_cloud_num);
    sorted_inpcl = (float*)malloc(sizeof(float) * 3 * _other_config->point_cloud_num);
}

void VoxelGrid::applyFilter(float *input_pcl, uint32_t inpcl_num, uint8_t *labels){
    if(inpcl_num == 0){
        filtered_pcl_num = 0;
        return; // 入力点群がない場合は何もしない
    }
    uint32_t index;
    uint32_t i, n;
    // リセット
    memset(labels, 0, sizeof(uint8_t) * inpcl_num);
    for (i = n = 0; i < inpcl_num; ++i) {
        index = voxel_config->getIndex(&input_pcl[i*3]);
        if(index >= voxel_config->maxXYZ){
            continue; // 範囲外は無視
        }
        voxel_index[n].voxel_index = index;
        voxel_index[n++].raw_index = i;
        labels[i] = 0b001; //範囲内点群
    }
    voxel_index_num = n;

    // ボクセルグリッドのソート
    boost::sort::spreadsort::integer_sort(voxel_index.begin(), voxel_index.begin() + voxel_index_num, voxel_rightshift_func);

    uint32_t now_index = voxel_index[0].voxel_index;
    voxel_range[0].start = 0;
    for (i = n = 0; i < voxel_index_num; ++i){
        if (voxel_index[i].voxel_index != now_index) {
            now_index = voxel_index[i].voxel_index;
            voxel_range[n].end = i;
            voxel_range[++n].start = i;
        }
        memcpy(&sorted_inpcl[i * 3], &input_pcl[voxel_index[i].raw_index*3], sizeof(float) * 3);
    }
    voxel_range[n].end = voxel_index_num;
    filtered_pcl_num = n;

    // ボクセルグリッドのフィルタリング
    for (i = 0; i < filtered_pcl_num; ++i){
        filtered_pcl[i].zero();
        for (n = voxel_range[i].start; n < voxel_range[i].end; ++n) {
            filtered_pcl[i][0] += sorted_inpcl[n * 3 + 0];
            filtered_pcl[i][1] += sorted_inpcl[n * 3 + 1];
            filtered_pcl[i][2] += sorted_inpcl[n * 3 + 2];
        }
        uint32_t voxel_num = voxel_range[i].end - voxel_range[i].start;
        float num_1 = (1.f) / (float)voxel_num;
        filtered_pcl[i] *= num_1; // 平均化
    }
}