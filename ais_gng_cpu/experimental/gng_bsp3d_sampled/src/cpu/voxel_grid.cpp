#include "voxel_grid.hpp"

#include <boost/sort/spreadsort/spreadsort.hpp>


uint32_t voxel_rightshift_func(const Voxel &x, const unsigned offset) {
    return x.voxel_index >> offset;
}

VoxelGrid::VoxelGrid(){

}
VoxelGrid::~VoxelGrid() {
}

void VoxelGrid::init(GridConfig *_grid_config, OtherConfig *_other_config) {
    voxel_config = _grid_config;
    enable_voxel_downsampling = _other_config->voxel_grid_unit > 0;
    voxel_index.resize(_other_config->point_cloud_num);
    voxel_range.resize(_other_config->point_cloud_num);
    filtered_pcl.resize(_other_config->point_cloud_num);
}

void VoxelGrid::applyFilter(vector<Vec3f> &input_pcl, uint32_t inpcl_num, vector<uint8_t> &labels){
    filtered_pcl_num = 0;
    voxel_index_num = 0;
    if(inpcl_num == 0){
        filtered_pcl_num = 0;
        return; // 入力点群がない場合は何もしない
    }
    if (!enable_voxel_downsampling) {
        // YAML範囲内の全点の保持。各点と元番号の一対一対応。
        std::fill(labels.begin(), labels.begin() + inpcl_num, 0);
        for (uint32_t idx = 0; idx < inpcl_num; ++idx) {
            if (!voxel_config->isRange(input_pcl[idx])) {continue;}
            const auto point_idx = filtered_pcl_num++;
            filtered_pcl[point_idx] = input_pcl[idx];
            voxel_index[point_idx] = Voxel(point_idx, idx);
            voxel_range[point_idx] = VoxelRange(point_idx, point_idx + 1);
            labels[idx] = 0b001;
        }
        voxel_index_num = filtered_pcl_num;
        return;
    }
    uint32_t index;
    uint32_t i, n;
    // リセット
    std::fill(labels.begin(), labels.begin() + inpcl_num, 0);
    for (i = n = 0; i < inpcl_num; ++i) {
        index = voxel_config->getIndex(input_pcl[i].p);
        if(index >= voxel_config->maxXYZ){
            continue; // 範囲外は無視
        }
        voxel_index[n].voxel_index = index;
        voxel_index[n++].raw_index = i;
        labels[i] = 0b001; //範囲内点群
    }
    voxel_index_num = n;
    if (voxel_index_num == 0) {return;}

    // ボクセルグリッドのソート
    boost::sort::spreadsort::integer_sort(voxel_index.data(), voxel_index.data() + voxel_index_num,
        [](const Voxel &voxel, unsigned offset) { return voxel.voxel_index >> offset; });

    uint32_t now_index = voxel_index[0].voxel_index;
    voxel_range[0].start = 0;
    for (i = n = 0; i < voxel_index_num; ++i){
        if (voxel_index[i].voxel_index != now_index) {
            now_index = voxel_index[i].voxel_index;
            voxel_range[n].end = i;
            voxel_range[++n].start = i;
        }
    }
    voxel_range[n].end = voxel_index_num;
    filtered_pcl_num = n + 1;

    // ボクセルグリッドのフィルタリング
    for (i = 0; i < filtered_pcl_num; ++i){
        float x = 0, y = 0, z = 0;
        const auto *points = input_pcl.data();
        const auto *indices = voxel_index.data();
        for (n = voxel_range[i].start; n < voxel_range[i].end; ++n) {
            x += points[indices[n].raw_index].p[0];
            y += points[indices[n].raw_index].p[1];
            z += points[indices[n].raw_index].p[2];
        }
        uint32_t voxel_num = voxel_range[i].end - voxel_range[i].start;
        float num_1 = (1.f) / (float)voxel_num;
        filtered_pcl[i].p[0] = x * num_1;
        filtered_pcl[i].p[1] = y * num_1;
        filtered_pcl[i].p[2] = z * num_1;
    }
}
