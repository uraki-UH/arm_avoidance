#include "voxel_grid.hpp"
#include "radix_sort.hpp"

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
#ifdef GNG_RADIX_VOXELS
    sort_buffer.resize(_other_config->point_cloud_num);
#endif
    voxel_range.resize(_other_config->point_cloud_num);
    filtered_pcl.resize(_other_config->point_cloud_num);
}

void VoxelGrid::applyFilter(vector<Vec3f> &input_pcl, uint32_t inpcl_num, vector<uint8_t> &labels){
    filtered_pcl_num = 0;
    voxel_index_num = 0;
    sort_ms = 0;
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

    // ボクセルグリッドのソート。計測対象は整列のみ。
    const auto sort_begin = std::chrono::steady_clock::now();
#ifdef GNG_RADIX_VOXELS
    radix_sort_voxels(voxel_index.data(), sort_buffer.data(), voxel_index_num);
#else
    boost::sort::spreadsort::integer_sort(voxel_index.data(), voxel_index.data() + voxel_index_num,
        [](const Voxel &voxel, unsigned offset) { return voxel.voxel_index >> offset; });
#endif
    sort_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - sort_begin).count();

    // 同じソート順・加算順での、セル範囲の確定と重心計算の単一走査。
    uint32_t begin_idx = 0;
    while (begin_idx < voxel_index_num) {
        const uint32_t cell_idx = voxel_index[begin_idx].voxel_index;
        uint32_t end_idx = begin_idx;
        float x = 0, y = 0, z = 0;
        do {
            const auto &point = input_pcl[voxel_index[end_idx].raw_index];
            x += point.p[0];
            y += point.p[1];
            z += point.p[2];
            ++end_idx;
        } while (end_idx < voxel_index_num && voxel_index[end_idx].voxel_index == cell_idx);
        const uint32_t voxel_num = end_idx - begin_idx;
        const float num_1 = 1.f / static_cast<float>(voxel_num);
        const uint32_t output_idx = filtered_pcl_num++;
        voxel_range[output_idx].start = begin_idx;
        voxel_range[output_idx].end = end_idx;
        filtered_pcl[output_idx].p[0] = x * num_1;
        filtered_pcl[output_idx].p[1] = y * num_1;
        filtered_pcl[output_idx].p[2] = z * num_1;
        begin_idx = end_idx;
    }
}
