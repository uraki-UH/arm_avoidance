#include "voxel_grid.hpp"

#include "radix_sort.hpp"


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
    sort_buffer.resize(_other_config->point_cloud_num);
    voxel_range.resize(_other_config->point_cloud_num);
    filtered_pcl.resize(_other_config->point_cloud_num);
}

void VoxelGrid::applyFilter(vector<Vec3f> &input_pcl, uint32_t inpcl_num, vector<uint8_t> &labels){
    if (tracking) {
        apply_filter<true>(input_pcl, inpcl_num, labels);
        tracking->evaluate();
        tracking = nullptr;
    } else {apply_filter<false>(input_pcl, inpcl_num, labels);}
}

template<bool enable_tracking>
void VoxelGrid::apply_filter(vector<Vec3f> &input_pcl, uint32_t inpcl_num, vector<uint8_t> &labels){
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
            if constexpr (enable_tracking) {tracking->fine_cells.push_back(tracking->add_point(input_pcl[idx].p));}
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

    // 全32bitセル番号による安定基数ソート。
    radix_sort_voxels(voxel_index.data(), sort_buffer.data(), voxel_index_num);

    // セル範囲と代表元点の確定。合成座標・重心計算なし。
    uint32_t begin_idx = 0;
    while (begin_idx < voxel_index_num) {
        const uint32_t cell_idx = voxel_index[begin_idx].voxel_index;
        uint32_t end_idx = begin_idx;
        uint32_t coarse_idx = 0;
        do {
            if constexpr (enable_tracking) {
                const auto &point = input_pcl[voxel_index[end_idx].raw_index];
                const auto idx = tracking->add_point(point.p);
                if (end_idx == begin_idx) {coarse_idx = idx;}
                else if (coarse_idx != idx) {coarse_idx = fuzzrobo::builtin_sampling::tracking_cells::mixed_cell;}
            }
            ++end_idx;
        } while (end_idx < voxel_index_num && voxel_index[end_idx].voxel_index == cell_idx);
        const uint32_t output_idx = filtered_pcl_num++;
        voxel_range[output_idx].start = begin_idx;
        voxel_range[output_idx].end = end_idx;
        filtered_pcl[output_idx] = input_pcl[voxel_index[begin_idx].raw_index];
        if constexpr (enable_tracking) {tracking->fine_cells.push_back(coarse_idx);}
        begin_idx = end_idx;
    }
}

bool VoxelGrid::has_occupied_cell(uint32_t cell_idx) const {
    // 既存の昇順セル範囲を直接参照。点群コピー・別占有グリッドなし。
    const auto end = voxel_range.begin() + filtered_pcl_num;
    const auto found = std::lower_bound(voxel_range.begin(), end, cell_idx,
        [&](const VoxelRange &range, uint32_t key) {return voxel_index[range.start].voxel_index < key;});
    return found != end && voxel_index[found->start].voxel_index == cell_idx;
}
