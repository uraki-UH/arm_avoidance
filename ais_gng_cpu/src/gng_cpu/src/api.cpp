#include "cpu/gng.hpp"

#define _GNU_SOURCE // dladdrのために必要
#include <dlfcn.h>      // dladdr

#if defined(_WIN32)
  // Windows用の設定
  #ifdef BUILDING_DLL
    #define MY_API __declspec(dllexport)
  #else
    #define MY_API __declspec(dllimport)
  #endif
#else
  // GCC/Clang用の設定
  #define MY_API __attribute__((visibility("default")))
#endif

extern "C" {

// ライブラリがロードされたときに自動実行されるコンストラクタ関数
void library_init(void) __attribute__((constructor));

void library_init(void) {
}

GNG gng;

MY_API int gng_init() {
    // ライブラリのパスを取得
    Dl_info info;
    if (!dladdr((void*)library_init, &info))
        return ERROR_CHECK_FILE;
    // /home/ubuntu/ros2_ws/install/libgng_cpu/lib/liblibgng_cpu.so
    // gng.log.println("Library path: %s", info.dli_fname);
    return gng.init(info.dli_fname);
}

MY_API int gng_setParameter(const char *paramerter_name, const uint32_t index, const float value){
    const std::string name(paramerter_name);
    auto &core = gng.n1;
    if (!std::isfinite(value)) {return false;}
    if (name == "node.covariance_enabled") {
        core.enable_covariance = value != 0;
        if (!core.enable_covariance) {for (auto &node : core.nodes) {node.winner_stats = {};}}
    } else if (name == "node.enable_support") {
        core.enable_support = value != 0;
        if (!core.enable_support) {for (auto &node : core.nodes) {node.support_stats = {};}}
    } else if (name == "node.covariance_winner_rank_max" && (value == 1 || value == 2)) {
        core.max_covariance_winner_rank = static_cast<uint16_t>(value);
    } else if (name == "node.support.sample_alpha" && value >= 0 && value <= 1) {
        core.support_sample_alpha = value;
    } else if (name == "node.support.second_weight" && value >= 0 && value <= 1) {
        core.support_second_weight = value;
    } else {return gng.param.setParameter(paramerter_name, index, value);}
    core.support_second_alpha = 1 - std::pow(1 - core.support_sample_alpha, core.support_second_weight);
    return true;
}

MY_API void gng_setPointCloud(const uint8_t *inpcl, const uint32_t input_pcl_num, const LiDAR_Config *config){
    gng.setPointCloud(inpcl, input_pcl_num, config);
}

MY_API void gng_exec() { gng.exec(); }

MY_API void gng_setMapDeltaCapture(uint8_t enable) {
    gng.n1.setMapDeltaCapture(enable != 0);
}

MY_API const GngMapDelta* gng_getTopologicalMapDelta() {
    return gng.n1.getMapDelta();
}

MY_API void gng_setTrainingEventCapture(uint8_t enable) {
    gng.n1.setTrainingEventCapture(enable != 0);
}

MY_API void gng_setTrainingEventMaxWinnerRank(uint16_t max_winner_rank) {
    gng.n1.setTrainingEventMaxWinnerRank(max_winner_rank);
}

MY_API const GngTrainingEvent* gng_getTrainingEvents(uint32_t *num) {
    return gng.n1.getTrainingEvents(num);
}

MY_API gng_node_statistics gng_get_node_statistics(uint16_t node_id) {
    gng_node_statistics result;
    if (node_id >= gng.n1.nodes.size() || gng.n1.nodes[node_id].id == NODE_NOID) {return result;}
    const auto &node = gng.n1.nodes[node_id];
    result.winner_point_count = node.winner_stats.count;
    result.support_weight_sum = node.support_stats.count;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            const auto idx = 3 * row + column;
            result.winner_point_covariance[idx] = node.winner_stats.covariance[idx] / std::max(1.0, node.winner_stats.count - 1);
            if (result.support_weight_sum > 0) {
                result.support_moment[idx] = node.support_stats.covariance[idx] +
                    (node.support_stats.mean[row] - node.pos.p[row]) * (node.support_stats.mean[column] - node.pos.p[column]);
            }
        }
    }
    return result;
}

MY_API TopologicalMap gng_getTopologicalMap(){
    return gng.getTopologicalMap();
}

MY_API uint8_t* gng_getDownSampling(uint32_t *label_num){
    return gng.getDownSampling(label_num);
}

// MY_API char* gng_getLog(uint32_t *len){
//     return gng.log.get(len);
// }

MY_API float* gng_getAffineTransformedInputPointCloud(uint32_t *num) {
    // Voxel Mass
    // int i, n;
    // for (i = 0; i < vg.filtered_pcl_num;++i) {
    //     gng.input_pcl[i * 3 + 0] = vg.filtered_pcl[i][0];
    //     gng.input_pcl[i * 3 + 1] = vg.filtered_pcl[i][1];
    //     gng.input_pcl[i * 3 + 2] = vg.filtered_pcl[i][2];
    // }
    // *num = gng.vg.filtered_pcl_num;

    // atention pcl
    // for (i = 0; i < gng.attention_pcl_num;++i) {
    //     gng.input_pcl[i * 3 + 0] = gng.attention_pcl[i][0];
    //     gng.input_pcl[i * 3 + 1] = gng.attention_pcl[i][1];
    //     gng.input_pcl[i * 3 + 2] = gng.attention_pcl[i][2];
    // }
    // *num = gng.attention_pcl_num;

    // attention voxel
    // for (i = n = 0; i < gng.vg.filtered_pcl_num;++i) {
    //     if(gng.labels[i] == 0){
    //         continue;
    //     }
    //     gng.input_pcl[n * 3 + 0] = gng.vg.filtered_pcl[i][0];
    //     gng.input_pcl[n * 3 + 1] = gng.vg.filtered_pcl[i][1];
    //     gng.input_pcl[n * 3 + 2] = gng.vg.filtered_pcl[i][2];
    //     n++;
    // }
    // *num = n;
    *num = gng.map.input_pcl_num;
    if (gng.map.input_pcl.empty()) {
        return nullptr;
    }
    return &gng.map.input_pcl[0].p[0];
}

MY_API void gng_setInferredClusterLabels(const uint32_t *cluster_ids, const uint32_t *cluster_ages, const uint8_t *cluster_labels, const uint32_t size) {
    gng.setInferredClusterLabels(cluster_ids, cluster_ages, cluster_labels, size);
}

MY_API void gng_setInferredNodeLabels(const NodeSemSeg* labels, const uint32_t num) {
   // ToDo
}

MY_API void gng_setNodePositions(const Vec3 *node_positions, const uint32_t num){
    // ToDo
}

}// extern "C"
