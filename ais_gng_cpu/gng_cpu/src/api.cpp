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
    return gng.param.setParameter(paramerter_name, index, value);
}

MY_API void gng_setPointCloud(const uint8_t *inpcl, const uint32_t input_pcl_num, const LiDAR_Config *config){
    gng.setPointCloud(inpcl, input_pcl_num, config);
}

MY_API void gng_exec() { gng.exec(); }

MY_API TopologicalMap gng_getTopologicalMap(){
    return gng.getTopologicalMap();
}

MY_API void gng_setInferredClusterLabels(const uint32_t *cluster_ids, const uint32_t *cluster_ages, const uint8_t *cluster_labels, const uint32_t size) {
    gng.setInferredClusterLabels(cluster_ids, cluster_ages, cluster_labels, size);
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
    *num = gng.input_pcl_num;
    return gng.input_pcl;
}
}// extern "C"