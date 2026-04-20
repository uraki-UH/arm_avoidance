#pragma once

#include "../utils/logger.hpp"
#include "../utils/param.hpp"
#include "../utils/utils.hpp"
#include "../auth/auth.hpp"
#include "../auth/yk_piv.hpp"
#include "cugng.hpp"
#include "voxel_grid.hpp"
#include "clustering.hpp"
#include "labelling.hpp"

struct Affine {
    float offset[3]; // 並行移動
    float rot[9]; // 回転行列
    void init(const LiDAR_Config *c){
        offset[0] = c->pos.x;
        offset[1] = c->pos.y;
        offset[2] = c->pos.z;
        float x, y, z, w, x2, y2, z2, w2;
        x = c->quat.x;
        y = c->quat.y;
        z = c->quat.z;
        w = c->quat.w;
        x2 = x * x;
        y2 = y * y;
        z2 = z * z;
        w2 = w * w;
        rot[0] = 2 * w2 + 2 * x2 - 1;
        rot[1] = 2 * x * y - 2 * z * w;
        rot[2] = 2 * x * z + 2 * y * w;
        rot[3] = 2 * x * y + 2 * z * w;
        rot[4] = 2 * w2 + 2 * y2 - 1;
        rot[5] = 2 * y * z - 2 * x * w;
        rot[6] = 2 * x * z - 2 * y * w;
        rot[7] = 2 * y * z + 2 * x * w;
        rot[8] = 2 * w2 + 2 * z2 - 1;
    }
    void initDiff(const LiDAR_Config *p1, const LiDAR_Config *p2){
        LiDAR_Config diff;
        diff.pos.x = p2->pos.x - p1->pos.x;
        diff.pos.y = p2->pos.y - p1->pos.y;
        diff.pos.z = p2->pos.z - p1->pos.z;
        float w1 = p2->quat.w;
        float x1 = p2->quat.x;
        float y1 = p2->quat.y;
        float z1 = p2->quat.z;
        // -q2 = (w1, -x1, -y1, -z1)
        float w2 =  p1->quat.w;
        float x2 = -p1->quat.x;
        float y2 = -p1->quat.y;
        float z2 = -p1->quat.z;
        // クォータニオンの差分
        // q2 - q1 = 
        // q1-1 = (w1, -x1, -y1, -z1)
        // dq = q2 * q1-1
        // https://zenn.dev/mebiusbox/books/132b654aa02124/viewer/2966c7
        // q1 * q2
        diff.quat.w = (w1 * w2) - (x1 * x2) - (y1 * y2) - (z1 * z2);
        diff.quat.x = (w1 * x2) + (w2 * x1) + (y1 * z2) - (z1 * y2);
        diff.quat.y = (w1 * y2) + (w2 * y1) + (z1 * x2) - (x1 * z2);
        diff.quat.z = (w1 * z2) + (w2 * z1) + (x1 * y2) - (y1 * x2);
        init(&diff);
    }
    void transform(Vec3f &in, Vec3f &out){
        out[0] = in[0] * rot[0] + in[1] * rot[1] + in[2] * rot[2] + offset[0];
        out[1] = in[0] * rot[3] + in[1] * rot[4] + in[2] * rot[5] + offset[1];
        out[2] = in[0] * rot[6] + in[1] * rot[7] + in[2] * rot[8] + offset[2];
    }
    void transform(float *in, Vec3f &out){
        out[0] = in[0] * rot[0] + in[1] * rot[1] + in[2] * rot[2] + offset[0];
        out[1] = in[0] * rot[3] + in[1] * rot[4] + in[2] * rot[5] + offset[1];
        out[2] = in[0] * rot[6] + in[1] * rot[7] + in[2] * rot[8] + offset[2];
    }
    void transform(float *in, float *out){
        out[0] = in[0] * rot[0] + in[1] * rot[1] + in[2] * rot[2] + offset[0];
        out[1] = in[0] * rot[3] + in[1] * rot[4] + in[2] * rot[5] + offset[1];
        out[2] = in[0] * rot[6] + in[1] * rot[7] + in[2] * rot[8] + offset[2];
    }
};

struct InPCL {
    uint32_t n = 0;  // 点群数
    float *inpcl;    // 点群データ
    Affine affine;   // 座標変換
};

class GNG{
    public:
    bool initialized = false;
    // output
    int input_pcl_num = 0;  // 入力点群数
    vector<Vec3f> attention_pcl;
    vector<uint8_t> voxel_labels;
    int attention_pcl_num = 0; // 注意点群数
    vector<Voxel> voxel2node_ids;
    uint32_t voxel2node_ids_num = 0; // ボクセルグリッドのノードID

    // 処理
    Auth auth; // 署名チェック
    YK_PIV yk_piv; // Yubikeyとの通信
    Param param;
    Logger log;
    VoxelGrid vg;
    CUGNG n1;
    Labelling la;
    Clustering cl;

    // output
    TopologicalMap map;
    uint16_t *clusted_node_ids = nullptr; // クラスタリングされたノードID
    uint8_t *labels = nullptr; // ラベル
    uint8_t *inpcl_labels = nullptr; // 入力点群のラベル
    uint32_t *inpcl_ids = nullptr; // 入力点群のID
    float *input_pcl = nullptr; // 入力点群データ

    GNG();
    ~GNG();
    // 入力
    int init(const char *binary_path);
    void setPointCloud(const uint8_t *inpcl, const uint32_t inpcl_num,
                       const LiDAR_Config *config);  // GPUメモリへの転送
    bool licenceAuthentication();

    // 出力
    TopologicalMap getTopologicalMap();

    void exec();
    void attention();
    void makeResult();
    void setInferredClusterLabels(const uint32_t *cluster_ids, const uint32_t *cluster_ages, const uint8_t *cluster_labels, const uint32_t size);
    void check_error();
    uint8_t* getDownSampling(uint32_t *label_num);

    const uint32_t ykey3[4] = {_YK_KEY3_1, _YK_KEY3_2, _YK_KEY3_3, _YK_KEY3_4};
    const uint32_t fkey3[4] = {_FILE_KEY3_1, _FILE_KEY3_2, _FILE_KEY3_3, _FILE_KEY3_4};
};