#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <cstddef>

// ラベル
typedef enum {
    DEFAULT = 0,
    SAFE_TERRAIN,
    WALL,
    UNKNOWN_OBJECT,
    HUMAN,
    CAR,
    LABEL_NUM
} GNG_LABEL_t;

// エラーコード
typedef enum {
    SUCCESS = 0,
    ERROR_CHECK_FILE,
    ERROR_CONNECT_YUBIKEY,
    ERROR_FAIL_AUTHENTICATION,
    ERROR_VOXEL_GRID_LEAF_SIZE,
} GNG_ERROR_CODE_t;

struct Vec3 {
    float x, y, z;
    operator float *() {
        return &x;
    }
};

struct Quaternion {
    float x=0, y=0, z=0, w=1;
};

struct TopologicalNode {
    uint16_t id=0;    // ID
    Vec3 pos={0,0,0};       // 位置
    Vec3 normal={0,0,0};    // 法線ベクトル
    float rho=1;      // 曲率
    uint8_t label=0;  // ラベル
    uint32_t frame=0; // ノード追加フレーム
    uint32_t *inpcl_ids=nullptr; // 入力点群ID
    uint32_t inpcl_num=0; // 入力点群の数
};

struct TopologicalCluster {
    uint32_t id=0;                  // ID
    uint8_t label=0;                // ラベル
    float label_reliability=0;        // 信頼度
    Vec3 pos={0,0,0};                     // 位置
    Vec3 scale={0,0,0};                   // サイズ
    Quaternion quat={0,0,0,1};              // 姿勢
    uint32_t frame=0;                 // 年齢
    float match=0;                  // 一致率
    Vec3 velocity={0,0,0};                // 速度
    uint16_t *nodes=nullptr;  // クラスタに属しているノードのID
    uint32_t node_num=0;  // クラスタに属しているノードの数
};

struct TopologicalMap {
    uint32_t frame_number=0;  // フレーム数
    uint32_t node_num=0;  // ノード数
    uint32_t cluster_num=0;  // クラスタ数
    uint32_t edge_num=0;  // エッジ数
    TopologicalNode *nodes=nullptr;        // ノード
    TopologicalCluster *clusters=nullptr;  // クラスタ
    uint16_t *edges=nullptr;  // エッジの隣接関係
    float *edges_distancesXY2=nullptr;  // エッジの距離（XY平面）
};

struct LiDAR_Config{
    Vec3 pos={0,0,0};
    Quaternion quat={0,0,0,1};
    uint32_t point_step=0;
};

struct NodeSemSeg{
    uint32_t node_id=0;
    uint32_t frame=0;
    uint32_t label=0;
};

/**
 * @brief 初期化
 */
int gng_init();

/**
 * @brief パラメーターを設定する
 * @param[in] paramerter_name 名前
 * @param[in] index インデックス
 * @param[in] value 値
 */
int gng_setParameter(const char *paramerter_name, const uint32_t index, const float value);

/**
 * @brief 点群を入力する
 * @param[in] inpcl 入力点群
 * @param[in] input_pcl_num 入力点群の数
 * @param[in] config LiDARの設定
 */
void gng_setPointCloud(const uint8_t *inpcl, const uint32_t input_pcl_num, const LiDAR_Config *config);

/**
 * @brief GNGを実行する
 */
void gng_exec();

/**
 * @brief GNGによるトポロジカルマップを返す
 * @return トポロジカルマップ
 */
TopologicalMap gng_getTopologicalMap();
/**
 * @brief ダウンサンプリングの結果を取得する
 * @return 点群のラベル(0: 入力範囲外, 1: 未知ノードが近くに無い，２: 人クラスタに属するノードが近くに無い)
 */
uint8_t* gng_getDownSampling(uint32_t *label_num);

/**
 * @brief アフィン変換された入力点群を取得する
 * @param[out] num 点群の数
 * @return 点群
 */
float* gng_getAffineTransformedInputPointCloud(uint32_t *num);

/**
 * @brief 推定されたクラスタラベルを設定する
 * @param[in] cluster_ids クラスタID
 * @param[in] cluster_ages クラスタ年齢
 * @param[in] cluster_labels クラスタラベル
 * @param[in] size クラスタの数
 */
void gng_setInferredClusterLabels(const uint32_t *cluster_ids, const uint32_t *cluster_ages, const uint8_t *cluster_labels, const uint32_t size);

/**
 * @brief 推定されたノードのラベルを設定する
 * @param[in] labels ノードのラベル配列
 * @param[in] num ノードの数
 */
void gng_setInferredNodeLabels(const NodeSemSeg* labels, const uint32_t num);

/**
 * @brief ノードの位置を設定する
 * @param[in] node_positions ノードの位置配列
 * @param[in] num ノードの数
 */
void gng_setNodePositions(const Vec3 *node_positions, const uint32_t num);

#ifdef __cplusplus
}
#endif