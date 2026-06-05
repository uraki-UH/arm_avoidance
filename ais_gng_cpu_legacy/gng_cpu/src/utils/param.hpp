#pragma once

#include "utils.hpp"
#include "vec3f.hpp"

struct GridConfig {
    float unit;
    float x_min, x_max;
    float y_min, y_max;
    float z_min, z_max;

    float unit_1;
    uint32_t max[3], maxXY, maxXYZ;
    bool init(GridConfig _c);
    bool isRange(float *pos);
    bool isRange(Vec3f &p);
    uint32_t getIndex(int i, int j, int k);
    uint32_t getIndex(Vec3f &p);
    uint32_t getIndex(float *p);
};

struct GNGConfig{
    int num_max; // ノード数の上限
    int learning_num; // 学習回数
    int unknown_learning_rate;
    /* 学習係数 */
    float eta_s1;            // s1更新時の学習係数
    float eta_s1_2;          // s1更新時の学習係数
    float eta_s2;            // s1更新時の学習係数
    float eta_s2_2;          // s1更新時の学習係数
    uint32_t edge_age_max;        // Edgeの年齢による切断の閾値
    float vigilance2[4]; //警戒領域
    int s1_age[4];
    int clusted_s1_age[4];
    float edge_distance_min2[4]; // エッジの距離の最小値

    /*ノードの情報*/
    // float dynamic_vel_min;   // 最小移動物体速度
    // float dynamic_lpf_time;  // 速度LPF 時定数

    float s1_reset_range2; // ノードの選択回数リセット範囲
    float ds_range_max2; // ダウンサンプリングの範囲

    int static_age_min; // 長期記憶の年齢
};

struct OtherConfig{
    bool local_coordinates; // ローカル座標系を使用するか
    uint32_t point_cloud_num; // 入力点群数
    float node_grid;  // ノードのグリッドサイズ
    float voxel_grid_unit;
    float x_min, x_max; // 範囲
    float y_min, y_max; // 範囲
    float z_min, z_max; // 範囲
};

struct ClusterConfig{
    size_t num_min;  // クラスタリングの個数の閾値
    float lpf_time; // ラベリングのLPF時定数
    int constant_age_ave_min;
    float plane_volume;
    float unknown_edge_distance_max2;
    float other_edge_distance_max2;
    // float human_dbscan_epsilon;
    // int human_dbscan_points_num_min;
    // bool classify_car;  // 車両の識別
    // float wall_normal_dot_max;
    int human_hysteresis_age; // ヒトのヒステリシス
    int human_confirmation_age; // ヒトの確認年齢
    int car_hysteresis_age; // 車両のヒステリシス
    int car_confirmation_age; // 車両の確認年齢
    float human_radius;
    // bool human_division;
};

struct LabelConfig{
    float fuzz_unknown_1; // 未知物体のメンバシップ関数
    float fuzz_min; // 未知物体のメンバシップ関数
    float lpf_time; // ラベリングのLPF時定数
    float fuzzy_unknown_label(float rho) {
        return MIN(1.f, rho * fuzz_unknown_1);
    }
};

struct DownSampleConfig{
    float clusted_node_range_max2;
};

class Param {
   public:
    ClusterConfig cluster;
    OtherConfig config;
    GNGConfig node;
    DownSampleConfig ds;
    LabelConfig label;
    // std::vector<uint16_t> s1_ids;

    Param();
    ~Param();
    void init();
    bool setParameter(const char *p, const uint32_t index, const float value);
};