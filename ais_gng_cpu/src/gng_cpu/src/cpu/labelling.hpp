#pragma once

#include "../utils/utils.hpp"
#include "cugng.hpp"

class Labelling {
public:
    Labelling();
    ~Labelling();

    void init(NodeConfig *_gng_config, LabelConfig *_label_config, CUGNG *_gng);

    void labelling_fuzzy();

    NodeConfig *gng_config;
    LabelConfig *label_config;
    CUGNG *gng;
    Time time;
    // 近傍参照用の連続配列。初期化時の確保とフレーム間再利用。
    vector<Vec3f> node_positions;
    vector<Vec3f> node_normals;

    float _fuzzy_safe_label(float angle);
    float _fuzzy_wall_label(float angle);

    const uint32_t ykey4[4] = {_YK_KEY4_1, _YK_KEY4_2, _YK_KEY4_3, _YK_KEY4_4};
    const uint32_t fkey4[4] = {_FILE_KEY4_1, _FILE_KEY4_2, _FILE_KEY4_3, _FILE_KEY4_4};
};
