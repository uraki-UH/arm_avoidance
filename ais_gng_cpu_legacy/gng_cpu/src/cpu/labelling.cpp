#include "labelling.hpp"

Labelling::Labelling(){

}

Labelling::~Labelling(){

}

void Labelling::init(GNGConfig *_gng_config, LabelConfig *_label_config, CUGNG *_gng) {
    // 初期化処理が必要な場合はここに記述
    gng_config = _gng_config;
    label_config = _label_config;
    gng = _gng;
}

void Labelling::labelling_fuzzy() {
    // ラベリング処理
    time.update(label_config->lpf_time);
    float h_exp[3];
    int i, num1, num2, num, max_label;
    float angle;
    Vec3f vec;
    float max_exp;
    for (auto &node : gng->nodes) {
        if (node.id == NODE_NOID)
            continue;
        gng->normal_vector(node);
    }
    for (auto &node : gng->nodes) {
        if (node.id == NODE_NOID)
            continue;
        // 新規追加
        // if (node.age != 0)
        //     node.vel = time->lpf_a * node.vel + time->lpf_dt_T * node.pos.norm(node.pos_prev);
        // node.pos_prev = node.pos;

        /* ファジィ法線ベクトル・曲率の算出 */
        if (node.edge_num >= 2) {
            gng->rho(node);
            // node.d_normal = acosf(node.normal.dot(normal) / (node.normal.norm() * normal.norm()));  // 1時刻前からの誤差
            angle = acosf(node.normal[2]);
            /* ファジィ法線ベクトルの算出 */
            h_exp[0] = _fuzzy_safe_label(angle);
            h_exp[2] = label_config->fuzzy_unknown_label(node.rho);
            h_exp[1] = _fuzzy_wall_label(angle) - h_exp[2] * 0.4;
        } else {  // 法線ベクトルが算出されない場合（先端ノードの場合）はファジィ法線ベクトルの値は0
            node.normal.zero();
            node.rho = 0.0;
            for (i = 0; i < 3; ++i)
                h_exp[i] = 0.0;
        }

        for (i = 0; i < 3; ++i){
            node.fuzzy_exp[i] = time.lpf_a * node.fuzzy_exp[i] + time.lpf_dt_T * h_exp[i];//LPF
            // node.fuzzy_exp[i] = h_exp[i]; // LPFなし
        }

        /* 外部入力 */
        max_label = 0;
        max_exp = node.fuzzy_exp[0];
        for (i = 1; i < 3;++i){
            if (node.fuzzy_exp[i] > max_exp) {
                max_exp = node.fuzzy_exp[i];
                max_label = i;
            }
        }
        if(max_exp > (label_config->fuzz_min)){
            node.label = max_label + 1;
        }else{
            node.label = DEFAULT;  // ラベルが決まらない場合はUNKNOWN_OBJECT
        }
    }
}

float Labelling::_fuzzy_safe_label(float angle) {
    // return expf(-((angle - M_PI_2) / a) * ((angle - M_PI_2) / a) / 2.0);
    if (angle < M_PI_2) {
        if (angle < M_PI_4)
            return (M_PI_4 - angle) / M_PI_4;  // 床の所
        else
            return 0;  // 水平
    } else {
        if (angle < M_PI_4 * 3)
            return 0;  // 水平
        else
            return (angle - M_PI_4 * 3) / M_PI_4;  // 天井
    }
}
float Labelling::_fuzzy_wall_label(float angle) {
    // return expf(-((angle - M_PI_2) / a) * ((angle - M_PI_2) / a) / 2.0);
    if (angle < M_PI_2) {
        if (angle < M_PI_4)
            return 0;  // 床の所
        else
            return (angle - M_PI_4) / M_PI_4;  // 水平
    } else {
        if (angle < M_PI_4 * 3)
            return 1.f - (angle - M_PI_2) / M_PI_4;  // 水平
        else
            return 0;  // 天井
    }
}