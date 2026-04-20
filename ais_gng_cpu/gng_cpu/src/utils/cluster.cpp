#include "cluster.hpp"

#define LIMIT_I 0.01f

/*
 * クラスタリングクラス
 */
Cluster::Cluster(const Cluster& c) {
    /* data */
    id = c.id;        // クラスタID
    ros_id = c.ros_id; // ROSID
    label = c.label;  // クラスタのラベル名
    label_inferred = c.label_inferred;  // 推定ラベル
    // err = c.err;      // クラスタ内部の積算誤差
    age = c.age;  // クラスタ
    age_inferred = c.age_inferred;
    count_inferred = c.count_inferred; // 推定ラベルのカウント
    match = c.match;
    // sustain = c.sustain;
    velocity = c.velocity;
    // non_dynamic_cnt = c.non_dynamic_cnt;

    center_pos = c.center_pos;  // バウンディングボックスの位置
    scale = c.scale;            // バウンディングボックスのサイズ
    // scale_2 = c.scale_2;

    nodes_ids.resize(c.nodes_ids.size());  // クラスタに属しているノードID一覧
    copy(c.nodes_ids.begin(), c.nodes_ids.end(), nodes_ids.begin());

    // nodes_pos.resize(c.nodes_pos.size());
    // copy(c.nodes_pos.begin(), c.nodes_pos.end(), nodes_pos.begin());

    size = c.size;

    quat.x = c.quat.x;
    quat.y = c.quat.y;
    quat.z = c.quat.z;
    quat.w = c.quat.w;

    // wall_n = c.wall_n;
    // wall_a = c.wall_a;
    // wall_d = c.wall_d;
    // wall_max = c.wall_max;
    // isWall = c.isWall;

    // integrated = c.integrated;
}
Cluster::Cluster(vector<int>& ids, vector<Node> &nodes, int _label) {
    int i;
    id = CLUSTER_DEFAULT_ID;
    ros_id = CLUSTER_DEFAULT_ROSID;
    // non_dynamic_cnt = 0;
    age = 1;
    age_inferred = 1;
    size = ids.size();
    nodes_ids.resize(size);
    velocity.zero();
    // velocity = 0;
    copy(ids.begin(), ids.end(), nodes_ids.begin());

    // 向き
    float Sx = 0;
    float Sxx = 0;
    float Sy = 0;
    float Syy = 0;
    float Sxy = 0;
    float x, y, z;
    for (auto& node_id : ids) {
        x = nodes[node_id].pos[0];
        y = nodes[node_id].pos[1];
        z = nodes[node_id].pos[2];
        // 方向
        Sx += x;
        Sxx += x * x;
        Sy += y;
        Syy += y * y;
        Sxy += x * y;
    }
    float N = size;

    // 最小距離２乗法
    float I = (Sx * Sy - N * Sxy);
    float I2 = I * I;
    if(I2 < LIMIT_I * LIMIT_I){
        I2 = LIMIT_I * LIMIT_I;
        I = (I >= 0) ? LIMIT_I : -LIMIT_I;
    }
    float K = N * (Syy - Sxx) + Sx * Sx - Sy * Sy;
    float a = (-K - sqrtf(K * K + 4.f * (I2))) / (2.f * I);
    float offset_x = 0;
    float offset_y = 0;
    float b = 0;
    bool vertical = fabs(a) > 1.f;
    float theta = 0;
    if (vertical) {
        I = (Sy * Sx - N * Sxy);
        I2 = I * I;
        if(I2 < LIMIT_I * LIMIT_I){
            I2 = LIMIT_I * LIMIT_I;
            I = (I >=0) ? LIMIT_I : -LIMIT_I;
        }
        K = N * (Sxx - Syy) + Sy * Sy - Sx * Sx;
        a = (-K - sqrtf(K * K + 4.f * (I2))) / (2.f * I);
        b = (Sx - a * Sy) / N;
        offset_x = b;
        offset_y = 0;
        theta = M_PI_2 - atanf(a);
    } else {
        b = (Sy - a * Sx) / N;
        offset_x = 0;
        offset_y = b;
        theta = atanf(a);
    }
    float c = cosf(theta);
    float s = sinf(theta);
    float c_2 = cosf(theta / 2.f);
    float s_2 = sinf(theta / 2.f);
    // 最小２乗法
    // float D = N * Sxx - Sx * Sx;
    // float a = (N * Sxy - Sx * Sy) / D;
    quat.x = 0;
    quat.y = 0;
    quat.z = s_2;
    quat.w = c_2;

    float r[3];
    float max_r[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
    float min_r[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
    for (auto& node_id : ids) {
        x = nodes[node_id].pos[0] - offset_x;
        y = nodes[node_id].pos[1] - offset_y;
        z = nodes[node_id].pos[2];

        r[0] = x * c + y * s;
        r[1] = -x * s + y * c;
        r[2] = z;
        for (i = 0; i < 3; i++) {
            if (max_r[i] < r[i])
                max_r[i] = r[i];
            if (min_r[i] > r[i])
                min_r[i] = r[i];
        }
    }
    /* ボックスの方向とスケールを表すベクトルと中心座標の作成 */
    for (i = 0; i < 3; i++) {  // xyz方向最大のベクトル
        scale[i] = fabs(max_r[i] - min_r[i]);
        r[i] = (max_r[i] + min_r[i]) / 2.f;
        // scale_2[i] = scale[i] * .5f;
    }
    // 同じ位置にノードがある
    if(scale[0] == 0 || scale[1] == 0 || scale[2] == 0){
        size = 0;
        return;
    }
    if(label == HUMAN){
        center_pos[0] = Sx / N;
        center_pos[1] = Sy / N;
        center_pos[2] = r[2];
    }else{
        center_pos[0] = offset_x + r[0] * c - r[1] * s;
        center_pos[1] = offset_y + r[0] * s + r[1] * c;
        center_pos[2] = r[2];
    }

    static array<int, LABEL_NUM> label_nums;
    // ラベル
    if(_label != DEFAULT){
        label = _label;
    }else{
        label_nums.fill(0);
        for(auto &node_id: ids){
            label_nums[nodes[node_id].label]++;
        }
        int max_num = -1;
        int max_label;
        for(int i =0;i<LABEL_NUM;++i){
            if(label_nums[i] > max_num){
                max_label = i;
                max_num = label_nums[i];
            }
        }
        label = max_label;
    }
    for (auto& node_id : ids) {
        if (nodes[node_id].normal.dot(center_pos) > 0) {
            nodes[node_id].normal *= -1;
        }
    }
}

Cluster::~Cluster() {

}

void Cluster::cylinder() {
    float max_scale = MAX(scale[0], scale[1]);
    scale[0] = max_scale;
    scale[1] = max_scale;
}

float Cluster::getArea(){
    if(scale[0] > scale[1]){
        if(scale[1] > scale[2]){
            return scale[0] * scale[1];//min: scale[2]
        }else{
            return scale[0] * scale[2];// min: scale1
        }
    }else {
        if(scale[0] > scale[2]){
            return scale[1] * scale[0];//min scale[2]
        }else{
            return scale[1] * scale[2];//min scale[0]
        }
    }
}

// bool Cluster::wallContain(Vec3f& pos, ParamCluster& param) {
//     if (!isWall)
//         return false;
//     float d2 = (wall_n.dot(pos) + wall_d);
//     // 壁からの距離が一定以上
//     if (d2 * d2 > param.wall_tickness2)
//         return false;
//     // 中心からの点へのベクトル
//     Vec3f center2pos = pos - center_pos;
//     // 壁への射影が含まれているか
//     if (wall_n[2] == 1) {
//         // 水平な壁
//         // x座標
//         if (fabs(wall_a.dot(center2pos)) > wall_max)
//             return false;
//     } else {
//         // 垂直な壁
//         // Z座標
//         if (fabs(center2pos[2]) > scale_2[2])
//             return false;
//         // x座標
//         if (fabs(wall_a.dot(center2pos)) > wall_max)
//             return false;
//     }
//     return true;
// }