#pragma once

#include "utils.hpp"
#include "vec3f.hpp"

// 勝者残差と入力座標の逐次統計。ノード初期化時の履歴破棄。
struct node_moments {
    double count = 0;
    double mean[3]{};
    double covariance[9]{}; // 残差は偏差積和、入力座標は共分散

    void add_residual(const Vec3f &point) {
        ++count;
        double delta[3]{}, delta2[3]{};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            delta[axis] = point.p[axis] - mean[axis];
            mean[axis] += delta[axis] / count;
            delta2[axis] = point.p[axis] - mean[axis];
        }
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                covariance[3 * row + column] += delta[row] * delta2[column];
            }
        }
    }

    void add_input(const Vec3f &point, double alpha, double weight) {
        if (weight <= 0) {return;}
        if (count == 0) {for (std::size_t axis = 0; axis < 3; ++axis) {mean[axis] = point.p[axis];}}
        count += weight;
        if (alpha == 0) {alpha = weight / count;}
        double delta[3]{};
        for (std::size_t axis = 0; axis < 3; ++axis) {delta[axis] = point.p[axis] - mean[axis];}
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                auto &value = covariance[3 * row + column];
                value = (1 - alpha) * (value + alpha * delta[row] * delta[column]);
            }
            mean[row] += alpha * delta[row];
        }
    }
};

class Node{
    public:
        Node();
        ~Node();
        void init(uint32_t id, float eta_s1, float eta_s2);
        void init(uint32_t id, float eta_s1, float eta_s2, Vec3f &pos);
        node_moments winner_stats, support_stats;
        uint32_t id;
        Vec3f pos;
        // Vec3f pos_prev;
        // Vec3f s1_w;
        // int s1_w_num;
        uint32_t edges[NODE_MAX_EDGE];
        uint32_t edge_num;  // 接続しているエッジの数
        uint32_t grid_i;//所属しているgridのvectorの中のindex
        uint32_t grid_vec_i;

        // 時間
        uint32_t frame;
        uint32_t age_s1 = 0;
        float eta_s1;
        float eta_s2;

        // flag
        bool static_node;

        // ラベル
        int label;
        Vec3f normal;
        float rho;
        // float vel;
        float fuzzy_exp[3];

        /* クラスタリング */
        uint64_t clusted_id;  // 以前のクラスタID
        int clusted_label;  // 以前のクラスタラベル
        bool clustered_flag;  // クラスタリング済みか記憶(true:クラスタリング済み)
};
