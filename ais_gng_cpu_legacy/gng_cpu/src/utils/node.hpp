#pragma once

#include "utils.hpp"
#include "vec3f.hpp"

class Node{
    public:
        Node();
        ~Node();
        void init(uint32_t id);
        void init(uint32_t id, Vec3f &pos);
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
        uint32_t age;
        uint32_t age_s1 = 0;

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