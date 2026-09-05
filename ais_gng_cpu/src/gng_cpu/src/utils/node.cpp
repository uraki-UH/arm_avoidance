#include "node.hpp"

Node::Node() {
    init(NODE_NOID, 0.f, 0.f);
}

Node::~Node() {
}

void Node::init(uint32_t _id, float _eta_s1, float _eta_s2) {
    winner_stats = {};
    support_stats = {};
    id = _id;
    edge_num = 0;

    // 時間
    frame = 0;
    age_s1 = 0;
    eta_s1 = _eta_s1;
    eta_s2 = _eta_s2;

    // flag
    static_node = false;

    // ラベル
    label = UNKNOWN_OBJECT; // 初期ラベルはUNKNOWN_OBJECT
    for (int i = 0; i < 3; ++i){
        fuzzy_exp[i] = 0.0;
    }
    normal.zero();
    rho = 0;
    // vel = 0;

    // クラスタリング
    clusted_id = CLUSTER_DEFAULT_ID;
    clusted_label = UNKNOWN_OBJECT;
    clustered_flag = false;
}
void Node::init(uint32_t _id, float _eta_s1, float _eta_s2, Vec3f &_pos) {
    init(_id, _eta_s1, _eta_s2);
    pos[0] = _pos[0];
    pos[1] = _pos[1];
    pos[2] = _pos[2];
    // pos_prev.zero();
}
