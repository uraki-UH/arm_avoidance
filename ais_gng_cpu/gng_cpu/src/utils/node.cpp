#include "node.hpp"

Node::Node() {
    Vec3f p;
    init(NODE_NOID);
}

Node::~Node() {
}

void Node::init(uint32_t _id) {
    id = _id;
    edge_num = 0;

    // 時間
    age = 0;
    age_s1 = 0;

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
void Node::init(uint32_t _id, Vec3f &_pos) {
    init(_id);
    pos[0] = _pos[0];
    pos[1] = _pos[1];
    pos[2] = _pos[2];
    // pos_prev.zero();
}