#include "cugng.hpp"

CUGNG::CUGNG(){

}

bool CUGNG::init(GNGConfig *_gng_config, OtherConfig *_other_config) {
    GridConfig c;
    // 範囲 (大きい)
    c.x_min = _other_config->x_min;
    c.x_max = _other_config->x_max;
    c.y_min = _other_config->y_min;
    c.y_max = _other_config->y_max;
    c.z_min = _other_config->z_min;
    c.z_max = _other_config->z_max;

    // Voxel Grid
    c.unit = _other_config->voxel_grid_unit;
    if (!voxel_config.init(c))
        return false;

    // Grid 範囲を広げる
    c.unit = _other_config->node_grid;
    c.x_min -= c.unit;
    c.y_min -= c.unit;
    c.z_min -= c.unit;
    c.x_max += c.unit;
    c.y_max += c.unit;
    c.z_max += c.unit;
    if (!grid_config.init(c))
        return false;
    
    // copy
    node_num_max = _gng_config->num_max;
    gng_config = *_gng_config;

    // clear
    node_num = 0;
    // nodes.clear();
    // tn_id.clear();
    edge_count.clear();
    grid.clear();

    // malloc
    nodes.resize(node_num_max);
    tn_id.resize(node_num_max);
    edge_count.resize(node_num_max * node_num_max);
    memset(edge_count.data(), 0, sizeof(uint8_t) * node_num_max * node_num_max);
    edge_distance.resize(node_num_max * node_num_max);
    grid.resize(grid_config.maxXYZ);
    for (auto& node : nodes)
        node.init(NODE_NOID);

    // 学習回数
    learning_num = 0;

    // 積算誤差による追加
#ifdef ADD_NODE_ERR
    Vec3f p(0, 0, 0);
    add_node(p);               // 初期ノードを追加
    p[0] = 1;
    add_node(p);  // 初期ノードを追加
#endif
    return true;
}
void CUGNG::free() {
    node_num = 0;
    nodes.clear();
    tn_id.clear();
    edge_count.clear();
    grid.clear();
}
void CUGNG::getDownSampling(vector<Vec3f> &inpcl, uint32_t input_pcl_num, vector<uint8_t> &labels, vector<Voxel> &voxel2node_ids, uint32_t &voxel2node_ids_num){
    uint32_t i, j;
    static Node_d n;
    // 全探索
    for (i = j = 0; i < input_pcl_num;++i){
        bool inpcl_is_in_vigilance = getDownSamplingGrid(inpcl[i], labels[i], n);
        if(!inpcl_is_in_vigilance){
            add_node(inpcl[i]);
        }
        if(n.id1 != NODE_NOID){
            voxel2node_ids[j].voxel_index = n.id1;
            voxel2node_ids[j++].raw_index = i;
            if(n.id2 != NODE_NOID){
                connect(n.id1, n.id2);
            }
        }
    }
    voxel2node_ids_num = j;
}
void CUGNG::check_edge_distance() {
    static uint32_t disconnect_ids[NODE_MAX_EDGE];
    int disconnect_num = 0, i;
    float norm2;
    if(node_num < 300){
        return; // ノード数が少ない場合は削除しない
    }
    for (auto& node : nodes) {
        if (node.id == NODE_NOID)
            continue;

        // ノードのラベルを信用する
        // if (node.label != WALL && node.label != SAFE_TERRAIN)
        //     continue;

        // 大きな平面に含まれるときのみ削除
        if (node.clusted_label != WALL && node.clusted_label != SAFE_TERRAIN)
            continue;

        // 次数が小さすぎる
        // if(node.edge_num <= 3){
        //     continue;
        // }
        disconnect_num = 0;
        for (i = 0; i < node.edge_num; i++) {
            int e_id = node.edges[i];
            auto& edge = nodes[e_id];
            if(node.label != edge.label)
                continue; // ラベルが違うエッジは無視->変な感じになる
            norm2 = node.pos.squaredNorm(edge.pos);
            if(norm2 < gng_config.edge_distance_min2[node.label]){
                disconnect_ids[disconnect_num++] = e_id; // エッジの切断
            } 
        }
        // エッジの削除
        for (i = 0; i < disconnect_num; ++i) {
            disconnect(node.id, disconnect_ids[i]);
        }
    }
}
// void CUGNG::learnBatch(vector<Vec3f> &inpcl, int input_pcl_num){
//     for(auto &node:nodes){
//         node.s1_w.zero();
//         node.s1_w_num = 0;
//     }

//     Node_d node_d;

//     for (int i = 0; i < input_pcl_num; ++i) {
//         // 全探索
//         getMinGrid(inpcl[i], node_d);
//         if(node_d.id1 == NODE_NOID){
//             continue; // 範囲外は無視
//         }
//         auto& node0 = nodes[node_d.id1];
//         node0.age_s1 = 0;  // ノードの年齢をリセット
//         if (node0.label == UNKNOWN_OBJECT || node0.label == DEFAULT) {
//             node0.s1_w += inpcl[i];
//             node0.s1_w_num++;
//         }
//         if(node_d.id2 != NODE_NOID){
//             connect(node_d.id1, node_d.id2);
//         }
//     }
//     float eta = 0.6;
//     for (auto& node : nodes) {
//         if(node.id == NODE_NOID)
//             continue;
//         if(node.s1_w_num > 0){
//             node.s1_w /= (float)node.s1_w_num;
//             node.s1_w_num = 0;
//             move_node(node, node.s1_w, eta, 1.f - eta);
//         }
//     }
// }
void CUGNG::learn(vector<Vec3f> &inpcl, int input_pcl_num, vector<Vec3f> &attention_pcl, int attention_pcl_num){
    random_device rnd;  // 非決定的な乱数生成器を生成
    mt19937 mt(rnd());  //  引数は初期シード値
    int i, j;
    if (input_pcl_num == 0)
        return;
    uniform_int_distribution<> rA(0, input_pcl_num - 1);  // 一様乱数
    if (attention_pcl_num == 0){
        for(i=0; i< gng_config.learning_num; ++i){
            // 学習
            learn_normal(inpcl[rA(mt)]);
        }
    }else{
        uniform_int_distribution<> rA_Attention(0, attention_pcl_num - 1);//一様乱数
        for(i=j=0; i< gng_config.learning_num; ++i){
            // j: 0 ~ 9
            if(++j == gng_config.unknown_learning_rate){
                learn_normal(inpcl[rA(mt)]);
                j = 0; // リセット
            } else {
                learn_normal(attention_pcl[rA_Attention(mt)]);
            }
        }
    }
}
void CUGNG::learn_normal(Vec3f& p) {
    static Node_d n;
    int i;
    // 全探索
    bool p_is_in_vigilance = getMinGrid(p, n);
    // Grid 探索
    // getMinAll(p, n);

    // pが警戒領域に無いときに追加
    if(!p_is_in_vigilance){
        add_node(p);
    }

    // s1が見つからない
    if(n.id1 == NODE_NOID){
        // add_node(p);
        return;
    }

    auto &node0 = nodes[n.id1];
    // ノードの移動
    // if (!node0.static_node){
    float eta = gng_config.eta_s1 / ((float)node0.age/200 + 1);
    float eta_2 = 1.f - eta;

    // move_node(node0, p, gng_config.eta_s1, gng_config.eta_s1_2);
    Vec3f new_pos = node0.pos.move(p, eta, eta_2);
    move_node(node0, new_pos);
    // }

    if(n.id2 != NODE_NOID){
        connect(n.id1, n.id2);
    }

    /* edge年齢の更新 */
    static uint32_t disconnect_ids[NODE_MAX_EDGE];
    int disconnect_num = 0;
    for (i = 0; i < node0.edge_num; ++i) {
        uint32_t edge_index = getEdgeIndex(node0.id, node0.edges[i]);
        edge_count[edge_index]++;
        /* Edgeの年齢による切断 */
        if (edge_count[edge_index] > (gng_config.edge_age_max)){
            disconnect_ids[disconnect_num++] = node0.edges[i];
        }
    }
    // エッジの削除
    for (i = 0; i < disconnect_num; ++i) {
        disconnect(node0.id, disconnect_ids[i]);
    }

    // 何もなくなったら削除
    if (node0.edge_num == 0) {
        // delete_node(node0.id);
        return;
    }

    // 隣接ノードの移動
    for (i = 0; i < node0.edge_num;++i) {
        auto& edge = nodes[node0.edges[i]];
        // if(!edge.static_node){
        // move_node(edge, p, gng_config.eta_s2, gng_config.eta_s2_2);
        // }
        eta = gng_config.eta_s2 / ((float)edge.age/200 + 1);
        eta_2 = 1.f - eta;
        new_pos = edge.pos.move(p, eta, eta_2);
        move_node(edge, new_pos);
    }

    // ノードのageをリセット
    node0.age_s1 = 0;
}

void CUGNG::getMinAll(Vec3f& p, Node_d& n){
    float norm2;

    n.id1 = NODE_NOID;
    n.id1_d2 = FLT_MAX;
    n.id2 = NODE_NOID;
    n.id2_d2 = FLT_MAX;

    // 全探索
    for (auto &node:nodes){
        if (node.id == NODE_NOID)
            continue;

        norm2 = p.squaredNorm(node.pos);  // input-nodeベクトル
        if (norm2 < n.id2_d2) {
            if (norm2 < n.id1_d2) {
                n.id2 = n.id1,
                n.id2_d2 = n.id1_d2;
                n.id1 = node.id, n.id1_d2 = norm2;
            } else {
                n.id2 = node.id, n.id2_d2 = norm2;
            }
        }
    }
}

bool CUGNG::getMinGrid(Vec3f& p, Node_d& n){
    int grid_mid_i, grid_mid_j, grid_mid_k, i, j, k;
    int grid_min_i, grid_max_i;
    int grid_min_j, grid_max_j;
    int grid_min_k, grid_max_k;
    float norm2;
    uint32_t grid_index;

    grid_mid_i = (int)((p[0] - grid_config.x_min) * grid_config.unit_1);
    grid_mid_j = (int)((p[1] - grid_config.y_min) * grid_config.unit_1);
    grid_mid_k = (int)((p[2] - grid_config.z_min) * grid_config.unit_1);

    grid_min_i = MAX(0, grid_mid_i - 1);
    grid_max_i = MIN((int)grid_config.max[0]-1, grid_mid_i + 1);
    grid_min_j = MAX(0, grid_mid_j - 1);
    grid_max_j = MIN((int)grid_config.max[1]-1, grid_mid_j + 1);
    grid_min_k = MAX(0, grid_mid_k - 1);
    grid_max_k = MIN((int)grid_config.max[2]-1, grid_mid_k + 1);

    n.id1 = NODE_NOID;
    n.id1_d2 = FLT_MAX;
    n.id2 = NODE_NOID;
    n.id2_d2 = FLT_MAX;

    bool p_is_in_vigilance = false;

    for (i = grid_min_i; i <= grid_max_i; ++i)
        for (j = grid_min_j; j <= grid_max_j; ++j)
            for (k = grid_min_k; k <= grid_max_k; ++k) {
                grid_index = grid_config.getIndex(i, j, k);
                if (grid_index >= grid_config.maxXYZ)
                    continue;
                for (auto& id : grid[grid_index]) {
                    auto &node = nodes[id];
                    norm2 = p.squaredNorm(node.pos);  // input-nodeベクトル
                    if (norm2 < n.id2_d2) {
                        if (norm2 < n.id1_d2) {
                            n.id2 = n.id1,
                            n.id2_d2 = n.id1_d2;
                            n.id1 = id, n.id1_d2 = norm2;
                        } else {
                            n.id2 = id, n.id2_d2 = norm2;
                        }
                    }
                    p_is_in_vigilance |= (norm2 < gng_config.vigilance2[node.label]);
                }
            }

    return p_is_in_vigilance;
}

bool CUGNG::getDownSamplingGrid(Vec3f& p, uint8_t& label, Node_d &n){
    int grid_mid_i, grid_mid_j, grid_mid_k, i, j, k;
    int grid_min_i, grid_max_i;
    int grid_min_j, grid_max_j;
    int grid_min_k, grid_max_k;
    float norm2;
    uint32_t grid_index;

    grid_mid_i = (int)((p[0] - grid_config.x_min) * grid_config.unit_1);
    grid_mid_j = (int)((p[1] - grid_config.y_min) * grid_config.unit_1);
    grid_mid_k = (int)((p[2] - grid_config.z_min) * grid_config.unit_1);

    grid_min_i = MAX(0, grid_mid_i - 1);
    grid_max_i = MIN((int)grid_config.max[0]-1, grid_mid_i + 1);
    grid_min_j = MAX(0, grid_mid_j - 1);
    grid_max_j = MIN((int)grid_config.max[1]-1, grid_mid_j + 1);
    grid_min_k = MAX(0, grid_mid_k - 1);
    grid_max_k = MIN((int)grid_config.max[2]-1, grid_mid_k + 1);

    bool p_is_in_vigilance = false;

    n.id1 = NODE_NOID;
    n.id1_d2 = FLT_MAX;
    n.id2 = NODE_NOID;
    n.id2_d2 = FLT_MAX;

    label = 0;

    for (i = grid_min_i; i <= grid_max_i; ++i)
        for (j = grid_min_j; j <= grid_max_j; ++j)
            for (k = grid_min_k; k <= grid_max_k; ++k) {
                grid_index = grid_config.getIndex(i, j, k);
                if (grid_index >= grid_config.maxXYZ)
                    continue;
                for (auto& id : grid[grid_index]) {
                    auto &node = nodes[id];
                    norm2 = p.squaredNorm(node.pos);  // input-nodeベクトル
                    if(norm2 < gng_config.s1_reset_range2){
                        node.age_s1 = 0;
                    }
                    if(norm2 < gng_config.ds_range_max2){
                        if(node.clusted_label == HUMAN){
                            label |= 0b111;
                        }else if(node.label == UNKNOWN_OBJECT){
                            label |= 0b011;
                        }
                    }
                    p_is_in_vigilance |= (norm2 < gng_config.vigilance2[node.label]);
                    if (norm2 < n.id2_d2) {
                        if (norm2 < n.id1_d2) {
                            n.id2 = n.id1,
                            n.id2_d2 = n.id1_d2;
                            n.id1 = id, n.id1_d2 = norm2;
                        } else {
                            n.id2 = id, n.id2_d2 = norm2;
                        }
                    }
                }
            }
    return p_is_in_vigilance;
}

void CUGNG::delete_node(uint32_t idx) {
    if (idx >= node_num_max || node_num <= 2)
        return;
    auto& node = nodes[idx];
    if (node.id == NODE_NOID)
        return;
    // gridから削除
    auto& g1 = grid[node.grid_i];
    nodes[g1.back()].grid_vec_i = node.grid_vec_i;
    g1[node.grid_vec_i] = g1.back();
    g1.pop_back();
    
    node_num--;

    disconnect_all(idx);
    node.id = NODE_NOID;
}

void CUGNG::move_node(Node& node, Vec3f& new_pos) {
    if(node.id == NODE_NOID){
        // assert(false);
        return;
    }

    if(!voxel_config.isRange(new_pos)){
        return;
    }

    uint32_t new_index = grid_config.getIndex(new_pos);
    if(new_index >= grid_config.maxXYZ){
        // 何もしない
        return;
    }
    node.pos[0] = new_pos[0];
    node.pos[1] = new_pos[1];
    node.pos[2] = new_pos[2];

    // 動かさない
    if (new_index == node.grid_i) {
        return;
    }

    // グリッドの更新
    // 削除
    auto& g1 = grid[node.grid_i];
    if(g1.empty()){
        assert(node_num);
    }

    nodes[g1.back()].grid_vec_i = node.grid_vec_i;
    g1[node.grid_vec_i] = g1.back();
    g1.pop_back();

    auto& g2 = grid[new_index];
    node.grid_i = new_index;
    node.grid_vec_i = (uint32_t)g2.size();
    g2.emplace_back(node.id);
}

uint32_t CUGNG::add_node(Vec3f &pos) {
    if(node_num == node_num_max){
        return NODE_NOID; // ノード数の上限に達している
    }
    uint32_t grid_i = grid_config.getIndex(pos);

    if(grid_i >= grid_config.maxXYZ){
        return NODE_NOID; // 範囲外
    }
    for (uint32_t i = 0; i < node_num_max; i++) {
        if (nodes[i].id == NODE_NOID) {
            auto& node = nodes[i];
            node.init(i, pos);
            node.grid_i = grid_i;
            auto& g1 = grid[grid_i];
            node.grid_vec_i = (uint32_t)g1.size();
            g1.emplace_back(i);
            node_num++;
            return i;
        }
    }
    return NODE_NOID;
}

void CUGNG::disconnect(uint32_t idx1, uint32_t idx2) {
    if (idx1 == idx2)
        return;
    int i;
    // エッジ
    auto& n1 = nodes[idx1];
    auto& n2 = nodes[idx2];
    auto& e1 = n1.edges;
    auto& e2 = n2.edges;
    // 自身のEdgesから相手を消す
    for (i = 0; i < n1.edge_num; ++i) {
        if (e1[i] == idx2) {
            e1[i] = e1[--n1.edge_num];
            break;
        }
    }

    // 相手のEdgesから自身を消す
    for (i = 0; i < n2.edge_num; ++i) {
        if (e2[i] == idx1) {
            e2[i] = e2[--n2.edge_num];
            break;
        }
    }
    edge_count[getEdgeIndex(idx1, idx2)] = EDGE_NO_CONNECT;
}

void CUGNG::disconnect_all(uint32_t idx) {
    uint32_t i, j;
    auto& n1 = nodes[idx];
    /* 相手側のEdgesから自分を消す。 */
    for (i = 0; i < n1.edge_num; ++i) {
        auto& n2_id = n1.edges[i];
        auto& n2 = nodes[n2_id];
        for (j = 0; j < n2.edge_num; ++j) {
            if (n2.edges[j] == idx) {
                n2.edges[j] = n2.edges[--n2.edge_num];
                edge_count[getEdgeIndex(idx, n2_id)] = EDGE_NO_CONNECT;
                break;
            }
        }
    }
    n1.edge_num = 0;
}

void CUGNG::connect(uint32_t idx1, uint32_t idx2) {
    // 同一のノード
    if (idx1 == idx2) return;
    // // すでに接続してる
    uint32_t edge_index = getEdgeIndex(idx1, idx2);
    if (edge_count[edge_index] != EDGE_NO_CONNECT) {
        edge_count[edge_index] = EDGE_CONNECT;
        return;
    }
    auto& n1 = nodes[idx1];
    auto& n2 = nodes[idx2];

    // エッジ上限
    if (n1.edge_num == NODE_MAX_EDGE || n2.edge_num == NODE_MAX_EDGE)
        return;
    edge_count[edge_index] = EDGE_CONNECT;
    // 末尾に追加
    n1.edges[n1.edge_num++] = idx2;
    n2.edges[n2.edge_num++] = idx1;
}

void CUGNG::check_delete_no_edge() {
    for (auto& node : nodes) {
        if (node.id != NODE_NOID && node.edge_num == 0)
            delete_node(node.id);
    }
}
uint32_t CUGNG::getEdgeIndex(uint32_t idx1, uint32_t idx2){
    if(idx1 < idx2)
        return idx1 + (uint32_t)node_num_max*idx2;
    return idx2 + (uint32_t)node_num_max*idx1;
}

void CUGNG::normal_vector(Node& node) {
    int i;
    if(node.edge_num <= 1){
        node.normal.zero();
    }else if (node.edge_num == 2){
        node.normal = (nodes[node.edges[0]].pos - nodes[node.id].pos).cross(nodes[node.edges[1]].pos - nodes[node.id].pos).normalized();
    }else{
        Vec3f normal0 =  (nodes[node.edges[0]].pos - nodes[node.id].pos).cross(nodes[node.edges[node.edge_num-1]].pos - nodes[node.id].pos);
        Vec3f normal_sum = normal0;
        for(i=0; i< (node.edge_num-1);++i){
            Vec3f normal = (nodes[node.edges[i]].pos - nodes[node.id].pos).cross(nodes[node.edges[i+1]].pos - nodes[node.id].pos).normalized();
            if(normal0.dot(normal) < 0)
                normal_sum += normal.reverse();
            else
                normal_sum += normal;
        }
        node.normal = normal_sum.normalized();
    }
}

void CUGNG::rho(Node& node) {
    /* 2つの隣接ノードとの関係を見てcos類似度を平均する */
    float rho_sum = 0;
    /* 2つの接線からcos類似度を計算*/
    int num = 0;
    float dot;
    for (int i = 0; i < node.edge_num;++i) {
        dot = node.normal.dot(nodes[node.edges[i]].normal);
        if(dot != 0.f){
            rho_sum += fabs(dot);
            num++;
        }
    }
    if(num == 0)
        node.rho = 0;
    else{
        float theta = rho_sum /(float)num;
        theta = MIN(theta, 1.f);
        node.rho = acosf(theta);
    }
}

void CUGNG::check_age(){
    // 選択回数に基づく削除
    int age;
    for (auto &node : nodes) {
        if (node.id == NODE_NOID)
            continue;
        
        if (node.clustered_flag)
            age = gng_config.clusted_s1_age[node.label];
        else
            age = gng_config.s1_age[node.label];
        
        if (node.static_node) {
            age = 100;
            if (node.age_s1 >= age) {
                delete_node(node.id);
            } else {
                node.age++;
                node.age_s1++;
                node.clustered_flag = false;
            }
        } else {
            if (node.age_s1 >= age) {
                delete_node(node.id);
            } else {
                node.age++;
                node.age_s1++;
                node.clustered_flag = false;
                if(gng_config.static_age_min > 0){
                    node.static_node |= node.age > gng_config.static_age_min;
                }
            }
        }
        if(node.clusted_label == HUMAN)
            node.static_node = false;
    }
}

void CUGNG::calc_edge_distanceXY(){
    int i;
    uint32_t edge_id;
    for (auto& node : nodes) {
        if(node.id == NODE_NOID)
            continue;
        for (i = 0; i < node.edge_num;++i){
            edge_id = node.edges[i];
            if(node.id < edge_id){
                edge_distance[getEdgeIndex(node.id, edge_id)] = node.pos.squaredNormXY(nodes[edge_id].pos);
            }
        }
    }
}