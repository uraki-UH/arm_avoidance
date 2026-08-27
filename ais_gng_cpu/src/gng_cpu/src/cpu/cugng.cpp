#include "cugng.hpp"

CUGNG::CUGNG(){

}

bool CUGNG::init(NodeConfig *_gng_config, EdgeConfig *_edge_config, OtherConfig *_other_config) {
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
    edge_config = _edge_config;

    // clear
    node_num = 0;
    // nodes.clear();
    // tn_id.clear();
    edge_count.clear();
    grid.clear();
    grid_node_num.clear();

    // malloc
    nodes.resize(node_num_max);
    tn_id.resize(node_num_max);
    edge_count.resize(node_num_max * node_num_max);
    memset(edge_count.data(), 0, sizeof(uint8_t) * node_num_max * node_num_max);
    edge_distance.resize(node_num_max * node_num_max);
    grid.resize(grid_config.maxXYZ);
    grid_node_num.resize(grid_config.maxXYZ, 0);
    for (auto& node : nodes)
        node.init(NODE_NOID, 0.f, 0.f);

    // 学習回数
    frame_number = 0;
    map_delta_frame_open = false;

    // 積算誤差による追加
#ifdef ADD_NODE_ERR
    Vec3f p(0, 0, 0);
    add_node(p);               // 初期ノードを追加
    p[0] = 1;
    add_node(p);  // 初期ノードを追加
#endif
    return true;
}
void CUGNG::clear() {
    node_num = 0;
    nodes.clear();
    tn_id.clear();
    edge_count.clear();
    grid.clear();
    grid_node_num.clear();
    training_events.clear();
    training_event_num = 0;
    node_update_recorded.clear();
    updated_node_ids.clear();
    node_deltas.clear();
    edge_deltas.clear();
    map_delta_capture_enabled = false;
    map_delta_frame_open = false;
}
void CUGNG::setTrainingEventCapture(bool enable) {
    if (enable && training_events.empty()) {
        resizeTrainingEventBuffer();
    }
    training_event_capture_enabled = enable;
    training_event_num = 0;
}

void CUGNG::setTrainingEventMaxWinnerRank(uint16_t max_winner_rank) {
    constexpr uint16_t supported_winner_rank_max = 2;
    training_event_winner_rank_max = std::clamp<uint16_t>(
        max_winner_rank,
        1,
        supported_winner_rank_max);
    if (training_event_capture_enabled) {
        resizeTrainingEventBuffer();
    }
    training_event_num = 0;
}

const GngTrainingEvent* CUGNG::getTrainingEvents(uint32_t *num) const {
    if (num != nullptr) {
        *num = training_event_num;
    }
    return training_event_num == 0 ? nullptr : training_events.data();
}

GngNodeKey CUGNG::nodeKey(const Node &node) {
    GngNodeKey key;
    key.id = static_cast<uint16_t>(node.id);
    key.frame = node.frame;
    return key;
}

void CUGNG::setMapDeltaCapture(bool enable) {
    if (enable == map_delta_capture_enabled) return;
    map_delta_capture_enabled = enable;
    map_delta_frame_open = false;
    updated_node_ids.clear();
    node_deltas.clear();
    edge_deltas.clear();
    if (enable) {
        node_update_recorded.assign(node_num_max, 0U);
    } else {
        node_update_recorded.clear();
    }
}

void CUGNG::beginMapDeltaFrame() {
    if (!map_delta_capture_enabled || map_delta_frame_open) {
        return;
    }
    for (const uint16_t id : updated_node_ids) {
        node_update_recorded[id] = 0U;
    }
    updated_node_ids.clear();
    node_deltas.clear();
    edge_deltas.clear();
    map_delta_frame_open = true;
}

void CUGNG::recordNodeDelta(const Node &node, uint8_t operation) {
    if (!map_delta_capture_enabled || node.id == NODE_NOID ||
        node.id >= node_update_recorded.size()) {
        return;
    }
    beginMapDeltaFrame();
    const uint16_t id = static_cast<uint16_t>(node.id);
    if (operation == GNG_DELTA_UPDATE) {
        if (node_update_recorded[id] != 0U) return;
        node_update_recorded[id] = 1U;
        updated_node_ids.push_back(id);
    }
    GngNodeDelta delta;
    delta.key = nodeKey(node);
    delta.operation = operation;
    node_deltas.push_back(delta);
}

void CUGNG::recordEdgeDelta(const Node &first, const Node &second, uint8_t operation) {
    if (!map_delta_capture_enabled || first.id == NODE_NOID || second.id == NODE_NOID ||
        first.id >= node_update_recorded.size() ||
        second.id >= node_update_recorded.size()) {
        return;
    }
    beginMapDeltaFrame();
    GngEdgeDelta delta;
    delta.first = nodeKey(first);
    delta.second = nodeKey(second);
    if (delta.second.id < delta.first.id ||
        (delta.second.id == delta.first.id && delta.second.frame < delta.first.frame)) {
        std::swap(delta.first, delta.second);
    }
    delta.operation = operation;
    edge_deltas.push_back(delta);
}

void CUGNG::finishMapDeltaFrame() {
    map_delta_frame_open = false;
}

const GngMapDelta* CUGNG::getMapDelta() {
    if (!map_delta_capture_enabled) {
        return nullptr;
    }
    map_delta_view.version = 1;
    map_delta_view.frame_number = frame_number;
    map_delta_view.node_delta_count = static_cast<uint32_t>(node_deltas.size());
    map_delta_view.edge_delta_count = static_cast<uint32_t>(edge_deltas.size());
    map_delta_view.node_deltas = node_deltas.empty() ? nullptr : node_deltas.data();
    map_delta_view.edge_deltas = edge_deltas.empty() ? nullptr : edge_deltas.data();
    return &map_delta_view;
}

void CUGNG::beginTrainingEvents() {
    training_event_num = 0;
}

void CUGNG::resizeTrainingEventBuffer() {
    const auto learning_num = static_cast<std::size_t>(std::max(0, gng_config.learning_num));
    training_events.resize(learning_num * training_event_winner_rank_max);
}

void CUGNG::recordTrainingEvent(
    uint16_t winner_rank,
    const Node &winner_node,
    const Vec3f &input_point) {
    if (!training_event_capture_enabled || training_event_num >= training_events.size()) {
        return;
    }

    auto &event = training_events[training_event_num++];
    event.winner_node_id = static_cast<uint16_t>(winner_node.id);
    event.winner_rank = winner_rank;
    event.winner_node_frame = winner_node.frame;
    event.residual.x = input_point.p[0] - winner_node.pos.p[0];
    event.residual.y = input_point.p[1] - winner_node.pos.p[1];
    event.residual.z = input_point.p[2] - winner_node.pos.p[2];
}

void CUGNG::recordTrainingEvents(const Node_d &winners, const Vec3f &input_point) {
    if (winners.id1 != NODE_NOID) {
        recordTrainingEvent(1, nodes[winners.id1], input_point);
    }
    if (training_event_winner_rank_max >= 2 && winners.id2 != NODE_NOID) {
        recordTrainingEvent(2, nodes[winners.id2], input_point);
    }
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
            if(norm2 < edge_config->distance_min2[node.label]){
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
    // frame
    frame_number++;
    beginTrainingEvents();

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
    recordTrainingEvents(n, p);
    // ノードの移動
    // if (!node0.static_node){
    Vec3f new_pos = node0.pos.move(p, node0.eta_s1, 1.f - node0.eta_s1);
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
        if (edge_count[edge_index] > edge_config->age_max){
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
        new_pos = edge.pos.move(p, edge.eta_s2, 1.f - edge.eta_s2);
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
                for (uint32_t grid_node_i = 0; grid_node_i < grid_node_num[grid_index]; ++grid_node_i) {
                    auto id = grid[grid_index][grid_node_i];
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
                for (uint32_t grid_node_i = 0; grid_node_i < grid_node_num[grid_index]; ++grid_node_i) {
                    auto id = grid[grid_index][grid_node_i];
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
    recordNodeDelta(node, GNG_DELTA_REMOVE);
    // gridから削除
    auto& g1 = grid[node.grid_i];
    uint32_t last = --grid_node_num[node.grid_i];
    if (node.grid_vec_i != last) {
        nodes[g1[last]].grid_vec_i = node.grid_vec_i;
        g1[node.grid_vec_i] = g1[last];
    }
    g1[last] = NODE_NOID;
    
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
    if (new_index != node.grid_i && grid_node_num[new_index] >= NODE_GRID_NODE_NUM_NAX) {
        return;
    }
    const bool position_changed = map_delta_capture_enabled &&
        (node.pos[0] != new_pos[0] || node.pos[1] != new_pos[1] ||
         node.pos[2] != new_pos[2]);
    node.pos[0] = new_pos[0];
    node.pos[1] = new_pos[1];
    node.pos[2] = new_pos[2];
    if (position_changed) {
        recordNodeDelta(node, GNG_DELTA_UPDATE);
    }

    // 動かさない
    if (new_index == node.grid_i) {
        return;
    }

    // グリッドの更新
    // 削除
    auto& g1 = grid[node.grid_i];
    if(grid_node_num[node.grid_i] == 0){
        assert(node_num);
    }

    uint32_t last = --grid_node_num[node.grid_i];
    if (node.grid_vec_i != last) {
        nodes[g1[last]].grid_vec_i = node.grid_vec_i;
        g1[node.grid_vec_i] = g1[last];
    }
    g1[last] = NODE_NOID;

    auto& g2 = grid[new_index];
    node.grid_i = new_index;
    node.grid_vec_i = grid_node_num[new_index]++;
    g2[node.grid_vec_i] = node.id;
}

uint32_t CUGNG::add_node(Vec3f &pos) {
    if(node_num == node_num_max){
        return NODE_NOID; // ノード数の上限に達している
    }
    uint32_t grid_i = grid_config.getIndex(pos);

    if(grid_i >= grid_config.maxXYZ){
        return NODE_NOID; // 範囲外
    }
    if (grid_node_num[grid_i] >= NODE_GRID_NODE_NUM_NAX) {
        return NODE_NOID; // グリッドセルの上限
    }
    for (uint32_t i = 0; i < node_num_max; i++) {
        if (nodes[i].id == NODE_NOID) {
            auto& node = nodes[i];
            node.init(i, gng_config.eta_s1, gng_config.eta_s2, pos);
            node.frame = frame_number;
            node.grid_i = grid_i;
            auto& g1 = grid[grid_i];
            node.grid_vec_i = grid_node_num[grid_i]++;
            g1[node.grid_vec_i] = i;
            node_num++;
            recordNodeDelta(node, GNG_DELTA_ADD);
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
    const bool was_connected = edge_count[getEdgeIndex(idx1, idx2)] != EDGE_NO_CONNECT;
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
    if (was_connected) {
        recordEdgeDelta(n1, n2, GNG_DELTA_REMOVE);
    }
}

void CUGNG::disconnect_all(uint32_t idx) {
    uint32_t i, j;
    auto& n1 = nodes[idx];
    /* 相手側のEdgesから自分を消す。 */
    for (i = 0; i < n1.edge_num; ++i) {
        // 相手のID
        auto& n2_id = n1.edges[i];
        auto& n2 = nodes[n2_id];
        // 相手側のエッジをすべて探索する
        for (j = 0; j < n2.edge_num; ++j) {
            // 相手から見たノードが自分のIndexのときなら
            if (n2.edges[j] == idx) {
                recordEdgeDelta(n1, n2, GNG_DELTA_REMOVE);
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
    recordEdgeDelta(n1, n2, GNG_DELTA_ADD);
}

void CUGNG::check_delete_no_edge_and_decay_eta() {
    for (auto& node : nodes) {
        if (node.id == NODE_NOID) {
            continue;
        }
        if (node.edge_num == 0) {
            delete_node(node.id);
        } else if (gng_config.eta_decay_rate < 1.f) {
            node.eta_s1 *= gng_config.eta_decay_rate;
            node.eta_s2 *= gng_config.eta_decay_rate;
        }
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

        const uint32_t node_age = frame_number - node.frame; // ノードの年齢
        
        if (node.clustered_flag)
            age = gng_config.clusted_s1_age[node.label];
        else
            age = gng_config.s1_age[node.label];
        
        if (node.static_node) {
            age = 100;
            if (node.age_s1 >= age) {
                delete_node(node.id);
            } else {
                node.age_s1++;
                node.clustered_flag = false;
            }
        } else {
            if (node.age_s1 >= age) {
                delete_node(node.id);
            } else {
                node.age_s1++;
                node.clustered_flag = false;
                if(gng_config.static_age_min > 0){
                    node.static_node |= node_age > gng_config.static_age_min;
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
