#include "cugng.hpp"

#include <numeric>

CUGNG::CUGNG(){

}

bool CUGNG::init(NodeConfig *node_config, EdgeConfig *edge_params, OtherConfig *input_config) {
    clear();
    min_input_pos = Vec3f(input_config->x_min, input_config->y_min, input_config->z_min);
    max_input_pos = Vec3f(input_config->x_max, input_config->y_max, input_config->z_max);
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(min_input_pos.p[axis]) || !std::isfinite(max_input_pos.p[axis]) ||
            min_input_pos.p[axis] > max_input_pos.p[axis]) {return false;}
    }
    node_num_max = node_config->num_max;
    gng_config = *node_config;
    edge_config = edge_params;
    nodes.resize(node_num_max);
    spatial_entries.assign(node_num_max, spatial_entry{});
    spatial_index = std::make_unique<spatial_tree>();
    tn_id.resize(node_num_max);
    // 既存エッジ表の維持。今回のグリッド撤去とは独立したメモリ構造。
    const auto num_edge_slots = static_cast<size_t>(node_num_max) * node_num_max;
    edge_count.assign(num_edge_slots, 0);
    edge_distance.resize(num_edge_slots);
    for (auto &node : nodes) {node.init(NODE_NOID, 0.0f, 0.0f);}
    frame_number = 0;
    sampling_statistics = {};
    return true;
}
void CUGNG::clear() {
    spatial_index.reset();
    spatial_entries.clear();
    observation_touched_ids.clear();
    observation_pixel_source = {};
    node_num = 0;
    nodes.clear();
    tn_id.clear();
    edge_count.clear();
    edge_distance.clear();
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

void CUGNG::update_winner_statistics(const Node_d &winners, const Vec3f &point) {
    if (!enable_covariance && !enable_support) {return;}
    if (!std::isfinite(point.p[0]) || !std::isfinite(point.p[1]) || !std::isfinite(point.p[2])) {return;}
    const uint32_t ids[]{winners.id1, winners.id2};
    const uint16_t max_rank = enable_support ? 2 : max_covariance_winner_rank;
    for (uint16_t rank = 1; rank <= max_rank; ++rank) {
        if (ids[rank - 1] == NODE_NOID) {continue;}
        auto &node = nodes[ids[rank - 1]];
        if (enable_covariance && rank <= (enable_support ? 1 : max_covariance_winner_rank)) {
            const Vec3f residual(point.p[0] - node.pos.p[0], point.p[1] - node.pos.p[1], point.p[2] - node.pos.p[2]);
            node.winner_stats.add_residual(residual);
        }
        if (enable_support) {
            node.support_stats.add_input(point, rank == 1 ? support_sample_alpha : support_second_alpha,
                rank == 1 ? 1.0 : support_second_weight);
        }
    }
}

void CUGNG::recordTrainingEvents(const Node_d &winners, const Vec3f &input_point) {
    if (winners.id1 != NODE_NOID) {
        recordTrainingEvent(1, nodes[winners.id1], input_point);
    }
    if (training_event_winner_rank_max >= 2 && winners.id2 != NODE_NOID) {
        recordTrainingEvent(2, nodes[winners.id2], input_point);
    }
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
// YAML範囲内の元点から直接選択する最小学習。全点探索・重点候補・空間索引の構築なし。
void CUGNG::learn_raw(vector<Vec3f> &points, const vector<uint32_t> &point_ids) {
    ++frame_number;
    beginTrainingEvents();
    has_observation_frame_origin = enable_observation_support && has_observation_origin;
    observation_pixel_hit_num = observation_ray_num = 0;
    observation_frame_origin = has_observation_frame_origin ? observation_origin : Vec3f{};
    for (const auto id : observation_touched_ids) {
        if (id < nodes.size()) {nodes[id].observation_range.clear();}
    }
    observation_touched_ids.clear();
    if (point_ids.empty()) {return;}
#ifdef GNG_DETERMINISTIC_BENCHMARK
    std::mt19937 random(frame_number);
#else
    std::random_device seed;
    std::mt19937 random(seed());
#endif
    std::uniform_int_distribution<size_t> select_point(0, point_ids.size() - 1);
    for (int iter = 0; iter < gng_config.learning_num; ++iter) {
        const auto raw_idx = point_ids[select_point(random)];
        learn_normal(points[raw_idx], nullptr, raw_idx);
    }
}

bool CUGNG::is_input_in_range(const Vec3f &point) const {
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(point.p[axis]) || point.p[axis] < min_input_pos.p[axis] ||
            point.p[axis] > max_input_pos.p[axis]) {return false;}
    }
    return true;
}

void CUGNG::learn_normal(Vec3f& p, const Vec3f *observation_point, uint32_t raw_idx, bool enable_statistics, const Node_d *selected_winners, bool is_selected_in_vigilance) {
    static Node_d n;
    int i;
    // 全探索
    sampling_statistics.num_zero_samples += p.p[0] == 0 && p.p[1] == 0 && p.p[2] == 0;
    const bool p_is_in_vigilance = selected_winners
        ? (n = *selected_winners, is_selected_in_vigilance) : getMinGrid(p, n);
    // Grid 探索
    // getMinAll(p, n);

    // pが警戒領域に無いときに追加
    if(!p_is_in_vigilance){
        // 全点照合に依存しない追加直後の接続。
        const auto node_idx = add_node(p);
        if (node_idx != NODE_NOID) {
            if (n.id1 != NODE_NOID) {connect(node_idx, n.id1);}
            if (n.id2 != NODE_NOID) {connect(node_idx, n.id2);}
        }

    }

    // s1が見つからない
    if(n.id1 == NODE_NOID){
        // add_node(p);
        return;
    }

    auto &node0 = nodes[n.id1];
    if (enable_statistics && enable_observation_support && has_observation_origin) {
        if (!node0.observation_range.has_support) {observation_touched_ids.push_back(n.id1);}
        const auto &point = observation_point ? *observation_point : p;
        const auto pixel_idx = observation_pixel_source.get(raw_idx);
        if (observation_angle_table && pixel_idx < observation_table_num) {
            node0.observation_range.add(observation_angle_table[pixel_idx]);
            ++observation_pixel_hit_num;
        } else {
            node0.observation_range.add_ray(
                static_cast<double>(point.p[0]) - observation_origin.p[0],
                static_cast<double>(point.p[1]) - observation_origin.p[1],
                static_cast<double>(point.p[2]) - observation_origin.p[2]);
            ++observation_ray_num;
        }
    }
    // 重点再学習の独立観測扱い防止。共分散・支持・統計用イベントは通常学習枠のみ。
    if (enable_statistics) {
        update_winner_statistics(n, p);
        if (training_event_capture_enabled) {recordTrainingEvents(n, p);}
    }
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

bool CUGNG::getMinGrid(Vec3f &point, Node_d &winners) {
    return query_spatial(point, winners, nullptr);
}

bool CUGNG::getDownSamplingGrid(Vec3f &point, uint8_t &label, Node_d &winners) {
    return query_spatial(point, winners, &label);
}

void CUGNG::delete_node(uint32_t idx) {
    if (idx >= static_cast<uint32_t>(node_num_max) || node_num <= 2) {return;}
    auto &node = nodes[idx];
    if (node.id == NODE_NOID) {return;}
    recordNodeDelta(node, GNG_DELTA_REMOVE);
    spatial_index->remove(&spatial_entries[idx]);
    --node_num;
    ++sampling_statistics.num_deleted_nodes;
    disconnect_all(idx);
    node.id = NODE_NOID;
}

void CUGNG::move_node(Node &node, Vec3f &new_pos) {
    if (node.id == NODE_NOID || !is_input_in_range(new_pos)) {return;}
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(new_pos.p[axis])) {return;}
    }
    const bool is_position_changed = node.pos.p[0] != new_pos.p[0] ||
        node.pos.p[1] != new_pos.p[1] || node.pos.p[2] != new_pos.p[2];
    node.pos = new_pos;
    // ノード座標と索引内の座標キャッシュの同期。セル収容上限による拒否なし。
    spatial_index->updatePosition(&spatial_entries[node.id],
        {new_pos.p[0], new_pos.p[1], new_pos.p[2]});
    ++sampling_statistics.num_tree_moves;
    if (map_delta_capture_enabled && is_position_changed) {recordNodeDelta(node, GNG_DELTA_UPDATE);}
}

uint32_t CUGNG::add_node(Vec3f &pos) {
    if (node_num == node_num_max || !is_input_in_range(pos)) {return NODE_NOID;}
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(pos.p[axis])) {return NODE_NOID;}
    }
    for (uint32_t idx = 0; idx < static_cast<uint32_t>(node_num_max); ++idx) {
        if (nodes[idx].id != NODE_NOID) {continue;}
        auto &node = nodes[idx];
        node.init(idx, gng_config.eta_s1, gng_config.eta_s2, pos);
        node.frame = frame_number;
        auto &entry = spatial_entries[idx];
        entry = spatial_entry{};
        entry.node_idx = idx;
        entry.position = {pos.p[0], pos.p[1], pos.p[2]};
        spatial_index->add(&entry);
        ++node_num;
        ++sampling_statistics.num_added_nodes;
        recordNodeDelta(node, GNG_DELTA_ADD);
        return idx;
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
            age = gng_config.max_static_s1_age;
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

bool CUGNG::query_spatial(Vec3f &point, Node_d &winners, uint8_t *label) {
    ++sampling_statistics.num_nearest_queries;
    winners.id1 = winners.id2 = NODE_NOID;
    winners.id1_d2 = winners.id2_d2 = FLT_MAX;
    if (label) {*label = 0;}
    if (!spatial_index) {return false;}
    std::array<bsp3d::SearchResult<spatial_entry>, 2> nearest;
    const int num_nearest = spatial_index->findNBest({point.p[0], point.p[1], point.p[2]}, 2, nearest);
    if (num_nearest > 0) {
        winners.id1 = nearest[0].element->node_idx;
        winners.id1_d2 = nearest[0].distance_sq;
    }
    if (num_nearest > 1) {
        winners.id2 = nearest[1].element->node_idx;
        winners.id2_d2 = nearest[1].distance_sq;
    }
    // 既存bsp3d版と同じ2近傍の判定。全ノードの観測寿命維持は別処理。
    const uint32_t ids[2]{winners.id1, winners.id2};
    const float dist2[2]{winners.id1_d2, winners.id2_d2};
    bool is_in_vigilance = false;
    for (int rank = 0; rank < 2; ++rank) {
        if (ids[rank] == NODE_NOID) {continue;}
        auto &node = nodes[ids[rank]];
        if (label) {
            if (dist2[rank] < gng_config.s1_reset_range2) {node.age_s1 = 0;}
            if (dist2[rank] < gng_config.ds_range_max2) {
                if (node.clusted_label == HUMAN) {*label |= 0b111;}
                else if (node.label == UNKNOWN_OBJECT) {*label |= 0b011;}
            }
        }
        is_in_vigilance |= dist2[rank] < gng_config.vigilance2[node.label];
    }
    return is_in_vigilance;
}
