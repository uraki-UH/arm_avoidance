#include "cugng.hpp"

#include <numeric>

CUGNG::CUGNG(){

}

bool CUGNG::init(NodeConfig *_gng_config, EdgeConfig *_edge_config, OtherConfig *_other_config) {
    observation_touched_ids.clear();
    observation_pixel_source = {};
    GridConfig c;
    // 範囲 (大きい)
    c.x_min = _other_config->x_min;
    c.x_max = _other_config->x_max;
    c.y_min = _other_config->y_min;
    c.y_max = _other_config->y_max;
    c.z_min = _other_config->z_min;
    c.z_max = _other_config->z_max;

    // Voxel Grid
    // 間引き無効時の範囲判定用グリッド。点の集約には不使用。
    c.unit = _other_config->voxel_grid_unit > 0
        ? _other_config->voxel_grid_unit : _other_config->node_grid;
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
    next_free_idx = 0;
    // nodes.clear();
    // tn_id.clear();
    edge_count.clear();
    grid.clear();
    grid_page_offsets.clear();
    grid_node_num.clear();

    // malloc
    nodes.resize(node_num_max);
    search_nodes.resize(node_num_max);
    orphan_node_words.assign((node_num_max + 63) / 64, 0);
    end_update_frame();
    is_search_batch = false;
    tn_id.resize(node_num_max);
    // ノード上限と次数上限から決まる、実在エッジ用領域の最大容量。
    const size_t max_edge_num = static_cast<size_t>(node_num_max) * NODE_MAX_EDGE / 2 + 1;
    edge_count.assign(1, EDGE_NO_CONNECT);
    edge_distance.assign(1, 0.f);
    edge_reference_num.assign(1, 0);
    edge_node_ids.assign(1, {0, 0});
    free_edge_ids.clear();
    edge_count.reserve(max_edge_num);
    edge_distance.reserve(max_edge_num);
    edge_reference_num.reserve(max_edge_num);
    edge_node_ids.reserve(max_edge_num);
    free_edge_ids.reserve(max_edge_num);
    edge_slots.resize(node_num_max);
    grid_page_offsets.assign((static_cast<size_t>(grid_config.maxXYZ) + grid_page_size - 1) / grid_page_size, UINT32_MAX);
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
    observation_touched_ids.clear();
    observation_pixel_source = {};
    node_num = 0;
    next_free_idx = 0;
    nodes.clear();
    search_nodes.clear();
    orphan_node_words.clear();
    end_update_frame();
    point_order.clear();
    is_search_batch = false;
    tn_id.clear();
    edge_count.clear();
    edge_distance.clear();
    edge_slots.clear();
    edge_reference_num.clear();
    edge_node_ids.clear();
    free_edge_ids.clear();
    grid.clear();
    grid_page_offsets.clear();
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
void CUGNG::begin_update_frame(bool enable_search_reuse, bool enable_orphan_updates, bool enable_edge_updates) {
    enable_frame_search_reuse = enable_search_reuse;
    enable_frame_orphan_updates = enable_orphan_updates;
    enable_frame_edge_updates = enable_edge_updates;
    is_search_batch = false;
    std::fill(orphan_node_words.begin(), orphan_node_words.end(), 0);
}

void CUGNG::end_update_frame() {
    is_search_batch = false;
    enable_frame_search_reuse = false;
    enable_frame_orphan_updates = false;
    enable_frame_edge_updates = false;
}

void CUGNG::begin_search_batch() {
    // 前フレームのラベル更新と、バッチ外のノード操作の反映。
    search_nodes.resize(nodes.size());
    for (const auto &node : nodes) {
        if (node.id != NODE_NOID) {
            search_nodes[node.id] = {node.pos, node.label, node.clusted_label};
            mark_orphan_node(node);
        }
    }
    is_search_batch = true;
}

void CUGNG::getDownSampling(vector<Vec3f> &inpcl, uint32_t input_pcl_num, vector<uint8_t> &labels){
    uint32_t i;
    static Node_d n;
    // ボクセル番号順による低いZ側へのノード枠の偏在防止。全入力の一度ずつの処理。
    begin_search_batch();
    point_order.resize(input_pcl_num);
    std::iota(point_order.begin(), point_order.end(), 0U);
    std::mt19937 random(frame_number);
    std::shuffle(point_order.begin(), point_order.end(), random);
    for (const uint32_t point_idx : point_order){
        i = point_idx;
        bool inpcl_is_in_vigilance = getDownSamplingGrid(inpcl[i], labels[i], n);
        if(!inpcl_is_in_vigilance){
            add_node(inpcl[i]);
        }
        if(n.id1 != NODE_NOID && n.id2 != NODE_NOID){
            connect(n.id1, n.id2);
        }
    }
    is_search_batch = enable_frame_search_reuse;
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
void CUGNG::learn(vector<Vec3f> &inpcl, int input_pcl_num, vector<Vec3f> &attention_pcl, int attention_pcl_num,
    const vector<Vec3f> *observation_points,
    const vector<uint32_t> *voxel_raw_ids, const vector<uint32_t> *attention_raw_ids,
    const vector<Vec3f> *raw_points, const VoxelGrid *source_voxels,
    const vector<observation_attention_span> *attention_spans, const vector<uint32_t> *attention_blocks){
    // frame
    frame_number++;
    beginTrainingEvents();
    has_observation_frame_origin = enable_observation_support && has_observation_origin;
    observation_pixel_hit_num = observation_ray_num = 0;
    observation_frame_origin = has_observation_frame_origin ? observation_origin : Vec3f{};
    // 支持更新済みノードのみの初期化。削除・再利用されたIDの重複クリアも同値。
    for (const auto id : observation_touched_ids) {
        if (id < nodes.size()) {nodes[id].observation_range.clear();}
    }
    observation_touched_ids.clear();
    const auto learn_input = [&](int idx) {
        if (raw_points && source_voxels && has_observation_frame_origin) {
            // voxel作成済みの対応から選択点だけの実測代表点・元番号参照。
            const auto raw_idx = source_voxels->voxel_index[source_voxels->voxel_range[idx].start].raw_index;
            learn_normal(inpcl[idx], &(*raw_points)[raw_idx], raw_idx);
            return;
        }
        learn_normal(inpcl[idx], observation_points ? &(*observation_points)[idx] : nullptr,
            voxel_raw_ids ? (*voxel_raw_ids)[idx] : UINT32_MAX);
    };

    random_device rnd;  // 非決定的な乱数生成器
    mt19937 mt(rnd());  // 初期シード値
    int i, j;
    if (input_pcl_num == 0)
        return;
    if (!is_search_batch) {begin_search_batch();}
    uniform_int_distribution<> rA(0, input_pcl_num - 1);  // 一様乱数
    const bool has_priority = raw_points && !priority_point_ids.empty() && priority_ratio > 0;
    uniform_int_distribution<> priority_dist(0, std::max(1, static_cast<int>(priority_point_ids.size())) - 1);
    const bool has_priority_weights = has_priority && priority_weights.size() == priority_point_ids.size();
    std::discrete_distribution<> weighted_priority_dist(priority_weights.begin(), priority_weights.end());
    {
        uniform_int_distribution<> rA_Attention(0, std::max(1, attention_pcl_num) - 1);
        for(i=j=0; i< gng_config.learning_num; ++i){
            // 総学習回数を固定した重点配分。通常学習の既存混合比は残余枠内で維持。
            if (has_priority && static_cast<int>((i + 1) * static_cast<double>(priority_ratio)) >
                static_cast<int>(i * static_cast<double>(priority_ratio))) {
                const auto raw_idx = priority_point_ids[has_priority_weights ? weighted_priority_dist(mt) : priority_dist(mt)];
                auto point = (*raw_points)[raw_idx];
                learn_normal(point, nullptr, raw_idx, false);
                continue;
            }
            if(attention_pcl_num == 0 || ++j == gng_config.unknown_learning_rate){
                learn_input(rA(mt));
                j = 0; // リセット
            } else {
                const auto idx = rA_Attention(mt);
                if (attention_raw_ids && raw_points) {
                    // 全attention点のXYZ複製なし。選択点の元番号からの直接参照。
                    const auto raw_idx = (*attention_raw_ids)[idx];
                    auto point = (*raw_points)[raw_idx];
                    learn_normal(point, nullptr, raw_idx);
                } else if (attention_spans && attention_blocks && raw_points && source_voxels) {
                    // 大入力時の圧縮索引。選択点だけの区間検索と実測XYZの読み出し。
                    const auto block = static_cast<uint32_t>(idx) / 64;
                    const auto begin = attention_spans->begin() + (*attention_blocks)[block];
                    const auto end = attention_spans->begin() + std::min<std::size_t>(
                        static_cast<std::size_t>((*attention_blocks)[block + 1]) + 1, attention_spans->size());
                    const auto span = idx < begin->end ? begin : std::upper_bound(begin + 1, end, idx,
                        [](uint32_t value, const auto &entry) {return value < entry.end;});
                    const auto raw_idx = source_voxels->voxel_index[span->source_begin + idx - span->begin].raw_index;
                    auto point = (*raw_points)[raw_idx];
                    learn_normal(point, nullptr, raw_idx);
                } else {
                    learn_normal(attention_pcl[idx], nullptr, attention_raw_ids ? (*attention_raw_ids)[idx] : UINT32_MAX);
                }
            }
        }
    }
    is_search_batch = enable_frame_search_reuse;
}
void CUGNG::learn_normal(Vec3f& p, const Vec3f *observation_point, uint32_t raw_idx, bool enable_statistics) {
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
        const uint32_t edge_idx = edge_slots[node0.id][i];
        edge_count[edge_idx]++;
        /* Edgeの年齢による切断 */
        if (edge_count[edge_idx] > edge_config->age_max){
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

bool CUGNG::getMinGrid(Vec3f &p, Node_d &n) {
    // 探索開始時の参照先選択。探索中の配列切替分岐と本体の重複を回避。
    const auto search = [&](const auto &node_source) {
        int grid_mid_i, grid_mid_j, grid_mid_k, i, j, k;
        int grid_min_i, grid_max_i;
        int grid_min_j, grid_max_j;
        int grid_min_k, grid_max_k;
        float norm2;
        uint32_t grid_index;

        grid_mid_i = (int)((p.p[0] - grid_config.x_min) * grid_config.unit_1);
        grid_mid_j = (int)((p.p[1] - grid_config.y_min) * grid_config.unit_1);
        grid_mid_k = (int)((p.p[2] - grid_config.z_min) * grid_config.unit_1);

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
                    grid_index = i + j * grid_config.max[0] + k * grid_config.maxXY;
                    if (grid_index >= grid_config.maxXYZ)
                        continue;
                    const auto num = grid_node_num[grid_index];
                    if (num == 0) {continue;}
                    const auto *ids = grid.data()[grid_page_offsets.data()[grid_index / grid_page_size] + grid_index % grid_page_size].data();
                    for (uint32_t grid_node_i = 0; grid_node_i < num; ++grid_node_i) {
                        const auto id = ids[grid_node_i];
                        const auto &node = node_source[id];
                        const float x = p.p[0] - node.pos.p[0];
                        const float y = p.p[1] - node.pos.p[1];
                        const float z = p.p[2] - node.pos.p[2];
                        norm2 = x * x + y * y + z * z;
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
    };
    return is_search_batch ? search(search_nodes) : search(nodes);
}

bool CUGNG::getDownSamplingGrid(Vec3f &p, uint8_t &label, Node_d &n) {
    // 探索開始時の参照先選択。探索中の配列切替分岐と本体の重複を回避。
    const auto search = [&](const auto &node_source) {
        int grid_mid_i, grid_mid_j, grid_mid_k, i, j, k;
        int grid_min_i, grid_max_i;
        int grid_min_j, grid_max_j;
        int grid_min_k, grid_max_k;
        float norm2;
        uint32_t grid_index;

        grid_mid_i = (int)((p.p[0] - grid_config.x_min) * grid_config.unit_1);
        grid_mid_j = (int)((p.p[1] - grid_config.y_min) * grid_config.unit_1);
        grid_mid_k = (int)((p.p[2] - grid_config.z_min) * grid_config.unit_1);

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
                    grid_index = i + j * grid_config.max[0] + k * grid_config.maxXY;
                    if (grid_index >= grid_config.maxXYZ)
                        continue;
                    const auto num = grid_node_num[grid_index];
                    if (num == 0) {continue;}
                    const auto *ids = grid.data()[grid_page_offsets.data()[grid_index / grid_page_size] + grid_index % grid_page_size].data();
                    for (uint32_t grid_node_i = 0; grid_node_i < num; ++grid_node_i) {
                        const auto id = ids[grid_node_i];
                        const auto &node = node_source[id];
                        const float x = p.p[0] - node.pos.p[0];
                        const float y = p.p[1] - node.pos.p[1];
                        const float z = p.p[2] - node.pos.p[2];
                        norm2 = x * x + y * y + z * z;
                        if(norm2 < gng_config.s1_reset_range2){
                            nodes[id].age_s1 = 0;
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
    };
    return is_search_batch ? search(search_nodes) : search(nodes);
}

void CUGNG::delete_node(uint32_t idx) {
    if (idx >= node_num_max || node_num <= 2)
        return;
    auto& node = nodes[idx];
    if (node.id == NODE_NOID)
        return;
    recordNodeDelta(node, GNG_DELTA_REMOVE);
    // gridから削除
    auto& g1 = grid_cell(node.grid_i);
    uint32_t last = --grid_node_num[node.grid_i];
    if (node.grid_vec_i != last) {
        nodes[g1[last]].grid_vec_i = node.grid_vec_i;
        g1[node.grid_vec_i] = g1[last];
    }
    g1[last] = NODE_NOID;
    
    node_num--;

    disconnect_all(idx);
    node.id = NODE_NOID;
    next_free_idx = std::min(next_free_idx, idx);
}

void CUGNG::move_node(Node& node, Vec3f& new_pos) {
    if(node.id == NODE_NOID){
        // assert(false);
        return;
    }

    if(!voxel_config.isRange(new_pos.p)){
        return;
    }

    uint32_t new_index = grid_config.getIndex(new_pos.p);
    if(new_index >= grid_config.maxXYZ){
        // 何もしない
        return;
    }
    if (new_index != node.grid_i && grid_node_num[new_index] >= NODE_GRID_NODE_NUM_NAX) {
        return;
    }
    const bool position_changed = map_delta_capture_enabled &&
        (node.pos.p[0] != new_pos.p[0] || node.pos.p[1] != new_pos.p[1] ||
         node.pos.p[2] != new_pos.p[2]);
    node.pos.p[0] = new_pos.p[0];
    node.pos.p[1] = new_pos.p[1];
    node.pos.p[2] = new_pos.p[2];
    if (is_search_batch) {search_nodes[node.id].pos = node.pos;}
    if (position_changed) {
        recordNodeDelta(node, GNG_DELTA_UPDATE);
    }

    // 動かさない
    if (new_index == node.grid_i) {
        return;
    }

    // グリッドの更新
    // 削除
    auto& g1 = grid_cell(node.grid_i);
    if(grid_node_num[node.grid_i] == 0){
        assert(node_num);
    }

    uint32_t last = --grid_node_num[node.grid_i];
    if (node.grid_vec_i != last) {
        nodes[g1[last]].grid_vec_i = node.grid_vec_i;
        g1[node.grid_vec_i] = g1[last];
    }
    g1[last] = NODE_NOID;

    auto& g2 = grid_cell(new_index);
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
    for (; next_free_idx < node_num_max; ++next_free_idx) {
        const uint32_t i = next_free_idx;
        if (nodes[i].id == NODE_NOID) {
            auto& node = nodes[i];
            node.init(i, gng_config.eta_s1, gng_config.eta_s2, pos);
            node.frame = frame_number;
            mark_orphan_node(node);
            if (is_search_batch) {
                search_nodes[i] = {node.pos, node.label, node.clusted_label};
            }
            node.grid_i = grid_i;
            auto& g1 = grid_cell(grid_i);
            node.grid_vec_i = grid_node_num[grid_i]++;
            g1[node.grid_vec_i] = i;
            node_num++;
            recordNodeDelta(node, GNG_DELTA_ADD);
            ++next_free_idx;
            return i;
        }
    }
    return NODE_NOID;
}

void CUGNG::disconnect(uint32_t idx1, uint32_t idx2) {
    if (idx1 == idx2) {return;}
    auto &first = nodes[idx1];
    auto &second = nodes[idx2];
    const uint32_t edge_idx = getEdgeIndex(idx1, idx2);
    const bool has_edge = edge_count[edge_idx] != EDGE_NO_CONNECT;
    // 従来と同じ末尾交換による削除順序と、共有IDの対応維持。
    for (uint32_t idx = 0; idx < first.edge_num; ++idx) {
        if (first.edges[idx] == idx2) {
            release_edge_slot(edge_slots[idx1][idx]);
            first.edges[idx] = first.edges[--first.edge_num];
            edge_slots[idx1][idx] = edge_slots[idx1][first.edge_num];
            break;
        }
    }
    for (uint32_t idx = 0; idx < second.edge_num; ++idx) {
        if (second.edges[idx] == idx1) {
            release_edge_slot(edge_slots[idx2][idx]);
            second.edges[idx] = second.edges[--second.edge_num];
            edge_slots[idx2][idx] = edge_slots[idx2][second.edge_num];
            break;
        }
    }
    edge_count[edge_idx] = EDGE_NO_CONNECT;
    if (has_edge) {recordEdgeDelta(first, second, GNG_DELTA_REMOVE);}
    mark_orphan_node(first);
    mark_orphan_node(second);
}

void CUGNG::disconnect_all(uint32_t idx) {
    auto &first = nodes[idx];
    // 対象ノードの元の隣接順による削除イベントと、相手側の末尾交換。
    for (uint32_t slot_idx = 0; slot_idx < first.edge_num; ++slot_idx) {
        const uint32_t second_idx = first.edges[slot_idx];
        auto &second = nodes[second_idx];
        const uint32_t edge_idx = edge_slots[idx][slot_idx];
        for (uint32_t other_slot_idx = 0; other_slot_idx < second.edge_num; ++other_slot_idx) {
            if (second.edges[other_slot_idx] == idx) {
                recordEdgeDelta(first, second, GNG_DELTA_REMOVE);
                release_edge_slot(edge_slots[second_idx][other_slot_idx]);
                second.edges[other_slot_idx] = second.edges[--second.edge_num];
                edge_slots[second_idx][other_slot_idx] = edge_slots[second_idx][second.edge_num];
                edge_count[edge_idx] = EDGE_NO_CONNECT;
                mark_orphan_node(second);
                break;
            }
        }
        release_edge_slot(edge_idx);
    }
    first.edge_num = 0;
    mark_orphan_node(first);
}

void CUGNG::connect(uint32_t idx1, uint32_t idx2) {
    if (idx1 == idx2) {return;}
    uint32_t edge_idx = getEdgeIndex(idx1, idx2);
    if (edge_count[edge_idx] != EDGE_NO_CONNECT) {
        edge_count[edge_idx] = EDGE_CONNECT;
        return;
    }
    auto &first = nodes[idx1];
    auto &second = nodes[idx2];
    if (first.edge_num == NODE_MAX_EDGE || second.edge_num == NODE_MAX_EDGE) {return;}
    if (edge_idx == 0) {
        if (!free_edge_ids.empty()) {
            edge_idx = free_edge_ids.back();
            free_edge_ids.pop_back();
            edge_distance[edge_idx] = 0.f;
        } else {
            edge_idx = edge_count.size();
            edge_count.push_back(EDGE_NO_CONNECT);
            edge_distance.push_back(0.f);
            edge_reference_num.push_back(0);
            edge_node_ids.push_back({0, 0});
        }
    }
    edge_count[edge_idx] = EDGE_CONNECT;
    edge_node_ids[edge_idx] = {std::min(idx1, idx2), std::max(idx1, idx2)};
    edge_reference_num[edge_idx] += 2;
    edge_slots[idx1][first.edge_num] = edge_idx;
    edge_slots[idx2][second.edge_num] = edge_idx;
    first.edges[first.edge_num++] = idx2;
    second.edges[second.edge_num++] = idx1;
    recordEdgeDelta(first, second, GNG_DELTA_ADD);
}

void CUGNG::check_delete_no_edge_and_decay_eta() {
    if (enable_frame_orphan_updates && !(gng_config.eta_decay_rate < 1.f)) {
        for (size_t word_idx = 0; word_idx < orphan_node_words.size(); ++word_idx) {
            auto word = orphan_node_words[word_idx];
            while (word != 0) {
                const uint32_t node_id = word_idx * 64 + __builtin_ctzll(word);
                word &= word - 1;
                auto &node = nodes[node_id];
                if (node.id != NODE_NOID && node.edge_num == 0) {delete_node(node_id);}
            }
        }
        return;
    }
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
uint32_t CUGNG::getEdgeIndex(uint32_t idx1, uint32_t idx2) {
    const auto &first = nodes[idx1];
    for (uint32_t slot_idx = 0; slot_idx < first.edge_num; ++slot_idx) {
        if (first.edges[slot_idx] == idx2) {return edge_slots[idx1][slot_idx];}
    }
    return 0;
}

void CUGNG::normal_vector(Node& node, Vec3f *node_positions) {
    // 連続座標配列がある場合の直接参照。単独呼出しでは従来のノード配列参照。
    const auto position = [&](uint32_t idx) -> Vec3f & {
        return node_positions ? node_positions[idx] : nodes[idx].pos;
    };
    if (node.edge_num <= 1) {
        node.normal.zero();
        return;
    }
    // 同じ差分ベクトルの再利用。外積・正規化・加算の順序は維持。
    const auto origin = position(node.id);
    auto previous = position(node.edges[0]) - origin;
    if (node.edge_num == 2) {
        node.normal = previous.cross(position(node.edges[1]) - origin).normalized();
        return;
    }
    const auto last = position(node.edges[node.edge_num - 1]) - origin;
    auto reference = previous.cross(last);
    auto normal_sum = reference;
    for (uint32_t idx = 1; idx < node.edge_num; ++idx) {
        auto next = position(node.edges[idx]) - origin;
        auto normal = previous.cross(next).normalized();
        if (reference.dot(normal) < 0) {normal_sum += normal.reverse();}
        else {normal_sum += normal;}
        previous = next;
    }
    node.normal = normal_sum.normalized();
}

void CUGNG::rho(Node& node, Vec3f *node_normals) {
    /* 2つの隣接ノードとの関係を見てcos類似度を平均する */
    float rho_sum = 0;
    /* 2つの接線からcos類似度を計算*/
    int num = 0;
    float dot;
    for (int i = 0; i < node.edge_num;++i) {
        dot = node.normal.dot(node_normals ? node_normals[node.edges[i]] : nodes[node.edges[i]].normal);
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
    if (enable_frame_edge_updates) {
        // 両端の隣接配列を重複走査しない、実在エッジ用プールの単一走査。
        for (uint32_t edge_idx = 1; edge_idx < edge_node_ids.size(); ++edge_idx) {
            if (edge_reference_num[edge_idx] == 0) {continue;}
            const auto &ids = edge_node_ids[edge_idx];
            auto &first = is_search_batch ? search_nodes[ids[0]].pos : nodes[ids[0]].pos;
            auto &second = is_search_batch ? search_nodes[ids[1]].pos : nodes[ids[1]].pos;
            edge_distance[edge_idx] = first.squaredNormXY(second);
        }
        return;
    }
    int i;
    uint32_t edge_id;
    for (auto& node : nodes) {
        if(node.id == NODE_NOID)
            continue;
        for (i = 0; i < node.edge_num;++i){
            edge_id = node.edges[i];
            if(node.id < edge_id){
                edge_distance[edge_slots[node.id][i]] = node.pos.squaredNormXY(nodes[edge_id].pos);
            }
        }
    }
}
