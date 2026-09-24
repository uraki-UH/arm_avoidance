#include "clustering.hpp"

Clustering::Clustering(){

}
Clustering::~Clustering(){

}

void Clustering::init(CUGNG *_gng, ClusterConfig *_cluster_config){
        gng = _gng;
    cluster_config = _cluster_config;

    // dbscan.init(_cluster_config->human_dbscan_epsilon, _cluster_config->human_dbscan_points_num_min); // corridor
    // dbscan.init(0.2, 10);
}

/* クラスタリング */
void Clustering::clustering() {
    // lpf 更新
    time.update(cluster_config->lpf_time);

    const uint32_t frame_number = gng->frame_number;

    new_clusters.clear();
    prev_cluster_order.clear();
    for (size_t idx = 0; idx < clusters.size(); ++idx) {
        prev_cluster_order.emplace_back(clusters[idx].id, idx);
    }
    // 重複ID時にも従来の線形探索と同じ先頭要素を選択する索引。
    sort(prev_cluster_order.begin(), prev_cluster_order.end());
    Center new_human_center; //新規の人クラスタ
    vector<vector<int>> new_human_clusters; //新規の人クラスタ

    set<uint64_t> used_id;  // 使用済みクラスタID
    set<uint32_t> used_rosid; // 使用済みクラスタROSID
    vector<pair<float, int>> nodes_tmp;
    vector<vector<int>> cluster_tmp_tmp;
    int i, j;
    /*未割り当てのIDを追加*/
    static uint64_t max_id = 1;

    /* 大きな床と壁をクラスタリング */
    wall_cluster_size = 0;
    safe_cluster_size = 0;
    for (auto &node : gng->nodes) {
        if (node.id == NODE_NOID || node.clustered_flag)
            continue;
        if (node.label != SAFE_TERRAIN && node.label != WALL)
            continue;
        cluster_tmp.clear();
        cluster_tmp.emplace_back(node.id);
        node.clustered_flag = true;
        _topologicalClusteringExtention(node.id, cluster_tmp, node.label);
        /* クラスタが安定していれば */
        if (cluster_tmp.size() >= cluster_config->node_num_min) {
            Cluster c(cluster_tmp, gng->nodes, node.label);
            if (c.size != 0 && (c.getArea() > (cluster_config->plane_volume))) {
                new_clusters.emplace_back(std::move(c));
                if(node.label == WALL)
                    wall_cluster_size++;
                else
                    safe_cluster_size++;
            }
        }
    }
    for(auto &node:gng->nodes){
        node.clustered_flag = false;
    }
    // 壁と床ノードをクラスタリング済み
    for(auto &new_cluster: new_clusters){
        for(auto &cluster_node_id: new_cluster.nodes_ids){
            gng->nodes[cluster_node_id].clustered_flag = true;
        }
    }
    
    /*その他をラベルなしクラスタリング*/
    unknown_cluster_size = 0;
    for (auto &node : gng->nodes) {
        if (node.id == NODE_NOID || node.clustered_flag)
            continue;
        cluster_tmp.clear();
        // ノードをクラスタリング
        // ラベル関係なし，static同士で
        _topologicalClusteringOther(node.id, cluster_tmp, node.static_node);
        /* クラスタが安定していれば */
        if (cluster_tmp.size() >= cluster_config->node_num_min) {
            Cluster c(cluster_tmp, gng->nodes, frame_number);
            if (c.size != 0) {
                new_clusters.emplace_back(std::move(c));
                unknown_cluster_size++;
            }
            // vector<Vec3f> cluster_node_pos;
            // cluster_node_pos.reserve(cluster_tmp.size());
            // for (auto &id : cluster_tmp) {
            //     cluster_node_pos.emplace_back(gng->nodes[id].pos);
            // }
            // // DBSCANクラスタリング
            // dbscan.fit(cluster_node_pos);
            // auto &labels = dbscan.get_labels();
            // int cluster_num = dbscan.get_cluster_num();
            
            // // 更に分割
            // for (int j = 1;j <= cluster_num; ++j) {
            //     vector<int> cluster_node_ids;
            //     for (int n = 0; n < cluster_tmp.size(); ++n) {
            //         if (labels[n] != j)
            //             continue;
            //         cluster_node_ids.emplace_back(cluster_tmp[n]);
            //     }
            //     if(cluster_node_ids.size() >= cluster_config->num_min){
            //         Cluster c(cluster_node_ids, gng->nodes);
            //         if (c.size != 0) {
            //             new_clusters.emplace_back(c);
            //             c.label == UNKNOWN_OBJECT;
            //             unknown_cluster_size++;
            //         }
            //     }
            // }
            continue;
        }
        for (auto &id : cluster_tmp) {
            gng->nodes[id].clustered_flag = false;
        }
    }

    /* Clusterに所属しているノードの以前のCluster IDの多数決で現在のID*/
    int clusters_size = new_clusters.size();
    if (clusters_size > 0) {
        uint64_t cluster_id;
        map<uint64_t, int> cluster_id_map;  // id, num
        sort(new_clusters.begin(), new_clusters.end(), [](Cluster &a, Cluster &b) { return a.size > b.size; });
        for (auto &cluster: new_clusters) {
            // ID が既に振られている
            if (cluster.id > CLUSTER_DEFAULT_ID)
                continue;
            cluster_id_map.clear();
            for (auto &node_id : cluster.nodes_ids) {
                cluster_id = gng->nodes[node_id].clusted_id;
                // 初めて追加されたノード, ラベルが切り替わったノード, 使用済みID
                if (cluster_id == CLUSTER_DEFAULT_ID || 
                    // gng->nodes[node_id].clusted_label != cluster.label || 
                    used_id.find(cluster_id) != used_id.end()
                ) {
                    continue;
                }
                // 単一の木探索による投票集計。同票時の昇順ID選択は維持。
                ++cluster_id_map.try_emplace(cluster_id, 0).first->second;
            }

            // IDマップができなかった
            if (cluster_id_map.size() == 0) {
                continue;
            }
            // ノード数でソート
            auto pr = max_element(cluster_id_map.begin(), cluster_id_map.end(), [](const auto &x, const auto &y) {
                return x.second < y.second;
            });
            cluster_id = pr->first;
            // 旧クラスタのID索引による引継ぎ先の検索。
            const auto previous = lower_bound(prev_cluster_order.begin(), prev_cluster_order.end(),
                pair<uint64_t, size_t>{cluster_id, 0});
            if (previous != prev_cluster_order.end() && previous->first == cluster_id) {
                _take_over_cluster(clusters[previous->second], cluster);
                used_id.insert(cluster_id);
            }
        }
        // ROSIDを先にマップ
        for (auto &cluster:new_clusters){
            if(cluster.ros_id != CLUSTER_DEFAULT_ROSID){
                used_rosid.insert(cluster.ros_id);
            }
        }
        // 使用済みROSIDの追加だけに対応した、最小空き候補の継続。
        uint32_t min_free_ros_id = 0;
        // 新規クラスタへの処理
        for (auto &cluster: new_clusters) {
            // ID が既に振られている
            if(cluster.id > CLUSTER_DEFAULT_ID){
                // 使用済みROSIDを調査
                continue;
            }
            cluster.id = max_id++;
            // over flow
            if (cluster.id == CLUSTER_DEFAULT_ID)
                cluster.id = max_id++;
            for (uint32_t ros_id = min_free_ros_id; ros_id < (UINT32_MAX - 1); ++ros_id) {
                // ROSIDID が未使用
                if (used_rosid.find(ros_id) == used_rosid.end()) {
                    cluster.ros_id = ros_id;
                    used_rosid.insert(ros_id);
                    min_free_ros_id = ros_id + 1;
                    break;
                }
            }
            cluster.frame = frame_number;
            cluster.match = 0;
            cluster.velocity.zero();
        }
        /* Cluster IDを記憶 */
        /* Cluster内の固定物体判定 */
        for (auto &cluster : new_clusters) {
            // 持続率など
            for (auto &node_id : cluster.nodes_ids) {
                auto &node = gng->nodes[node_id];
                node.clusted_id = cluster.id;
                node.clustered_flag = true;
                node.clusted_label = cluster.label;
            }
        }
    }
    // クラスタに属さなかったノードの処理
    for (auto &node : gng->nodes) {
        if (node.id != NODE_NOID && !node.clustered_flag) {
            node.clusted_id = CLUSTER_DEFAULT_ID;
            node.clusted_label = UNKNOWN_OBJECT;
        }
    }

    // 所属ノード配列の深いコピーを伴わない、構築結果と旧領域の交換。
    clusters.swap(new_clusters);

    // vector<Cluster> new_disable_clusters;
    // for(auto &cluster:disable_clusters){
    //     if(used_id.count(cluster.id) <= 0 && cluster.age++ < 100){
    //         new_disable_clusters.emplace_back(cluster);
    //     }
    // }
    // disable_clusters = new_disable_clusters;
    // for(auto &cluster:disable_clusters){
    //     log.println("id: %d, age: %d", cluster.id, cluster.age);
    // }
}
/* クラスタリングの再帰処理関数 */
void Clustering::_topologicalClustering(int idx, vector<int> &ids) {
    /* あるノード周りを走査 */
    auto &node = gng->nodes[idx];
    for (int i = 0; i < node.edge_num; ++i) {
        int edge_id = node.edges[i];
        auto &node_edge = gng->nodes[edge_id];
        if (node_edge.clustered_flag)
            continue;
        if (gng->nodes[idx].label == node_edge.label) {
            ids.emplace_back(edge_id);
            node_edge.clustered_flag = true;
            _topologicalClustering(edge_id, ids);
        }
    }
}
void Clustering::_topologicalClusteringExtention(int idx, vector<int> &ids, int label) {
    /* あるノード周りを走査 */
    auto &node = gng->nodes[idx];
    int i, j;
    for (i = 0; i < node.edge_num; ++i) {
        int edge_id = node.edges[i];
        auto &node_edge = gng->nodes[edge_id];
        if (node_edge.clustered_flag)
            continue;
        if (label == node_edge.label) {
            ids.emplace_back(edge_id);
            node_edge.clustered_flag = true;
            _topologicalClusteringExtention(edge_id, ids, label);
        }else{
            for (j = 0; j < node_edge.edge_num; ++j){
                int edge_edge_id = node_edge.edges[j];
                auto &node_edge_edge = gng->nodes[edge_edge_id];
                if (label == node_edge_edge.label && !node_edge_edge.clustered_flag) {
                    ids.emplace_back(edge_edge_id);
                    node_edge_edge.clustered_flag = true;
                    _topologicalClusteringExtention(edge_edge_id, ids, label);
                }
            } 
        }
    }
}
void Clustering::_topologicalClusteringOther(int idx, vector<int> &ids, bool static_node) {
    /* あるノード周りを走査 */
    auto &node = gng->nodes[idx];
    for (int i = 0; i < node.edge_num; ++i) {
        int edge_id = node.edges[i];
        auto &node_edge = gng->nodes[edge_id];
        float edge_d2 =
            (node.label == UNKNOWN_OBJECT && node_edge.label == UNKNOWN_OBJECT)
                ? cluster_config->unknown_edge_distance_max2
                : cluster_config->other_edge_distance_max2;
        const uint32_t edge_idx = gng->edge_slots[idx][i];
        if (!node_edge.clustered_flag 
            && (node_edge.static_node == static_node)
            && gng->edge_distance[edge_idx] < edge_d2
        ) {
            ids.emplace_back(edge_id);
            node_edge.clustered_flag = true;
            _topologicalClusteringOther(edge_id, ids, static_node);
        }
    }
}
void Clustering::_take_over_cluster(Cluster &prev, Cluster &now) {
    // 引き継ぎ
    const uint32_t frame_number = gng->frame_number;

    now.id = prev.id;
    now.ros_id = prev.ros_id;
    now.frame = prev.frame;
    now.frame_inferred = prev.frame_inferred;
    now.label_inferred = prev.label_inferred;
    now.count_inferred = prev.count_inferred;

    if (now.label_inferred == HUMAN || now.label_inferred == CAR) {
        const bool is_human = now.label_inferred == HUMAN;
        const auto max_inference_age = static_cast<uint32_t>(is_human
            ? cluster_config->human_hysteresis_age : cluster_config->car_hysteresis_age);
        const auto min_confirmation_count = static_cast<uint32_t>(is_human
            ? cluster_config->human_confirmation_age : cluster_config->car_confirmation_age);
        // 推論の保持期限切れによる失効と、再検出時への確認回数持越し防止。
        if (frame_number - now.frame_inferred > max_inference_age) {
            now.label_inferred = DEFAULT;
            now.count_inferred = 0;
        } else if (now.count_inferred >= min_confirmation_count) {
            now.label = now.label_inferred;
            if (is_human) {
                // 人クラスタの表示幅制限。
                float radius = MIN(MAX(now.scale[0], now.scale[1]), cluster_config->human_radius);
                now.scale[0] = radius;
                now.scale[1] = radius;
            }
        }
    }
    int sum_same = 0;
    for (auto &id : now.nodes_ids) {
        auto &node = gng->nodes[id];
        const uint32_t node_age = frame_number - node.frame; // ノードの年齢
        if (node.clusted_id == prev.id && node_age >= 2)//１：現フレームで生成，２：前フレームで生成
            sum_same++;
    }
    now.match = (float)sum_same / (float)now.size;
    now.velocity = prev.velocity * time.lpf_a + (now.center_pos - prev.center_pos) * time.lpf_dt_T;
}
