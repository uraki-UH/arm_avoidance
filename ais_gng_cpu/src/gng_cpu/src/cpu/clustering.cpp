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

    vector<Cluster> new_clusters;  // 現在のノード情報で新規にクラスタリング
    Center new_human_center; //新規の人クラスタ
    vector<vector<int>> new_human_clusters; //新規の人クラスタ

    set<uint64_t> used_id;  // 使用済みクラスタID
    set<uint32_t> used_rosid; // 使用済みクラスタROSID
    vector<int> cluster_tmp;
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
                new_clusters.emplace_back(c);
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
                new_clusters.emplace_back(c);
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
                // 新規ID
                if (cluster_id_map.find(cluster_id) == cluster_id_map.end())
                    cluster_id_map.emplace(cluster_id, 1);
                else
                    cluster_id_map.at(cluster_id)++;
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
            // 見つかった場合
            for (auto &prev_cluster : clusters) {
                if (prev_cluster.id == cluster_id) {
                    _take_over_cluster(prev_cluster, cluster);
                    used_id.insert(cluster_id);
                    break;
                }
            }
        }
        // ROSIDを先にマップ
        for (auto &cluster:new_clusters){
            if(cluster.ros_id != CLUSTER_DEFAULT_ROSID){
                used_rosid.insert(cluster.ros_id);
            }
        }
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
            for (uint32_t ros_id = 0; ros_id < (UINT32_MAX - 1); ++ros_id) {
                // ROSIDID が未使用
                if (used_rosid.find(ros_id) == used_rosid.end()) {
                    cluster.ros_id = ros_id;
                    used_rosid.insert(ros_id);
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

    clusters.clear();
    clusters = new_clusters;

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
        uint32_t edge_index = gng->getEdgeIndex(idx, edge_id);
        if (!node_edge.clustered_flag 
            && (node_edge.static_node == static_node)
            && gng->edge_distance[edge_index] < edge_d2
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

    if(now.label_inferred == HUMAN){// 人
        if((now.count_inferred >= cluster_config->human_confirmation_age) && 
           (frame_number - now.frame_inferred) > cluster_config->human_hysteresis_age){
            now.label = HUMAN;
            // 見た目の大きさを制限する
            float radius = MIN(MAX(now.scale[0], now.scale[1]), cluster_config->human_radius);
            now.scale[0] = radius;
            now.scale[1] = radius;
        }
    }else if(now.label_inferred == CAR){ // 車
        if((now.count_inferred >= cluster_config->car_confirmation_age) && 
           (frame_number - now.frame_inferred) > cluster_config->car_hysteresis_age){
            now.label = CAR;
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