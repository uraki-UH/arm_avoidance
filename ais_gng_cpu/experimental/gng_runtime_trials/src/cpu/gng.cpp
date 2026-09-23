#include "gng.hpp"
#include <numeric>

// #include "../certification/yubikey.hpp"

GNG::GNG() {
}
GNG::~GNG(){
    if (!initialized) return;
    n1.clear();
}
int GNG::init(const char *binary_path) {
    if (!n1.init(&param.node, &param.edge, &param.config)) {return ERROR_VOXEL_GRID_LEAF_SIZE;}
#ifdef GNG_USE_INPUT_VOXELS
    if (!voxel_input.init(param.config)) {return ERROR_VOXEL_GRID_LEAF_SIZE;}
#endif
    input_point_ids.reserve(param.config.point_cloud_num);
    la.init(&param.node, &param.label, &n1);
    map.init(param.node.num_max, param.cluster.num_max, param.edge.num_max, param.config.point_cloud_num);
    initialized = true;
    return SUCCESS;
}

void GNG::setPointCloud(const uint8_t *inpcl, const uint32_t _in_num, const LiDAR_Config *_config) {
    static LiDAR_Config prev_config;
    static bool no_prev_config = true;
    if (!initialized) return;
    n1.beginMapDeltaFrame();
    n1.priority_point_ids.clear();
    n1.priority_weights.clear();
    n1.priority_ratio = 0;
    // 入力点群の確保
    n1.has_observation_origin = false;
    n1.observation_pixel_source = {};
    n1.observation_angle_table = nullptr;
    n1.observation_table_num = 0;
    int i;

    // 入力点群の最大値制限
    input_pcl_num  = MIN(_in_num, (uint32_t)map.input_pcl.size());
    map.input_pcl_num = input_pcl_num;

    Affine affine;

    // 通常版
    if(!param.config.local_coordinates){
        // グローバル座標で動かす
        affine.init(_config);
        // 座標チェック
        float pos[3];
        for (i = 0; i < input_pcl_num; ++i) {
            memcpy((uint8_t*)pos, &inpcl[i*_config->point_step], 4 * 3);
            affine.transform(pos, map.input_pcl[i].p);
        }
    }else{
        // 入力点群のコピー
        // ローカル座標で動かす
        for (i = 0; i < input_pcl_num; ++i){
            memcpy((uint8_t*)&map.input_pcl[i].p[0], &inpcl[i*_config->point_step], 4 * 3);
        }
    }
#if defined(VERSION_MOVE)
        // 前回の設定がない場合
        if(no_prev_config){
            prev_config = *_config;
            no_prev_config = false;
        }else{
            // GNGのノードを変換
            affine.initDiff(_config, &prev_config);
            prev_config = *_config;
            Vec3f new_pos;

            // ノードの座標変換
            for (auto &node : n1.nodes) {
                if(node.id == NODE_NOID)
                    continue; // 無効なノード
                // 座標変換
                affine.transform(node.pos, new_pos);
                // ノードの移動
                n1.move_node(node, new_pos);
            }
        }
#endif
}

TopologicalMap GNG::getTopologicalMap() {
    makeResult();
    TopologicalMap result;
    result.frame_number = n1.frame_number;
    result.nodes = map.nodes.data();
    result.clusters = map.clusters.data();
    result.edges = map.edges.data();
    result.node_num = map.node_num;
    result.cluster_num = map.cluster_num;
    result.edge_num = map.edge_num;
    return result;
}
uint8_t* GNG::getDownSampling(uint32_t *label_num){
    (*label_num) = input_pcl_num;
    return map.inpcl_labels.data();
}

void GNG::exec() {
    if (!initialized) {return;}
    n1.beginMapDeltaFrame();
    n1.sampling_statistics = {};
    const auto start = std::chrono::steady_clock::now();
    input_point_ids.clear();
#ifdef GNG_USE_INPUT_VOXELS
    voxel_input.prepare(map.input_pcl, input_pcl_num, map.inpcl_labels);
    input_point_ids.resize(voxel_input.points.size());
    std::iota(input_point_ids.begin(), input_point_ids.end(), 0);
    n1.sampling_statistics.num_input_points = voxel_input.num_input_points;
#else
    std::fill(map.inpcl_labels.begin(), map.inpcl_labels.begin() + input_pcl_num, 0);
    // 設定範囲と有限座標の確認のみ。ソート・重心計算・セル分割なし。
    for (uint32_t idx = 0; idx < static_cast<uint32_t>(input_pcl_num); ++idx) {
        if (!n1.is_input_in_range(map.input_pcl[idx])) {continue;}
        input_point_ids.push_back(idx);
        map.inpcl_labels[idx] = 0b001;
    }
    n1.sampling_statistics.num_input_points = input_point_ids.size();
#endif
    n1.sampling_statistics.num_training_points = input_point_ids.size();
    const auto after_input = std::chrono::steady_clock::now();
#ifdef GNG_USE_INPUT_VOXELS
    // 重心は元画素番号を持たないため、観測支持は方向ベクトルで算出。
    n1.observation_pixel_source = {};
    n1.observation_angle_table = nullptr;
    n1.observation_table_num = 0;
    n1.learn_raw(voxel_input.points, input_point_ids);
#else
    n1.learn_raw(map.input_pcl, input_point_ids);
#endif
    n1.has_observation_origin = false;
    n1.observation_pixel_source = {};
    n1.observation_angle_table = nullptr;
    n1.observation_table_num = 0;
    const auto after_learn = std::chrono::steady_clock::now();
    // グラフ出力に用いる既存の法線・ラベル計算。
    la.labelling_fuzzy();
    const auto after_label = std::chrono::steady_clock::now();
    n1.check_age();
    n1.check_delete_no_edge_and_decay_eta();
    const auto after_maintenance = std::chrono::steady_clock::now();
    auto &statistics = n1.sampling_statistics;
    statistics.input_prepare_ms = std::chrono::duration<double, std::milli>(after_input - start).count();
#ifdef GNG_USE_INPUT_VOXELS
    statistics.voxel_ms = statistics.input_prepare_ms;
#endif
    statistics.learn_ms = std::chrono::duration<double, std::milli>(after_learn - after_input).count();
    statistics.label_ms = std::chrono::duration<double, std::milli>(after_label - after_learn).count();
    statistics.maintenance_ms = std::chrono::duration<double, std::milli>(after_maintenance - after_label).count();
    // 最小版では平面・曲面クラスタリングなし。
    n1.finishMapDeltaFrame();
}

void GNG::makeResult(){
    int i, j, k, l, m, n, o;
    uint32_t *ids;

    uint32_t edge_num = 0;
    for (i = j = l = m = 0; i < n1.node_num_max; ++i) {
        auto &node = n1.nodes[i];
        if (node.id == NODE_NOID)
            continue;
        auto &n = map.nodes[j];
        n.id = node.id;
        n.pos.x = node.pos[0];
        n.pos.y = node.pos[1];
        n.pos.z = node.pos[2];
        n.normal.x = node.normal[0];
        n.normal.y = node.normal[1];
        n.normal.z = node.normal[2];
        n.rho = node.rho;
        n.label = node.label;
        n.frame = node.frame;
        n.inpcl_ids = nullptr; // TODO
        n.inpcl_num = 0;
        n1.tn_id[node.id] = j;
        for (k = 0; k < node.edge_num; ++k) {
            if (node.id < node.edges[k]) {
                edge_num += 2;
            }
        }
        j++;
    }
    map.node_num = j;
    // IDの変換&エッジの挿入
    map.edge_num = MIN(edge_num, map.edges.size());
    for(i = j = 0; i < n1.node_num_max; ++i){
        auto &node = n1.nodes[i];
        if (node.id == NODE_NOID)
            continue;
        for(k = 0; k < node.edge_num; ++k) {
            if (node.id < node.edges[k]) {
                if(j + 1 >= map.edge_num){
                    // log.println("Edge num over: %d", edge_num);
                    break;
                }
                map.edges[j++] = n1.tn_id[node.id];
                map.edges[j++] = n1.tn_id[node.edges[k]];
            }
        }
    }
    map.cluster_num = 0;

}

void GNG::setInferredClusterLabels(const uint32_t *cluster_ids, const uint32_t *cluster_ages, const uint8_t *cluster_labels, const uint32_t size){
#if (defined(VERSION_STATIC)) || (defined(VERSION_MOVE))
    for(int i=0; i< size; ++i){
        if(cluster_labels[i] != HUMAN && cluster_labels[i] != CAR)// ヒトか車の判定
            continue;
        for(auto &cluster: cl.clusters){
            const uint32_t cluster_age = n1.frame_number - cluster.frame; // クラスタの年齢
            if(cluster.ros_id == cluster_ids[i] &&
                cluster_age >= cluster_ages[i]
                ){
                cluster.label_inferred = cluster_labels[i];
                cluster.frame_inferred = n1.frame_number;
                cluster.count_inferred++;
                // log.println("Detect %s, %d, age: %d",
                // cluster_labels[i] == HUMAN ? "Human" : "CAR",
                // cluster.ros_id, cluster_age);
                break;
            }
        }
    }
#endif
}

