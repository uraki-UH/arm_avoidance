#include "gng.hpp"

#include <boost/sort/spreadsort/spreadsort.hpp>
// #include "../certification/yubikey.hpp"

GNG::GNG() {
}
GNG::~GNG(){
    if (!initialized) return;
    n1.free();
    free(map.nodes);
    free(map.clusters);
    free(map.edges);
    free(clusted_node_ids);
    free(inpcl_labels);
    free(inpcl_ids);
    free(input_pcl);
}
int GNG::init(const char *binary_path) {
    // このプログラムの改ざんチェック
#ifdef _RELEASE	
    if(!auth.checkFile(binary_path, cl.fkey1, n1.fkey2, fkey3, la.fkey4, vg.fkey5)){
        return ERROR_CHECK_FILE;
    }
    // Yubikeyとの通信を開始
    if(!yk_piv.init()){
        return ERROR_CONNECT_YUBIKEY;
    }
    // 認証
    unsigned char req[AUTH_REQUEST_LEN] = {0};
    unsigned char res[AUTH_RESPONSE_LEN] = {0};
    size_t res_len = 0;
    // 認証要求を取得
    if(!auth.getRequest(req))
        return ERROR_FAIL_AUTHENTICATION;
    // Yubikeyに認証要求を送信
    if(!yk_piv.challenge(req, res, &res_len))
        return ERROR_FAIL_AUTHENTICATION;
    // 署名チェック
    if(!auth.checkResponse(res, res_len, cl.ykey1, n1.ykey2, ykey3, la.ykey4, vg.ykey5)){
        return ERROR_FAIL_AUTHENTICATION;
    }
#endif
    
    // GNG初期化
    if(!n1.init(&param.node, &param.config)){
        return ERROR_VOXEL_GRID_LEAF_SIZE;
    }
    voxel_labels.resize(param.config.point_cloud_num);
    attention_pcl.resize(param.config.point_cloud_num);
    voxel2node_ids.resize(param.config.point_cloud_num);

    vg.init(&n1.voxel_config, &param.config);
    la.init(&param.node, &param.label, &n1);
    cl.init(&param.cluster, &n1);

    // outputの初期化
    map.nodes = (TopologicalNode*)malloc(sizeof(TopologicalNode) * param.node.num_max);
    map.clusters = nullptr;
    map.edges = nullptr;
    map.node_num = 0;
    map.cluster_num = 0;
    map.edge_num = 0;
    inpcl_labels = (uint8_t*)malloc(sizeof(uint8_t) * param.config.point_cloud_num);
    inpcl_ids = (uint32_t*)malloc(sizeof(uint32_t) * param.config.point_cloud_num);
    input_pcl = (float*)malloc(sizeof(float) * 3 * param.config.point_cloud_num);

    // 初期化完了
    initialized = true;

    return SUCCESS;
}

#define SECRET_KEY (0x98fad9be)

bool GNG::licenceAuthentication(){
    static bool wait_for_response = false;
    static uint32_t encrypted_count = 0  ^ SECRET_KEY; // 認証済みかどうか

    uint32_t decrypted_count = encrypted_count ^ SECRET_KEY;

    // 100回までならカウントする
    if(decrypted_count < 100){
        decrypted_count++;
    }

    // まだ焦るときではない
    if(decrypted_count <= 25){
        encrypted_count = decrypted_count ^ SECRET_KEY;
        return true;
    }
    // yubikeyからの反応待ち
    if(wait_for_response){
        unsigned char res[AUTH_RESPONSE_LEN] = {0};
        size_t res_len = 0;
        if(yk_piv.checkFinished(res, &res_len)){
            // 署名チェック
            if(auth.checkResponse(res, res_len, cl.ykey1, n1.ykey2, ykey3, la.ykey4, vg.ykey5)){
                // 認証成功
                decrypted_count = 0;
                wait_for_response = false;
            }
        }
    }else{
        // 認証要求を送信
        unsigned char req[AUTH_REQUEST_LEN] = {0};
        if(auth.getRequest(req)){
            if(yk_piv.challengeAsync(req)){
                wait_for_response = true;
            }
        }
    }

    encrypted_count = decrypted_count ^ SECRET_KEY;

    // 制限
    return decrypted_count < 100;
}

void GNG::setPointCloud(const uint8_t *inpcl, const uint32_t _in_num, const LiDAR_Config *_config) {
    static LiDAR_Config prev_config;
    static bool no_prev_config = true;
    if (!initialized) return;
    // 入力点群の確保
    int i;

    // 入力点群の最大値制限
    input_pcl_num  = MIN(_in_num, param.config.point_cloud_num);

    Affine affine;

    // 通常版
    if(!param.config.local_coordinates){
        // グローバル座標で動かす
        affine.init(_config);
        // 座標チェック
        float pos[3];
        for (i = 0; i < input_pcl_num; ++i) {
            // Vec3f p;
            memcpy((uint8_t*)pos, &inpcl[i*_config->point_step], 4 * 3);
            // affine.transform(pos, input_pcl[i]);
            affine.transform(pos, &input_pcl[i*3]);
        }
    }
#if defined(VERSION_MOVE)
    else{
        // 入力点群のコピー
        // ローカル座標で動かす
        for (i = 0; i < input_pcl_num; ++i){
            memcpy((uint8_t*)&input_pcl[i*3], &inpcl[i*_config->point_step], 4 * 3);
        }
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
    }
#endif
}

TopologicalMap GNG::getTopologicalMap() {
    makeResult();
    return map;
}
uint8_t* GNG::getDownSampling(uint32_t *label_num){
    (*label_num) = input_pcl_num;
    return inpcl_labels;
}

void GNG::exec() {
    if (!initialized)
        return;

#ifdef _RELEASE
    // ライセンス認証ブロック
    if(!licenceAuthentication()){
        return;
    }
#endif

    auto t0 = std::chrono::system_clock::now();
    // クラスタリング（CPU）
    vg.applyFilter(input_pcl, input_pcl_num, inpcl_labels);
    auto t1 = std::chrono::system_clock::now();
    // ダウンサンプリング
    attention();
    auto t2 = std::chrono::system_clock::now();
    // 学習
    // n1.learn_normal(input_pcl, input_pcl_num);// 元点群
    n1.learn(vg.filtered_pcl, vg.filtered_pcl_num, attention_pcl, attention_pcl_num);
    auto t3 = std::chrono::system_clock::now();
    // ラベリング
    la.labelling_fuzzy();
    auto t4 = std::chrono::system_clock::now();
    // エッジが短いのは削除
    n1.check_edge_distance();
    // 年齢に基づく削除
    n1.check_age();
    // エッジが無いノードの削除
    n1.check_delete_no_edge();
    // クラスタリングのために，エッジの距離を計算
    n1.calc_edge_distanceXY();
    auto t5 = std::chrono::system_clock::now();
    // クラスタリング
    cl.clustering();
    auto t6 = std::chrono::system_clock::now();

#ifndef _RELEASE
    log.println("I: %d, V: %d, A: %d, Nodes: %d, Clusters: %d", 
        input_pcl_num,
        vg.filtered_pcl_num,
        attention_pcl_num,
        n1.node_num, 
        cl.clusters.size());
    log.println(
        "[%d]: V: %d, A: %d, L:%d, La:%d, Ch:%d, Cl:%d",
        (int)std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t0)
            .count(),
        (int)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
            .count(),
        (int)std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
            .count(),
        (int)std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
            .count(),
        (int)std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3)
            .count(),
        (int)std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4)
            .count(),
        (int)std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5)
            .count());
    log.println("Wall: %d, Safe: %d, Human: %d",
                cl.wall_cluster_size,
                cl.safe_cluster_size,
                cl.human_cluster_size);
#endif
}

void GNG::attention(){
    n1.getDownSampling(vg.filtered_pcl, vg.filtered_pcl_num, voxel_labels, voxel2node_ids, voxel2node_ids_num);
    int i, j;
    for (i = attention_pcl_num = 0; i < vg.filtered_pcl_num; ++i){
        if (voxel_labels[i] == 0)
            continue;
        for (j = vg.voxel_range[i].start; j < vg.voxel_range[i].end; ++j) {
            inpcl_labels[vg.voxel_index[j].raw_index] = voxel_labels[i];

            attention_pcl[attention_pcl_num][0] = vg.sorted_inpcl[j * 3 + 0];
            attention_pcl[attention_pcl_num][1] = vg.sorted_inpcl[j * 3 + 1];
            attention_pcl[attention_pcl_num][2] = vg.sorted_inpcl[j * 3 + 2];
            attention_pcl_num++;

            // memcpy(&, &vg.sorted_inpcl[j*3], sizeof(float) * 3);
        }
    }
    boost::sort::spreadsort::integer_sort(voxel2node_ids.begin(),
    voxel2node_ids.begin() + voxel2node_ids_num, voxel_rightshift_func);
}

void GNG::makeResult(){
    int i, j, k, l, m, n, o;
    uint32_t *ids;

    map.edge_num = 0;
    for (i = j = l = m = 0; i < n1.node_num_max; ++i) {
        auto &node = n1.nodes[i];
        if (node.id == NODE_NOID)
            continue;
        map.nodes[j].id = node.id;
        map.nodes[j].pos.x = node.pos[0];
        map.nodes[j].pos.y = node.pos[1];
        map.nodes[j].pos.z = node.pos[2];
        map.nodes[j].normal.x = node.normal[0];
        map.nodes[j].normal.y = node.normal[1];
        map.nodes[j].normal.z = node.normal[2];
        map.nodes[j].rho = node.rho;
        map.nodes[j].label = node.label;
        map.nodes[j].age = node.age;
        n1.tn_id[node.id] = j;
        for (k = 0; k < node.edge_num; ++k) {
            if (node.id < node.edges[k]) {
                map.edge_num += 2;
            }
        }
        // 以前に合った
        if(node.age > 1){
            // ofset
            for (; voxel2node_ids[l].voxel_index < node.id; ++l){

            }
            // ノードと同じ入力点群
            o = 0;
            ids = &inpcl_ids[m];
            for (; voxel2node_ids[l].voxel_index == node.id; ++l) {
                n = voxel2node_ids[l].raw_index;
                for (k = vg.voxel_range[n].start; k < vg.voxel_range[n].end; ++k) {
                    inpcl_ids[m++] = vg.voxel_index[k].raw_index;
                    o++;
                }
            }
            map.nodes[j].inpcl_num = o;
            if(o > 0){
                map.nodes[j].inpcl_ids = ids;
            }else{
                map.nodes[j].inpcl_ids = nullptr; // 入力点群が無い
            }
        }
        j++;
    }
    map.node_num = j;
    // IDの変換&エッジの挿入
    map.edges = (uint16_t*)realloc(map.edges, sizeof(uint16_t) * map.edge_num);
    for(i = j = 0; i < n1.node_num_max; ++i){
        auto &node = n1.nodes[i];
        if (node.id == NODE_NOID)
            continue;
        for(k = 0; k < node.edge_num; ++k) {
            if (node.id < node.edges[k]) {
                map.edges[j++] = n1.tn_id[node.id];
                map.edges[j++] = n1.tn_id[node.edges[k]];
            }
        }
    }
#if defined(VERSION_STATIC) || defined(VERSION_MOVE)
    // クラスタリング結果
    int cluster_num = cl.clusters.size();
    map.cluster_num = 0;
    int clustered_node_num = 0;
    vector<int> cluster_ids;
    cluster_ids.reserve(cluster_num);
    for (i = 0; i < cluster_num; ++i) {
        if (cl.clusters[i].id == CLUSTER_DEFAULT_ID)
            continue;
        clustered_node_num += cl.clusters[i].nodes_ids.size();
        cluster_ids.emplace_back(i);
    }
    map.cluster_num = cluster_ids.size();
    map.clusters = (TopologicalCluster*)realloc(map.clusters, sizeof(TopologicalCluster) * map.cluster_num);
    clusted_node_ids = (uint16_t*)realloc(clusted_node_ids, sizeof(uint16_t) * clustered_node_num);
    i = j = 0;
    for (auto &id : cluster_ids) {
        auto &cluster = cl.clusters[id];
        // ID 新規割当
        auto &c = map.clusters[i++];
        c.id = cluster.ros_id;
        c.label = cluster.label;
        c.label_inferred = DEFAULT;
        c.pos.x = cluster.center_pos[0];
        c.pos.y = cluster.center_pos[1];
        c.pos.z = cluster.center_pos[2];
        c.scale.x = cluster.scale[0];
        c.scale.y = cluster.scale[1];
        c.scale.z = cluster.scale[2];
        c.quat.x = cluster.quat.x;
        c.quat.y = cluster.quat.y;
        c.quat.z = cluster.quat.z;
        c.quat.w = cluster.quat.w;
        c.age = cluster.age;
        c.match = cluster.match;
        c.velocity = cluster.velocity.toVec3();
        // クラスタに含まれるノード
        c.nodes = clusted_node_ids + j;
        for (auto &node_id : cluster.nodes_ids) {
            clusted_node_ids[j++] = n1.tn_id[node_id];
        }
        c.node_num = cluster.nodes_ids.size();
    } 
#elif defined(VERSION_MAP)
    map.cluster_num = 0;
#endif
}

void GNG::setInferredClusterLabels(const uint32_t *cluster_ids, const uint32_t *cluster_ages, const uint8_t *cluster_labels, const uint32_t size){
#if (defined(VERSION_STATIC)) || (defined(VERSION_MOVE))
    for(int i=0; i< size; ++i){
        if(cluster_labels[i] != HUMAN && cluster_labels[i] != CAR)// ヒトか車の判定
            continue;
        for(auto &cluster: cl.clusters){
            if(cluster.ros_id == cluster_ids[i] &&
                cluster.age >= cluster_ages[i]
                ){
                cluster.label_inferred = cluster_labels[i];
                cluster.age_inferred = cluster.age;
                cluster.count_inferred++;
                // log.println("Detect %s, %d, age: %d",
                // cluster_labels[i] == HUMAN ? "Human" : "CAR",
                // cluster.ros_id, cluster.age);
                break;
            }
        }
    }
#endif
}

void GNG::check_error(){
    // エッジが単一かどうかのエラーチェック
    int error_count = 0;
    int i, j;
    for (auto &node : n1.nodes) {
        if (node.id != -1) {
            int error = 0;
            for (i = 0; i < node.edge_num; ++i) {
                int count = 0;
                int edge_id = node.edges[i];
                auto &edge = n1.nodes[edge_id];
                for (j = 0; j < edge.edge_num;++j) {
                    int edge2_id = edge.edges[j];

                    if (edge_id == edge2_id)
                        count++;
                }
                if (count != 1) {
                    error |= 0b1;
                    error_count++;
                }
                uint32_t edge_index = n1.getEdgeIndex(node.id, edge_id);
                if (n1.edge_count[edge_index] == EDGE_NO_CONNECT)
                    error |= 0b10;
            }

            if (error) {
                log.print("%d[%d]: ", node.id, error);
                for (i = 0; i < node.edge_num; ++i) {
                    log.print("%d, ", node.edges[i]);
                }
                log.print("\n");
            }
        }
    }
    // int i, j;
    // for(i=0; i< NODE_NUM_MAX;++i){
    //     for(j=0;j < NODE_NUM_MAX;++j){
    //         bool detect = false;
    //         for(auto &edge:c.nodes[i].edges){
    //             if(edge == j){
    //                 detect = true;
    //                 break;
    //             }
    //         }
    //         if((c.edge_age[i][j] == 0) == (detect)){
    //             error_count++;
    //             log.print("%d->%d: ", i, j);
    //             for (auto &edge_id : c.nodes[i].edges) {
    //                 log.print("%d, ", edge_id);
    //             }
    //             log.print("\n");
    //         }
    //     }
    // }
    // log.println("err: %d", error_count);
    set<int> ids;
    for(auto &c: cl.clusters){
        bool zero = false;
        bool multi = false;
        ids.clear();
        if(c.scale.isZero() || c.id == CLUSTER_DEFAULT_ID)
            zero = true;
        
        for(auto &id: c.nodes_ids){
            if(ids.find(id) == ids.end()){
                ids.insert(id);
            }else{
                multi = true;
                break;
            }
        }
        if(zero || multi){
            log.println("[%d] id: %d, l: %u, S:%d", zero*2 + multi*1, c.id, static_cast<uint8_t>(c.label), c.size);
            // error check
            for (auto &id:c.nodes_ids) {
                log.println("%.3f, %.3f, %.3f", n1.nodes[id].pos[0], n1.nodes[id].pos[1], n1.nodes[id].pos[2]);
            }
        }
    }
    // cluster_id
    set<uint64_t> used_id;
    set<uint32_t> used_rosid;
    for(auto &cluster: cl.clusters){
        // 未割り当て
        if(cluster.id == CLUSTER_DEFAULT_ID){
            log.println("cluster id none");
        }
        if(cluster.ros_id == CLUSTER_DEFAULT_ROSID){
            log.println("cluster rosid none");
        }
        // ダブってる
        if(used_id.find(cluster.id) == used_id.end()){
            used_id.insert(cluster.id);
        }else{
            log.println("cluster id error %ld", cluster.id);
        }
        if (used_rosid.find(cluster.ros_id) == used_rosid.end()) {
            used_rosid.insert(cluster.ros_id);
        } else {
            log.println("cluster ros_id error %ld", cluster.ros_id);
        }
    }
}
