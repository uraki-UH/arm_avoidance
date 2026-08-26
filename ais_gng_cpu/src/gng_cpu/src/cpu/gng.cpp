#include "gng.hpp"

#include <boost/sort/spreadsort/spreadsort.hpp>
// #include "../certification/yubikey.hpp"

GNG::GNG() {
}
GNG::~GNG(){
    if (!initialized) return;
    n1.clear();
}
int GNG::init(const char *binary_path) {
    // このプログラムの改ざんチェック
#ifdef GNG_ENABLE_AUTHENTICATION
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
    if(!n1.init(&param.node, &param.edge, &param.config)){
        return ERROR_VOXEL_GRID_LEAF_SIZE;
    }
    voxel_labels.resize(param.config.point_cloud_num);
    attention_pcl.resize(param.config.point_cloud_num);
    voxel2node_ids.resize(param.config.point_cloud_num);

    vg.init(&n1.voxel_config, &param.config);
    la.init(&param.node, &param.label, &n1);
    cl.init(&n1, &param.cluster);

    // outputの初期化
    map.init(param.node.num_max,
            param.cluster.num_max,
            param.edge.num_max,
            param.config.point_cloud_num);

    // inpcl_labels = (uint8_t*)malloc(sizeof(uint8_t) * param.config.point_cloud_num);
    // inpcl_ids = (uint32_t*)malloc(sizeof(uint32_t) * param.config.point_cloud_num);
    // input_pcl = (float*)malloc(sizeof(float) * 3 * param.config.point_cloud_num);

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
    if (!initialized)
        return;

#ifdef GNG_ENABLE_AUTHENTICATION
    // ライセンス認証ブロック
    if(!licenceAuthentication()){
        return;
    }
#endif

    auto t0 = std::chrono::system_clock::now();
    // クラスタリング（CPU）
    vg.applyFilter(map.input_pcl, input_pcl_num, map.inpcl_labels);
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
    // エッジが無いノードの削除と学習係数の減衰
    n1.check_delete_no_edge_and_decay_eta();
    // クラスタリングのために，エッジの距離を計算
    n1.calc_edge_distanceXY();
    auto t5 = std::chrono::system_clock::now();
    // クラスタリング
    cl.clustering();
    auto t6 = std::chrono::system_clock::now();

#ifdef GNG_ENABLE_FRAME_LOG
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
    // check_error();
    log.screen();
#endif
}

void GNG::attention(){
    n1.getDownSampling(vg.filtered_pcl, vg.filtered_pcl_num, voxel_labels, voxel2node_ids, voxel2node_ids_num);
    int i, j;
    for (i = attention_pcl_num = 0; i < vg.filtered_pcl_num; ++i){
        if (voxel_labels[i] == 0)
            continue;
        for (j = vg.voxel_range[i].start; j < vg.voxel_range[i].end; ++j) {
            map.inpcl_labels[vg.voxel_index[j].raw_index] = voxel_labels[i];
            attention_pcl[attention_pcl_num] = vg.sorted_inpcl[j];
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
#if defined(VERSION_STATIC) || defined(VERSION_MOVE)
    // クラスタリング結果
    int cluster_num = MIN(cl.clusters.size(), map.clusters.size());
    map.cluster_num = cluster_num;
    i = j = 0;
    for (i=0; i< cluster_num; ++i) {
        auto &cluster = cl.clusters[i];
        // ID 新規割当
        auto &c = map.clusters[i];
        c.id = cluster.ros_id;
        c.label = cluster.label;
        // c.label_inferred = DEFAULT;
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
        c.frame = cluster.frame;
        c.match = cluster.match;
        c.velocity = cluster.velocity.toVec3();
        // クラスタに含まれるノード
        c.nodes = map.clusters_nodes.data() + j;
        for (auto &node_id: cluster.nodes_ids) {
            map.clusters_nodes[j++] = n1.tn_id[node_id];
        }
        c.node_num = 0;
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

void GNG::check_error(){
    bool check_node = true;
    bool check_edge = false;
    bool check_cluster = true;
    // エッジが単一かどうかのエラーチェック
    int error_count = 0;
    int i, j;

    if(check_node){
        for(i =0;i < n1.node_num_max; ++i){
            auto &node = n1.nodes[i];
            for(j=i+1; j < n1.node_num_max; ++j){
                auto &node2 = n1.nodes[j];
                if(node.id != NODE_NOID && node.id == node2.id){
                    log.println("Node ID Duplicate Error: %d", node.id);
                    error_count++;
                }
            }
        }
        if(error_count == 0){
            log.println("No node ID duplicate error found.");
        }
    }else{
        log.println("Skip node ID duplicate check.");
    }


    if(check_edge){
        bool check_edge_error = false;
        for (auto &node : n1.nodes) {
            if (node.id != -1) {
                bool dublicate = false;
                for (i = 0; i < node.edge_num; ++i) {
                    for(j = i + 1; j < node.edge_num; ++j){
                        if(node.edges[i] == node.edges[j]){
                            dublicate = true;
                            break;
                        }
                    }
                }                
                if (dublicate) {
                    log.print("Edge Duplicate Error(node_id: %d), Edges: ", node.id);
                    for (i = 0; i < node.edge_num; ++i) {
                        log.print("%d, ", node.edges[i]);
                    }
                    log.print("\n");
                    check_edge_error = true;
                }
            }
        }
        if(!check_edge_error){
            log.println("No edge consistency error found.");
        }
    }else{
        log.println("Skip edge consistency check.");
    }

    if(check_cluster){
        bool check_cluster_error = false;
        for(auto &c: cl.clusters){
            int multi_count = 0;
            if(c.scale.isZero()){
                log.println("Cluster scale error, id: %d", c.id);
                check_cluster_error = true;
                continue;
            }
            
            for(int i=0; i < c.size; ++i){
                for(j=i+1; j < c.size; ++j){
                    if(c.nodes_ids[i] == c.nodes_ids[j]){
                        multi_count++;
                    }
                }
            }
            if(multi_count > 0){
                log.println("Cluster Inner node duplicate error, cluster_id: %d, multi_count: %d", c.id, multi_count);
                check_cluster_error = true;
            }
        }
        for(int i=0; i < cl.clusters.size(); ++i){
            for(auto &id: cl.clusters[i].nodes_ids){
                for(int j=i+1; j < cl.clusters.size(); ++j){
                    for(auto &id2: cl.clusters[j].nodes_ids){
                        if(id == id2){
                            log.println("Cluster Inter node duplicate error, cluster_id1: %d, cluster_id2: %d, node_id: %d", cl.clusters[i].id, cl.clusters[j].id, id);
                            check_cluster_error = true;
                        }
                    }
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
                check_cluster_error = true;
            }
            if(cluster.ros_id == CLUSTER_DEFAULT_ROSID){
                log.println("cluster rosid none");
                check_cluster_error = true;
            }
            // ダブってる
            if(used_id.find(cluster.id) == used_id.end()){
                used_id.insert(cluster.id);
            }else{
                log.println("cluster id error %ld", cluster.id);
                check_cluster_error = true;
            }
            if (used_rosid.find(cluster.ros_id) == used_rosid.end()) {
                used_rosid.insert(cluster.ros_id);
            } else {
                log.println("cluster ros_id error %ld", cluster.ros_id);
                check_cluster_error = true;
            }
        }
        if(!check_cluster_error){
            log.println("No cluster consistency error found.");
        }
    }else{
        log.println("Skip cluster consistency check.");
    }
}
