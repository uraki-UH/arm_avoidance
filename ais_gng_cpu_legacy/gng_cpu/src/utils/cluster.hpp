#pragma once

#include <vector>

#include "node.hpp"
#include "param.hpp"
#include "utils.hpp"
#include "vec3f.hpp"

struct Center{
    vector<uint64_t> cluter_id;
    vector<Vec3f> center_pos;
    set<int> node_ids;
};

/* クラスタリングクラス */
class Cluster {
   public:
    /* data */
    uint64_t id = CLUSTER_DEFAULT_ID;    // クラスタID
    uint32_t ros_id = CLUSTER_DEFAULT_ROSID;  // ROS2でのID
    int label = DEFAULT;  // クラスタのラベル名
    int label_inferred = DEFAULT;  // 推定ラベル
    uint32_t age = 1;  // クラスタ
    uint32_t age_inferred = 1;
    uint32_t count_inferred = 0; // 推定ラベルのカウント
    float match = 0;
    Vec3f velocity;
    Vec3f center_pos;  // バウンディングボックスの位置
    Vec3f scale;       // バウンディングボックスのサイズ
    vector<int> nodes_ids;  // クラスタに属しているノードID一覧
    int size;
    Quaternion quat;

   public:
    Cluster(const Cluster& cluster);
    Cluster(vector<int>& ids, vector<Node> &nodes, int _label = DEFAULT);
    ~Cluster();
    void cylinder();
    float getArea();
    // bool wallContain(Vec3f& pos, ParamCluster& param);
};