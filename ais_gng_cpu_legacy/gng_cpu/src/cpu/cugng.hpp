#pragma once

#include "../utils/utils.hpp"
#include "../utils/node.hpp"
#include "../utils/param.hpp"
#include "voxel_grid.hpp"

#include "define.h"

struct Node_d{
    uint32_t id1;
    float id1_d2;
    uint32_t id2;
    float id2_d2;
};

class CUGNG {
   public:
    int node_num = 0;
    int node_num_max = 0;
    int learning_num = 0;
    vector<Node> nodes;
    vector<uint8_t> edge_count;
    vector<vector<uint32_t>> grid;
    vector<uint16_t> tn_id;
    GNGConfig gng_config;
    GridConfig voxel_config;
    GridConfig grid_config;
    vector<float> edge_distance; // エッジの距離

    CUGNG();
    bool init(GNGConfig *_gng_config, OtherConfig *_other_config);
    void free();
    // void learnBatch(vector<Vec3f> &inpcl, int input_pcl_num);
    void learn(vector<Vec3f> &inpcl, int input_pcl_num, vector<Vec3f> &attention_pcl, int attention_pcl_num);
    void learn_normal(Vec3f& input_point);

    void getMinAll(Vec3f& p, Node_d& result);
    bool getMinGrid(Vec3f& p, Node_d& result);
    void getDownSampling(vector<Vec3f> &inpcl, uint32_t input_pcl_num, vector<uint8_t> &labels, vector<Voxel> &voxel2node_ids, uint32_t &voxel2node_ids_num);
    bool getDownSamplingGrid(Vec3f& p, uint8_t &label, Node_d &n);
    void move_node(Node& node, Vec3f& new_pos);

    /* ノードを削除する関数 */
    void delete_node(uint32_t idx);
    /* ノードを追加する関数 */
    uint32_t add_node(Vec3f &pos);
    /* エッジを切断する関数 */
    void disconnect(uint32_t idx1, uint32_t idx2);
    /* 全てのエッジを切断する関数 */
    void disconnect_all(uint32_t idx);
    /* ノード同士を接続する関数 */
    void connect(uint32_t idx1, uint32_t idx2);
    /* エッジ数が0のノードを削除する関数 */
    void check_delete_no_edge();
    /* エッジIDを検索 */
    uint32_t getEdgeIndex(uint32_t idx1, uint32_t idx2);
    /* ノードの法線ベクトルの算出 */
    void normal_vector(Node& node);
    /* ノードのCOS類似度の算出 */
    void rho(Node& node);
    /* ノードの年齢チェック */
    void check_age();
    /* エッジの長さチェック */
    void check_edge_distance();
    /* エッジの距離を計算 */
    void calc_edge_distanceXY();

    const uint32_t ykey2[4] = {_YK_KEY2_1, _YK_KEY2_2, _YK_KEY2_3, _YK_KEY2_4};
    const uint32_t fkey2[4] = {_FILE_KEY2_1, _FILE_KEY2_2, _FILE_KEY2_3, _FILE_KEY2_4};
};