#include "dbscan.hpp"

DBSCAN::DBSCAN() {
}
DBSCAN::~DBSCAN(){

}

void DBSCAN::init(float epsilon, int minPts){
    epsilon_2 = epsilon * epsilon;  // 距離の2乗
    min_pts_ = minPts;
}

void DBSCAN::fit(vector<Vec3f>& data) {
    int num_points = data.size();
    labels_.assign(num_points, UNCLASSIFIED);
    cluster_num = 0;

    for (int i = 0; i < num_points; ++i) {
        // すでに分類済みの点はスキップ
        if (labels_[i] != UNCLASSIFIED) {
            continue;
        }

        // 点iの近傍点を探す
        vector<int> neighbors = query_region(data, i);

        // 近傍点の数がminPts未満なら、現時点ではノイズ候補
        if (neighbors.size() < min_pts_) {
            labels_[i] = NOISE;
        } else {
            // 新しいクラスタを開始
            cluster_num++;
            expand_cluster(data, i, neighbors, cluster_num);
        }
    }
}

vector<int>& DBSCAN::get_labels(){
    return labels_;
}
int DBSCAN::get_cluster_num() {
    return cluster_num;
}

vector<int> DBSCAN::query_region(vector<Vec3f>& data, int point_index) {
    vector<int> neighbors;

    auto &p = data[point_index];

    for (int i = 0; i < data.size(); ++i) {
        if (i == point_index)
            continue;  // 自分自身は除外
        if (p.squaredNormXY(data[i]) < epsilon_2) {
            neighbors.emplace_back(i);
        }
    }
    return neighbors;
}

void DBSCAN::expand_cluster(vector<Vec3f>& data, int point_index, vector<int>& neighbors, int cluster_id) {
    // まず起点となるコア点をクラスタに割り当てる
    labels_[point_index] = cluster_id;

    // 近傍点のリストをキューのように使って探索を広げる
    // ループ中にneighborsが拡張される可能性があるため、この形式でループする
    for (size_t i = 0; i < neighbors.size(); ++i) {
        int current_neighbor_idx = neighbors[i];

        // 以前ノイズと判断された点も、コア点から到達可能ならクラスタの一部にする
        if (labels_[current_neighbor_idx] == NOISE) {
            labels_[current_neighbor_idx] = cluster_id;
        }

        // まだ未分類の点なら、処理を続ける
        if (labels_[current_neighbor_idx] == UNCLASSIFIED) {
            labels_[current_neighbor_idx] = cluster_id;

            // この近傍点が新たなコア点かチェック
            vector<int> new_neighbors = query_region(data, current_neighbor_idx);
            if (new_neighbors.size() >= min_pts_) {
                // 新たなコア点なら、その近傍点を探索リストに追加
                neighbors.insert(neighbors.end(), new_neighbors.begin(), new_neighbors.end());
            }
        }
    }
}