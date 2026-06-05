#pragma once

#include "../utils/utils.hpp"
#include "../utils/vec3f.hpp"

// クラスタの状態を示す定数
constexpr int NOISE = 0;
constexpr int UNCLASSIFIED = -1;

class DBSCAN {
    public:
        DBSCAN();
        ~DBSCAN();

        /**
     * @brief DBSCANのコンストラクタ
     * @param epsilon 半径 (epsilon)
     * @param minPts クラスタを形成するための最小点数
     */
    void init(float epsilon, int minPts);

    /**
     * @brief データに対してDBSCANクラスタリングを実行する
     * @param data データ点。各行が1つのデータ点を表すEigenのMatrix。
     */

        void fit(vector<Vec3f>& data);
    

        /**
         * @brief クラスタリング結果のラベルを返す
         * @return 各データ点のクラスタID (0はノイズ) を格納したvector
         */
        vector<int>& get_labels();

        /**
         * @brief クラスタ数を取得
         * @return クラスタ数
         */
        int get_cluster_num();

    private:
        /**
         * @brief 指定された点のepsilon近傍にある点のインデックスを返す
         */
        vector<int> query_region(vector<Vec3f>& data, int point_index);

        /**
         * @brief コア点を起点にクラスタを拡張する
         */
        void expand_cluster(vector<Vec3f>& data, int point_index, vector<int>& neighbors, int cluster_id);

        float epsilon_2;
        int min_pts_;
        vector<int> labels_;
        int cluster_num = 0;
};