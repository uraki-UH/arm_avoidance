#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include "node_spatial_sensor_mapper.hpp"
#include "../spatial/ispatial_index.hpp"

namespace robot_sim {
namespace analysis {

/**
 * @brief SafetyNodeSensorMapper
 * 
 * 「ノードをセンサーとして扱う」安全判定ロジックの実装。
 * 障害物ボクセル -> 手先（センサー）ノード -> 影響を受けるノード群 という流れで
 * 衝突カウントを更新する。
 */
class SafetyNodeSensorMapper {
public:
    SafetyNodeSensorMapper() = default;

    /**
     * @brief 初期化
     * @param total_nodes 全ノード数
     * @param sensor_mapper NodeID -> AffectedNodeIDs のマッピング
     * @param spatial_index Voxel -> SensorNodeID の逆引き用VLUT
     */
    void initialize(size_t total_nodes, 
                    std::shared_ptr<NodeSpatialSensorMapper> sensor_mapper,
                    std::shared_ptr<ISpatialIndex> spatial_index) {
        sensor_mapper_ = sensor_mapper;
        spatial_index_ = spatial_index;
        node_collision_counts_.assign(total_nodes, 0);
    }

    /**
     * @brief 差分更新 (added/removed voxels) を受けてノードの衝突状態を更新
     */
    void updateFromDiff(const std::vector<long>& added_vids, 
                        const std::vector<long>& removed_vids) {
        if (!sensor_mapper_ || !spatial_index_) return;

        // 占有ボクセルの追加によるカウントアップ
        for (long vid : added_vids) {
            updateCountsForVoxel(vid, 1);
        }

        // 占有ボクセルの除去によるカウントダウン
        for (long vid : removed_vids) {
            updateCountsForVoxel(vid, -1);
        }
    }

    /**
     * @brief 特定のボクセルが検知された際の処理
     * @param vid ボクセルID
     * @param delta +1 または -1
     */
    void updateCountsForVoxel(long vid, int delta) {
        // 1. そのボクセルを「センサー（手先）」としているノードを取得
        // 注: ここでの spatial_index_ は Link 7 (手先) を中心に構築されている必要がある
        std::vector<int> sensor_node_ids = spatial_index_->getNodesInVoxel(vid);

        for (int sid : sensor_node_ids) {
            // 2. そのセンサーノードが反応した場合に、トポロジー/空間的に影響を受けるノード群を取得
            const std::vector<int>& affected = sensor_mapper_->getAffectedNodes(sid);
            
            // 3. 影響を受けるノードすべてのカウントを更新
            for (int aid : affected) {
                if (aid >= 0 && (size_t)aid < node_collision_counts_.size()) {
                    node_collision_counts_[aid] += delta;
                }
            }

            // センサーノード自身もカウントを更新
            if (sid >= 0 && (size_t)sid < node_collision_counts_.size()) {
                node_collision_counts_[sid] += delta;
            }
        }
    }

    const std::vector<int>& getCollisionCounts() const { return node_collision_counts_; }

    void reset() {
        std::fill(node_collision_counts_.begin(), node_collision_counts_.end(), 0);
    }

private:
    std::shared_ptr<NodeSpatialSensorMapper> sensor_mapper_;
    std::shared_ptr<ISpatialIndex> spatial_index_;
    std::vector<int> node_collision_counts_;
};

} // namespace analysis
} // namespace robot_sim
