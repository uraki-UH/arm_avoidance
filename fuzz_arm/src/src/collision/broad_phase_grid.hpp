#pragma once

#include "collision/collision_detector.hpp"
#include <unordered_map>
#include <unordered_set>

namespace collision {

/**
 * @brief 空間ハッシュグリッドによるブロードフェーズ衝突検出
 * 多数の動的オブジェクトの効率的な衝突判定のための前処理
 */
class BroadPhaseGrid {
public:
    struct GridCell {
        std::vector<int> object_ids;
    };
    
    /**
     * @brief グリッドを初期化
     * @param cell_size グリッドセルのサイズ
     * @param world_bounds ワールドの境界
     */
    BroadPhaseGrid(double cell_size, const AABB& world_bounds);
    
    /**
     * @brief オブジェクトをグリッドに登録
     */
    void insertObject(int object_id, const AABB& bounds);
    
    /**
     * @brief オブジェクトをグリッドから削除
     */
    void removeObject(int object_id);
    
    /**
     * @brief オブジェクトの位置を更新
     */
    void updateObject(int object_id, const AABB& new_bounds);
    
    /**
     * @brief 指定されたAABBと重複する可能性のあるオブジェクトIDを取得
     */
    std::vector<int> queryOverlaps(const AABB& query_bounds) const;
    
    /**
     * @brief 全てのグリッドをクリア
     */
    void clear();

private:
    double cell_size_;
    AABB world_bounds_;
    std::unordered_map<uint64_t, GridCell> grid_;
    std::unordered_map<int, std::unordered_set<uint64_t>> object_to_cells_;
    
    struct GridCoord {
        int x, y, z;
    };
    
    GridCoord worldToGrid(const Eigen::Vector3d& world_pos) const;
    uint64_t gridCoordToKey(const GridCoord& coord) const;
    std::vector<GridCoord> aabbToGridCoords(const AABB& bounds) const;
};

} // namespace collision