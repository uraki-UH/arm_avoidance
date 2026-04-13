#include "collision/broad_phase_grid.hpp"
#include <algorithm>

namespace collision {

BroadPhaseGrid::BroadPhaseGrid(double cell_size, const AABB& world_bounds)
    : cell_size_(cell_size), world_bounds_(world_bounds) {
}

void BroadPhaseGrid::insertObject(int object_id, const AABB& bounds) {
    auto grid_coords = aabbToGridCoords(bounds);
    
    for (const auto& coord : grid_coords) {
        uint64_t key = gridCoordToKey(coord);
        grid_[key].object_ids.push_back(object_id);
        object_to_cells_[object_id].insert(key);
    }
}

void BroadPhaseGrid::removeObject(int object_id) {
    auto it = object_to_cells_.find(object_id);
    if (it == object_to_cells_.end()) return;
    
    for (uint64_t cell_key : it->second) {
        auto& cell = grid_[cell_key];
        cell.object_ids.erase(
            std::remove(cell.object_ids.begin(), cell.object_ids.end(), object_id),
            cell.object_ids.end());
        
        if (cell.object_ids.empty()) {
            grid_.erase(cell_key);
        }
    }
    
    object_to_cells_.erase(it);
}

void BroadPhaseGrid::updateObject(int object_id, const AABB& new_bounds) {
    removeObject(object_id);
    insertObject(object_id, new_bounds);
}

std::vector<int> BroadPhaseGrid::queryOverlaps(const AABB& query_bounds) const {
    std::unordered_set<int> result_set;
    auto grid_coords = aabbToGridCoords(query_bounds);
    
    for (const auto& coord : grid_coords) {
        uint64_t key = gridCoordToKey(coord);
        auto it = grid_.find(key);
        if (it != grid_.end()) {
            for (int object_id : it->second.object_ids) {
                result_set.insert(object_id);
            }
        }
    }
    
    return std::vector<int>(result_set.begin(), result_set.end());
}

void BroadPhaseGrid::clear() {
    grid_.clear();
    object_to_cells_.clear();
}

// ============================================================================
// Private Helper Methods
// ============================================================================

BroadPhaseGrid::GridCoord BroadPhaseGrid::worldToGrid(const Eigen::Vector3d& world_pos) const {
    GridCoord coord;
    coord.x = static_cast<int>(std::floor((world_pos.x() - world_bounds_.min.x()) / cell_size_));
    coord.y = static_cast<int>(std::floor((world_pos.y() - world_bounds_.min.y()) / cell_size_));
    coord.z = static_cast<int>(std::floor((world_pos.z() - world_bounds_.min.z()) / cell_size_));
    return coord;
}

uint64_t BroadPhaseGrid::gridCoordToKey(const GridCoord& coord) const {
    // Simple hash combining x, y, z coordinates
    uint64_t key = 0;
    key |= (static_cast<uint64_t>(coord.x & 0x1FFFFF));
    key |= (static_cast<uint64_t>(coord.y & 0x1FFFFF) << 21);
    key |= (static_cast<uint64_t>(coord.z & 0x3FFFFF) << 42);
    return key;
}

std::vector<BroadPhaseGrid::GridCoord> BroadPhaseGrid::aabbToGridCoords(const AABB& bounds) const {
    std::vector<GridCoord> coords;
    
    GridCoord min_coord = worldToGrid(bounds.min);
    GridCoord max_coord = worldToGrid(bounds.max);
    
    for (int x = min_coord.x; x <= max_coord.x; ++x) {
        for (int y = min_coord.y; y <= max_coord.y; ++y) {
            for (int z = min_coord.z; z <= max_coord.z; ++z) {
                coords.push_back({x, y, z});
            }
        }
    }
    
    return coords;
}

} // namespace collision