#pragma once

#include <algorithm>
#include <vector>
#include <memory>
#include "../spatial/ispatial_index.hpp"
#include "common/voxel_utils.hpp"

namespace robot_sim {
namespace analysis {

/**
 * @brief SafetyVlutMapper
 * Maps raw voxel occupancies into GNG topological node counts.
 * Exclusively designed for portability in ROS 2 standalone components.
 */
class SafetyVlutMapper {
public:
    SafetyVlutMapper() = default;

    void initialize(size_t total_nodes, std::shared_ptr<ISpatialIndex> spatial_index) {
        spatial_index_ = spatial_index;
        if (node_collision_counts_.size() < total_nodes) {
            node_collision_counts_.assign(total_nodes, 0);
            node_danger_counts_.assign(total_nodes, 0);
        }
    }

    void ensureCapacity(size_t nodes_size) {
        if (node_collision_counts_.size() < nodes_size) {
            node_collision_counts_.resize(nodes_size, 0);
            node_danger_counts_.resize(nodes_size, 0);
        }
    }

    void updateFromVoxels(const std::vector<long>& current_occupied,
                          const std::vector<long>& current_danger) {
        if (!spatial_index_) return;

        auto normalized_occupied = current_occupied;
        auto normalized_danger = current_danger;
        std::sort(normalized_occupied.begin(), normalized_occupied.end());
        normalized_occupied.erase(std::unique(normalized_occupied.begin(), normalized_occupied.end()),
                                  normalized_occupied.end());
        std::sort(normalized_danger.begin(), normalized_danger.end());
        normalized_danger.erase(std::unique(normalized_danger.begin(), normalized_danger.end()),
                                normalized_danger.end());

        added_occ_.clear(); removed_occ_.clear();
        std::set_difference(prev_occupied_voxels_.begin(), prev_occupied_voxels_.end(),
                            normalized_occupied.begin(), normalized_occupied.end(),
                            std::back_inserter(removed_occ_));
        std::set_difference(normalized_occupied.begin(), normalized_occupied.end(),
                            prev_occupied_voxels_.begin(), prev_occupied_voxels_.end(),
                            std::back_inserter(added_occ_));

        added_dan_.clear(); removed_dan_.clear();
        std::set_difference(prev_danger_voxels_.begin(), prev_danger_voxels_.end(),
                            normalized_danger.begin(), normalized_danger.end(),
                            std::back_inserter(removed_dan_));
        std::set_difference(normalized_danger.begin(), normalized_danger.end(),
                            prev_danger_voxels_.begin(), prev_danger_voxels_.end(),
                            std::back_inserter(added_dan_));

        updateFromDiff(added_occ_, removed_occ_, added_dan_, removed_dan_);

        prev_occupied_voxels_ = std::move(normalized_occupied);
        prev_danger_voxels_   = std::move(normalized_danger);
    }

    /**
     * @brief Directly update counts using voxel differences.
     * Useful when the sensing node already provides differential information.
     */
    void updateFromDiff(const std::vector<long>& added_occ, const std::vector<long>& removed_occ,
                        const std::vector<long>& added_dan, const std::vector<long>& removed_dan) {
        if (!spatial_index_) return;

        if (!added_occ.empty()) spatial_index_->updateCounts(added_occ, node_collision_counts_, 1);
        if (!removed_occ.empty()) spatial_index_->updateCounts(removed_occ, node_collision_counts_, -1);
        if (!added_dan.empty()) spatial_index_->updateCounts(added_dan, node_danger_counts_, 1);
        if (!removed_dan.empty()) spatial_index_->updateCounts(removed_dan, node_danger_counts_, -1);
    }

    const std::vector<int>& getCollisionCounts() const { return node_collision_counts_; }
    const std::vector<int>& getDangerCounts() const { return node_danger_counts_; }

    const std::vector<long>& getAddedOccupied() const { return added_occ_; }
    const std::vector<long>& getRemovedOccupied() const { return removed_occ_; }
    const std::vector<long>& getAddedDanger() const { return added_dan_; }
    const std::vector<long>& getRemovedDanger() const { return removed_dan_; }

    void resetState() {
        std::fill(node_collision_counts_.begin(), node_collision_counts_.end(), 0);
        std::fill(node_danger_counts_.begin(), node_danger_counts_.end(), 0);
        prev_occupied_voxels_.clear();
        prev_danger_voxels_.clear();
    }

    const std::vector<long>& getPrevOccupiedVoxels() const { return prev_occupied_voxels_; }
    const std::vector<long>& getPrevDangerVoxels() const { return prev_danger_voxels_; }

private:
    std::shared_ptr<ISpatialIndex> spatial_index_;
    
    std::vector<int> node_collision_counts_;
    std::vector<int> node_danger_counts_;

    std::vector<long> prev_occupied_voxels_;
    std::vector<long> prev_danger_voxels_;

    std::vector<long> added_occ_, removed_occ_, added_dan_, removed_dan_;
};

} // namespace analysis
} // namespace robot_sim
