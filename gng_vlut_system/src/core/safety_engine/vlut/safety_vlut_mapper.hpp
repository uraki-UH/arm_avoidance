#pragma once

#include <algorithm>
#include <vector>
#include <memory>
#include "../indexing/ispatial_index.hpp"
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

        auto normalized_occupied = normalize(current_occupied);
        auto normalized_danger = normalize(current_danger);

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

    void updateFromVlutDistance(const std::vector<long>& current_occupied,
                                float danger_dist) {
        if (!spatial_index_) return;

        auto normalized_occupied = normalize(current_occupied);
        added_occ_.clear();
        removed_occ_.clear();
        added_dan_.clear();
        removed_dan_.clear();
        std::set_difference(prev_occupied_voxels_.begin(), prev_occupied_voxels_.end(),
                            normalized_occupied.begin(), normalized_occupied.end(),
                            std::back_inserter(removed_occ_));
        std::set_difference(normalized_occupied.begin(), normalized_occupied.end(),
                            prev_occupied_voxels_.begin(), prev_occupied_voxels_.end(),
                            std::back_inserter(added_occ_));

        const float clamped_danger_dist = std::max(0.0f, danger_dist);
        constexpr float collision_dist = 1e-6f;
        if (!added_occ_.empty()) {
            spatial_index_->updateCounts(
                added_occ_, node_collision_counts_, 1, collision_dist);
            spatial_index_->updateCounts(
                added_occ_, node_danger_counts_, 1, clamped_danger_dist);
        }
        if (!removed_occ_.empty()) {
            spatial_index_->updateCounts(
                removed_occ_, node_collision_counts_, -1, collision_dist);
            spatial_index_->updateCounts(
                removed_occ_, node_danger_counts_, -1, clamped_danger_dist);
        }

        prev_occupied_voxels_ = std::move(normalized_occupied);
        prev_danger_voxels_.clear();
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
    static std::vector<long> normalize(const std::vector<long>& values) {
        auto normalized = values;
        std::sort(normalized.begin(), normalized.end());
        normalized.erase(std::unique(normalized.begin(), normalized.end()), normalized.end());
        return normalized;
    }

    std::shared_ptr<ISpatialIndex> spatial_index_;
    
    std::vector<int> node_collision_counts_;
    std::vector<int> node_danger_counts_;

    std::vector<long> prev_occupied_voxels_;
    std::vector<long> prev_danger_voxels_;

    std::vector<long> added_occ_, removed_occ_, added_dan_, removed_dan_;
};

} // namespace analysis
} // namespace robot_sim
