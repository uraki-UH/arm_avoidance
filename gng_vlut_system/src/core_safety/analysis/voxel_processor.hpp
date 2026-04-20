#pragma once

#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include "common/voxel_utils.hpp"
#include "core_safety/spatial/index_voxel_grid.hpp"

namespace robot_sim {
namespace analysis {

/**
 * @brief High-performance processor for spatial environment data.
 * Converts raw coordinates (points) or occupancy grids into GNG-ready voxel IDs.
 * Optimized for ROS 2 standalone usage without simulation dependencies.
 */
class VoxelProcessor {
public:
    VoxelProcessor(double voxel_size) : voxel_size_(voxel_size) {}

    /**
     * @brief Converts a set of 3D points into a unique list of voxel IDs.
     * Uses Vector-Sort-Unique for O(N log N) performance without hash overhead.
     */
    std::vector<long> voxelize(const std::vector<Eigen::Vector3f>& points) {
        std::vector<long> vids;
        vids.reserve(points.size());

        for (const auto& p : points) {
            Eigen::Vector3i idx = (p / (float)voxel_size_).array().floor().cast<int>();
            vids.push_back(::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx));
        }

        std::sort(vids.begin(), vids.end());
        vids.erase(std::unique(vids.begin(), vids.end()), vids.end());
        return vids;
    }

    /**
     * @brief Generates danger voxels by dilating occupied voxels by a given radius.
     * @param occupied_vids Primary occupied voxel IDs.
     * @param radius Dilation radius in meters.
     * @return Unique list of danger voxel IDs (may overlap with occupied).
     */
    std::vector<long> dilate(const std::vector<long>& occupied_vids, float radius) {
        if (radius <= 0.0f) return {};

        std::vector<long> danger_vids;
        danger_vids.reserve(occupied_vids.size() * 27); // Heuristic for 1-voxel dilation

        for (long vid : occupied_vids) {
            Eigen::Vector3i center_idx = ::GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);
            Eigen::Vector3f center_pos = (center_idx.cast<float>() + Eigen::Vector3f::Constant(0.5f)) * (float)voxel_size_;

            // Use the sphere generator utility
            auto sphere_vids = ::common::geometry::VoxelUtils::getSphereVoxels(
                center_pos, radius, (float)voxel_size_, 
                [](const Eigen::Vector3i& idx) {
                    return ::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);
                });
            
            danger_vids.insert(danger_vids.end(), sphere_vids.begin(), sphere_vids.end());
        }

        // Deduplicate the resulting danger field
        std::sort(danger_vids.begin(), danger_vids.end());
        danger_vids.erase(std::unique(danger_vids.begin(), danger_vids.end()), danger_vids.end());
        
        return danger_vids;
    }

private:
    double voxel_size_;
};

} // namespace analysis
} // namespace robot_sim
