#pragma once

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <memory>
#include "spatial/index_voxel_grid.hpp"

namespace robot_sim {
namespace digital_twin {

enum class ReceptorType { NONE, EEF, LINK, DANGER };

struct ReceptorMatch {
    int node_id;
    ReceptorType type;
};

/**
 * @brief "Node as Receptor" Engine.
 * 
 * Maps end-effector (receptor) positions to configurations (nodes) that would collide
 * if an obstacle were present at those positions.
 */
class ReceptorNodeEngine {
public:
    ReceptorNodeEngine(double voxel_size) 
        : voxel_grid_(std::make_unique<::GNG::Analysis::IndexVoxelGrid>(voxel_size)) {}

    /**
     * @brief Clear the current index.
     */
    void clear() {
        voxel_grid_->clear();
        receptor_to_affected_nodes_.clear();
    }

    /**
     * @brief Associate a receptor position (usually a Node's EEF) with a set of 
     * configurations that would collide if an obstacle were at that position.
     * 
     * @param receptor_node_id The ID of the node whose EEF acts as the receptor.
     * @param pos The world position of the receptor.
     * @param affected_node_ids List of Node IDs that should "fire" when this receptor is hit.
     */
    void registerReceptor(int receptor_node_id, const Eigen::Vector3d& pos, 
                         const std::vector<int>& affected_node_ids) {
        // Map receptor position to affected nodes
        receptor_to_affected_nodes_[receptor_node_id] = affected_node_ids;
        
        // Add to spatial index using EEF as primary key
        // Note: Using float version of pos for IndexVoxelGrid
        voxel_grid_->insert(pos.cast<float>(), receptor_node_id);
    }

    /**
     * @brief Given a detected obstacle position (world frame), find and return
     * the IDs of all nodes whose configurations are affected.
     */
    std::vector<int> queryAffectedNodes(const Eigen::Vector3d& obstacle_pos) const {
        std::vector<int> affected_all;
        
        // 1. Find the receptor node(s) near the obstacle
        std::vector<int> hit_receptors = voxel_grid_->getIndicesAt(obstacle_pos.cast<float>());
        
        // 2. Aggregate all nodes affected by these hit receptors
        for (int receptor_id : hit_receptors) {
            auto it = receptor_to_affected_nodes_.find(receptor_id);
            if (it != receptor_to_affected_nodes_.end()) {
                affected_all.insert(affected_all.end(), it->second.begin(), it->second.end());
            }
        }
        
        return affected_all;
    }

private:
    std::unique_ptr<::GNG::Analysis::IndexVoxelGrid> voxel_grid_;
    
    // ReceptorID -> List of Config-Node IDs that collision-match this position
    std::unordered_map<int, std::vector<int>> receptor_to_affected_nodes_;
};

} // namespace digital_twin
} // namespace robot_sim
