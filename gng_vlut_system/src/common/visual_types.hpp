#pragma once

#include <Eigen/Dense>
#include <vector>

namespace simulation {

/**
 * @brief Simple data structures for GNG visualization.
 * Rescued from legacy simulation module.
 */
struct VisualNode {
    int id;
    Eigen::Vector3f position;
    float level;
    bool is_surface = false;
    bool is_active_surface = false;
    bool active = true;
    bool is_collision = false;
    bool is_hazard = false;
    bool is_influence = false;
    bool is_path = false;
    bool is_mainland = true;
    bool is_island = false;
    Eigen::Vector4f color = {1.f, 1.f, 1.f, 1.f};
};

struct VisualEdge {
    int node1_id;
    int node2_id;
    float level;
    bool is_path = false;
    Eigen::Vector4f color = {0.5f, 0.5f, 0.5f, 0.5f};
};

} // namespace simulation
