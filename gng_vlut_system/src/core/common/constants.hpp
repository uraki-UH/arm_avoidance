#pragma once

namespace robot_sim {
namespace common {

/**
 * @brief System-wide default constants for the GNG safety engine.
 * Consolidated here to avoid magic numbers scattered across the codebase.
 */
struct Constants {
    // Spatial Defaults
    static constexpr double DEFAULT_VOXEL_SIZE = 0.02;
    static constexpr float  DEFAULT_VOXEL_SIZE_F = 0.02f;
    static constexpr double DEFAULT_SAFETY_MARGIN = 0.05;
    
    // Voxel Indexing Defaults
    static constexpr int DEFAULT_X_SHIFT = 42;
    static constexpr int DEFAULT_Y_SHIFT = 21;
    static constexpr int DEFAULT_Z_SHIFT = 0;
    static constexpr long DEFAULT_OFFSET = 1000000L;

    // Node Defaults
    static constexpr double DEFAULT_UPDATE_HZ = 20.0;
    static constexpr double DEFAULT_VIEWER_HZ = 30.0;
    static constexpr const char* DEFAULT_BASE_FRAME = "base_link";
    static constexpr const char* DEFAULT_FOOTPRINT_FRAME = "base_footprint";
    static constexpr const char* DEFAULT_WORLD_FRAME = "world";
    static constexpr const char* DEFAULT_MASK_TOPIC = "/self_recognition/voxel_mask";

    // Numerical Tolerances
    static constexpr double GEOM_EPSILON = 1e-7;
    static constexpr double BOUNDARY_EPSILON_SCALE = 0.1;
    static constexpr double JOINT_LIMIT_TOLERANCE = 1e-4;
    static constexpr double NANO_TO_SEC = 1e-9;

    // PSO (Particle Swarm Optimization) Parameters
    static constexpr double PSO_INERTIA_WEIGHT = 0.7;
    static constexpr double PSO_COGNITIVE_WEIGHT = 0.8;
    static constexpr double PSO_SOCIAL_WEIGHT = 0.9;
    static constexpr double PSO_VELOCITY_SCALE = 0.1;
};

} // namespace common
} // namespace robot_sim
