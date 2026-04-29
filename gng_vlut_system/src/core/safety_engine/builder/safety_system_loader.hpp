#pragma once

#include <memory>
#include <string>
#include "safety_engine/gng/GrowingNeuralGas.hpp"
#include "safety_engine/indexing/dense_spatial_index.hpp"
#include "safety_engine/vlut/safety_vlut_mapper.hpp"
#include "safety_engine/vlut/voxel_processor.hpp"

namespace robot_sim {
namespace analysis {

struct SafetySystemContext {
    std::shared_ptr<::GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>> gng;
    std::shared_ptr<::robot_sim::analysis::ISpatialIndex> spatial_index;
    std::shared_ptr<::robot_sim::analysis::SafetyVlutMapper> mapper;
    std::shared_ptr<::robot_sim::analysis::VoxelProcessor> processor;

    size_t num_nodes = 0;
    double voxel_size = 0.02;

    void update(const std::vector<long>& occupied_vids, const std::vector<long>& danger_vids) {
        if (mapper) {
            mapper->updateFromVoxels(occupied_vids, danger_vids);
        }
    }
};

/**
 * @brief Standalone loader for the GNG/VLUT environment.
 * Decoupled from SimulationApp, making it ideal for ROS 2 nodes.
 */
class SafetySystemLoader {
public:
    static std::shared_ptr<SafetySystemContext> load(const std::string& gng_bin, 
                                                   const std::string& vlut_bin,
                                                   int angle_dim) {
        auto ctx = std::make_shared<SafetySystemContext>();

        // 1. Initialize GNG (without Kinematics for pure analysis node)
        ctx->gng = std::make_shared<::GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>>(angle_dim, 3, nullptr);
        if (!ctx->gng->load(gng_bin)) {
            std::cerr << "[SafetySystemLoader] Failed to load GNG: " << gng_bin << std::endl;
            return nullptr;
        }
        ctx->num_nodes = ctx->gng->getMaxNodeNum();

        // 2. Load Spatial Index (V-LUT)
        // Note: Default bounds will be overridden by the file header (Version 2)
        auto index = std::make_shared<::robot_sim::analysis::DenseSpatialIndex>(
            0.02, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
        
        if (!index->load(vlut_bin)) {
            std::cerr << "[SafetySystemLoader] Failed to load VLUT: " << vlut_bin << std::endl;
            return nullptr;
        }
        ctx->spatial_index = index;
        ctx->voxel_size = index->getVoxelSize();

        // 3. Initialize Mapper
        ctx->mapper = std::make_shared<::robot_sim::analysis::SafetyVlutMapper>();
        ctx->mapper->initialize(ctx->num_nodes, ctx->spatial_index);

        // 4. Initialize Processor
        ctx->processor = std::make_shared<::robot_sim::analysis::VoxelProcessor>(ctx->voxel_size);

        std::cout << "[SafetySystemLoader] Successfully initialized context with " 
                  << ctx->num_nodes << " nodes." << std::endl;
        
        return ctx;
    }
};

} // namespace analysis
} // namespace robot_sim
