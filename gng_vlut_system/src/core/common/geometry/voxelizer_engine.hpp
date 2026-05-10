#pragma once

#include <Eigen/Dense>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <cmath>

#include "common/voxel_utils.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"

namespace robot_sim {
namespace common {

/**
 * @brief 幾何形状を数学的サンプリングに基づきボクセル化するエンジン
 * ロボットモデルやROSから独立した、純粋な幾何学処理ライブラリ
 */
class VoxelizerEngine {
public:
    /**
     * @brief Boxをボクセル化
     */
    static void voxelizeBox(const Eigen::Vector3d& size, const Eigen::Isometry3d& tf, 
                           const ::GNG::Analysis::IndexVoxelGrid& grid, 
                           std::unordered_set<long>& vids) {
        double v_size = grid.getVoxelSize();
        Eigen::Vector3d half = size * 0.5;
        double eps = v_size * ::robot_sim::common::Constants::BOUNDARY_EPSILON_SCALE;
        for (double x = -half.x(); x <= half.x() + eps; x += v_size) {
            for (double y = -half.y(); y <= half.y() + eps; y += v_size) {
                for (double z = -half.z(); z <= half.z() + eps; z += v_size) {
                    Eigen::Vector3d p = tf * Eigen::Vector3d(x, y, z);
                    vids.insert(grid.getFlatVoxelId(
                        ::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), (float)v_size)));
                }
            }
        }
    }

    /**
     * @brief Sphereをボクセル化
     */
    static void voxelizeSphere(double radius, const Eigen::Isometry3d& tf, 
                               const ::GNG::Analysis::IndexVoxelGrid& grid, 
                               std::unordered_set<long>& vids) {
        double v_size = grid.getVoxelSize();
        double r2 = radius * radius;
        double eps = v_size * ::robot_sim::common::Constants::BOUNDARY_EPSILON_SCALE;
        for (double x = -radius; x <= radius + eps; x += v_size) {
            for (double y = -radius; y <= radius + eps; y += v_size) {
                for (double z = -radius; z <= radius + eps; z += v_size) {
                    if (x*x + y*y + z*z <= r2 + ::robot_sim::common::Constants::GEOM_EPSILON) {
                        Eigen::Vector3d p = tf * Eigen::Vector3d(x, y, z);
                        vids.insert(grid.getFlatVoxelId(
                            ::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), (float)v_size)));
                    }
                }
            }
        }
    }

    /**
     * @brief Cylinderをボクセル化
     */
    static void voxelizeCylinder(double radius, double length, const Eigen::Isometry3d& tf, 
                                const ::GNG::Analysis::IndexVoxelGrid& grid, 
                                std::unordered_set<long>& vids) {
        double v_size = grid.getVoxelSize();
        double r2 = radius * radius;
        double h_half = length * 0.5;
        double eps = v_size * ::robot_sim::common::Constants::BOUNDARY_EPSILON_SCALE;
        for (double z = -h_half; z <= h_half + eps; z += v_size) {
            for (double x = -radius; x <= radius + eps; x += v_size) {
                for (double y = -radius; y <= radius + eps; y += v_size) {
                    if (x*x + y*y <= r2 + ::robot_sim::common::Constants::GEOM_EPSILON) {
                        Eigen::Vector3d p = tf * Eigen::Vector3d(x, y, z);
                        vids.insert(grid.getFlatVoxelId(
                            ::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), (float)v_size)));
                    }
                }
            }
        }
    }

    /**
     * @brief 三角形メッシュのリストをボクセル化
     */
    static void voxelizeMeshTriangles(const std::vector<Eigen::Vector3d>& vertices, 
                                     const ::GNG::Analysis::IndexVoxelGrid& grid, 
                                     std::unordered_set<long>& vids) {
        double v_size = grid.getVoxelSize();
        for (size_t t = 0; t < vertices.size() / 3; ++t) {
            const Eigen::Vector3d& v0 = vertices[t*3];
            const Eigen::Vector3d& v1 = vertices[t*3+1];
            const Eigen::Vector3d& v2 = vertices[t*3+2];
            
            double d12 = (v1-v0).norm(), d13 = (v2-v0).norm();
            int s1 = std::max(1, (int)std::ceil(d12/v_size)), s2 = std::max(1, (int)std::ceil(d13/v_size));
            for(int i1=0; i1<=s1; ++i1) {
                double t1 = (double)i1/s1;
                for(int i2=0; i2<=std::max(0, (int)(s2*(1.0-t1))); ++i2) {
                    double t2 = (double)i2/s2;
                    Eigen::Vector3d p = v0 + t1*(v1-v0) + t2*(v2-v0);
                    vids.insert(grid.getFlatVoxelId(
                        ::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), (float)v_size)));
                }
            }
        }
    }
};

} // namespace common
} // namespace robot_sim
