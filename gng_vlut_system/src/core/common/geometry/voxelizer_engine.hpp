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
     * @brief Boxをボクセル化 (AABB + 包含判定)
     */
    static void voxelizeBox(const Eigen::Vector3d& size, const Eigen::Isometry3d& tf, 
                           const ::GNG::Analysis::IndexVoxelGrid& grid, 
                           std::unordered_set<long>& vids) {
        double v_size = grid.getVoxelSize();
        Eigen::Vector3d half = size * 0.5;

        // 1. ワールド座標系でのAABBを計算
        Eigen::Vector3d corners[8];
        for(int i=0; i<8; ++i) {
            corners[i] = tf * Eigen::Vector3d((i&1)?half.x():-half.x(), (i&2)?half.y():-half.y(), (i&4)?half.z():-half.z());
        }
        Eigen::Vector3d w_min = corners[0], w_max = corners[0];
        for(int i=1; i<8; ++i) {
            w_min = w_min.cwiseMin(corners[i]);
            w_max = w_max.cwiseMax(corners[i]);
        }

        // 2. ボクセルインデックスの範囲を特定
        Eigen::Vector3i min_idx = ::common::geometry::VoxelUtils::worldToVoxel(w_min.cast<float>(), (float)v_size);
        Eigen::Vector3i max_idx = ::common::geometry::VoxelUtils::worldToVoxel(w_max.cast<float>(), (float)v_size);

        // 3. 範囲内のボクセルを走査
        Eigen::Isometry3d inv_tf = tf.inverse();
        for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
            for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
                for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
                    Eigen::Vector3i idx(x, y, z);
                    Eigen::Vector3d wp = ::common::geometry::VoxelUtils::voxelToWorld(idx, (float)v_size).template cast<double>();
                    Eigen::Vector3d lp = inv_tf * wp; // ローカル座標に変換

                    if (std::abs(lp.x()) <= half.x() + ::robot_sim::common::Constants::GEOM_EPSILON &&
                        std::abs(lp.y()) <= half.y() + ::robot_sim::common::Constants::GEOM_EPSILON &&
                        std::abs(lp.z()) <= half.z() + ::robot_sim::common::Constants::GEOM_EPSILON) {
                        vids.insert(grid.getFlatVoxelId(idx));
                    }
                }
            }
        }
    }

    /**
     * @brief Sphereをボクセル化 (AABB + 包含判定)
     */
    static void voxelizeSphere(double radius, const Eigen::Isometry3d& tf, 
                               const ::GNG::Analysis::IndexVoxelGrid& grid, 
                               std::unordered_set<long>& vids) {
        double v_size = grid.getVoxelSize();
        Eigen::Vector3d center = tf.translation();
        Eigen::Vector3d w_min = center - Eigen::Vector3d::Constant(radius);
        Eigen::Vector3d w_max = center + Eigen::Vector3d::Constant(radius);

        Eigen::Vector3i min_idx = ::common::geometry::VoxelUtils::worldToVoxel(w_min.cast<float>(), (float)v_size);
        Eigen::Vector3i max_idx = ::common::geometry::VoxelUtils::worldToVoxel(w_max.cast<float>(), (float)v_size);

        double r2 = (radius + ::robot_sim::common::Constants::GEOM_EPSILON) * (radius + ::robot_sim::common::Constants::GEOM_EPSILON);
        for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
            for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
                for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
                    Eigen::Vector3i idx(x, y, z);
                    Eigen::Vector3d wp = ::common::geometry::VoxelUtils::voxelToWorld(idx, (float)v_size).template cast<double>();
                    if ((wp - center).squaredNorm() <= r2) {
                        vids.insert(grid.getFlatVoxelId(idx));
                    }
                }
            }
        }
    }

    /**
     * @brief Cylinderをボクセル化 (AABB + 包含判定)
     */
    static void voxelizeCylinder(double radius, double length, const Eigen::Isometry3d& tf, 
                                const ::GNG::Analysis::IndexVoxelGrid& grid, 
                                std::unordered_set<long>& vids) {
        double v_size = grid.getVoxelSize();
        double h_half = length * 0.5;

        // AABB (simplified)
        Eigen::Vector3d center = tf.translation();
        double diag = std::sqrt(radius*radius + h_half*h_half);
        Eigen::Vector3d w_min = center - Eigen::Vector3d::Constant(diag);
        Eigen::Vector3d w_max = center + Eigen::Vector3d::Constant(diag);

        Eigen::Vector3i min_idx = ::common::geometry::VoxelUtils::worldToVoxel(w_min.cast<float>(), (float)v_size);
        Eigen::Vector3i max_idx = ::common::geometry::VoxelUtils::worldToVoxel(w_max.cast<float>(), (float)v_size);

        Eigen::Isometry3d inv_tf = tf.inverse();
        double r2 = (radius + ::robot_sim::common::Constants::GEOM_EPSILON) * (radius + ::robot_sim::common::Constants::GEOM_EPSILON);

        for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
            for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
                for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
                    Eigen::Vector3i idx(x, y, z);
                    Eigen::Vector3d wp = ::common::geometry::VoxelUtils::voxelToWorld(idx, (float)v_size).template cast<double>();
                    Eigen::Vector3d lp = inv_tf * wp;

                    if (std::abs(lp.z()) <= h_half + ::robot_sim::common::Constants::GEOM_EPSILON) {
                        if (lp.x()*lp.x() + lp.y()*lp.y() <= r2) {
                            vids.insert(grid.getFlatVoxelId(idx));
                        }
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
        double step = v_size * 0.5;
        for (size_t t = 0; t < vertices.size() / 3; ++t) {
            const Eigen::Vector3d& v0 = vertices[t*3];
            const Eigen::Vector3d& v1 = vertices[t*3+1];
            const Eigen::Vector3d& v2 = vertices[t*3+2];
            
            double d12 = (v1-v0).norm(), d13 = (v2-v0).norm();
            int s1 = std::max(1, (int)std::ceil(d12/step)), s2 = std::max(1, (int)std::ceil(d13/step));
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
