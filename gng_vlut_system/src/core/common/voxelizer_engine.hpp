#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_set>
#include <vector>

#include "core/common/constants.hpp"
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

        Eigen::Vector3d corners[8];
        for (int i = 0; i < 8; ++i) {
            corners[i] = tf * Eigen::Vector3d((i & 1) ? half.x() : -half.x(),
                                              (i & 2) ? half.y() : -half.y(),
                                              (i & 4) ? half.z() : -half.z());
        }
        Eigen::Vector3d w_min = corners[0], w_max = corners[0];
        for (int i = 1; i < 8; ++i) {
            w_min = w_min.cwiseMin(corners[i]);
            w_max = w_max.cwiseMax(corners[i]);
        }

        Eigen::Vector3i min_idx =
            ::common::geometry::VoxelUtils::worldToVoxel(w_min.cast<float>(),
                                                         static_cast<float>(v_size));
        Eigen::Vector3i max_idx =
            ::common::geometry::VoxelUtils::worldToVoxel(w_max.cast<float>(),
                                                         static_cast<float>(v_size));

        Eigen::Isometry3d inv_tf = tf.inverse();
        for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
            for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
                for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
                    Eigen::Vector3i idx(x, y, z);
                    Eigen::Vector3d wp =
                        ::common::geometry::VoxelUtils::voxelToWorld(idx, static_cast<float>(v_size))
                            .template cast<double>();
                    Eigen::Vector3d lp = inv_tf * wp;

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

        Eigen::Vector3i min_idx =
            ::common::geometry::VoxelUtils::worldToVoxel(w_min.cast<float>(),
                                                         static_cast<float>(v_size));
        Eigen::Vector3i max_idx =
            ::common::geometry::VoxelUtils::worldToVoxel(w_max.cast<float>(),
                                                         static_cast<float>(v_size));

        double r2 = (radius + ::robot_sim::common::Constants::GEOM_EPSILON) *
                    (radius + ::robot_sim::common::Constants::GEOM_EPSILON);
        for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
            for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
                for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
                    Eigen::Vector3i idx(x, y, z);
                    Eigen::Vector3d wp =
                        ::common::geometry::VoxelUtils::voxelToWorld(idx, static_cast<float>(v_size))
                            .template cast<double>();
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

        Eigen::Vector3d center = tf.translation();
        double diag = std::sqrt(radius * radius + h_half * h_half);
        Eigen::Vector3d w_min = center - Eigen::Vector3d::Constant(diag);
        Eigen::Vector3d w_max = center + Eigen::Vector3d::Constant(diag);

        Eigen::Vector3i min_idx =
            ::common::geometry::VoxelUtils::worldToVoxel(w_min.cast<float>(),
                                                         static_cast<float>(v_size));
        Eigen::Vector3i max_idx =
            ::common::geometry::VoxelUtils::worldToVoxel(w_max.cast<float>(),
                                                         static_cast<float>(v_size));

        Eigen::Isometry3d inv_tf = tf.inverse();
        double r2 = (radius + ::robot_sim::common::Constants::GEOM_EPSILON) *
                    (radius + ::robot_sim::common::Constants::GEOM_EPSILON);

        for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
            for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
                for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
                    Eigen::Vector3i idx(x, y, z);
                    Eigen::Vector3d wp =
                        ::common::geometry::VoxelUtils::voxelToWorld(idx, static_cast<float>(v_size))
                            .template cast<double>();
                    Eigen::Vector3d lp = inv_tf * wp;

                    if (std::abs(lp.z()) <= h_half + ::robot_sim::common::Constants::GEOM_EPSILON) {
                        if (lp.x() * lp.x() + lp.y() * lp.y() <= r2) {
                            vids.insert(grid.getFlatVoxelId(idx));
                        }
                    }
                }
            }
        }
    }

    /**
     * @brief 三角形メッシュ外形の保守的ボクセル化
     */
    static void voxelizeMeshTriangles(const std::vector<Eigen::Vector3d>& vertices,
                                      const ::GNG::Analysis::IndexVoxelGrid& grid,
                                      std::unordered_set<long>& vids) {
        const double voxel_size = grid.getVoxelSize();
        const double voxel_half = voxel_size * 0.5;
        const double bbox_margin = voxel_half +
                                   ::robot_sim::common::Constants::GEOM_EPSILON;
        for (size_t t = 0; t < vertices.size() / 3; ++t) {
            const Eigen::Vector3d& v0 = vertices[t * 3];
            const Eigen::Vector3d& v1 = vertices[t * 3 + 1];
            const Eigen::Vector3d& v2 = vertices[t * 3 + 2];

            const Eigen::Vector3d triangle_min =
                v0.cwiseMin(v1).cwiseMin(v2) -
                Eigen::Vector3d::Constant(bbox_margin);
            const Eigen::Vector3d triangle_max =
                v0.cwiseMax(v1).cwiseMax(v2) +
                Eigen::Vector3d::Constant(bbox_margin);
            const Eigen::Vector3i min_idx =
                ::common::geometry::VoxelUtils::worldToVoxel(
                    triangle_min.cast<float>(), static_cast<float>(voxel_size));
            const Eigen::Vector3i max_idx =
                ::common::geometry::VoxelUtils::worldToVoxel(
                    triangle_max.cast<float>(), static_cast<float>(voxel_size));

            for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
                for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
                    for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
                        const Eigen::Vector3i idx(x, y, z);
                        const Eigen::Vector3d center =
                            ::common::geometry::VoxelUtils::voxelToWorld(
                                idx, static_cast<float>(voxel_size))
                                .template cast<double>();
                        if (hasTriangleVoxelOverlap(v0, v1, v2, center,
                                                    voxel_half)) {
                            vids.insert(grid.getFlatVoxelId(idx));
                        }
                    }
                }
            }
        }
    }

    /**
     * @brief ローカルセルを剛体変換後のグリッドへ保守的に再収録
     *
     * 回転したリンクでセル中心だけを再量子化すると、元の表面セル間に
     * 出力グリッド上の隙間が発生する問題への対応。
     */
    static void appendTransformedVoxelCell(
        const Eigen::Vector3d& local_center,
        const Eigen::Isometry3d& local_to_target,
        const ::GNG::Analysis::IndexVoxelGrid& grid,
        std::vector<long>& voxel_ids) {
        const double voxel_size = grid.getVoxelSize();
        const double voxel_half = voxel_size * 0.5;
        const Eigen::Vector3d target_center = local_to_target * local_center;
        const Eigen::Matrix3d rotation = local_to_target.rotation();

        Eigen::Vector3d source_extent = Eigen::Vector3d::Zero();
        for (int axis_idx = 0; axis_idx < 3; ++axis_idx) {
            source_extent[axis_idx] =
                voxel_half * rotation.row(axis_idx).cwiseAbs().sum();
        }

        const Eigen::Vector3d candidate_min =
            target_center - source_extent -
            Eigen::Vector3d::Constant(voxel_half);
        const Eigen::Vector3d candidate_max =
            target_center + source_extent +
            Eigen::Vector3d::Constant(voxel_half);
        const Eigen::Vector3i min_idx(
            minVoxelIdxWithPositiveOverlap(candidate_min.x(), voxel_size),
            minVoxelIdxWithPositiveOverlap(candidate_min.y(), voxel_size),
            minVoxelIdxWithPositiveOverlap(candidate_min.z(), voxel_size));
        const Eigen::Vector3i max_idx(
            maxVoxelIdxWithPositiveOverlap(candidate_max.x(), voxel_size),
            maxVoxelIdxWithPositiveOverlap(candidate_max.y(), voxel_size),
            maxVoxelIdxWithPositiveOverlap(candidate_max.z(), voxel_size));

        for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
            for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
                for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
                    const Eigen::Vector3i idx(x, y, z);
                    const Eigen::Vector3d voxel_center =
                        ::common::geometry::VoxelUtils::voxelToWorld(
                            idx, static_cast<float>(voxel_size))
                            .template cast<double>();
                    if (hasOrientedVoxelOverlap(
                            target_center, rotation, voxel_half, voxel_center,
                            voxel_half)) {
                        voxel_ids.push_back(grid.getFlatVoxelId(idx));
                    }
                }
            }
        }
    }

    /**
     * @brief 回転を伴わないグリッド整列姿勢の判定
     */
    static bool isVoxelGridAlignedTransform(
        const Eigen::Isometry3d& local_to_target) {
        constexpr double alignment_eps = 1e-9;
        const Eigen::Matrix3d rotation = local_to_target.rotation();
        for (int column_idx = 0; column_idx < 3; ++column_idx) {
            int aligned_axis_num = 0;
            for (int row_idx = 0; row_idx < 3; ++row_idx) {
                const double component = std::abs(rotation(row_idx, column_idx));
                if (std::abs(component - 1.0) <= alignment_eps) {
                    ++aligned_axis_num;
                } else if (component > alignment_eps) {
                    return false;
                }
            }
            if (aligned_axis_num != 1) {
                return false;
            }
        }
        return true;
    }

private:
    /**
     * @brief 正の体積交差を持つ最小ボクセル添字
     */
    static int minVoxelIdxWithPositiveOverlap(double min_value,
                                               double voxel_size) {
        constexpr double index_eps = 1e-9;
        return static_cast<int>(std::floor(
                   min_value / voxel_size - 0.5 + index_eps)) +
               1;
    }

    /**
     * @brief 正の体積交差を持つ最大ボクセル添字
     */
    static int maxVoxelIdxWithPositiveOverlap(double max_value,
                                               double voxel_size) {
        constexpr double index_eps = 1e-9;
        return static_cast<int>(std::ceil(
                   max_value / voxel_size - 0.5 - index_eps)) -
               1;
    }

    /**
     * @brief 回転セルと軸整列セルの分離軸判定
     */
    static bool hasOrientedVoxelOverlap(
        const Eigen::Vector3d& source_center,
        const Eigen::Matrix3d& source_rotation, double source_half,
        const Eigen::Vector3d& target_center, double target_half) {
        const Eigen::Vector3d center_delta = target_center - source_center;
        const std::array<Eigen::Vector3d, 3> source_axes{
            source_rotation.col(0), source_rotation.col(1),
            source_rotation.col(2)};
        const std::array<Eigen::Vector3d, 3> target_axes{
            Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
            Eigen::Vector3d::UnitZ()};

        const auto overlaps_axis =
            [&center_delta, &source_axes, source_half, target_half](
                const Eigen::Vector3d& axis) {
                if (axis.squaredNorm() <=
                    ::robot_sim::common::Constants::GEOM_EPSILON) {
                    return true;
                }
                double source_radius = 0.0;
                for (const auto& source_axis : source_axes) {
                    source_radius += std::abs(axis.dot(source_axis));
                }
                source_radius *= source_half;
                const double target_radius =
                    target_half * axis.cwiseAbs().sum();
                return std::abs(axis.dot(center_delta)) <
                       source_radius + target_radius -
                           ::robot_sim::common::Constants::GEOM_EPSILON;
            };

        for (const auto& source_axis : source_axes) {
            if (!overlaps_axis(source_axis)) {
                return false;
            }
        }
        for (const auto& target_axis : target_axes) {
            if (!overlaps_axis(target_axis)) {
                return false;
            }
        }
        for (const auto& source_axis : source_axes) {
            for (const auto& target_axis : target_axes) {
                if (!overlaps_axis(source_axis.cross(target_axis))) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * @brief 三角形と軸整列ボクセルの分離軸判定
     */
    static bool hasTriangleVoxelOverlap(const Eigen::Vector3d& v0,
                                        const Eigen::Vector3d& v1,
                                        const Eigen::Vector3d& v2,
                                        const Eigen::Vector3d& voxel_center,
                                        double voxel_half) {
        const Eigen::Vector3d local_v0 = v0 - voxel_center;
        const Eigen::Vector3d local_v1 = v1 - voxel_center;
        const Eigen::Vector3d local_v2 = v2 - voxel_center;
        const std::array<Eigen::Vector3d, 3> local_vertices{
            local_v0, local_v1, local_v2};
        const std::array<Eigen::Vector3d, 3> triangle_edges{
            local_v1 - local_v0, local_v2 - local_v1,
            local_v0 - local_v2};
        const std::array<Eigen::Vector3d, 3> box_axes{
            Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
            Eigen::Vector3d::UnitZ()};

        const auto overlaps_axis = [&local_vertices, voxel_half](
                                       const Eigen::Vector3d& axis) {
            if (axis.squaredNorm() <=
                ::robot_sim::common::Constants::GEOM_EPSILON) {
                return true;
            }
            double min_projection = axis.dot(local_vertices[0]);
            double max_projection = min_projection;
            for (std::size_t i = 1; i < local_vertices.size(); ++i) {
                const double projection = axis.dot(local_vertices[i]);
                min_projection = std::min(min_projection, projection);
                max_projection = std::max(max_projection, projection);
            }
            const double voxel_radius =
                voxel_half * axis.cwiseAbs().sum();
            return min_projection <= voxel_radius +
                                       ::robot_sim::common::Constants::GEOM_EPSILON &&
                   max_projection >= -voxel_radius -
                                       ::robot_sim::common::Constants::GEOM_EPSILON;
        };

        for (const Eigen::Vector3d& box_axis : box_axes) {
            if (!overlaps_axis(box_axis)) {
                return false;
            }
        }
        for (const Eigen::Vector3d& triangle_edge : triangle_edges) {
            for (const Eigen::Vector3d& box_axis : box_axes) {
                if (!overlaps_axis(triangle_edge.cross(box_axis))) {
                    return false;
                }
            }
        }

        const Eigen::Vector3d triangle_normal =
            triangle_edges[0].cross(triangle_edges[1]);
        return overlaps_axis(triangle_normal);
    }
};

} // namespace common
} // namespace robot_sim
