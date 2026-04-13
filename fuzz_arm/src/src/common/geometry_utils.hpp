#pragma once

#include "simulation/robot/robot_model.hpp"
#include "spatial/index_voxel_grid.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <vector>

namespace common {
namespace geometry {

/**
 * @brief
 * GJKアルゴリズムで使用する、凸形状のサポート写像（最も遠い点）を計算するためのクラス
 */
class SupportProvider {
public:
  virtual ~SupportProvider() = default;

  /**
   * @brief 指定された方向(world)に対して、形状の最も遠い点(world)を返す
   */
  virtual Eigen::Vector3d support(const Eigen::Vector3d &direction) const = 0;
};

/**
 * @brief simulation::Geometry に対応するサポート写像実装
 */
class PrimitiveSupport : public SupportProvider {
public:
  PrimitiveSupport(const simulation::Geometry &geom,
                   const Eigen::Isometry3d &transform)
      : geom_(geom), transform_(transform) {}

  Eigen::Vector3d support(const Eigen::Vector3d &direction) const override;

private:
  simulation::Geometry geom_;
  Eigen::Isometry3d transform_;
};

/**
 * @brief 単一のボクセル（AABB）に対応するサポート写像実装
 */
class VoxelSupport : public SupportProvider {
public:
  VoxelSupport(const Eigen::Vector3d &center, double size)
      : center_(center), extents_(Eigen::Vector3d::Constant(size * 0.5)) {}

  Eigen::Vector3d support(const Eigen::Vector3d &direction) const override;

private:
  Eigen::Vector3d center_;
  Eigen::Vector3d extents_;
};

/**
 * @brief GJKアルゴリズムを用いて2つの凸形状が干渉しているかを判定する
 */
class GJK {
public:
  static bool intersect(const SupportProvider &shapeA,
                        const SupportProvider &shapeB, int max_iterations = 32);

private:
  // ミンコフスキー差のサポート写像を計算するヘルパー
  static Eigen::Vector3d minkowskiSupport(const SupportProvider &shapeA,
                                          const SupportProvider &shapeB,
                                          const Eigen::Vector3d &direction);
};

/**
 * @brief Utilities for conversion between world coordinates and voxel indices.
 */
class VoxelUtils {
public:
  static Eigen::Vector3i worldToVoxel(const Eigen::Vector3f &pos,
                                      float voxel_size) {
    return (pos.array() / voxel_size).floor().cast<int>();
  }

  static Eigen::Vector3f voxelToWorld(const Eigen::Vector3i &idx,
                                      float voxel_size) {
    return idx.cast<float>() * voxel_size +
           Eigen::Vector3f::Constant(voxel_size * 0.5f);
  }

  /**
   * @brief Get a set of flat voxel IDs covered by a sphere.
   */
  template <typename IdProvider>
  static std::vector<long> getSphereVoxels(const Eigen::Vector3f &center,
                                           float radius, float voxel_size,
                                           IdProvider id_provider) {
    std::vector<long> voxels;
    Eigen::Vector3i min_idx =
        worldToVoxel(center - Eigen::Vector3f::Constant(radius), voxel_size);
    Eigen::Vector3i max_idx =
        worldToVoxel(center + Eigen::Vector3f::Constant(radius), voxel_size);

    int nx = max_idx.x() - min_idx.x() + 1;
    int ny = max_idx.y() - min_idx.y() + 1;
    int nz = max_idx.z() - min_idx.z() + 1;
    if (nx > 0 && ny > 0 && nz > 0) {
      voxels.reserve(nx * ny * nz);
    }

    float r_sq = radius * radius;
    float half_v = voxel_size * 0.5f;

    // Pre-calculate world bounds aligned to voxel centers
    float z_world_base = min_idx.z() * voxel_size + half_v - center.z();
    float y_world_base = min_idx.y() * voxel_size + half_v - center.y();
    float x_world_base = min_idx.x() * voxel_size + half_v - center.x();

    for (int z = min_idx.z(), k = 0; z <= max_idx.z(); ++z, ++k) {
      float dz = z_world_base + k * voxel_size;
      float dz_sq = dz * dz;

      for (int y = min_idx.y(), j = 0; y <= max_idx.y(); ++y, ++j) {
        float dy = y_world_base + j * voxel_size;
        float dzy_sq = dz_sq + dy * dy;

        // Skip entire X row if Z and Y distance already exceeds radius
        if (dzy_sq > r_sq + (nx * voxel_size * nx * voxel_size)) {
          // Rough heuristic, but if completely out of bounds skip early
        }

        for (int x = min_idx.x(), i = 0; x <= max_idx.x(); ++x, ++i) {
          float dx = x_world_base + i * voxel_size;
          if (dzy_sq + dx * dx <= r_sq) {
            Eigen::Vector3i current_idx(x, y, z);
            long flat_id = id_provider(current_idx);
            if (flat_id >= 0)
              voxels.push_back(flat_id);
          }
        }
      }
    }
    return voxels;
  }

  /**
   * @brief Compute the centroid of a set of voxels.
   */
  static Eigen::Vector3f calculateCentroid(const std::vector<long> &voxels,
                                           float voxel_size) {
    if (voxels.empty())
      return Eigen::Vector3f::Zero();
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    for (long vid : voxels) {
      Eigen::Vector3i idx =
          ::GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);
      sum += (idx.cast<float>() + Eigen::Vector3f::Constant(0.5f)) * voxel_size;
    }
    return sum / (float)voxels.size();
  }

  /**
   * @brief Compute added and removed voxels between two sets.
   */
  static void computeDiff(const std::vector<long> &old_voxels,
                          const std::vector<long> &new_voxels,
                          std::vector<long> &added,
                          std::vector<long> &removed) {
    std::vector<long> sorted_old = old_voxels;
    std::vector<long> sorted_new = new_voxels;
    std::sort(sorted_old.begin(), sorted_old.end());
    std::sort(sorted_new.begin(), sorted_new.end());

    std::set_difference(sorted_old.begin(), sorted_old.end(),
                        sorted_new.begin(), sorted_new.end(),
                        std::back_inserter(removed));
    std::set_difference(sorted_new.begin(), sorted_new.end(),
                        sorted_old.begin(), sorted_old.end(),
                        std::back_inserter(added));
  }
};

} // namespace geometry
} // namespace common
