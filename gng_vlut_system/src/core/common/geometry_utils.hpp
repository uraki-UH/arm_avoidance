#pragma once
#include "common/voxel_utils.hpp"
#include "robot_model/robot_model.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"
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

} // namespace geometry
} // namespace common
