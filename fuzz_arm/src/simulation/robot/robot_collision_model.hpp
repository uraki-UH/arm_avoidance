#pragma once

#include "simulation/robot/robot_model.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

namespace simulation {

/**
 * @brief リンクごとの衝突判定形状と、ジョイントからのオフセットを保持する構造体
 */
struct LinkCollisionGeometry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string link_name;
  int parent_joint_index; // KinematicChainにおけるジョイントインデックス
  Geometry geometry;      // 形状情報
  Eigen::Isometry3d origin_offset; // ジョイント原点から形状中心までのオフセット
                                   // (URDF <origin>)
  Eigen::Isometry3d inertial_origin; // ジョイント原点からCOMまでのオフセット
                                     // (URDF <inertial><origin>)

  LinkCollisionGeometry()
      : parent_joint_index(-1), origin_offset(Eigen::Isometry3d::Identity()),
        inertial_origin(Eigen::Isometry3d::Identity()) {}
};

/**
 * @brief ロボットの全衝突形状を管理するクラス
 *        KinematicChainの計算結果（各ジョイントのワールド座標）を受け取り、
 *        各リンクの衝突形状のワールド位置を計算する役割を担う。
 */
class RobotCollisionModel {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void addCollisionGeometry(const LinkCollisionGeometry &geom) {
    geometries_.push_back(geom);
  }

  const std::vector<LinkCollisionGeometry> &getGeometries() const {
    return geometries_;
  }

  /**
   * @brief
   * ジョイントのワールド変換行列の配列を受け取り、各衝突形状のワールド変換行列を計算する
   * @param joint_transforms KinematicChainから取得した[Base, J1, J2, ...,
   * Tip]の変換行列
   * @return 衝突形状ごとのワールド変換行列
   */
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
  calculateCollisionTransforms(
      const std::vector<Eigen::Isometry3d,
                        Eigen::aligned_allocator<Eigen::Isometry3d>>
          &joint_transforms) const {

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
        collision_tfs;
    collision_tfs.reserve(geometries_.size());

    for (const auto &geom : geometries_) {
      if (geom.parent_joint_index >= 0 &&
          geom.parent_joint_index < (int)joint_transforms.size()) {
        // ODE側と同じ計算（物理シミュレーションのため、ボディ基準で配置）:
        // 1. ボディ（COM）のワールド変換 = ジョイント変換 * inertial.origin
        // 2. ジオメトリのボディ基準オフセット = inertial.origin.inverse() *
        // collision.origin
        // 3. ジオメトリのワールド変換 = ボディ変換 * ボディ基準オフセット
        Eigen::Isometry3d body_world =
            joint_transforms[geom.parent_joint_index] * geom.inertial_origin;
        Eigen::Isometry3d origin_in_body =
            geom.inertial_origin.inverse() * geom.origin_offset;
        collision_tfs.push_back(body_world * origin_in_body);
      } else {
        // インデックス異常時は単位行列
        collision_tfs.push_back(Eigen::Isometry3d::Identity());
      }
    }
    return collision_tfs;
  }

private:
  std::vector<LinkCollisionGeometry> geometries_;
};

/**
 * @brief RobotModelからRobotCollisionModelを構築するヘルパー
 * @param model ソースとなるRobotModel
 * @param leaf_link_name
 * KinematicChainの終端リンク名（チェーンに含まれるリンクを特定するため）
 * @return 構築されたRobotCollisionModel
 */
RobotCollisionModel
createCollisionModelFromRobot(const RobotModel &model,
                              const std::string &leaf_link_name);

} // namespace simulation
