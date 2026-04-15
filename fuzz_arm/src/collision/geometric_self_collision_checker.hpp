#pragma once

#include "collision/iself_collision_checker.hpp"
#include "collision/self_collision_checker.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "simulation/robot/robot_model.hpp"
#include <Eigen/Dense>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#ifdef USE_FCL
#include "collision/fcl/fcl_collision_detector.hpp"
#endif

namespace simulation {

/**
 * @brief ODEを使用しない、幾何計算ベースの自己干渉チェッカー
 * include/collision_detector ライブラリを使用する
 */
class GeometricSelfCollisionChecker : public ISelfCollisionChecker {
public:
  GeometricSelfCollisionChecker(const RobotModel &model,
                                const kinematics::KinematicChain &chain);
  ~GeometricSelfCollisionChecker() override = default;

  void updateBodyPoses(
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
      const std::vector<Eigen::Quaterniond,
                        Eigen::aligned_allocator<Eigen::Quaterniond>>
          &orientations) override;

  bool checkCollision() override;

  // 衝突除外ペアの管理
  void addCollisionExclusion(const std::string &link1,
                             const std::string &link2);
  bool shouldSkipCollision(const std::string &link1,
                           const std::string &link2) const;

#ifdef USE_FCL
  void setStrictMode(bool strict) { strict_mode_ = strict; }
  collision::FCLCollisionDetector &getFCLDetector() { return fcl_detector_; }
  std::shared_ptr<fcl::CollisionObject<double>> getFCLObject(int index) const {
    if (index >= 0 && index < (int)object_fcl_ids_.size()) {
      int fcl_id = object_fcl_ids_[index];
      return (fcl_id != -1) ? fcl_detector_.getRobotLink(fcl_id) : nullptr;
    }
    return nullptr;
  }
#endif

  const std::vector<collision::SelfCollisionChecker::CollisionObject> &
  getCollisionObjects() const {
    return collision_objects_;
  }

  std::string getLinkNameForObject(int obj_idx) const {
    if (obj_idx >= 0 && obj_idx < (int)object_map_.size()) {
      return object_map_[obj_idx].link_name;
    }
    return "";
  }

private:
  // 内部チェッカー
  collision::SelfCollisionChecker checker_;

  // 管理している衝突オブジェクト
  std::vector<collision::SelfCollisionChecker::CollisionObject>
      collision_objects_;

#ifdef USE_FCL
  bool strict_mode_ = false;
  collision::FCLCollisionDetector fcl_detector_;
  std::vector<int> object_fcl_ids_; // Mapping from collision_objects_ index to FCL ID
  std::vector<std::pair<int, int>> fcl_ignore_pairs_; // Pairs of FCL IDs to ignore
#endif

  // 各オブジェクトがどのリンク(KinematicChain上のインデックス)に紐付いているか
  struct ObjectLinkMap {
    int link_index;             // KinematicChainのリンクインデックス
    std::string link_name;      // リンク名（固定リンク位置計算用）
    Eigen::Isometry3d local_tf; // リンク原点からのオフセット
  };
  std::vector<ObjectLinkMap> object_map_;

  // KinematicChainへの参照（全リンク姿勢構築用）
  const kinematics::KinematicChain &chain_;

  // 固定リンク情報: リンク名 -> (親リンク名, ジョイントオフセット)
  std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
      fixed_link_info_;

  // 固定リンクのトポロジー: 子リンク名 -> 親リンク名
  std::map<std::string, std::string> fixed_link_connectivity_;

  // 衝突除外ペア（環境障害物との衝突をスキップするリンクペア）
  std::set<std::pair<std::string, std::string>> collision_exclusion_pairs_;
};

} // namespace simulation
