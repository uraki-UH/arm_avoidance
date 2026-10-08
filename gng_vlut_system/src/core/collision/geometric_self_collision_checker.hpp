#pragma once

#include "safety_engine/vlut/iself_collision_checker.hpp"
#include "collision/self_collision_checker.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "robot_model/robot_model.hpp"
#include <Eigen/Dense>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <stdexcept>
#include <vector>

#ifdef USE_FCL
#include "collision/fcl/fcl_collision_detector.hpp"
namespace collision { struct solid_voxel_geometry; }
#endif

namespace simulation {

/**
 * @brief ODEを使用しない、幾何計算ベースの自己干渉チェッカー
 * include/collision_detector ライブラリを使用する
 */
class GeometricSelfCollisionChecker : public ISelfCollisionChecker {
public:
  GeometricSelfCollisionChecker(const RobotModel &model,
                                const kinematics::KinematicChain &chain,
                                bool enable_fcl_backend = true,
                                double voxel_size = 0.0);
  ~GeometricSelfCollisionChecker() override = default;

  void updateBodyPoses(
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
      const std::vector<Eigen::Quaterniond,
                        Eigen::aligned_allocator<Eigen::Quaterniond>>
          &orientations) override;

  bool checkCollision() override;

  // 不変な形状だけを共有する、並列問い合わせ用の独立した姿勢・探索状態
  std::unique_ptr<GeometricSelfCollisionChecker> clone_for_queries() const;

  /**
   * @brief 現在の姿勢で自己衝突しているリンクペアを列挙する
   * 既に除外済みのペアは含めない。
   */
  std::vector<std::pair<std::string, std::string>>
  collectSelfCollisionPairs() const;

  // 衝突除外ペアの管理
  void addCollisionExclusion(const std::string &link1,
                             const std::string &link2);
  bool shouldSkipCollision(const std::string &link1,
                           const std::string &link2) const;

#ifdef USE_FCL
  void setStrictMode(bool enable_strict_mode) {
    if (!enable_strict_mode && has_mesh_geometry_) {
      throw std::invalid_argument("Mesh collision requires strict geometry checks");
    }
    strict_mode_ = enable_strict_mode;
  }
  void setUseFCLBackend(bool enable_fcl_backend) {
    if (enable_fcl_backend && object_fcl_ids_.size() != collision_objects_.size()) {
      throw std::invalid_argument("Cannot enable an unregistered geometry backend");
    }
    if (!enable_fcl_backend && has_mesh_geometry_) {
      throw std::invalid_argument("Mesh collision requires the geometry backend");
    }
    use_fcl_backend_ = enable_fcl_backend;
  }
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
  GeometricSelfCollisionChecker(const GeometricSelfCollisionChecker &source);
  // 内部チェッカー
  collision::SelfCollisionChecker checker_;

  // 管理している衝突オブジェクト
  std::vector<collision::SelfCollisionChecker::CollisionObject>
      collision_objects_;

#ifdef USE_FCL
  bool strict_mode_ = true;
  bool use_fcl_backend_ = true;
  collision::FCLCollisionDetector fcl_detector_;
  // 表面交差と完全内包の併用。占有木はリンクローカル座標で共有
  bool enable_solid_containment_ = false;
  bool has_mesh_geometry_ = false;
  std::vector<std::shared_ptr<const collision::solid_voxel_geometry>> solid_geometries_;
  bool has_point_inside(std::size_t obj_idx, const Eigen::Vector3d &point) const;
  bool has_solid_containment(std::size_t first_idx, std::size_t second_idx) const;
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
