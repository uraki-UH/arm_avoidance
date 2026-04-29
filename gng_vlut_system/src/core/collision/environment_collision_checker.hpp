#pragma once

#include "collision/collision_detector.hpp"
#include "collision/self_collision_checker.hpp"
#include <string>
#include <vector>

namespace simulation {

/**
 * @brief 環境障害物（床、壁、柱など）との衝突判定を行うクラス
 * 幾何プリミティブ（Box, Sphere,
 * Capsule）を環境として保持し、ロボットリンクとの交差を判定する
 */
class EnvironmentCollisionChecker {
public:
  struct Obstacle {
    std::string name;
    enum class Type { PRIMITIVE, MESH } type = Type::PRIMITIVE;
    collision::SelfCollisionChecker::CollisionObject geometry;
    collision::Mesh mesh;
  };

  void addBoxObstacle(const std::string &name, const Eigen::Vector3d &center,
                      const Eigen::Matrix3d &rotation,
                      const Eigen::Vector3d &extents);
  void addSphereObstacle(const std::string &name, const Eigen::Vector3d &center,
                         double radius);
  void addCapsuleObstacle(const std::string &name, const Eigen::Vector3d &a,
                          const Eigen::Vector3d &b, double radius);

  /**
   * @brief 複雑な形状（メッシュ）を不変障害物として追加
   */
  void addMeshObstacle(const std::string &name, const collision::Mesh &mesh);

  /**
   * @brief 特定のロボットリンクを環境判定から除外する設定を行う
   */
  void addIgnoreRobotLink(int link_id) { ignore_link_ids_.insert(link_id); }

  /**
   * @brief
   * 指定されたロボットの衝突形状群と、全ての環境障害物との間で衝突があるか判定する
   * is_fixed_to_base が true の形状や、addIgnoreRobotLink
   * で指定されたリンクはスキップする
   */
  bool checkCollision(
      const std::vector<collision::SelfCollisionChecker::CollisionObject>
          &robot_objects) const;

private:
  std::vector<Obstacle> obstacles_;
  std::unordered_set<int> ignore_link_ids_;
  collision::SelfCollisionChecker
      internal_checker_; // checkPair を流用するための内部チェッカー
};

} // namespace simulation
