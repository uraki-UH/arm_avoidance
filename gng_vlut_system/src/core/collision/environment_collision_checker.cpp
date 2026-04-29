#include "collision/environment_collision_checker.hpp"

namespace simulation {

void EnvironmentCollisionChecker::addBoxObstacle(
    const std::string &name, const Eigen::Vector3d &center,
    const Eigen::Matrix3d &rotation, const Eigen::Vector3d &extents) {
  Obstacle obs;
  obs.name = name;
  obs.geometry.type = collision::SelfCollisionChecker::ShapeType::BOX;
  obs.geometry.box.center = center;
  obs.geometry.box.rotation = rotation;
  obs.geometry.box.extents = extents;
  obs.geometry.id = static_cast<int>(obstacles_.size());
  obstacles_.push_back(obs);
}

void EnvironmentCollisionChecker::addSphereObstacle(
    const std::string &name, const Eigen::Vector3d &center, double radius) {
  Obstacle obs;
  obs.name = name;
  obs.geometry.type = collision::SelfCollisionChecker::ShapeType::SPHERE;
  obs.geometry.sphere.center = center;
  obs.geometry.sphere.radius = radius;
  obs.geometry.id = static_cast<int>(obstacles_.size());
  obstacles_.push_back(obs);
}

void EnvironmentCollisionChecker::addCapsuleObstacle(const std::string &name,
                                                     const Eigen::Vector3d &a,
                                                     const Eigen::Vector3d &b,
                                                     double radius) {
  Obstacle obs;
  obs.name = name;
  obs.geometry.type = collision::SelfCollisionChecker::ShapeType::CAPSULE;
  obs.geometry.capsule.a = a;
  obs.geometry.capsule.b = b;
  obs.geometry.capsule.radius = radius;
  obs.geometry.id = static_cast<int>(obstacles_.size());
  obstacles_.push_back(obs);
}

void EnvironmentCollisionChecker::addMeshObstacle(const std::string &name,
                                                  const collision::Mesh &mesh) {
  Obstacle obs;
  obs.name = name;
  obs.type = Obstacle::Type::MESH;
  obs.mesh = mesh;
  obstacles_.push_back(obs);
}

bool EnvironmentCollisionChecker::checkCollision(
    const std::vector<collision::SelfCollisionChecker::CollisionObject>
        &robot_objects) const {
  for (const auto &rob_obj : robot_objects) {
    // 土台に固定されているリンク（base_linkなど）または明示的に無視するよう設定されたリンクは環境判定をスキップ
    if (rob_obj.is_fixed_to_base || ignore_link_ids_.count(rob_obj.id) > 0) {
      continue;
    }

    for (const auto &env_obs : obstacles_) {
      if (env_obs.type == Obstacle::Type::MESH) {
        // ロボット側の形状が Capsule の場合のみ Mesh との高速判定が利用可能
        if (rob_obj.type ==
            collision::SelfCollisionChecker::ShapeType::CAPSULE) {
          if (collision::CollisionQuery::testCollision(rob_obj.capsule,
                                                       env_obs.mesh)) {
            return true;
          }
        }
      } else {
        if (internal_checker_.checkPair(rob_obj, env_obs.geometry)) {
          return true;
        }
      }
    }
  }
  return false;
}

} // namespace simulation
