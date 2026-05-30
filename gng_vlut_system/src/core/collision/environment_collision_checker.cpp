#include "collision/environment_collision_checker.hpp"

namespace simulation {

bool EnvironmentCollisionChecker::checkCollision(
    const collision::SelfCollisionChecker::CollisionObject &robot_object) const {
  if (robot_object.is_fixed_to_base || ignore_link_ids_.count(robot_object.id) > 0) {
    return false;
  }

  for (const auto &env_obs : obstacles_) {
    if (env_obs.type == Obstacle::Type::MESH) {
      if (robot_object.type ==
          collision::SelfCollisionChecker::ShapeType::CAPSULE) {
        if (collision::CollisionQuery::testCollision(robot_object.capsule,
                                                     env_obs.mesh)) {
          return true;
        }
      } else if (robot_object.type ==
                 collision::SelfCollisionChecker::ShapeType::SPHERE) {
        if (collision::CollisionQuery::testCollision(robot_object.sphere,
                                                     env_obs.mesh)) {
          return true;
        }
      }
    } else {
      if (internal_checker_.checkPair(robot_object, env_obs.geometry)) {
        return true;
      }
    }
  }
  return false;
}

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
    if (checkCollision(rob_obj)) {
      return true;
    }
  }
  return false;
}

} // namespace simulation
