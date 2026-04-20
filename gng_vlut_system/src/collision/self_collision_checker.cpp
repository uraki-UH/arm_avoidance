#include "collision/self_collision_checker.hpp"

namespace collision {

void SelfCollisionChecker::setIgnorePair(int id1, int id2) {
  ignore_pairs.insert(pairKey(id1, id2));
}

bool SelfCollisionChecker::checkSelfCollision(
    const std::vector<CollisionObject> &objects) const {
  for (size_t i = 0; i < objects.size(); ++i) {
    for (size_t j = i + 1; j < objects.size(); ++j) {
      if (ignore_pairs.find(pairKey(objects[i].id, objects[j].id)) !=
          ignore_pairs.end()) {
        continue;
      }

      if (checkPair(objects[i], objects[j])) {
        return true;
      }
    }
  }
  return false;
}

bool SelfCollisionChecker::checkMultiArmCollision(
    const std::vector<std::vector<CollisionObject>> &arms,
    const std::vector<CollisionObject> &shared_bodies) const {

  // 1. アーム間の衝突チェック
  for (size_t arm_i = 0; arm_i < arms.size(); ++arm_i) {
    for (size_t arm_j = arm_i + 1; arm_j < arms.size(); ++arm_j) {
      for (const auto &obj_i : arms[arm_i]) {
        for (const auto &obj_j : arms[arm_j]) {
          if (checkPair(obj_i, obj_j)) {
            return true;
          }
        }
      }
    }
  }

  // 2. 各アーム内の自己衝突チェック（隣接リンクは除外）
  for (size_t arm_idx = 0; arm_idx < arms.size(); ++arm_idx) {
    const auto &arm = arms[arm_idx];
    for (size_t i = 0; i < arm.size(); ++i) {
      for (size_t j = i + 2; j < arm.size(); ++j) {
        if (checkPair(arm[i], arm[j])) {
          return true;
        }
      }
    }
  }

  // 3. アームと共有ベースの衝突チェック
  for (const auto &arm : arms) {
    for (const auto &arm_obj : arm) {
      for (const auto &base_obj : shared_bodies) {
        if (checkPair(arm_obj, base_obj)) {
          return true;
        }
      }
    }
  }

  return false;
}

// Helper for Capsule-Capsule
static double segmentToSegmentDist(const Eigen::Vector3d &p1,
                                   const Eigen::Vector3d &q1,
                                   const Eigen::Vector3d &p2,
                                   const Eigen::Vector3d &q2) {
  Eigen::Vector3d d1 = q1 - p1;
  Eigen::Vector3d d2 = q2 - p2;
  Eigen::Vector3d r = p1 - p2;
  double a = d1.dot(d1), b = d1.dot(d2), c = d2.dot(d2);
  double d = d1.dot(r), e = d2.dot(r);
  double denom = a * c - b * b;
  double s = 0.0, t = 0.0;
  if (denom < 1e-6) {
    s = 0.0;
    t = (b > c ? d / b : e / c);
  } else {
    s = (b * e - c * d) / denom;
    t = (a * e - b * d) / denom;
  }
  s = std::max(0.0, std::min(1.0, s));
  t = std::max(0.0, std::min(1.0, t));
  Eigen::Vector3d cp1 = p1 + s * d1;
  Eigen::Vector3d cp2 = p2 + t * d2;
  return (cp1 - cp2).norm();
}

bool SelfCollisionChecker::checkPair(const CollisionObject &obj1,
                                     const CollisionObject &obj2) const {
  // AABB Check
  AABB aabb1 = calculateAABB(obj1);
  AABB aabb2 = calculateAABB(obj2);
  if (!aabb1.intersects(aabb2))
    return false;

  // Capsule-Capsule
  if (obj1.type == ShapeType::CAPSULE && obj2.type == ShapeType::CAPSULE) {
    double dist = segmentToSegmentDist(obj1.capsule.a, obj1.capsule.b,
                                       obj2.capsule.a, obj2.capsule.b);
    return dist < (obj1.capsule.radius + obj2.capsule.radius);
  }

  // Box-Box
  if (obj1.type == ShapeType::BOX && obj2.type == ShapeType::BOX) {
    return CollisionQuery::testCollision(obj1.box, obj2.box);
  }

  // Box-Capsule
  if (obj1.type == ShapeType::BOX && obj2.type == ShapeType::CAPSULE) {
    return CollisionQuery::testCollision(obj1.box, obj2.capsule);
  }
  if (obj1.type == ShapeType::CAPSULE && obj2.type == ShapeType::BOX) {
    return CollisionQuery::testCollision(obj2.box, obj1.capsule);
  }

  // Capsule-Sphere
  if (obj1.type == ShapeType::CAPSULE && obj2.type == ShapeType::SPHERE) {
    return CollisionQuery::testCollision(obj1.capsule, obj2.sphere);
  }
  if (obj1.type == ShapeType::SPHERE && obj2.type == ShapeType::CAPSULE) {
    return CollisionQuery::testCollision(obj2.capsule, obj1.sphere);
  }

  // Mesh-Capsule
  if (obj1.type == ShapeType::MESH && obj2.type == ShapeType::CAPSULE) {
    return CollisionQuery::testCollision(obj2.capsule, obj1.mesh);
  }
  if (obj1.type == ShapeType::CAPSULE && obj2.type == ShapeType::MESH) {
    return CollisionQuery::testCollision(obj1.capsule, obj2.mesh);
  }

  // Add other pairs as needed (Sphere-Sphere etc)
  if (obj1.type == ShapeType::SPHERE && obj2.type == ShapeType::SPHERE) {
    double distSq = (obj1.sphere.center - obj2.sphere.center).squaredNorm();
    double rSum = obj1.sphere.radius + obj2.sphere.radius;
    return distSq < rSum * rSum;
  }

  return false;
}

AABB SelfCollisionChecker::calculateAABB(const CollisionObject &obj) const {
  AABB aabb;
  if (obj.type == ShapeType::CAPSULE) {
    aabb.expand(obj.capsule.a);
    aabb.expand(obj.capsule.b);
    Eigen::Vector3d r(obj.capsule.radius, obj.capsule.radius,
                      obj.capsule.radius);
    aabb.min -= r;
    aabb.max += r;
  } else if (obj.type == ShapeType::BOX) {
    // Compute AABB of OBB
    Eigen::Vector3d center = obj.box.center;
    Eigen::Vector3d extent = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; ++i) {
      extent += obj.box.rotation.col(i).cwiseAbs() * obj.box.extents(i);
    }
    aabb.min = center - extent;
    aabb.max = center + extent;
  } else if (obj.type == ShapeType::SPHERE) {
    Eigen::Vector3d r(obj.sphere.radius, obj.sphere.radius, obj.sphere.radius);
    aabb.min = obj.sphere.center - r;
    aabb.max = obj.sphere.center + r;
  } else if (obj.type == ShapeType::MESH) {
    aabb = obj.mesh.bounds;
  }
  return aabb;
}

} // namespace collision