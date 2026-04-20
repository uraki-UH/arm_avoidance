#pragma once

#include "collision_detector.hpp"
#include <string>
#include <unordered_set>
#include <vector>

namespace collision {

class SelfCollisionChecker {
public:
  enum class ShapeType { CAPSULE, BOX, SPHERE, MESH };

  struct CollisionObject {
    int id;
    std::string name;
    ShapeType type;
    Capsule capsule;
    Box box;
    Sphere sphere;
    Mesh mesh; // Added Mesh support
    bool is_fixed_to_base;
  };

  void setIgnorePair(int id1, int id2);
  bool checkSelfCollision(const std::vector<CollisionObject> &objects) const;
  bool checkMultiArmCollision(
      const std::vector<std::vector<CollisionObject>> &arms,
      const std::vector<CollisionObject> &shared_bodies) const;

  bool checkPair(const CollisionObject &obj1,
                 const CollisionObject &obj2) const;
  AABB calculateAABB(const CollisionObject &obj) const;

private:
  std::unordered_set<uint64_t> ignore_pairs;

  uint64_t pairKey(int id1, int id2) const {
    if (id1 > id2)
      std::swap(id1, id2);
    return (static_cast<uint64_t>(id1) << 32) | static_cast<uint32_t>(id2);
  }

  double segmentDistance(const Capsule &c1, const Capsule &c2) const;
  double segmentToSegmentDistance(const Eigen::Vector3d &p1,
                                  const Eigen::Vector3d &q1,
                                  const Eigen::Vector3d &p2,
                                  const Eigen::Vector3d &q2) const;
};

} // namespace collision