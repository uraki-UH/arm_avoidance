#pragma once

#ifdef USE_FCL

#include "collision/collision_detector.hpp" // For existing data structures like Capsule, Mesh, Contact, etc.

// FCL Headers
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/capsule.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/math/bv/utility.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>

#include <memory>
#include <unordered_map>
#include <vector>

namespace collision {

// Hash for Eigen::Vector3d to use in std::unordered_map
struct Vector3dHash {
  std::size_t operator()(const Eigen::Vector3d &v) const {
    std::size_t seed = 0;
    std::hash<double> hasher;
    seed ^= hasher(v.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hasher(v.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hasher(v.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

// Helper function to convert Eigen::Vector3d to fcl::Vector3d
inline fcl::Vector3d toFCL(const Eigen::Vector3d &v) {
  return fcl::Vector3d(v.x(), v.y(), v.z());
}

// Helper function to convert Eigen::Matrix3d to fcl::Matrix3d
inline fcl::Matrix3d toFCL(const Eigen::Matrix3d &m) {
  fcl::Matrix3d fcl_m;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      fcl_m(i, j) = m(i, j);
    }
  }
  return fcl_m;
}

// Helper function to convert fcl::Vector3d to Eigen::Vector3d
inline Eigen::Vector3d toEigen(const fcl::Vector3d &v) {
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

struct FCLContactData {
  std::vector<Contact> contacts;
};

class FCLCollisionDetector {
public:
  FCLCollisionDetector();
  ~FCLCollisionDetector();

  // Add obstacles to the collision world
  void addObstacle(const Sphere &s);
  void addObstacle(const Box &b);
  void addObstacle(const Mesh &m);
  // Add other primitive types as needed

  // Check collision between robot links and obstacles
  bool checkRobotCollision(const std::vector<Capsule> &robot_links) const;

  // Get all contacts between robot links and obstacles
  std::vector<Contact>
  getAllContacts(const std::vector<Capsule> &robot_links) const;

  // Clear all obstacles from the collision world
  void clearObstacles();

private:
  std::vector<std::shared_ptr<fcl::CollisionObject<double>>> obstacles_;

  // Helper to convert collision::Capsule to FCL CollisionObject
  std::shared_ptr<fcl::CollisionObject<double>>
  createFCLCollisionObject(const Capsule &capsule) const;

  // Helper to convert collision::Sphere to FCL CollisionObject
  std::shared_ptr<fcl::CollisionObject<double>>
  createFCLCollisionObject(const Sphere &sphere) const;

  // Helper to convert collision::Box to FCL CollisionObject
  std::shared_ptr<fcl::CollisionObject<double>>
  createFCLCollisionObject(const Box &box) const;

  // Helper to convert collision::Mesh to FCL CollisionObject
  std::shared_ptr<fcl::CollisionObject<double>>
  createFCLCollisionObject(const Mesh &mesh) const;
};

} // namespace collision

#endif // USE_FCL
