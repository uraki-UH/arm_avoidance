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

  // --- Obstacle Management ---

  // Add obstacles to the collision world
  void addObstacle(const Sphere &s);
  void addObstacle(const Box &b);
  void addObstacle(const Mesh &m);
  
  // New: Add mesh obstacle with caching
  void addMeshObstacle(const std::string &stl_path, const Eigen::Vector3d &scale, 
                       const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot);

  // Clear all obstacles from the collision world
  void clearObstacles();

  // --- Robot Link Management ---

  // Register a robot link (Capsule or Mesh)
  int addRobotLink(const Capsule &capsule);
  int addRobotMeshLink(const std::string &stl_path, const Eigen::Vector3d &scale);

  // Update robot link pose (should be called before checkRobotCollision)
  void updateRobotLinkPose(int index, const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot);
  void updateRobotLinkPose(int index, const Eigen::Isometry3d &tf);
  
  // Alternative for capsules: update endpoints
  void updateRobotLinkCapsule(int index, const Eigen::Vector3d &a, const Eigen::Vector3d &b);

  // --- Collision Queries ---

  // Check collision between robot links and obstacles
  bool checkRobotCollision() const;

  // Check self-collision among robot links
  bool checkSelfCollision(const std::vector<std::pair<int, int>> &ignore_pairs = {}) const;

  // Check collision between specific capsules and obstacles (Legacy support)
  bool checkRobotCollision(const std::vector<Capsule> &robot_links) const;

  // Get all contacts between robot links and obstacles
  std::vector<Contact> getAllContacts() const;

  // --- Getters ---
  std::shared_ptr<fcl::CollisionObject<double>> getRobotLink(int index) const {
    if (index >= 0 && index < (int)robot_links_.size()) {
      return robot_links_[index];
    }
    return nullptr;
  }

  const std::vector<std::shared_ptr<fcl::CollisionObject<double>>> &getRobotLinks() const {
    return robot_links_;
  }

private:
  // Persistent FCL objects
  std::vector<std::shared_ptr<fcl::CollisionObject<double>>> obstacles_;
  std::vector<std::shared_ptr<fcl::CollisionObject<double>>> robot_links_;

  // Broadphase Managers for high-performance collision checking
  mutable fcl::DynamicAABBTreeCollisionManager<double> robot_manager_;
  mutable fcl::DynamicAABBTreeCollisionManager<double> obstacle_manager_;
  mutable bool robot_manager_dirty_ = true;
  mutable bool obstacle_manager_dirty_ = true;

  // Geometry Cache: Avoid rebuilding BVH for the same mesh
  using BVHModelPtr = std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>>;
  std::unordered_map<std::string, BVHModelPtr> geometry_cache_;

  // Helper to load or get mesh from cache
  BVHModelPtr getOrLoadMesh(const std::string &stl_path, const Eigen::Vector3d &scale);

  // Helper to convert collision primitives to FCL objects
  std::shared_ptr<fcl::CollisionObject<double>>
  createFCLCollisionObject(const Capsule &capsule) const;
  std::shared_ptr<fcl::CollisionObject<double>>
  createFCLCollisionObject(const Sphere &sphere) const;
  std::shared_ptr<fcl::CollisionObject<double>>
  createFCLCollisionObject(const Box &box) const;
  std::shared_ptr<fcl::CollisionObject<double>>
  createFCLCollisionObject(const Mesh &mesh) const;
};

} // namespace collision

#endif // USE_FCL
