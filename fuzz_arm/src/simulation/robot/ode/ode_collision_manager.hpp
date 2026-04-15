#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <map>
#include <memory>
#include <ode/ode.h>
#include <set>
#include <string>
#include <vector>

namespace simulation {

struct MeshEntry;

// Enum for collision categories, using powers of 2 for bitmasking
enum CollisionCategory : uint32_t {
  NOTHING = 0,
  ROBOT_PART = 1 << 0,
  GROUND = 1 << 1,
  MOVABLE_OBJECT = 1 << 2,
  ALL = 0xFFFFFFFF
};

// Struct to hold per-geometry data
struct GeomData {
  std::string name;
  uint32_t categoryBits = CollisionCategory::ALL;
  uint32_t collideBits = CollisionCategory::ALL;

  // Material properties
  double friction = 1.0;
  double restitution = 0.1; // Bounciness
  // Add other dContact surface properties as needed (e.g., slip, soft_cfm)
  std::shared_ptr<MeshEntry> visual_mesh_entry; // Added for visual override
  Eigen::Vector3d visual_mesh_center = Eigen::Vector3d::Zero(); // Mesh center offset
};

class CollisionManager {
public:
  CollisionManager(dWorldID world, dJointGroupID contactGroup);
  ~CollisionManager();

  // Register a new geometry with specific properties
  void registerGeom(dGeomID geom, const std::string &name,
                    uint32_t categoryBits, uint32_t collideBits,
                    double friction = 1.0, double restitution = 0.1,
                    std::shared_ptr<MeshEntry> visual_mesh_entry = nullptr,
                    const Eigen::Vector3d &visual_mesh_center =
                        Eigen::Vector3d::Zero());

  // Update material properties for an already registered geometry
  bool updateMaterial(const std::string &name, double friction,
                      double restitution);

  // The static callback function to be passed to ODE
  static void nearCallback(void *data, dGeomID o1, dGeomID o2);

  // Clear all tracked contacts (should be called before dSpaceCollide each
  // step)
  void clearContacts();

  // Get list of names in contact with the given object name
  std::vector<std::string> getContacts(const std::string &name) const;

  // Get the name associated with a geometry
  std::string getGeomName(dGeomID geom) const;

  // 衝突除外ペアの管理
  void addCollisionExclusion(const std::string &name1,
                             const std::string &name2);
  bool shouldSkipCollision(const std::string &name1,
                           const std::string &name2) const;

  dJointGroupID getContactGroupID() const { return contactGroup_; }

private:
  // The actual collision handling logic
  void handleCollision(dGeomID o1, dGeomID o2);

  dWorldID world_;
  dJointGroupID contactGroup_;

  // Storage for GeomData. We use pointers to avoid invalidation.
  std::vector<std::unique_ptr<GeomData>> geomDataStore_;
  // Map for easy lookup by name
  std::map<std::string, GeomData *> geomDataNameMap_;

  // Contact tracking: maps a geometry to a set of geometries it is touching
  std::map<dGeomID, std::set<dGeomID>> activeContacts_;

  // 衝突除外ペア（名前ベース）
  std::set<std::pair<std::string, std::string>> collisionExclusionPairs_;
};

} // namespace simulation
