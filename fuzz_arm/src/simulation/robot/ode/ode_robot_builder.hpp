#pragma once

#include "simulation/robot/robot_model.hpp"
#include <cstdint>
#include <map>
#include <ode/ode.h>
#include <string>
#include <vector>

namespace robot_sim {
namespace simulation {
class MeshCache;
struct MeshEntry;
}
} // namespace robot_sim

namespace simulation {

using robot_sim::simulation::MeshCache;
class CollisionManager; // Forward declaration

// ODEで生成されたボディ、ジオメトリ、ジョイントの情報を保持する構造体
struct OdeRobotComponent {
  dBodyID body_id = nullptr;
  std::vector<dGeomID> geom_ids;
  dJointID joint_id = nullptr;
  std::string link_name;
  std::string joint_name;
};

class OdeRobotBuilder {
public:
  OdeRobotBuilder(dWorldID world, dSpaceID space,
                  CollisionManager *collision_manager, MeshCache *mesh_cache);
  ~OdeRobotBuilder();

  std::map<std::string, OdeRobotComponent>
  build(const RobotModel &model, uint32_t categoryBits, uint32_t collideBits,
        const Eigen::Vector3d &base_position = Eigen::Vector3d::Zero(),
        const Eigen::Quaterniond &base_orientation =
            Eigen::Quaterniond::Identity(),
        bool fix_root_link = true);

  void
  buildRecursive(const RobotModel &model, const std::string &current_link_name,
                 dBodyID parent_body_id,
                 const Eigen::Isometry3d &
                     parent_joint_world_transform, // 親ジョイントのワールド変換
                 std::map<std::string, OdeRobotComponent> &components,
                 uint32_t categoryBits, uint32_t collideBits);

  void setUseMesh(bool use) { use_mesh_ = use; }
private:
  struct TriMeshCacheEntry {
    dTriMeshDataID data_id = nullptr;
    std::vector<dReal> vertices;
    std::vector<dTriIndex> indices;
    ~TriMeshCacheEntry() {
      if (data_id) {
        dGeomTriMeshDataDestroy(data_id);
      }
    }
  };

  dWorldID world_;
  dSpaceID space_;
  CollisionManager *collision_manager_; // Pointer to the collision manager
  MeshCache *mesh_cache_;               // Pointer to the mesh cache
  bool use_mesh_ = true;                // Added toggle flag
  std::map<const robot_sim::simulation::MeshEntry *, dTriMeshDataID>
      tri_mesh_data_cache_;

  dGeomID createOdeGeometry(const std::string &link_name, int geom_index,
                            const Geometry &geom, dBodyID body,
                            const Eigen::Isometry3d &origin,
                            uint32_t categoryBits, uint32_t collideBits);
  dTriMeshDataID
  createTriMeshData(robot_sim::simulation::MeshEntry &mesh_entry);

  void setOdeInertia(dBodyID body, const Inertial &inertial,
                     const LinkProperties *link_props);
};

} // namespace simulation
