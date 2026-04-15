#ifdef USE_FCL

#include "collision/fcl/fcl_collision_detector.hpp"

// FCL collision and distance request/result objects
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include "simulation/robot/stl_loader.hpp"
#include "common/resource_utils.hpp"
#include <iostream>

namespace collision {

FCLCollisionDetector::FCLCollisionDetector() {}

FCLCollisionDetector::~FCLCollisionDetector() { clearObstacles(); }

void FCLCollisionDetector::clearObstacles() {
  robot_manager_.clear();
  obstacle_manager_.clear();
  obstacles_.clear();
  robot_links_.clear();
  robot_manager_dirty_ = true;
  obstacle_manager_dirty_ = true;
  // We keep the geometry_cache_ for possible reuse
}

void FCLCollisionDetector::addObstacle(const Sphere &s) {
  auto obj = createFCLCollisionObject(s);
  obstacles_.push_back(obj);
  obstacle_manager_.registerObject(obj.get());
  obstacle_manager_dirty_ = true;
}

void FCLCollisionDetector::addObstacle(const Box &b) {
  auto obj = createFCLCollisionObject(b);
  obstacles_.push_back(obj);
  obstacle_manager_.registerObject(obj.get());
  obstacle_manager_dirty_ = true;
}

void FCLCollisionDetector::addObstacle(const Mesh &m) {
  auto obj = createFCLCollisionObject(m);
  obstacles_.push_back(obj);
  obstacle_manager_.registerObject(obj.get());
  obstacle_manager_dirty_ = true;
}

void FCLCollisionDetector::addMeshObstacle(const std::string &stl_path,
                                           const Eigen::Vector3d &scale,
                                           const Eigen::Vector3d &pos,
                                           const Eigen::Matrix3d &rot) {
  auto mesh_geom = getOrLoadMesh(stl_path, scale);
  if (!mesh_geom)
    return;

  fcl::Transform3d tf;
  tf.translation() = toFCL(pos);
  tf.linear() = toFCL(rot);

  auto obj = std::make_shared<fcl::CollisionObject<double>>(mesh_geom, tf);
  obstacles_.push_back(obj);
  obstacle_manager_.registerObject(obj.get());
  obstacle_manager_dirty_ = true;
}

FCLCollisionDetector::BVHModelPtr
FCLCollisionDetector::getOrLoadMesh(const std::string &stl_path,
                                    const Eigen::Vector3d &scale) {
  std::string key = stl_path + "_" + std::to_string(scale.x()) + "_" +
                    std::to_string(scale.y()) + "_" + std::to_string(scale.z());

  auto it = geometry_cache_.find(key);
  if (it != geometry_cache_.end()) {
    return it->second;
  }

  // Resolve path
  std::string resolved_path = robot_sim::common::resolvePath(stl_path);

  // Load using StlLoader
  using ::simulation::MeshData;
  using ::simulation::StlLoader;
  MeshData mesh_data;
  try {
    mesh_data = StlLoader::loadBinaryStl(resolved_path, scale);
  } catch (const std::exception &e) {
    std::cerr << "[FCLDetector] Error loading STL " << stl_path << ": "
              << e.what() << std::endl;
    return nullptr;
  }

  if (mesh_data.vertices.empty())
    return nullptr;

  auto fcl_mesh = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();
  fcl_mesh->beginModel();

  std::vector<fcl::Vector3d> vertices;
  std::vector<fcl::Triangle> triangles;
  vertices.reserve(mesh_data.vertices.size() / 3);
  triangles.reserve(mesh_data.indices.size() / 3);

  for (size_t i = 0; i < mesh_data.vertices.size(); i += 3) {
    vertices.emplace_back(mesh_data.vertices[i], mesh_data.vertices[i + 1],
                          mesh_data.vertices[i + 2]);
  }
  for (size_t i = 0; i < mesh_data.indices.size(); i += 3) {
    triangles.emplace_back(mesh_data.indices[i], mesh_data.indices[i + 1],
                           mesh_data.indices[i + 2]);
  }

  fcl_mesh->addSubModel(vertices, triangles);
  fcl_mesh->endModel();

  geometry_cache_[key] = fcl_mesh;
  return fcl_mesh;
}

int FCLCollisionDetector::addRobotLink(const Capsule &capsule) {
  auto obj = createFCLCollisionObject(capsule);
  int index = (int)robot_links_.size();
  obj->setUserData((void*)(intptr_t)index);
  robot_links_.push_back(obj);
  robot_manager_.registerObject(obj.get());
  robot_manager_dirty_ = true;
  return index;
}
int FCLCollisionDetector::addRobotMeshLink(const std::string &stl_path,
                                           const Eigen::Vector3d &scale) {
  auto mesh_geom = getOrLoadMesh(stl_path, scale);
  if (!mesh_geom) {
    return -1;
  }
  auto obj = std::make_shared<fcl::CollisionObject<double>>(
      mesh_geom, fcl::Transform3d::Identity());
  int index = (int)robot_links_.size();
  obj->setUserData((void*)(intptr_t)index);
  robot_links_.push_back(obj);
  robot_manager_.registerObject(obj.get());
  robot_manager_dirty_ = true;
  return index;
}

void FCLCollisionDetector::updateRobotLinkPose(int index,
                                               const Eigen::Vector3d &pos,
                                               const Eigen::Matrix3d &rot) {
  if (index < 0 || index >= (int)robot_links_.size())
    return;
  fcl::Transform3d tf;
  tf.translation() = toFCL(pos);
  tf.linear() = toFCL(rot);
  robot_links_[index]->setTransform(tf);
  robot_links_[index]->computeAABB();
  robot_manager_.update(robot_links_[index].get());
}

void FCLCollisionDetector::updateRobotLinkPose(int index,
                                               const Eigen::Isometry3d &tf) {
  if (index < 0 || index >= (int)robot_links_.size())
    return;
  fcl::Transform3d fcl_tf;
  fcl_tf.translation() = toFCL(Eigen::Vector3d(tf.translation()));
  fcl_tf.linear() = toFCL(Eigen::Matrix3d(tf.rotation()));
  robot_links_[index]->setTransform(fcl_tf);
  robot_links_[index]->computeAABB();
  robot_manager_.update(robot_links_[index].get());
}

void FCLCollisionDetector::updateRobotLinkCapsule(int index,
                                                  const Eigen::Vector3d &a,
                                                  const Eigen::Vector3d &b) {
  if (index < 0 || index >= (int)robot_links_.size())
    return;

  Eigen::Vector3d vec = b - a;
  double length = vec.norm();
  Eigen::Vector3d mid_point = (a + b) / 2.0;

  fcl::Transform3d tf;
  tf.translation() = toFCL(mid_point);

  if (length > std::numeric_limits<double>::epsilon()) {
    Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d capsule_axis = vec.normalized();
    Eigen::Quaterniond quat =
        Eigen::Quaterniond::FromTwoVectors(z_axis, capsule_axis);
    tf.linear() = toFCL(quat.toRotationMatrix());
  } else {
    tf.linear() = fcl::Matrix3d::Identity();
  }

  robot_links_[index]->setTransform(tf);
  robot_links_[index]->computeAABB();
  robot_manager_.update(robot_links_[index].get());
}

std::shared_ptr<fcl::CollisionObject<double>>
FCLCollisionDetector::createFCLCollisionObject(const Capsule &capsule) const {
  // FCL capsule requires half-length and radius.
  // Our capsule has two endpoints a and b, and a radius.
  // The half-length is half the distance between a and b.
  Eigen::Vector3d vec = capsule.b - capsule.a;
  double length = vec.norm();
  double half_length = length / 2.0;

  // FCL capsule is defined along the Z-axis, centered at origin.
  // We need to apply a transformation later.
  auto fcl_capsule =
      std::make_shared<fcl::Capsule<double>>(capsule.radius, length);

  // Calculate the transform from FCL's canonical capsule (centered, along Z)
  // to our capsule's position and orientation.
  fcl::Transform3d tf;

  // Translation: Midpoint of a and b
  Eigen::Vector3d mid_point = (capsule.a + capsule.b) / 2.0;
  tf.translation() = toFCL(mid_point);

  // Rotation: Align FCL's Z-axis with the capsule's axis (vec)
  // Handle the case where vec is zero to avoid issues with `vec.normalized()`
  if (length < std::numeric_limits<double>::epsilon()) {
    tf.linear() = fcl::Matrix3d::Identity(); // No rotation if points are same
  } else {
    Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d capsule_axis = vec.normalized();
    Eigen::Quaterniond quat =
        Eigen::Quaterniond::FromTwoVectors(z_axis, capsule_axis);
    tf.linear() = toFCL(quat.toRotationMatrix());
  }

  return std::make_shared<fcl::CollisionObject<double>>(fcl_capsule, tf);
}

std::shared_ptr<fcl::CollisionObject<double>>
FCLCollisionDetector::createFCLCollisionObject(const Sphere &sphere) const {
  auto fcl_sphere = std::make_shared<fcl::Sphere<double>>(sphere.radius);
  fcl::Transform3d tf;
  tf.translation() = toFCL(sphere.center);
  return std::make_shared<fcl::CollisionObject<double>>(fcl_sphere, tf);
}

std::shared_ptr<fcl::CollisionObject<double>>
FCLCollisionDetector::createFCLCollisionObject(const Box &box) const {
  auto fcl_box = std::make_shared<fcl::Box<double>>(
      box.extents.x() * 2.0, // FCL Box uses full lengths
      box.extents.y() * 2.0, box.extents.z() * 2.0);
  fcl::Transform3d tf;
  tf.translation() = toFCL(box.center);
  tf.linear() = toFCL(box.rotation);
  return std::make_shared<fcl::CollisionObject<double>>(fcl_box, tf);
}

std::shared_ptr<fcl::CollisionObject<double>>
FCLCollisionDetector::createFCLCollisionObject(const Mesh &mesh) const {
  // FCL requires a BVHModel for meshes
  auto fcl_mesh = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();

  // Add vertices and triangles to the FCL mesh model
  std::vector<fcl::Vector3d> vertices;
  std::vector<fcl::Triangle> triangles;
  vertices.reserve(mesh.triangles.size() * 3); // Pre-allocate
  triangles.reserve(mesh.triangles.size());    // Pre-allocate

  std::unordered_map<Eigen::Vector3d, int, collision::Vector3dHash> vertex_map;
  int current_vertex_idx = 0;

  for (const auto &tri : mesh.triangles) {
    std::array<int, 3> tri_indices;

    auto add_vertex_get_index = [&](const Eigen::Vector3d &v_eigen) {
      if (vertex_map.find(v_eigen) == vertex_map.end()) {
        vertex_map[v_eigen] = current_vertex_idx;
        vertices.push_back(toFCL(v_eigen));
        current_vertex_idx++;
      }
      return vertex_map[v_eigen];
    };

    tri_indices[0] = add_vertex_get_index(tri.v0);
    tri_indices[1] = add_vertex_get_index(tri.v1);
    tri_indices[2] = add_vertex_get_index(tri.v2);

    triangles.emplace_back(tri_indices[0], tri_indices[1], tri_indices[2]);
  }

  // Setup BVHModel
  fcl_mesh->beginModel();
  fcl_mesh->addSubModel(vertices, triangles);
  fcl_mesh->endModel();

  // The transform for the mesh itself is typically identity if vertices are in
  // world frame or corresponds to the mesh's root transform. Assuming
  // mesh.triangles are already in world coordinates.
  fcl::Transform3d tf; // Identity transform
  return std::make_shared<fcl::CollisionObject<double>>(fcl_mesh, tf);
}

// Stateful Self-Collision Callback
struct SelfCollisionData {
  const std::vector<std::pair<int, int>> &ignore_pairs;
  const std::vector<std::shared_ptr<fcl::CollisionObject<double>>> &links;
  bool is_collision = false;

  SelfCollisionData(
      const std::vector<std::pair<int, int>> &ignores,
      const std::vector<std::shared_ptr<fcl::CollisionObject<double>>> &l)
      : ignore_pairs(ignores), links(l) {}
};

bool FCLSelfCollisionCallback(fcl::CollisionObject<double> *o1,
                               fcl::CollisionObject<double> *o2, void *data) {
  SelfCollisionData *col_data = static_cast<SelfCollisionData *>(data);
  if (col_data->is_collision)
    return true; // Already found a collision

  // O(1) index lookup via UserData
  int idx1 = (int)(intptr_t)o1->getUserData();
  int idx2 = (int)(intptr_t)o2->getUserData();

  // AABB Fast Filter: Skip if AABBs are more than 5mm apart
  const auto& aabb1 = o1->getAABB();
  const auto& aabb2 = o2->getAABB();
  if (aabb1.distance(aabb2) > 0.005) {
    return false;
  }

  // Check if idx1 and idx2 are an ignored pair
  if (idx1 != -1 && idx2 != -1) {
    for (const auto &pair : col_data->ignore_pairs) {
      if ((pair.first == idx1 && pair.second == idx2) ||
          (pair.first == idx2 && pair.second == idx1)) {
        return false; // Skip this pair
      }
    }
  }

  fcl::CollisionRequest<double> request;
  fcl::CollisionResult<double> result;
  fcl::collide(o1, o2, request, result);

  if (result.isCollision()) {
    col_data->is_collision = true;
    return true;
  }
  return false;
}

// A simple collision callback function
bool FCLCollisionCallback(fcl::CollisionObject<double> *o1,
                          fcl::CollisionObject<double> *o2, void * /*data*/) {
  // Request collision details
  fcl::CollisionRequest<double> request;
  request.num_max_contacts = 1; // We only need to know if a collision occurs
  fcl::CollisionResult<double> result;
  fcl::collide(o1, o2, request, result);
  return result.isCollision();
}

bool FCLCollisionDetector::checkRobotCollision() const {
  if (robot_links_.empty() || obstacles_.empty())
    return false;

  if (robot_manager_dirty_) {
    robot_manager_.setup();
    robot_manager_dirty_ = false;
  }
  if (obstacle_manager_dirty_) {
    obstacle_manager_.setup();
    obstacle_manager_dirty_ = false;
  }

  fcl::DefaultCollisionData<double> collision_data;
  collision_data.request.num_max_contacts = 1;

  robot_manager_.collide(&obstacle_manager_, &collision_data,
                         fcl::DefaultCollisionFunction<double>);
  return collision_data.result.isCollision();
}

bool FCLCollisionDetector::checkSelfCollision(
    const std::vector<std::pair<int, int>> &ignore_pairs) const {
  if (robot_links_.size() < 2)
    return false;

  if (robot_manager_dirty_) {
    robot_manager_.setup();
    robot_manager_dirty_ = false;
  }

  SelfCollisionData col_data(ignore_pairs, robot_links_);
  robot_manager_.collide(&col_data, FCLSelfCollisionCallback);

  return col_data.is_collision;
}

bool FCLCollisionDetector::checkRobotCollision(
    const std::vector<Capsule> &robot_links) const {
  if (robot_links.empty() || obstacles_.empty())
    return false;

  // Temporary manager for provided links (Legacy/Utility)
  fcl::DynamicAABBTreeCollisionManager<double> robot_manager;
  std::vector<std::shared_ptr<fcl::CollisionObject<double>>> temp_links;
  std::vector<fcl::CollisionObject<double> *> robot_fcl_ptrs;

  for (const auto &link : robot_links) {
    auto obj = createFCLCollisionObject(link);
    temp_links.push_back(obj);
    robot_fcl_ptrs.push_back(obj.get());
  }
  robot_manager.registerObjects(robot_fcl_ptrs);
  robot_manager.setup();

  // Manager for obstacles
  fcl::DynamicAABBTreeCollisionManager<double> obstacle_manager;
  std::vector<fcl::CollisionObject<double> *> obstacle_ptrs;
  for (const auto &obj : obstacles_) {
    obstacle_ptrs.push_back(obj.get());
  }
  obstacle_manager.registerObjects(obstacle_ptrs);
  obstacle_manager.setup();

  fcl::DefaultCollisionData<double> collision_data;
  collision_data.request.num_max_contacts = 1;

  robot_manager.collide(&obstacle_manager, &collision_data,
                        fcl::DefaultCollisionFunction<double>);
  return collision_data.result.isCollision();
}

// Extended collision callback to get contact information
bool FCLContactCallback(fcl::CollisionObject<double> *o1,
                        fcl::CollisionObject<double> *o2, void *data) {
  FCLContactData *contact_data = static_cast<FCLContactData *>(data);
  fcl::CollisionRequest<double> request;
  request.num_max_contacts =
      std::numeric_limits<int>::max(); // Get all contacts
  request.enable_contact = true;
  request.gjk_solver_type =
      fcl::GJKSolverType::GST_LIBCCD; // Use libccd for GJK

  fcl::CollisionResult<double> result;
  fcl::collide(o1, o2, request, result);

  if (result.isCollision()) {
    std::vector<fcl::Contact<double>> fcl_contacts;
    result.getContacts(fcl_contacts);
    for (const auto &fcl_contact : fcl_contacts) {
      Contact contact;
      contact.colliding = true;
      // FCL contact point is typically the point on the first object.
      // Normal points from o1 to o2.
      contact.pos_a = toEigen(fcl_contact.pos);
      // Need to calculate pos_b based on normal and depth for accurate
      // representation
      contact.pos_b = toEigen(
          fcl_contact.pos - fcl_contact.normal * fcl_contact.penetration_depth);
      contact.normal = toEigen(fcl_contact.normal);
      contact.depth = fcl_contact.penetration_depth;
      // primitive_id and link_parameter are not directly available from FCL
      // contacts and might require more sophisticated mapping for meshes or
      // specific primitives.
      contact_data->contacts.push_back(contact);
    }
    return true; // Continue to find more contacts
  }
  return false; // No collision, stop checking between these two objects
}

std::vector<Contact> FCLCollisionDetector::getAllContacts() const {
  std::vector<Contact> contacts;

  for (const auto &robot_obj : robot_links_) {
    for (const auto &obstacle_obj : obstacles_) {
      fcl::CollisionRequest<double> request;
      request.num_max_contacts = std::numeric_limits<int>::max();
      request.enable_contact = true;
      request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

      fcl::CollisionResult<double> result;
      fcl::collide(robot_obj.get(), obstacle_obj.get(), request, result);

      if (result.isCollision()) {
        std::vector<fcl::Contact<double>> fcl_contacts;
        result.getContacts(fcl_contacts);
        for (const auto &fcl_contact : fcl_contacts) {
          Contact contact;
          contact.colliding = true;
          contact.pos_a = toEigen(fcl_contact.pos);
          contact.pos_b = toEigen(
              fcl_contact.pos - fcl_contact.normal * fcl_contact.penetration_depth);
          contact.normal = toEigen(fcl_contact.normal);
          contact.depth = fcl_contact.penetration_depth;
          contacts.push_back(contact);
        }
      }
    }
  }
  return contacts;
}

} // namespace collision

// Define a hash function for Eigen::Vector3d for use in std::unordered_map
namespace std {
template <> struct hash<Eigen::Vector3d> {
  size_t operator()(const Eigen::Vector3d &v) const {
    size_t seed = 0;
    hash<double> hasher;
    seed ^= hasher(v.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hasher(v.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hasher(v.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

} // namespace std

#endif // USE_FCL
