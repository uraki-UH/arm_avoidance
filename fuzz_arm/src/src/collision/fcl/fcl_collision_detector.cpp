#ifdef USE_FCL

#include "collision/fcl/fcl_collision_detector.hpp"

// FCL collision and distance request/result objects
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>

namespace collision {

FCLCollisionDetector::FCLCollisionDetector() {
  // Constructor: Nothing specific to initialize for FCL here,
  // as collision objects are created on demand when obstacles are added.
}

FCLCollisionDetector::~FCLCollisionDetector() {
  // Destructor: FCL collision objects are managed by shared_ptr,
  // so they will be deallocated automatically.
  clearObstacles();
}

void FCLCollisionDetector::clearObstacles() { obstacles_.clear(); }

void FCLCollisionDetector::addObstacle(const Sphere &s) {
  obstacles_.push_back(createFCLCollisionObject(s));
}

void FCLCollisionDetector::addObstacle(const Box &b) {
  obstacles_.push_back(createFCLCollisionObject(b));
}

void FCLCollisionDetector::addObstacle(const Mesh &m) {
  obstacles_.push_back(createFCLCollisionObject(m));
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

bool FCLCollisionDetector::checkRobotCollision(
    const std::vector<Capsule> &robot_links) const {
  // Manager for robot links
  fcl::DynamicAABBTreeCollisionManager<double> robot_manager;
  std::vector<std::shared_ptr<fcl::CollisionObject<double>>> robot_fcl_links;
  std::vector<fcl::CollisionObject<double> *> robot_fcl_ptrs;
  robot_fcl_links.reserve(robot_links.size());
  robot_fcl_ptrs.reserve(robot_links.size());

  for (const auto &link : robot_links) {
    auto obj = createFCLCollisionObject(link);
    robot_fcl_links.push_back(obj);
    robot_fcl_ptrs.push_back(obj.get());
  }
  robot_manager.registerObjects(robot_fcl_ptrs);
  robot_manager.setup();

  // Manager for obstacles
  fcl::DynamicAABBTreeCollisionManager<double> obstacle_manager;
  std::vector<fcl::CollisionObject<double> *> obstacle_ptrs;
  obstacle_ptrs.reserve(obstacles_.size());
  for (const auto &obj : obstacles_) {
    obstacle_ptrs.push_back(obj.get());
  }
  obstacle_manager.registerObjects(obstacle_ptrs);
  obstacle_manager.setup();

  // Perform collision check between robot links and obstacles
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

std::vector<Contact> FCLCollisionDetector::getAllContacts(
    const std::vector<Capsule> &robot_links) const {
  std::vector<Contact> contacts;

  // Create FCL collision objects for robot links
  std::vector<std::shared_ptr<fcl::CollisionObject<double>>> robot_fcl_links;
  robot_fcl_links.reserve(robot_links.size());
  for (const auto &link : robot_links) {
    robot_fcl_links.push_back(createFCLCollisionObject(link));
  }

  // Iterate through each robot link and check against all obstacles
  for (const auto &robot_obj : robot_fcl_links) {
    for (const auto &obstacle_obj : obstacles_) {
      fcl::CollisionRequest<double> request;
      request.num_max_contacts =
          std::numeric_limits<int>::max(); // Get all contacts
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
          // FCL's contact point is on one surface, penetration depth and normal
          // are relative. To get the point on the second object,
          // we move along the normal by the penetration depth.
          contact.pos_b =
              toEigen(fcl_contact.pos -
                      fcl_contact.normal * fcl_contact.penetration_depth);
          contact.normal = toEigen(fcl_contact.normal);
          contact.depth = fcl_contact.penetration_depth;
          // primitive_id and link_parameter are not directly available from FCL
          // contacts and might require more sophisticated mapping for meshes or
          // specific primitives.
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
