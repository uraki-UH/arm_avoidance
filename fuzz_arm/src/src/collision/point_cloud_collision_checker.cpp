#include "collision/point_cloud_collision_checker.hpp"

namespace simulation {

PointCloudCollisionChecker::PointCloudCollisionChecker(
    const RobotModel &model, const kinematics::KinematicChain &chain)
    : model_(model), chain_(chain) {
  initializeShapes();
}

void PointCloudCollisionChecker::initializeShapes() {
  shapes_.clear();

  // Extract fixed link info for FK
  for (const auto &jp : model_.getJoints()) {
    if (jp.second.type == kinematics::JointType::Fixed) {
      fixed_link_info_[jp.second.child_link] = {jp.second.parent_link,
                                                jp.second.origin};
    }
  }

  // Iterate through links and define their proxy collision shapes
  for (const auto &link_pair : model_.getLinks()) {
    const auto &link = link_pair.second;
    LinkShape shape;
    shape.name = link.name;
    shape.link_idx = -1;

    // Try to find matching link in KinematicChain
    for (int i = 0; i < chain_.getNumJoints(); ++i) {
      if (chain_.getLinkName(i) == link.name) {
        shape.link_idx = i;
        break;
      }
    }

    // Define a simplified capsule for each link based on its geometry or vector
    // For now, we use a conservative sphere-swept-line (capsule)
    // based on the link's length and some radius
    double radius = 0.05; // Default safety margin
    if (!link.collisions.empty()) {
      const auto &first_col = link.collisions[0];
      if (first_col.geometry.type == GeometryType::BOX) {
        radius = first_col.geometry.size.head<2>().maxCoeff() * 0.5;
      } else if (first_col.geometry.type == GeometryType::CYLINDER) {
        radius = first_col.geometry.size[0];
      }
    }

    shape.local_capsule.a = Eigen::Vector3d::Zero();
    // Link[i] world transform is at joint[i+1] (child).
    // The link vector (getLinkVector(i)) is parent->child.
    // To cover link[i], capsule goes from 0 (child) to -link_vector (parent).
    if (shape.link_idx != -1) {
      shape.local_capsule.b = -chain_.getLinkVector(shape.link_idx);
    } else {
      shape.local_capsule.b = Eigen::Vector3d::Zero();
    }
    shape.local_capsule.radius = radius;

    // Pre-calculate local AABB of the capsule
    shape.local_aabb.expand(shape.local_capsule.a -
                            Eigen::Vector3d::Constant(radius));
    shape.local_aabb.expand(shape.local_capsule.a +
                            Eigen::Vector3d::Constant(radius));
    shape.local_aabb.expand(shape.local_capsule.b -
                            Eigen::Vector3d::Constant(radius));
    shape.local_aabb.expand(shape.local_capsule.b +
                            Eigen::Vector3d::Constant(radius));

    shapes_.push_back(shape);
  }
}

collision::AABB
PointCloudCollisionChecker::computeLinkAABB(const LinkShape &shape,
                                            const Eigen::Isometry3d &tf) const {
  collision::AABB world_aabb;
  // Fast way: transform the 8 corners of the local AABB
  // Or even faster: transform the centers and add radius
  Eigen::Vector3d wa = tf * shape.local_capsule.a;
  Eigen::Vector3d wb = tf * shape.local_capsule.b;
  double r = shape.local_capsule.radius;

  world_aabb.expand(wa - Eigen::Vector3d::Constant(r));
  world_aabb.expand(wa + Eigen::Vector3d::Constant(r));
  world_aabb.expand(wb - Eigen::Vector3d::Constant(r));
  world_aabb.expand(wb + Eigen::Vector3d::Constant(r));

  return world_aabb;
}

bool PointCloudCollisionChecker::checkCollision(
    const std::vector<double> &joints, const VoxelGrid &grid,
    const collision::AABB &pc_bounds, double margin) const {

  // FK to get all transforms
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> j_pos;
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
      j_ori;
  chain_.forwardKinematicsAt(joints, j_pos, j_ori);

  std::map<std::string, Eigen::Isometry3d> link_tfs;
  chain_.buildAllLinkTransforms(j_pos, j_ori, fixed_link_info_, link_tfs);

  // --- Phase 0: Robot-level AABB ---
  collision::AABB robot_aabb;
  std::vector<std::pair<const LinkShape *, Eigen::Isometry3d>> active_links;

  for (const auto &link_shape : shapes_) {
    auto it = link_tfs.find(link_shape.name);
    if (it != link_tfs.end()) {
      collision::AABB link_aabb = computeLinkAABB(link_shape, it->second);
      robot_aabb.expand(link_aabb);
      active_links.push_back({&link_shape, it->second});
    }
  }

  if (!robot_aabb.intersects(pc_bounds)) {
    return false; // NO COLLISION (Phase 0 Exit)
  }

  // --- Phase 1: Link-level AABB vs Grid ---
  for (const auto &item : active_links) {
    const LinkShape *shape = item.first;
    const Eigen::Isometry3d &tf = item.second;

    collision::AABB link_aabb = computeLinkAABB(*shape, tf);
    if (!link_aabb.intersects(pc_bounds))
      continue;

    std::vector<Eigen::Vector3d> candidate_points;
    grid.getPointsInAABB(link_aabb, candidate_points);

    if (candidate_points.empty())
      continue;

    // --- Phase 2: Precise Capsule-Point Check ---
    Eigen::Vector3d wa = tf * shape->local_capsule.a;
    Eigen::Vector3d wb = tf * shape->local_capsule.b;
    double r = shape->local_capsule.radius + margin;
    double r_sq = r * r;
    Eigen::Vector3d ab = wb - wa;
    double length_sq = ab.squaredNorm();

    for (const auto &p : candidate_points) {
      // Point-to-segment distance squared
      Eigen::Vector3d ap = p - wa;
      double t = ap.dot(ab) / length_sq;
      t = std::max(0.0, std::min(1.0, t));
      Eigen::Vector3d closest = wa + t * ab;
      if ((p - closest).squaredNorm() < r_sq) {
        return true; // COLLISION DETECTED
      }
    }
  }

  return false;
}

} // namespace simulation
