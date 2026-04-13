#include "simulation/robot/urdf_loader.hpp"
#include "common/resource_utils.hpp"
#include "simulation/robot/robot_model.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace simulation {

// Helper function to convert urdf::Vector3 to Eigen::Vector3d
Eigen::Vector3d toEigen(const urdf::Vector3 &vec) {
  return Eigen::Vector3d(vec.x, vec.y, vec.z);
}

// Helper function to convert urdf::Rotation to Eigen::Quaterniond
Eigen::Quaterniond toEigen(const urdf::Rotation &rot) {
  double x, y, z, w;
  rot.getQuaternion(x, y, z, w);
  return Eigen::Quaterniond(w, x, y, z);
}

// Helper function to convert urdf::Pose to Eigen::Isometry3d
Eigen::Isometry3d toEigen(const urdf::Pose &pose) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(toEigen(pose.position));
  T.rotate(toEigen(pose.rotation));
  return T;
}

RobotModel loadRobotFromUrdf(const std::string &urdf_path) {
  // Resolve the path using utility
  std::string resolved_path = robot_sim::common::resolvePath(urdf_path);

  // 1. Parse the URDF file using urdfdom
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(resolved_path);

  if (!urdf_model) {
    throw std::runtime_error("Failed to parse URDF file: " + urdf_path);
  }

  std::cout << "Successfully parsed URDF file: " << urdf_model->getName()
            << std::endl;

  RobotModel robot_model;
  robot_model.setName(urdf_model->getName());
  robot_model.setRootLinkName(urdf_model->getRoot()->name);

  // 2. Iterate through all links in the URDF model
  for (auto const &[name, link_ptr] : urdf_model->links_) {
    LinkProperties link_props;
    link_props.name = link_ptr->name;

    // 2a. Convert Inertial properties
    if (link_ptr->inertial) {
      link_props.inertial.mass = link_ptr->inertial->mass;
      link_props.inertial.origin = toEigen(link_ptr->inertial->origin);
      const auto &i = link_ptr->inertial; // URDF inertia tensor
      link_props.inertial.inertia_tensor(0, 0) = i->ixx;
      link_props.inertial.inertia_tensor(0, 1) = i->ixy;
      link_props.inertial.inertia_tensor(0, 2) = i->ixz;
      link_props.inertial.inertia_tensor(1, 0) = i->ixy;
      link_props.inertial.inertia_tensor(1, 1) = i->iyy;
      link_props.inertial.inertia_tensor(1, 2) = i->iyz;
      link_props.inertial.inertia_tensor(2, 0) = i->ixz;
      link_props.inertial.inertia_tensor(2, 1) = i->iyz;
      link_props.inertial.inertia_tensor(2, 2) = i->izz;
    }

    // 2b. Convert Visual properties
    for (const auto &visual_ptr : link_ptr->visual_array) {
      Visual visual_prop;
      visual_prop.name = visual_ptr->name;
      visual_prop.origin = toEigen(visual_ptr->origin);

      if (visual_ptr->material) {
        visual_prop.material.name = visual_ptr->material->name;
        visual_prop.material.color = Eigen::Vector4d(
            visual_ptr->material->color.r, visual_ptr->material->color.g,
            visual_ptr->material->color.b, visual_ptr->material->color.a);
      }

      // Geometry
      if (visual_ptr->geometry->type == urdf::Geometry::BOX) {
        visual_prop.geometry.type = GeometryType::BOX;
        auto box = std::static_pointer_cast<urdf::Box>(visual_ptr->geometry);
        visual_prop.geometry.size = toEigen(box->dim);
      } else if (visual_ptr->geometry->type == urdf::Geometry::CYLINDER) {
        visual_prop.geometry.type = GeometryType::CYLINDER;
        auto cylinder =
            std::static_pointer_cast<urdf::Cylinder>(visual_ptr->geometry);
        visual_prop.geometry.size =
            Eigen::Vector3d(cylinder->radius, cylinder->length, 0);
      } else if (visual_ptr->geometry->type == urdf::Geometry::SPHERE) {
        visual_prop.geometry.type = GeometryType::SPHERE;
        auto sphere =
            std::static_pointer_cast<urdf::Sphere>(visual_ptr->geometry);
        visual_prop.geometry.size = Eigen::Vector3d(sphere->radius, 0, 0);
      } else if (visual_ptr->geometry->type == urdf::Geometry::MESH) {
        visual_prop.geometry.type = GeometryType::MESH;
        auto mesh = std::static_pointer_cast<urdf::Mesh>(visual_ptr->geometry);
        visual_prop.geometry.mesh_filename = mesh->filename;
        visual_prop.geometry.size = toEigen(mesh->scale);
      }
      link_props.visuals.push_back(visual_prop);
    }

    // 2c. Convert Collision properties (similar to visual)
    for (const auto &collision_ptr : link_ptr->collision_array) {
      Collision collision_prop;
      collision_prop.name = collision_ptr->name;
      collision_prop.origin = toEigen(collision_ptr->origin);

      // Geometry
      if (collision_ptr->geometry->type == urdf::Geometry::BOX) {
        collision_prop.geometry.type = GeometryType::BOX;
        auto box = std::static_pointer_cast<urdf::Box>(collision_ptr->geometry);
        collision_prop.geometry.size = toEigen(box->dim);
      } else if (collision_ptr->geometry->type == urdf::Geometry::CYLINDER) {
        collision_prop.geometry.type = GeometryType::CYLINDER;
        auto cylinder =
            std::static_pointer_cast<urdf::Cylinder>(collision_ptr->geometry);
        collision_prop.geometry.size =
            Eigen::Vector3d(cylinder->radius, cylinder->length, 0);
      } else if (collision_ptr->geometry->type == urdf::Geometry::SPHERE) {
        collision_prop.geometry.type = GeometryType::SPHERE;
        auto sphere =
            std::static_pointer_cast<urdf::Sphere>(collision_ptr->geometry);
        collision_prop.geometry.size = Eigen::Vector3d(sphere->radius, 0, 0);
      } else if (collision_ptr->geometry->type == urdf::Geometry::MESH) {
        collision_prop.geometry.type = GeometryType::MESH;
        auto mesh =
            std::static_pointer_cast<urdf::Mesh>(collision_ptr->geometry);
        collision_prop.geometry.mesh_filename = mesh->filename;
        collision_prop.geometry.size = toEigen(mesh->scale);
      }
      link_props.collisions.push_back(collision_prop);
    }

    robot_model.addLink(link_props);
  }

  // 3. Iterate through all joints in the URDF model
  for (auto const &[name, joint_ptr] : urdf_model->joints_) {
    JointProperties joint_props;
    joint_props.name = joint_ptr->name;
    joint_props.parent_link = joint_ptr->parent_link_name;
    joint_props.child_link = joint_ptr->child_link_name;
    joint_props.origin = toEigen(joint_ptr->parent_to_joint_origin_transform);
    joint_props.axis = toEigen(joint_ptr->axis);

    // Convert joint type
    switch (joint_ptr->type) {
    case urdf::Joint::REVOLUTE:
      joint_props.type = kinematics::JointType::Revolute;
      break;
    case urdf::Joint::CONTINUOUS:
      joint_props.type = kinematics::JointType::Revolute;
      break;
    case urdf::Joint::PRISMATIC:
      joint_props.type = kinematics::JointType::Prismatic;
      break;
    case urdf::Joint::FIXED:
      joint_props.type = kinematics::JointType::Fixed;
      break;
    case urdf::Joint::FLOATING:
      // KinematicChain does not support floating joints, treat as Fixed.
      joint_props.type = kinematics::JointType::Fixed;
      break;
    case urdf::Joint::PLANAR:
      // KinematicChain does not support planar joints, treat as Fixed.
      joint_props.type = kinematics::JointType::Fixed;
      break;
    default:
      joint_props.type = kinematics::JointType::Fixed;
      break;
    }

    // Convert limits
    if (joint_ptr->limits) {
      joint_props.limits.lower = joint_ptr->limits->lower;
      joint_props.limits.upper = joint_ptr->limits->upper;
      joint_props.limits.effort = joint_ptr->limits->effort;
      joint_props.limits.velocity = joint_ptr->limits->velocity;
    }

    // Convert dynamics
    if (joint_ptr->dynamics) {
      joint_props.dynamics.damping = joint_ptr->dynamics->damping;
      joint_props.dynamics.friction = joint_ptr->dynamics->friction;
    }

    robot_model.addJoint(joint_props);
  }

  return robot_model;
}

} // namespace simulation
