#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace grasping_system::core
{

enum class ObjectShapeKind
{
  kRigidMesh,
  kRigidPrimitive,
  kSoftObject,
  kArticulatedObject
};

enum class ObjectRepresentationKind
{
  kGraph,
  kVoxel,
  kPrimitive
};

struct GraspObject
{
  std::string object_id;
  std::string object_class;
  std::string reference_frame;
  ObjectShapeKind shape_kind{ObjectShapeKind::kRigidMesh};
  ObjectRepresentationKind representation_kind{ObjectRepresentationKind::kVoxel};
  geometry_msgs::msg::Pose pose_in_world{};
  std::string mesh_resource;
  double voxel_resolution{0.0};
  std::string graph_resource;
  std::string primitive_resource;

  bool valid() const noexcept { return !object_id.empty(); }
};

struct GraspAttachment
{
  std::string robot_name;
  std::string eef_link;
  std::string object_id;
  geometry_msgs::msg::Pose eef_in_world{};
  geometry_msgs::msg::Pose object_in_world{};
  geometry_msgs::msg::Pose object_in_eef{};
};

struct GraspRelation
{
  std::string parent_frame;
  std::string child_frame;
  geometry_msgs::msg::Pose child_in_parent{};
};

struct GraspCollisionPolicy
{
  std::vector<std::string> ignored_contact_links;
  std::vector<std::string> hard_collision_links;

  bool allowsContactLink(const std::string &link_name) const
  {
    return std::find(ignored_contact_links.begin(), ignored_contact_links.end(), link_name) !=
           ignored_contact_links.end();
  }

  bool isHardCollisionLink(const std::string &link_name) const
  {
    return std::find(hard_collision_links.begin(), hard_collision_links.end(), link_name) !=
           hard_collision_links.end();
  }
};

}  // namespace grasping_system::core
