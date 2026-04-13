#include "simulation/robot/robot_collision_model.hpp"
#include <iostream>
#include <stdexcept>

namespace simulation {

RobotCollisionModel
createCollisionModelFromRobot(const RobotModel &model,
                              const std::string &leaf_link_name) {
  RobotCollisionModel collision_model;

  // 1. Traverse to build the child-to-parent and child-link-to-joint maps (same
  // as KinematicAdapter)
  std::map<std::string, std::string> child_to_parent_link;
  std::map<std::string, const JointProperties *> child_link_to_joint;
  for (const auto &kv : model.getJoints()) {
    const auto &joint_props = kv.second;
    child_to_parent_link[joint_props.child_link] = joint_props.parent_link;
    child_link_to_joint[joint_props.child_link] = &joint_props;
  }

  std::string leaf_name = leaf_link_name;
  if (leaf_name.empty()) {
    // Find a link that is not a parent to any joint (simplified logic)
    for (const auto &kv : model.getLinks()) {
      const auto &link_name = kv.first;
      bool is_parent = false;
      for (const auto &j_kv : model.getJoints()) {
        if (j_kv.second.parent_link == link_name) {
          is_parent = true;
          break;
        }
      }
      if (!is_parent && link_name != model.getRootLinkName()) {
        leaf_name = link_name;
        break;
      }
    }
  }

  if (leaf_name.empty()) {
    throw std::runtime_error("Could not determine the end-effector link for "
                             "collision model construction.");
  }

  // 2. Traverse from leaf back to root
  std::vector<std::string> chain_links_reversed;
  std::string current_link_name = leaf_name;
  while (current_link_name != model.getRootLinkName() &&
         !current_link_name.empty()) {
    chain_links_reversed.push_back(current_link_name);
    auto it = child_to_parent_link.find(current_link_name);
    if (it != child_to_parent_link.end()) {
      current_link_name = it->second;
    } else {
      break;
    }
  }

  // 3. Process links in base-to-tip order

  int joint_index = 1;
  for (auto it = chain_links_reversed.rbegin();
       it != chain_links_reversed.rend(); ++it, ++joint_index) {
    const std::string &link_name = *it;
    const LinkProperties *link_props = model.getLink(link_name);
    if (!link_props)
      continue;

    // Add all collisions for this link
    for (const auto &collision : link_props->collisions) {
      LinkCollisionGeometry link_geom;
      link_geom.link_name = link_name;
      link_geom.parent_joint_index = joint_index;
      link_geom.geometry = collision.geometry;
      link_geom.origin_offset = collision.origin;
      link_geom.inertial_origin = link_props->inertial.origin;
      collision_model.addCollisionGeometry(link_geom);
    }

    // If no collisions, check visuals as fallback
    if (link_props->collisions.empty()) {
      for (const auto &visual : link_props->visuals) {
        LinkCollisionGeometry link_geom;
        link_geom.link_name = link_name;
        link_geom.parent_joint_index = joint_index;
        link_geom.geometry = visual.geometry;
        link_geom.origin_offset = visual.origin;
        link_geom.inertial_origin = link_props->inertial.origin;
        collision_model.addCollisionGeometry(link_geom);
      }
    }
  }

  return collision_model;
}

} // namespace simulation
