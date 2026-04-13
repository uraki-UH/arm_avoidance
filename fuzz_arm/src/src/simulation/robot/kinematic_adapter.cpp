#include "simulation/robot/kinematic_adapter.hpp"
#include <iostream>
#include <stdexcept>

namespace simulation {

kinematics::KinematicChain
createKinematicChainFromModel(const RobotModel &model,
                              const std::string &end_effector_name,
                              const Eigen::Vector3d &base_position) {
  kinematics::KinematicChain chain;
  chain.setBase(base_position,
                Eigen::Quaterniond::Identity()); // Set base position

  // 1. Build maps for easier traversal
  std::map<std::string, std::string> child_to_parent_link;
  std::map<std::string, const JointProperties *> child_link_to_joint;
  for (const auto &[joint_name, joint_props] : model.getJoints()) {
    child_to_parent_link[joint_props.child_link] = joint_props.parent_link;
    child_link_to_joint[joint_props.child_link] = &joint_props;
  }

  // 2. Find the end-effector link
  std::string leaf_name = end_effector_name;
  if (leaf_name.empty()) {
    // Find a link that is not a parent to any joint
    for (const auto &[link_name, link_props] : model.getLinks()) {
      bool is_parent = false;
      for (const auto &[j_name, j_props] : model.getJoints()) {
        if (j_props.parent_link == link_name) {
          is_parent = true;
          break;
        }
      }
      if (!is_parent && link_name != model.getRootLinkName()) {
        leaf_name = link_name;
        break; // Assuming one end-effector for now
      }
    }
  }

  if (leaf_name.empty()) {
    throw std::runtime_error(
        "Could not determine the end-effector link of the chain.");
  }

  std::cout << "Building KinematicChain up to leaf link: " << leaf_name
            << std::endl;

  // 3. Traverse from end-effector back to the root to get the correct order
  std::vector<const JointProperties *> chain_joints_reversed;
  std::string current_link_name = leaf_name;

  while (current_link_name != model.getRootLinkName() &&
         !current_link_name.empty()) {
    const JointProperties *joint = child_link_to_joint[current_link_name];
    if (!joint) {
      throw std::runtime_error(
          "Chain is broken. No joint found for child link: " +
          current_link_name);
    }
    chain_joints_reversed.push_back(joint);
    current_link_name = joint->parent_link;
  }

  // 4. Add segments to the KinematicChain in base-to-tip order
  for (auto it = chain_joints_reversed.rbegin();
       it != chain_joints_reversed.rend(); ++it) {
    const JointProperties *sim_joint_props = *it;
    const LinkProperties *sim_link_props =
        model.getLink(sim_joint_props->child_link);
    if (!sim_link_props) {
      throw std::runtime_error("Link not found in model: " +
                               sim_joint_props->child_link);
    }

    // Convert simulation::LinkProperties -> kinematics::Link
    kinematics::Link kin_link;
    kin_link.name = sim_link_props->name;

    // The "vector" of a kinematic link is the transform from the previous
    // joint's origin to the current joint's origin. This is defined in the
    // current joint's <origin> tag in URDF.
    kin_link.vector = sim_joint_props->origin.translation();

    // Convert simulation::JointProperties -> kinematics::Joint
    kinematics::Joint kin_joint;
    kin_joint.name = sim_joint_props->name;
    kin_joint.type = static_cast<kinematics::JointType>(sim_joint_props->type);
    kin_joint.local_rotation =
        Eigen::Quaterniond(sim_joint_props->origin.rotation());
    kin_joint.axis1 = sim_joint_props->axis;

    // Add limits
    if (sim_joint_props->limits.lower < sim_joint_props->limits.upper) {
      kin_joint.min_limits = {sim_joint_props->limits.lower};
      kin_joint.max_limits = {sim_joint_props->limits.upper};
    }

    // Add the joint and link to the chain
    chain.addSegment(kin_link, kin_joint);
  }

  const LinkProperties *leaf_props = model.getLink(leaf_name);
  if (leaf_props) {
    Eigen::Vector3d center_offset = Eigen::Vector3d::Zero();
    Eigen::Vector3d geom_size = Eigen::Vector3d::Zero();
    bool found_geom = false;

    if (!leaf_props->visuals.empty()) {
      center_offset = leaf_props->visuals[0].origin.translation();
      geom_size = leaf_props->visuals[0].geometry.size;
      found_geom = true;
    } else if (!leaf_props->collisions.empty()) {
      center_offset = leaf_props->collisions[0].origin.translation();
      geom_size = leaf_props->collisions[0].geometry.size;
      found_geom = true;
    }

    Eigen::Vector3d tip_offset = center_offset;
    if (found_geom) {
      auto g_type = (!leaf_props->visuals.empty())
                        ? leaf_props->visuals[0].geometry.type
                        : leaf_props->collisions[0].geometry.type;

      if (g_type == GeometryType::BOX) {
        tip_offset.z() += geom_size.z() / 2.0;
      } else if (g_type == GeometryType::CYLINDER) {
        tip_offset.z() += geom_size.y() / 2.0; // Cylinder length stored in Y
      } else if (g_type == GeometryType::SPHERE) {
        tip_offset.z() += geom_size.x(); // Sphere radius stored in X
      } else {
        // Fallback or default for other types
        tip_offset.z() += geom_size.z() / 2.0;
      }
    }

    chain.setEEFOffset(tip_offset);
    std::cout << "[DEBUG Adapter] Set EEF Offset (Tip) for '" << leaf_name
              << "': " << tip_offset.transpose() << " (Center was "
              << center_offset.transpose() << ")" << std::endl;
  }

  return chain;
}

} // namespace simulation