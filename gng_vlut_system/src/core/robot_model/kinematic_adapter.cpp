#include "robot_model/kinematic_adapter.hpp"
#include <cstddef>
#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

namespace simulation {

void completeMissingBranchLinkTransforms(
    const RobotModel &model,
    const std::map<std::string, double> &joint_value_hints,
    std::map<std::string, Eigen::Isometry3d> &link_transforms) {
  const auto joint_value = [&](const std::string &name) {
    const auto it = joint_value_hints.find(name);
    return it == joint_value_hints.end() ? 0.0 : it->second;
  };
  const auto joint_motion = [](const JointProperties &joint, double value) {
    Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
    if (joint.type == kinematics::JointType::Revolute) {
      motion.rotate(Eigen::AngleAxisd(value, joint.axis.normalized()));
    } else if (joint.type == kinematics::JointType::Prismatic) {
      motion.translate(joint.axis.normalized() * value);
    }
    return motion;
  };

  bool changed = true;
  std::size_t iteration = 0;
  while (changed && iteration++ < model.getJoints().size() + 1) {
    changed = false;
    for (const auto &[name, joint] : model.getJoints()) {
      (void)name;
      if (link_transforms.count(joint.child_link) > 0) {
        continue;
      }
      const auto parent = link_transforms.find(joint.parent_link);
      if (parent == link_transforms.end()) {
        continue;
      }

      double value = joint_value(joint.name);
      if (joint.has_mimic) {
        value = joint_value(joint.mimic_joint_name) * joint.mimic_multiplier +
                joint.mimic_offset;
      }
      link_transforms[joint.child_link] =
          parent->second * joint.origin * joint_motion(joint, value);
      changed = true;
    }
  }
}

kinematics::KinematicChain
createKinematicChainFromModel(const RobotModel &model,
                              const std::string &end_effector_name,
                              const Eigen::Vector3d &base_position,
                              const std::string &root_link_name) {
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

  // 3. Traverse from end-effector back to the TRUE model root
  std::vector<const JointProperties *> total_path_reversed;
  std::string path_link = leaf_name;
  std::string true_root = model.getRootLinkName();

  while (path_link != true_root && !path_link.empty()) {
    const JointProperties *joint = child_link_to_joint[path_link];
    if (!joint) break;
    total_path_reversed.push_back(joint);
    path_link = joint->parent_link;
  }

  // 4. Calculate base offset up to root_link_name and build segments after that
  std::string target_root = root_link_name.empty() ? true_root : root_link_name;
  Eigen::Isometry3d accumulated_base_tf = Eigen::Isometry3d::Identity();
  accumulated_base_tf.translate(base_position);

  bool reached_target_root = (true_root == target_root);
  
  for (auto it = total_path_reversed.rbegin(); it != total_path_reversed.rend(); ++it) {
    const JointProperties *sim_joint = *it;

    if (!reached_target_root) {
      // Accumulate transform into base offset
      accumulated_base_tf = accumulated_base_tf * sim_joint->origin;
      if (sim_joint->child_link == target_root) {
        reached_target_root = true;
      }
      continue;
    }

    // After target_root, add as segments
    const LinkProperties *sim_link_props = model.getLink(sim_joint->child_link);
    if (!sim_link_props) {
      throw std::runtime_error("Link not found: " + sim_joint->child_link);
    }

    kinematics::Link kin_link;
    kin_link.name = sim_link_props->name;
    kin_link.vector = sim_joint->origin.translation();

    kinematics::Joint kin_joint;
    kin_joint.name = sim_joint->name;
    kin_joint.type = static_cast<kinematics::JointType>(sim_joint->type);
    kin_joint.local_rotation = Eigen::Quaterniond(sim_joint->origin.rotation());
    kin_joint.axis1 = sim_joint->axis;

    if (sim_joint->limits.lower < sim_joint->limits.upper) {
      kin_joint.min_limits = {sim_joint->limits.lower};
      kin_joint.max_limits = {sim_joint->limits.upper};
    }
    chain.addSegment(kin_link, kin_joint);
  }

  // Set the calculated base pose
  chain.setBase(accumulated_base_tf.translation(), Eigen::Quaterniond(accumulated_base_tf.rotation()));

  if (!reached_target_root && !root_link_name.empty()) {
     std::cerr << "[WARNING Adapter] root_link_name '" << root_link_name << "' was not found in the path from '" << leaf_name << "' to root!" << std::endl;
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
  }

  return chain;
}

std::unique_ptr<kinematics::KinematicChain>
createMultiArmKinematicChain(
    const RobotModel &model,
    const std::vector<ArmConfig> &arm_configs,
    const Eigen::Vector3d &base_position) {
  if (arm_configs.empty()) {
    throw std::runtime_error("No arm configurations were provided.");
  }

  std::vector<MultiArmKinematicAdapter::ArmEntry,
              Eigen::aligned_allocator<MultiArmKinematicAdapter::ArmEntry>>
      arms;
  arms.reserve(arm_configs.size());
  for (const auto &cfg : arm_configs) {
    MultiArmKinematicAdapter::ArmEntry arm_entry;
    arm_entry.chain = createKinematicChainFromModel(
        model, cfg.leaf_link, base_position, cfg.root_link);
    arm_entry.prefix = cfg.prefix;
    // Store the calculated base as relative offset
    arm_entry.relative_base_pos = arm_entry.chain.getBasePosition();
    arm_entry.relative_base_ori = arm_entry.chain.getBaseOrientation();
    arms.push_back(std::move(arm_entry));
  }

  auto adapter =
      std::make_unique<MultiArmKinematicAdapter>(std::move(arms), 0);
  adapter->setBase(base_position, Eigen::Quaterniond::Identity());
  return adapter;
}

std::unique_ptr<kinematics::KinematicChain>
createMultiArmKinematicChainFromModels(
    const RobotModel &model, const std::vector<std::string> &end_effector_names,
    const std::vector<std::string> &prefixes,
    const Eigen::Vector3d &base_position,
    const std::string &root_link_name) {
  
  std::vector<ArmConfig> configs;
  for (size_t i = 0; i < end_effector_names.size(); ++i) {
    ArmConfig cfg;
    cfg.root_link = root_link_name;
    cfg.leaf_link = end_effector_names[i];
    if (i < prefixes.size()) {
      cfg.prefix = prefixes[i];
    } else {
      // 未指定時はURDF上の素のリンク名をそのまま使う。
      // offline URDF trainer や viewer bridge では、voxel/TF 側の名前と
      // 一致させるために prefix を付けない方が自然。
      cfg.prefix = "";
    }
    configs.push_back(cfg);
  }
  return createMultiArmKinematicChain(model, configs, base_position);
}


MultiArmKinematicAdapter::MultiArmKinematicAdapter(
    std::vector<ArmEntry, Eigen::aligned_allocator<ArmEntry>> arms,
    std::size_t primary_arm_index)
    : arms_(std::move(arms)) {
  if (arms_.empty()) {
    throw std::runtime_error("MultiArmKinematicAdapter requires at least one arm.");
  }
  primary_arm_index_ =
      std::min(primary_arm_index, arms_.size() - 1);
  if (primary_arm_index_ != 0) {
    auto primary = std::move(arms_[primary_arm_index_]);
    arms_.erase(arms_.begin() + static_cast<std::ptrdiff_t>(primary_arm_index_));
    arms_.insert(arms_.begin(), std::move(primary));
  }
  primary_arm_index_ = 0;
  syncCachedState();
}

void MultiArmKinematicAdapter::updateJointValuesByName(const std::vector<std::string> &names, const std::vector<double> &values) {
  for (auto &arm : arms_) {
    // 各腕の updateJointValuesByName を呼び出す
    // 名前がプレフィックス付きで送られてくることを想定
    // ただし、子アーム側ではプレフィックスなしの名前を期待している可能性があるため
    // プレフィックスを剥がして渡すか、あるいは子アーム側で判定させる
    
    std::vector<std::string> local_names;
    std::vector<double> local_values;
    
    for (size_t i = 0; i < names.size(); ++i) {
      if (arm.prefix.empty() || startsWith(names[i], arm.prefix)) {
        std::string stripped_name = names[i];
        if (!arm.prefix.empty()) {
            stripped_name = names[i].substr(arm.prefix.length());
        }
        local_names.push_back(stripped_name);
        local_values.push_back(values[i]);
      }
    }
    
    if (!local_names.empty()) {
        arm.chain.updateJointValuesByName(local_names, local_values);
    }
  }
  forwardKinematics();
}

void MultiArmKinematicAdapter::setBase(const Eigen::Vector3d &position,
                                       const Eigen::Quaterniond &orientation) {
    base_position_ = position;
    base_orientation_ = orientation;
    
    for (auto &arm : arms_) {
        // Compose global base with relative arm offset
        Eigen::Vector3d arm_pos = orientation * arm.relative_base_pos + position;
        Eigen::Quaterniond arm_ori = orientation * arm.relative_base_ori;
        arm.chain.setBase(arm_pos, arm_ori);
    }
  forwardKinematics();
}

std::size_t MultiArmKinematicAdapter::armOffset(std::size_t arm_index) const {
  std::size_t offset = 0;
  for (std::size_t i = 0; i < arm_index && i < arms_.size(); ++i) {
    offset += static_cast<std::size_t>(arms_[i].chain.getTotalDOF());
  }
  return offset;
}

std::size_t
MultiArmKinematicAdapter::armPositionOffset(std::size_t arm_index) const {
  std::size_t offset = 0;
  for (std::size_t i = 0; i < arm_index && i < arms_.size(); ++i) {
    offset += arms_[i].chain.getLinkPositions().size();
  }
  return offset;
}

std::size_t MultiArmKinematicAdapter::armIndexForJoint(int joint_index) const {
  if (joint_index < 0) {
    return arms_.size();
  }
  std::size_t cursor = 0;
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    int n = arms_[i].chain.getNumJoints();
    if (joint_index < static_cast<int>(cursor + n)) {
      return i;
    }
    cursor += static_cast<std::size_t>(n);
  }
  return arms_.size();
}

std::size_t MultiArmKinematicAdapter::armIndexForLink(int link_index) const {
  return armIndexForJoint(link_index);
}

std::size_t
MultiArmKinematicAdapter::armIndexForPositionIndex(int position_index) const {
  if (position_index < 0) {
    return arms_.size();
  }
  std::size_t cursor = 0;
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    const std::size_t block_size = arms_[i].chain.getLinkPositions().size();
    if (position_index < static_cast<int>(cursor + block_size)) {
      return i;
    }
    cursor += block_size;
  }
  return arms_.size();
}

bool MultiArmKinematicAdapter::setJointValues(const std::vector<double> &values) {
  const std::size_t total_dof = static_cast<std::size_t>(getTotalDOF());
  if (values.size() > total_dof) {
    return false;
  }

  std::vector<double> full = getJointValues();
  if (full.size() != total_dof) {
    full.assign(total_dof, 0.0);
  }
  std::copy(values.begin(), values.end(), full.begin());

  std::size_t offset = 0;
  for (auto &arm : arms_) {
    const std::size_t dof = static_cast<std::size_t>(arm.chain.getTotalDOF());
    std::vector<double> slice(full.begin() + offset, full.begin() + offset + dof);
    arm.chain.setJointValues(slice);
    offset += dof;
  }
  forwardKinematics();
  return true;
}

std::vector<double> MultiArmKinematicAdapter::getJointValues() const {
  std::vector<double> values;
  for (const auto &arm : arms_) {
    auto arm_values = arm.chain.getJointValues();
    values.insert(values.end(), arm_values.begin(), arm_values.end());
  }
  return values;
}

bool MultiArmKinematicAdapter::isWithinLimits(
    const std::vector<double> &values) const {
  if (values.size() != static_cast<std::size_t>(getTotalDOF())) {
    return false;
  }
  std::size_t offset = 0;
  for (const auto &arm : arms_) {
    const std::size_t dof = static_cast<std::size_t>(arm.chain.getTotalDOF());
    std::vector<double> slice(values.begin() + offset, values.begin() + offset + dof);
    if (!arm.chain.isWithinLimits(slice)) {
      return false;
    }
    offset += dof;
  }
  return true;
}

void MultiArmKinematicAdapter::clampToLimits(std::vector<double> &values) const {
  if (values.size() != static_cast<std::size_t>(getTotalDOF())) {
    return;
  }
  std::size_t offset = 0;
  for (const auto &arm : arms_) {
    const std::size_t dof = static_cast<std::size_t>(arm.chain.getTotalDOF());
    std::vector<double> slice(values.begin() + offset, values.begin() + offset + dof);
    arm.chain.clampToLimits(slice);
    std::copy(slice.begin(), slice.end(), values.begin() + offset);
    offset += dof;
  }
}

void MultiArmKinematicAdapter::updateKinematics(
    const std::vector<double> &values) {
  if (setJointValues(values)) {
    forwardKinematics();
  }
}

void MultiArmKinematicAdapter::forwardKinematicsAt(
    const std::vector<double> &values,
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        &out_positions,
    std::vector<Eigen::Quaterniond,
                Eigen::aligned_allocator<Eigen::Quaterniond>>
        &out_orientations) const {
  std::vector<double> full = getJointValues();
  if (full.size() != static_cast<std::size_t>(getTotalDOF())) {
    full.assign(static_cast<std::size_t>(getTotalDOF()), 0.0);
  }
  std::copy(values.begin(), values.begin() + std::min(values.size(), full.size()),
            full.begin());

  out_positions.clear();
  out_orientations.clear();

  auto append_arm = [&](const ArmEntry &arm, std::size_t offset) {
    std::vector<double> slice;
    const std::size_t dof = static_cast<std::size_t>(arm.chain.getTotalDOF());
    slice.assign(full.begin() + offset, full.begin() + offset + dof);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        arm_positions;
    std::vector<Eigen::Quaterniond,
                Eigen::aligned_allocator<Eigen::Quaterniond>>
        arm_orientations;
    arm.chain.forwardKinematicsAt(slice, arm_positions, arm_orientations);

    if (arm_positions.empty()) {
      return;
    }

    out_positions.insert(out_positions.end(), arm_positions.begin(),
                         arm_positions.end());
    out_orientations.insert(out_orientations.end(), arm_orientations.begin(),
                            arm_orientations.end());
  };

  std::size_t offset = 0;
  if (primary_arm_index_ < arms_.size()) {
    append_arm(arms_[primary_arm_index_], offset);
    offset += static_cast<std::size_t>(arms_[primary_arm_index_].chain.getTotalDOF());
  }
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    if (i == primary_arm_index_) {
      continue;
    }
    append_arm(arms_[i], offset);
    offset += static_cast<std::size_t>(arms_[i].chain.getTotalDOF());
  }
}

void MultiArmKinematicAdapter::forwardKinematicsAt(
    const std::vector<double> &values) {
  if (setJointValues(values)) {
      forwardKinematics();
  }
}

void MultiArmKinematicAdapter::forwardKinematics() {
  for (auto &arm : arms_) {
    arm.chain.forwardKinematics();
  }
  syncCachedState();
}

Eigen::MatrixXd MultiArmKinematicAdapter::calculateJacobianAt(
    int target_joint_index, const std::vector<double> &values) const {
  if (arms_.empty()) {
    return Eigen::MatrixXd::Zero(6, 0);
  }

  if (target_joint_index < 0) {
    return Eigen::MatrixXd::Zero(6, getTotalDOF());
  }

  const int link_index = target_joint_index - 1;
  const std::size_t arm_index = armIndexForLink(link_index);
  if (arm_index >= arms_.size()) {
    return Eigen::MatrixXd::Zero(6, getTotalDOF());
  }

  std::size_t link_offset = 0;
  for (std::size_t i = 0; i < arm_index && i < arms_.size(); ++i) {
    link_offset += static_cast<std::size_t>(arms_[i].chain.getNumJoints());
  }

  const std::size_t dof_offset = armOffset(arm_index);
  const auto &arm = arms_[arm_index].chain;

  const std::size_t arm_positions = arm.getLinkPositions().size();
  const int local_target = link_index - static_cast<int>(link_offset) + 1;
  if (local_target < 0 ||
      local_target >= static_cast<int>(arm_positions)) {
    return Eigen::MatrixXd::Zero(6, getTotalDOF());
  }

  const std::size_t arm_dof = static_cast<std::size_t>(arm.getTotalDOF());

  std::vector<double> full = getJointValues();
  if (full.size() != static_cast<std::size_t>(getTotalDOF())) {
    full.assign(static_cast<std::size_t>(getTotalDOF()), 0.0);
  }
  if (values.size() >= full.size()) {
    std::copy(values.begin(), values.begin() + full.size(), full.begin());
  } else {
    std::copy(values.begin(), values.end(), full.begin());
  }

  std::vector<double> local_values(full.begin() + dof_offset,
                                   full.begin() + dof_offset + arm_dof);
  Eigen::MatrixXd J_local =
      arm.calculateJacobianAt(local_target, local_values);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, getTotalDOF());
  if (J_local.cols() == static_cast<int>(arm_dof)) {
    J.block(0, static_cast<int>(dof_offset), 6, J_local.cols()) = J_local;
  }
  return J;
}

Eigen::MatrixXd
MultiArmKinematicAdapter::calculateJacobian(int target_joint_index) const {
  return calculateJacobianAt(target_joint_index, getJointValues());
}

int MultiArmKinematicAdapter::getNumJoints() const {
  int total = 0;
  for (const auto &arm : arms_) {
    total += arm.chain.getNumJoints();
  }
  return total;
}

int MultiArmKinematicAdapter::getTotalDOF() const {
  int total = 0;
  for (const auto &arm : arms_) {
    total += arm.chain.getTotalDOF();
  }
  return total;
}

std::size_t MultiArmKinematicAdapter::getArmCount() const {
  return arms_.size();
}

Eigen::Vector3d
MultiArmKinematicAdapter::getJointPosition(int joint_index) const {
  if (joint_index < 0 || joint_index >= static_cast<int>(cached_positions_.size())) {
    return Eigen::Vector3d::Zero();
  }
  return cached_positions_[joint_index];
}

Eigen::Quaterniond
MultiArmKinematicAdapter::getJointOrientation(int joint_index) const {
  if (joint_index < 0 ||
      joint_index >= static_cast<int>(cached_orientations_.size())) {
    return Eigen::Quaterniond::Identity();
  }
  return cached_orientations_[joint_index];
}

std::string MultiArmKinematicAdapter::getJointName(int joint_index) const {
  std::size_t cursor = 0;
  for (const auto &arm : arms_) {
    int n = arm.chain.getNumJoints();
    if (joint_index < static_cast<int>(cursor + n)) {
        return arm.prefix + arm.chain.getJointName(joint_index - static_cast<int>(cursor));
    }
    cursor += static_cast<std::size_t>(n);
  }
  return "";
}

int MultiArmKinematicAdapter::getJointDOF(int joint_index) const {
  std::size_t cursor = 0;
  for (const auto &arm : arms_) {
    int n = arm.chain.getNumJoints();
    if (joint_index < static_cast<int>(cursor + n)) {
      return arm.chain.getJointDOF(joint_index - static_cast<int>(cursor));
    }
    cursor += static_cast<std::size_t>(n);
  }
  return 0;
}

std::string MultiArmKinematicAdapter::getLinkName(int link_index) const {
  std::size_t cursor = 0;
  for (const auto &arm : arms_) {
    int n = arm.chain.getNumJoints();
    if (link_index < static_cast<int>(cursor + n)) {
        return arm.prefix + arm.chain.getLinkName(link_index - static_cast<int>(cursor));
    }
    cursor += static_cast<std::size_t>(n);
  }
  return "";
}

Eigen::Vector3d MultiArmKinematicAdapter::getEEFPosition() const {
  if (arms_.empty()) {
    return base_position_;
  }
  return arms_[primary_arm_index_].chain.getEEFPosition();
}

Eigen::Vector3d
MultiArmKinematicAdapter::getEEFPosition(std::size_t arm_index) const {
  if (arms_.empty() || arm_index >= arms_.size()) {
    return base_position_;
  }
  return arms_[arm_index].chain.getEEFPosition();
}

Eigen::Quaterniond MultiArmKinematicAdapter::getEEFOrientation() const {
  if (arms_.empty()) {
    return base_orientation_;
  }
  return arms_[primary_arm_index_].chain.getEEFOrientation();
}

Eigen::Quaterniond
MultiArmKinematicAdapter::getEEFOrientation(std::size_t arm_index) const {
  if (arms_.empty() || arm_index >= arms_.size()) {
    return base_orientation_;
  }
  return arms_[arm_index].chain.getEEFOrientation();
}

const std::vector<Eigen::Vector3d,
                  Eigen::aligned_allocator<Eigen::Vector3d>> &
MultiArmKinematicAdapter::getLinkPositions() const {
  syncCachedState();
  return cached_positions_;
}

const std::vector<Eigen::Quaterniond,
                  Eigen::aligned_allocator<Eigen::Quaterniond>> &
MultiArmKinematicAdapter::getLinkOrientations() const {
  syncCachedState();
  return cached_orientations_;
}

void MultiArmKinematicAdapter::setEEFOffset(const Eigen::Vector3d &offset) {
  if (!arms_.empty()) {
    arms_[primary_arm_index_].chain.setEEFOffset(offset);
  }
}

Eigen::Vector3d MultiArmKinematicAdapter::getEEFOffset() const {
  if (arms_.empty()) {
    return Eigen::Vector3d::Zero();
  }
  return arms_[primary_arm_index_].chain.getEEFOffset();
}

bool MultiArmKinematicAdapter::startsWith(const std::string &value,
                                          const std::string &prefix) {
  return value.rfind(prefix, 0) == 0;
}

void MultiArmKinematicAdapter::buildAllLinkTransforms(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
    const std::vector<Eigen::Quaterniond,
                      Eigen::aligned_allocator<Eigen::Quaterniond>> &orientations,
    const std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
        &fixed_link_info,
    std::map<std::string, Eigen::Isometry3d> &link_transforms) const {
  link_transforms.clear();

  std::size_t offset = 0;
  auto process_arm = [&](const ArmEntry &arm, std::size_t arm_offset) {
    const auto &default_positions = arm.chain.getLinkPositions();
    const auto &default_orientations = arm.chain.getLinkOrientations();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        arm_positions;
    std::vector<Eigen::Quaterniond,
                Eigen::aligned_allocator<Eigen::Quaterniond>>
        arm_orientations;

    const std::size_t block_size = default_positions.size();
    if (positions.size() >= arm_offset + block_size &&
        orientations.size() >= arm_offset + block_size && block_size > 0) {
      arm_positions.assign(positions.begin() + arm_offset,
                           positions.begin() + arm_offset + block_size);
      arm_orientations.assign(orientations.begin() + arm_offset,
                              orientations.begin() + arm_offset + block_size);
    } else {
      arm_positions = default_positions;
      arm_orientations = default_orientations;
    }

    std::map<std::string, Eigen::Isometry3d> arm_map;
    arm.chain.buildAllLinkTransforms(arm_positions, arm_orientations,
                                     fixed_link_info, arm_map);
    auto is_identity = [](const Eigen::Isometry3d &tf) {
      return tf.matrix().isApprox(Eigen::Isometry3d::Identity().matrix(), 1e-9);
    };
    for (const auto &[name, tf] : arm_map) {
      const std::string key = arm.prefix + name;
      auto it = link_transforms.find(key);
      if (it == link_transforms.end()) {
        link_transforms.emplace(key, tf);
        continue;
      }
      if (is_identity(tf) && !is_identity(it->second)) {
        continue;
      }
      if (!is_identity(tf) || is_identity(it->second)) {
        it->second = tf;
      }
    }
  };

  for (const auto &arm : arms_) {
    process_arm(arm, offset);
    offset += arm.chain.getLinkPositions().size();
  }
}

std::vector<double> MultiArmKinematicAdapter::sampleRandomJointValues() const {
  std::vector<double> values;
  for (const auto &arm : arms_) {
    auto arm_values = arm.chain.sampleRandomJointValues();
    values.insert(values.end(), arm_values.begin(), arm_values.end());
  }
  return values;
}

void MultiArmKinematicAdapter::sampleRandomJointValues(
    std::vector<double> &out_values) const {
  out_values.clear();
  for (const auto &arm : arms_) {
    std::vector<double> arm_values;
    arm.chain.sampleRandomJointValues(arm_values);
    out_values.insert(out_values.end(), arm_values.begin(), arm_values.end());
  }
}

std::vector<double>
MultiArmKinematicAdapter::sampleRandomJointValue(int joint_index) const {
  std::vector<double> values = getJointValues();
  std::size_t cursor = 0;
  std::size_t offset = 0;
  for (const auto &arm : arms_) {
    int n = arm.chain.getNumJoints();
    if (joint_index < static_cast<int>(cursor + n)) {
      auto sample = arm.chain.sampleRandomJointValue(joint_index - static_cast<int>(cursor));
      std::copy(sample.begin(), sample.end(), values.begin() + offset);
      return values;
    }
    cursor += static_cast<std::size_t>(n);
    offset += static_cast<std::size_t>(arm.chain.getTotalDOF());
  }
  return values;
}

void MultiArmKinematicAdapter::sampleRandomJointValue(
    int joint_index, std::vector<double> &out_values) const {
  out_values = getJointValues();
  std::size_t cursor = 0;
  std::size_t offset = 0;
  for (const auto &arm : arms_) {
    int n = arm.chain.getNumJoints();
    if (joint_index < static_cast<int>(cursor + n)) {
      std::vector<double> sample;
      arm.chain.sampleRandomJointValue(joint_index - static_cast<int>(cursor),
                                       sample);
      std::copy(sample.begin(), sample.end(), out_values.begin() + offset);
      return;
    }
    cursor += static_cast<std::size_t>(n);
    offset += static_cast<std::size_t>(arm.chain.getTotalDOF());
  }
}

std::vector<double> MultiArmKinematicAdapter::sampleRandomJointValues(
    const std::vector<int> &joint_indices) const {
  std::vector<double> values = getJointValues();
  std::size_t cursor = 0;
  std::size_t offset = 0;
  for (const auto &arm : arms_) {
    std::vector<int> local;
    int n = arm.chain.getNumJoints();
    for (int idx : joint_indices) {
      if (idx >= static_cast<int>(cursor) &&
          idx < static_cast<int>(cursor + n)) {
        local.push_back(idx - static_cast<int>(cursor));
      }
    }
    if (!local.empty()) {
      auto sample = arm.chain.sampleRandomJointValues(local);
      std::copy(sample.begin(), sample.end(), values.begin() + offset);
    }
    cursor += static_cast<std::size_t>(n);
    offset += static_cast<std::size_t>(arm.chain.getTotalDOF());
  }
  return values;
}

void MultiArmKinematicAdapter::sampleRandomJointValues(
    const std::vector<int> &joint_indices,
    std::vector<double> &out_values) const {
  out_values = getJointValues();
  std::size_t cursor = 0;
  std::size_t offset = 0;
  for (const auto &arm : arms_) {
    std::vector<int> local;
    int n = arm.chain.getNumJoints();
    for (int idx : joint_indices) {
      if (idx >= static_cast<int>(cursor) &&
          idx < static_cast<int>(cursor + n)) {
        local.push_back(idx - static_cast<int>(cursor));
      }
    }
    if (!local.empty()) {
      std::vector<double> sample;
      arm.chain.sampleRandomJointValues(local, sample);
      std::copy(sample.begin(), sample.end(), out_values.begin() + offset);
    }
    cursor += static_cast<std::size_t>(n);
    offset += static_cast<std::size_t>(arm.chain.getTotalDOF());
  }
}

void MultiArmKinematicAdapter::syncCachedState() const {
  cached_positions_.clear();
  cached_orientations_.clear();
  for (const auto &arm : arms_) {
    const auto &p = arm.chain.getLinkPositions();
    const auto &o = arm.chain.getLinkOrientations();
    cached_positions_.insert(cached_positions_.end(), p.begin(), p.end());
    cached_orientations_.insert(cached_orientations_.end(), o.begin(), o.end());
  }
}

} // namespace simulation
