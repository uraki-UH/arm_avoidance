#include "kinematics/kinematic_chain.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

namespace kinematics {

// 内部で使うヘルパ：全DOFベクトルを取得（現在値）
static std::vector<double> buildCurrentValuesVector(
    const std::vector<Joint, Eigen::aligned_allocator<Joint>> &joints,
    int total_dof) {
  std::vector<double> vals;
  vals.reserve(total_dof);
  for (const auto &joint : joints) {
    for (int i = 0; i < joint.getDOF(); ++i)
      vals.push_back(joint.values[i]);
  }
  return vals;
}

// ヘルパ: 引数 values が部分ベクトルでも total_dof 長の配列を作る（out を上書き）
static void buildFullValues(
    const std::vector<Joint, Eigen::aligned_allocator<Joint>> &joints,
    int total_dof, const std::vector<double> &values,
    std::vector<double> &out) {
  out.resize(total_dof);
  int idx = 0;
  int provided = (int)values.size();
  for (const auto &joint : joints) {
    int dof = joint.getDOF();
    for (int d = 0; d < dof; ++d) {
      if (idx < provided)
        out[idx] = values[idx];
      else if ((size_t)d < joint.values.size())
        out[idx] = joint.values[d];
      else
        out[idx] = 0.0;
      ++idx;
    }
  }
}

// ヘルパ：指定 joint と与えられた局所 DOF 値配列から相対回転を計算
static Eigen::Quaterniond
computeRelativeRotationFromValues(const Joint &joint,
                                  const std::vector<double> &vals) {
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  switch (joint.type) {
  case JointType::Revolute:
    rotation = Eigen::AngleAxisd(vals.size() > 0 ? vals[0] : joint.values[0],
                                 joint.axis1);
    break;
  case JointType::Universal:
    rotation = Eigen::AngleAxisd(vals.size() > 0 ? vals[0] : joint.values[0],
                                 joint.axis1) *
               Eigen::AngleAxisd(vals.size() > 1 ? vals[1] : joint.values[1],
                                 joint.axis2);
    break;
  case JointType::Spherical:
    rotation = Eigen::AngleAxisd(vals.size() > 0 ? vals[0] : joint.values[0],
                                 joint.axis1) *
               Eigen::AngleAxisd(vals.size() > 1 ? vals[1] : joint.values[1],
                                 joint.axis2) *
               Eigen::AngleAxisd(vals.size() > 2 ? vals[2] : joint.values[2],
                                 joint.axis3);
    break;
  default:
    break;
  }
  return rotation;
}

static Eigen::Vector3d
computeRelativeTranslationFromValues(const Joint &joint,
                                     const std::vector<double> &vals) {
  if (joint.type == JointType::Prismatic) {
    double v = vals.size() > 0 ? vals[0] : joint.values[0];
    return joint.prismatic_axis * v;
  }
  return Eigen::Vector3d::Zero();
}

void KinematicChain::setBase(const Eigen::Vector3d &position,
                             const Eigen::Quaterniond &orientation) {
  base_position_ = position;
  base_orientation_ = orientation;
}

void KinematicChain::addSegment(const Link &link, const Joint &joint) {
  Joint corrected_joint = joint;

  // Spherical ジョイントの軸を検証
  if (corrected_joint.type == JointType::Spherical) {
    if (!corrected_joint.validateSphericalAxes()) {
      std::cerr << "Warning: Spherical joint '" << corrected_joint.name
                << "' has invalid axes." << std::endl;

      // 自動修正を試行
      std::cerr << "Attempting to normalize axes..." << std::endl;
      corrected_joint.normalizeSphericalAxes();

      if (!corrected_joint.validateSphericalAxes()) {
        std::cerr << "Error: Failed to correct axes. Using default XYZ."
                   << std::endl;
        corrected_joint.setDefaultSphericalAxes();
      } else {
        std::cerr << "Successfully corrected axes for joint '"
                  << corrected_joint.name << "'" << std::endl;
      }
    }
  }

  links_.push_back(link);
  joints_.push_back(corrected_joint);
  total_dof_ += corrected_joint.getDOF();

  // Index 0: Base
  // Index 1..N: Joint Origins
  // Index N+1: End-Effector Tip (Joint Origin N + EEF Offset)
  joint_positions_world_.resize(joints_.size() + 2);
  joint_orientations_world_.resize(joints_.size() + 2);
}

void KinematicChain::setEEFOffset(const Eigen::Vector3d &offset) {
  eef_offset_ = offset;
}

Eigen::Vector3d KinematicChain::getEEFOffset() const { return eef_offset_; }

bool KinematicChain::setJointValues(const std::vector<double> &values) {
  if (values.size() > (size_t)total_dof_) {
    std::cerr << "Error: Number of joint values (" << values.size()
              << ") exceeds total DOF (" << total_dof_ << ")."
              << std::endl;
    return false;
  }

  size_t current_value_idx = 0;
  for (auto &joint : joints_) {
    int dof = joint.getDOF();
    if (dof == 0)
      continue;

    if (current_value_idx >= values.size()) {
      break;
    }

    bool use_limits = (joint.min_limits.size() == (size_t)dof &&
                       joint.max_limits.size() == (size_t)dof);

    for (int j = 0; j < dof; ++j) {
      if (current_value_idx + j >= values.size())
        break;

      double value = values[current_value_idx + j];
      if (use_limits) {
        value = std::max(joint.min_limits[j], std::min(value, joint.max_limits[j]));
      }
      joint.values[j] = value;
    }
    current_value_idx += dof;
  }
  return true;
}

std::vector<double> KinematicChain::getJointValues() const {
  std::vector<double> values;
  values.reserve(total_dof_);
  for (const auto &joint : joints_) {
    for (int i = 0; i < joint.getDOF(); ++i) {
      values.push_back(joint.values[i]);
    }
  }
  return values;
}

bool KinematicChain::isWithinLimits(const std::vector<double> &values) const {
  if (values.size() != (size_t)total_dof_)
    return false;
  int idx = 0;
  for (const auto &joint : joints_) {
    int dof = joint.getDOF();
    bool has_limits = (joint.min_limits.size() == (size_t)dof &&
                       joint.max_limits.size() == (size_t)dof);
    if (has_limits) {
      for (int d = 0; d < dof; ++d) {
        if (values[idx + d] < joint.min_limits[d] - 1e-4 ||
            values[idx + d] > joint.max_limits[d] + 1e-4)
          return false;
      }
    }
    idx += dof;
  }
  return true;
}

void KinematicChain::clampToLimits(std::vector<double> &values) const {
  if (values.size() != (size_t)total_dof_)
    return;
  int idx = 0;
  for (const auto &joint : joints_) {
    int dof = joint.getDOF();
    bool has_limits = (joint.min_limits.size() == (size_t)dof &&
                       joint.max_limits.size() == (size_t)dof);
    if (has_limits) {
      for (int d = 0; d < dof; ++d) {
        values[idx + d] =
            std::max(joint.min_limits[d],
                     std::min(values[idx + d], joint.max_limits[d]));
      }
    }
    idx += dof;
  }
}

void KinematicChain::forwardKinematicsAt(
    const std::vector<double> &values,
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        &out_positions,
    std::vector<Eigen::Quaterniond,
                Eigen::aligned_allocator<Eigen::Quaterniond>> &out_orientations)
    const {
  std::vector<double> full;
  buildFullValues(joints_, total_dof_, values, full);

  out_positions.resize(joints_.size() + 2);
  out_orientations.resize(joints_.size() + 2);

  Eigen::Vector3d current_pos = base_position_;
  Eigen::Quaterniond current_orient = base_orientation_;
  out_positions[0] = current_pos;
  out_orientations[0] = current_orient;

  int val_idx = 0;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const Joint &joint = joints_[i];
    int dof = joint.getDOF();

    current_pos += current_orient * links_[i].vector;

    std::vector<double> local;
    local.reserve(dof);
    for (int d = 0; d < dof; ++d)
      local.push_back(full[val_idx + d]);
    Eigen::Quaterniond rel = computeRelativeRotationFromValues(joint, local);
    current_orient = current_orient * joint.local_rotation * rel;
    current_orient.normalize();

    if (joint.type == kinematics::JointType::Prismatic) {
      Eigen::Vector3d rel_trans =
          computeRelativeTranslationFromValues(joint, local);
      current_pos += current_orient * rel_trans;
    }

    out_positions[i + 1] = current_pos;
    out_orientations[i + 1] = current_orient;
    val_idx += dof;
  }

  out_positions.back() = out_positions[joints_.size()];
  out_orientations.back() = out_orientations[joints_.size()];
  if (!joints_.empty()) {
    out_positions.back() += out_orientations.back() * eef_offset_;
  }
}

void KinematicChain::forwardKinematicsAt(const std::vector<double> &values) {
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      out_positions(joint_positions_world_.size(), Eigen::Vector3d::Zero());
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
      dummy_orientations(joint_orientations_world_.size(),
                         Eigen::Quaterniond::Identity());
  forwardKinematicsAt(values, out_positions, dummy_orientations);
}

void KinematicChain::forwardKinematics() {
  Eigen::Vector3d current_pos = base_position_;
  Eigen::Quaterniond current_orient = base_orientation_;

  joint_positions_world_[0] = current_pos;
  joint_orientations_world_[0] = current_orient;

  for (size_t i = 0; i < joints_.size(); ++i) {
    current_pos += current_orient * links_[i].vector;
    current_orient = current_orient * joints_[i].local_rotation *
                     joints_[i].getRelativeRotation();
    current_orient.normalize();

    if (joints_[i].type == kinematics::JointType::Prismatic) {
      current_pos += current_orient * joints_[i].getRelativeTranslation();
    }

    joint_positions_world_[i + 1] = current_pos;
    joint_orientations_world_[i + 1] = current_orient;
  }

  joint_positions_world_.back() = joint_positions_world_[joints_.size()];
  joint_orientations_world_.back() = joint_orientations_world_[joints_.size()];
  if (!joints_.empty()) {
    joint_positions_world_.back() +=
        joint_orientations_world_.back() * eef_offset_;
  }
}

int KinematicChain::getNumJoints() const { return joints_.size(); }

int KinematicChain::getTotalDOF() const { return total_dof_; }

Eigen::Vector3d KinematicChain::getJointPosition(int joint_index) const {
  if (joint_index < 0 || (size_t)joint_index >= joint_positions_world_.size() - 1) {
    std::cerr << "Error: Invalid joint index for getJointPosition." << std::endl;
    return Eigen::Vector3d::Zero();
  }
  return joint_positions_world_[joint_index];
}

Eigen::Quaterniond KinematicChain::getJointOrientation(int joint_index) const {
  if (joint_index < 0 || (size_t)joint_index >= joint_orientations_world_.size() - 1) {
    std::cerr << "Error: Invalid joint index for getJointOrientation." << std::endl;
    return Eigen::Quaterniond::Identity();
  }
  return joint_orientations_world_[joint_index];
}

Eigen::Vector3d KinematicChain::getEEFPosition() const {
  if (joint_positions_world_.empty())
    return base_position_;
  return joint_positions_world_.back();
}

Eigen::Quaterniond KinematicChain::getEEFOrientation() const {
  if (joint_orientations_world_.empty())
    return base_orientation_;
  return joint_orientations_world_.back();
}

Eigen::MatrixXd
KinematicChain::calculateJacobianAt(int target_joint_index,
                                    const std::vector<double> &values) const {
  std::vector<double> full;
  buildFullValues(joints_, total_dof_, values, full);

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts(joints_.size() + 2);
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orients(joints_.size() + 2);
  forwardKinematicsAt(full, pts, orients);

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, total_dof_);
  const Eigen::Vector3d &p_e = pts[target_joint_index];

  Eigen::Quaterniond cumulative = base_orientation_;
  int col = 0;
  int val_idx = 0;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const Joint &joint = joints_[i];
    int dof = joint.getDOF();

    if (joint.type == JointType::Fixed) {
      std::vector<double> loc;
      loc.reserve(dof);
      for (int d = 0; d < dof; ++d) loc.push_back(full[val_idx + d]);
      cumulative = cumulative * joint.local_rotation * computeRelativeRotationFromValues(joint, loc);
      val_idx += dof;
      continue;
    }

    std::vector<double> loc;
    loc.reserve(dof);
    for (int d = 0; d < dof; ++d) loc.push_back(full[val_idx + d]);

    if (i + 1 >= (size_t)target_joint_index)
      break;

    Eigen::Quaterniond q_base = cumulative * joint.local_rotation;
    const Eigen::Vector3d &p_i = pts[i + 1];

    switch (joint.type) {
    case JointType::Revolute: {
      Eigen::Vector3d axis = q_base * joint.axis1;
      J.block<3, 1>(0, col) = axis.cross(p_e - p_i);
      J.block<3, 1>(3, col) = axis;
      col++;
      break;
    }
    case JointType::Prismatic: {
      Eigen::Vector3d axis = q_base * joint.prismatic_axis;
      J.block<3, 1>(0, col) = axis;
      J.block<3, 1>(3, col).setZero();
      col++;
      break;
    }
    case JointType::Universal: {
      Eigen::Vector3d a1 = q_base * joint.axis1;
      J.block<3, 1>(0, col) = a1.cross(p_e - p_i);
      J.block<3, 1>(3, col) = a1;
      col++;
      Eigen::Quaterniond q2 = q_base * Eigen::AngleAxisd(loc.size() > 0 ? loc[0] : joint.values[0], joint.axis1);
      Eigen::Vector3d a2 = q2 * joint.axis2;
      J.block<3, 1>(0, col) = a2.cross(p_e - p_i);
      J.block<3, 1>(3, col) = a2;
      col++;
      break;
    }
    case JointType::Spherical: {
      Eigen::Vector3d a1 = q_base * joint.axis1;
      J.block<3, 1>(0, col) = a1.cross(p_e - p_i);
      J.block<3, 1>(3, col) = a1;
      col++;
      Eigen::Quaterniond q2 = q_base * Eigen::AngleAxisd(loc.size() > 0 ? loc[0] : joint.values[0], joint.axis1);
      Eigen::Vector3d a2 = q2 * joint.axis2;
      J.block<3, 1>(0, col) = a2.cross(p_e - p_i);
      J.block<3, 1>(3, col) = a2;
      col++;
      Eigen::Quaterniond q3 = q2 * Eigen::AngleAxisd(loc.size() > 1 ? loc[1] : joint.values[1], joint.axis2);
      Eigen::Vector3d a3 = q3 * joint.axis3;
      J.block<3, 1>(0, col) = a3.cross(p_e - p_i);
      J.block<3, 1>(3, col) = a3;
      col++;
      break;
    }
    default:
      break;
    }

    cumulative = cumulative * joint.local_rotation * computeRelativeRotationFromValues(joint, loc);
    val_idx += dof;
  }
  return J;
}

Eigen::MatrixXd
KinematicChain::calculateJacobian(int target_joint_index) const {
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, total_dof_);

  if (target_joint_index < 0 || (size_t)target_joint_index >= joint_positions_world_.size()) {
    std::cerr << "Error: Invalid target_joint_index for calculateJacobian." << std::endl;
    return J;
  }

  const Eigen::Vector3d &p_e = joint_positions_world_[target_joint_index];

  int dof_col_idx = 0;
  Eigen::Quaterniond cumulative_orientation = base_orientation_;

  for (size_t i = 0; i < joints_.size(); ++i) {
    if (i + 1 >= (size_t)target_joint_index)
      break;

    const Joint &joint = joints_[i];
    const Eigen::Vector3d &p_i = joint_positions_world_[i + 1];

    Eigen::Quaterniond q_current_base = cumulative_orientation * joint.local_rotation;

    switch (joint.type) {
    case JointType::Fixed:
      break;

    case JointType::Revolute: {
      Eigen::Vector3d axis_world = q_current_base * joint.axis1;
      J.block<3, 1>(0, dof_col_idx) = axis_world.cross(p_e - p_i);
      J.block<3, 1>(3, dof_col_idx) = axis_world;
      dof_col_idx++;
      break;
    }

    case JointType::Prismatic: {
      Eigen::Vector3d axis_world = q_current_base * joint.prismatic_axis;
      J.block<3, 1>(0, dof_col_idx) = axis_world;
      J.block<3, 1>(3, dof_col_idx).setZero();
      dof_col_idx++;
      break;
    }

    case JointType::Universal: {
      Eigen::Quaterniond q_axis1 = q_current_base;
      Eigen::Vector3d axis1_world = q_axis1 * joint.axis1;
      J.block<3, 1>(0, dof_col_idx) = axis1_world.cross(p_e - p_i);
      J.block<3, 1>(3, dof_col_idx) = axis1_world;
      dof_col_idx++;

      Eigen::Quaterniond q_axis2 = q_axis1 * Eigen::AngleAxisd(joint.values[0], joint.axis1);
      Eigen::Vector3d axis2_world = q_axis2 * joint.axis2;
      J.block<3, 1>(0, dof_col_idx) = axis2_world.cross(p_e - p_i);
      J.block<3, 1>(3, dof_col_idx) = axis2_world;
      dof_col_idx++;
      break;
    }

    case JointType::Spherical: {
      Eigen::Quaterniond q_axis1 = q_current_base;
      Eigen::Vector3d axis1_world = q_axis1 * joint.axis1;
      J.block<3, 1>(0, dof_col_idx) = axis1_world.cross(p_e - p_i);
      J.block<3, 1>(3, dof_col_idx) = axis1_world;
      dof_col_idx++;

      Eigen::Quaterniond q_axis2 = q_axis1 * Eigen::AngleAxisd(joint.values[0], joint.axis1);
      Eigen::Vector3d axis2_world = q_axis2 * joint.axis2;
      J.block<3, 1>(0, dof_col_idx) = axis2_world.cross(p_e - p_i);
      J.block<3, 1>(3, dof_col_idx) = axis2_world;
      dof_col_idx++;

      Eigen::Quaterniond q_axis3 = q_axis2 * Eigen::AngleAxisd(joint.values[1], joint.axis2);
      Eigen::Vector3d axis3_world = q_axis3 * joint.axis3;
      J.block<3, 1>(0, dof_col_idx) = axis3_world.cross(p_e - p_i);
      J.block<3, 1>(3, dof_col_idx) = axis3_world;
      dof_col_idx++;
      break;
    }
    }
    cumulative_orientation = cumulative_orientation * joint.local_rotation * joint.getRelativeRotation();
  }
  return J;
}

bool KinematicChain::inverseKinematicsAt(
    int target_joint_index, const Eigen::Vector3d &target_position,
    const Eigen::Quaterniond &target_orientation,
    std::vector<double> initial_values, 
    int max_iterations, double pos_tolerance, double ori_tolerance,
    std::vector<double> &out_solution) const {
  if (target_joint_index < 0 || (size_t)target_joint_index >= joints_.size() + 2)
    return false;

  std::vector<double> vals = initial_values.empty()
                                 ? buildCurrentValuesVector(joints_, total_dof_)
                                 : initial_values;
  if ((int)vals.size() != total_dof_) vals.resize(total_dof_, 0.0);

  const double lambda = 0.5;
  const double step_size = 0.5;

  for (int iter = 0; iter < max_iterations; ++iter) {
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts(joints_.size() + 2);
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orients(joints_.size() + 2);
    forwardKinematicsAt(vals, pts, orients);

    const Eigen::Vector3d &current_pos = pts[target_joint_index];
    const Eigen::Quaterniond &current_orient = orients[target_joint_index];

    Eigen::Vector3d pos_error = target_position - current_pos;
    Eigen::Quaterniond orient_error_q = target_orientation * current_orient.inverse();
    Eigen::AngleAxisd orient_error_aa(orient_error_q);
    Eigen::Vector3d ori_error = orient_error_aa.axis() * orient_error_aa.angle();

    if (pos_error.norm() < pos_tolerance && ori_error.norm() < ori_tolerance) {
      out_solution = vals;
      return true;
    }

    Eigen::MatrixXd J = calculateJacobianAt(target_joint_index, vals);
    Eigen::VectorXd error_vec(6);
    error_vec.head(3) = pos_error;
    error_vec.tail(3) = ori_error;

    Eigen::MatrixXd A = J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd delta_theta = J.transpose() * A.completeOrthogonalDecomposition().solve(error_vec);

    if ((size_t)delta_theta.size() != vals.size()) break;
    int idx = 0;
    for (auto &joint : joints_) {
      int dof = joint.getDOF();
      for (int d = 0; d < dof; ++d) {
        vals[idx] += step_size * delta_theta(idx);
        if ((size_t)d < joint.min_limits.size() && (size_t)d < joint.max_limits.size()) {
          vals[idx] = std::max(joint.min_limits[d], std::min(vals[idx], joint.max_limits[d]));
        }
        ++idx;
      }
    }
  }
  return false;
}

bool KinematicChain::inverseKinematicsAt(
    int target_joint_index, const Eigen::Vector3d &target_position,
    std::vector<double> initial_values, int max_iterations,
    double pos_tolerance, std::vector<double> &out_solution) const {

  if (target_joint_index < 0 || (size_t)target_joint_index >= joints_.size() + 2) {
    return false;
  }
  std::vector<double> vals = initial_values.empty()
                                 ? buildCurrentValuesVector(joints_, total_dof_)
                                 : initial_values;
  if ((int)vals.size() != total_dof_) vals.resize(total_dof_, 0.0);

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts(joints_.size() + 2);
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orients(joints_.size() + 2);

  const double lambda = 0.1;
  const double step_size = 0.5;

  for (int iter = 0; iter < max_iterations; ++iter) {
    Eigen::MatrixXd J_full = calculateJacobianAt(target_joint_index, vals);
    forwardKinematicsAt(vals, pts, orients);
    Eigen::Vector3d pos_error = target_position - pts[target_joint_index];
    double error_norm = pos_error.norm();

    if (error_norm < pos_tolerance) {
      out_solution = vals;
      return true;
    }

    Eigen::MatrixXd Jp = J_full.topRows(3);
    Eigen::MatrixXd A = Jp * Jp.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd delta = Jp.transpose() * A.completeOrthogonalDecomposition().solve(pos_error);

    if ((size_t)delta.size() != vals.size()) break;

    for (size_t i = 0; i < vals.size(); ++i)
      vals[i] += step_size * delta(i);
    
    int idx = 0;
    for (const auto &joint : joints_) {
      int dof = joint.getDOF();
      bool has_limits = (joint.min_limits.size() == (size_t)dof && joint.max_limits.size() == (size_t)dof);
      for (int d = 0; d < dof; ++d) {
        if (has_limits) {
          vals[idx] = std::max(joint.min_limits[d], std::min(vals[idx], joint.max_limits[d]));
        }
        idx++;
      }
    }
  }
  return false;
}

bool KinematicChain::inverseKinematicsPSOAt(
    int target_joint_index, const Eigen::Vector3d &target_position,
    int swarm_size, int max_iterations, double pos_tolerance,
    std::vector<double> &out_solution) const {
  if (target_joint_index < 0 || (size_t)target_joint_index >= joints_.size() + 2)
    return false;

  struct Particle {
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd best_position;
    double best_error = std::numeric_limits<double>::max();
  };

  const double w = 0.7, c1 = 0.8, c2 = 0.9;
  std::random_device rd;
  std::mt19937 gen(rd());

  std::vector<std::pair<double, double>> limits;
  std::vector<double> max_vel;
  for (const auto &joint : joints_) {
    for (int j = 0; j < joint.getDOF(); ++j) {
      bool has_limits = (joint.min_limits.size() == (size_t)joint.getDOF() && joint.max_limits.size() == (size_t)joint.getDOF());
      double minv = has_limits ? joint.min_limits[j] : -M_PI;
      double maxv = has_limits ? joint.max_limits[j] : M_PI;
      limits.emplace_back(minv, maxv);
      max_vel.push_back((maxv - minv) * 0.1);
    }
  }
  int dofN = total_dof_;

  auto random_in_range = [&](int i) {
    std::uniform_real_distribution<> d(limits[i].first, limits[i].second);
    return d(gen);
  };
  auto random_vel = [&](int i) {
    std::uniform_real_distribution<> d(-max_vel[i], max_vel[i]);
    return d(gen);
  };

  std::vector<Particle> swarm(swarm_size);
  Eigen::VectorXd global_best_pos(dofN);
  double global_best_error = std::numeric_limits<double>::max();

  for (auto &p : swarm) {
    p.position = Eigen::VectorXd(dofN);
    p.velocity = Eigen::VectorXd(dofN);
    for (int i = 0; i < dofN; ++i) {
      p.position(i) = random_in_range(i);
      p.velocity(i) = random_vel(i);
    }
    p.best_position = p.position;
    p.best_error = std::numeric_limits<double>::max();
  }

  for (int iter = 0; iter < max_iterations; ++iter) {
    for (auto &p : swarm) {
      std::vector<double> vals(p.position.data(), p.position.data() + p.position.size());
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts(joints_.size() + 2);
      std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orients(joints_.size() + 2);
      forwardKinematicsAt(vals, pts, orients);
      Eigen::Vector3d cur = pts[target_joint_index];
      double err = (target_position - cur).norm();
      if (err < p.best_error) {
        p.best_error = err;
        p.best_position = p.position;
      }
      if (err < global_best_error) {
        global_best_error = err;
        global_best_pos = p.position;
      }
    }

    if (global_best_error < pos_tolerance) {
      out_solution.assign(global_best_pos.data(), global_best_pos.data() + global_best_pos.size());
      return true;
    }

    for (auto &p : swarm) {
      std::uniform_real_distribution<> r(0.0, 1.0);
      double r1 = r(gen), r2 = r(gen);
      p.velocity = w * p.velocity + c1 * r1 * (p.best_position - p.position) +
                   c2 * r2 * (global_best_pos - p.position);
      for (int i = 0; i < dofN; ++i) {
        p.velocity(i) = std::max(-max_vel[i], std::min(p.velocity(i), max_vel[i]));
        p.position(i) += p.velocity(i);
        p.position(i) = std::max(limits[i].first, std::min(p.position(i), limits[i].second));
      }
    }
  }

  return false;
}

bool KinematicChain::inverseKinematics(
    int target_joint_index, const Eigen::Vector3d &target_position,
    const Eigen::Quaterniond &target_orientation, int max_iterations,
    double pos_tolerance, double ori_tolerance) {
  std::vector<double> sol;
  bool ok = inverseKinematicsAt(
      target_joint_index, target_position, target_orientation,
      std::vector<double>(), max_iterations, pos_tolerance, ori_tolerance, sol);
  if (ok) setJointValues(sol);
  return ok;
}

bool KinematicChain::inverseKinematics(int target_joint_index,
                                       const Eigen::Vector3d &target_position,
                                       int max_iterations,
                                       double pos_tolerance) {
  std::vector<double> sol;
  bool ok = inverseKinematicsAt(target_joint_index, target_position,
                                std::vector<double>(), max_iterations,
                                pos_tolerance, sol);
  if (ok) setJointValues(sol);
  return ok;
}

bool KinematicChain::inverseKinematicsPSO(
    int target_joint_index, const Eigen::Vector3d &target_position,
    int swarm_size, int max_iterations, double pos_tolerance) {
  std::vector<double> sol;
  bool ok = inverseKinematicsPSOAt(target_joint_index, target_position, swarm_size,
                             max_iterations, pos_tolerance, sol);
  if (ok) setJointValues(sol);
  return ok;
}

std::vector<double> KinematicChain::sampleRandomJointValues() const {
  std::vector<double> new_values = buildCurrentValuesVector(joints_, total_dof_);
  int current_dof_idx = 0;
  for (const auto &joint : joints_) {
    int dof = joint.getDOF();
    if (dof == 0) {
      current_dof_idx += dof;
      continue;
    }
    bool has_limits = (joint.min_limits.size() == (size_t)dof && joint.max_limits.size() == (size_t)dof);
    for (int j = 0; j < dof; ++j) {
      double min_val = has_limits ? joint.min_limits[j] : -M_PI;
      double max_val = has_limits ? joint.max_limits[j] : M_PI;
      std::uniform_real_distribution<> dis(min_val, max_val);
      new_values[current_dof_idx + j] = dis(rng_);
    }
    current_dof_idx += dof;
  }
  return new_values;
}

std::vector<double>
KinematicChain::sampleRandomJointValue(int joint_index) const {
  std::vector<double> new_values = buildCurrentValuesVector(joints_, total_dof_);
  if (joint_index < 0 || (size_t)joint_index >= joints_.size()) return new_values;
  int current_dof_idx = 0;
  for (int i = 0; i < joint_index; ++i) current_dof_idx += joints_[i].getDOF();
  const Joint &joint = joints_[joint_index];
  int dof = joint.getDOF();
  if (dof == 0) return new_values;
  bool has_limits = (joint.min_limits.size() == (size_t)dof && joint.max_limits.size() == (size_t)dof);
  for (int j = 0; j < dof; ++j) {
    double min_val = has_limits ? joint.min_limits[j] : -M_PI;
    double max_val = has_limits ? joint.max_limits[j] : M_PI;
    std::uniform_real_distribution<> dis(min_val, max_val);
    new_values[current_dof_idx + j] = dis(rng_);
  }
  return new_values;
}

std::vector<double> KinematicChain::sampleRandomJointValues(
    const std::vector<int> &joint_indices) const {
  std::vector<double> new_values = buildCurrentValuesVector(joints_, total_dof_);
  for (int target_joint_idx : joint_indices) {
    if (target_joint_idx < 0 || (size_t)target_joint_idx >= joints_.size()) continue;
    int current_dof_idx = 0;
    for (int i = 0; i < target_joint_idx; ++i) current_dof_idx += joints_[i].getDOF();
    const Joint &joint = joints_[target_joint_idx];
    int dof = joint.getDOF();
    if (dof == 0) continue;
    bool has_limits = (joint.min_limits.size() == (size_t)dof && joint.max_limits.size() == (size_t)dof);
    for (int j = 0; j < dof; ++j) {
      double min_val = has_limits ? joint.min_limits[j] : -M_PI;
      double max_val = has_limits ? joint.max_limits[j] : M_PI;
      std::uniform_real_distribution<> dis(min_val, max_val);
      new_values[current_dof_idx + j] = dis(rng_);
    }
  }
  return new_values;
}

std::string KinematicChain::getJointName(int joint_index) const {
  if (joint_index < 0 || (size_t)joint_index >= joints_.size()) {
    return "";
  }
  return joints_[joint_index].name;
}

int KinematicChain::getJointDOF(int joint_index) const {
  if (joint_index < 0 || (size_t)joint_index >= joints_.size()) {
    return 0;
  }
  return joints_[joint_index].getDOF();
}

std::string KinematicChain::getLinkName(int link_index) const {
  if (link_index < 0 || (size_t)link_index >= links_.size()) {
    return "";
  }
  return links_[link_index].name;
}

void KinematicChain::buildAllLinkTransforms(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
    const std::vector<Eigen::Quaterniond,
                      Eigen::aligned_allocator<Eigen::Quaterniond>>
        &orientations,
    const std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
        &fixed_link_info,
    std::map<std::string, Eigen::Isometry3d> &link_transforms) const {

  link_transforms.clear();

  Eigen::Isometry3d base_tf = Eigen::Isometry3d::Identity();
  base_tf.translate(base_position_);
  base_tf.rotate(base_orientation_);
  link_transforms["base_link"] = base_tf;

  for (size_t i = 0; i < links_.size(); ++i) {
    if (i < positions.size() - 1) {
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translate(positions[i + 1]);
      tf.rotate(orientations[i + 1]);
      link_transforms[links_[i].name] = tf;
    }
  }

  std::function<Eigen::Isometry3d(const std::string &)> get_transform;
  get_transform = [&](const std::string &link_name) -> Eigen::Isometry3d {
    auto it = link_transforms.find(link_name);
    if (it != link_transforms.end()) {
      return it->second;
    }

    auto fixed_it = fixed_link_info.find(link_name);
    if (fixed_it == fixed_link_info.end()) {
      return Eigen::Isometry3d::Identity();
    }

    const std::string &parent_name = fixed_it->second.first;
    const Eigen::Isometry3d &joint_origin = fixed_it->second.second;

    Eigen::Isometry3d parent_tf = get_transform(parent_name);
    Eigen::Isometry3d child_tf = parent_tf * joint_origin;
    link_transforms[link_name] = child_tf;

    return child_tf;
  };

  for (const auto &[link_name, info] : fixed_link_info) {
    get_transform(link_name);
  }
}

} // namespace kinematics
