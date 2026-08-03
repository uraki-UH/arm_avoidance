#pragma once

#include "kinematics/kinematic_chain.hpp"
#include "robot_model/robot_model.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace simulation {

/**
 * @brief Creates a kinematics::KinematicChain object from a
 * simulation::RobotModel.
 *
 * This function serves as an adapter, translating the general-purpose,
 * tree-like RobotModel structure (loaded from a URDF) into the linear chain
 * representation required by the KinematicChain class for kinematic
 * calculations.
 *
 * @param model The RobotModel object to convert.
 * @return A configured kinematics::KinematicChain object.
 * @throws std::runtime_error if the model is empty or describes a structure
 *         that cannot be represented as a single kinematic chain.
 */
struct ArmConfig {
    std::string root_link;
    std::string leaf_link;
    std::string prefix;
    // 今後「除外リンク」などを追加する場合もここを拡張すれば良い
};

kinematics::KinematicChain
createKinematicChainFromModel(const RobotModel &model,
                              const std::string &end_effector_name = "",
                              const Eigen::Vector3d &base_position = Eigen::Vector3d::Zero(),
                              const std::string &root_link_name = "");

std::unique_ptr<kinematics::KinematicChain>
createMultiArmKinematicChain(
    const RobotModel &model,
    const std::vector<ArmConfig> &arm_configs,
    const Eigen::Vector3d &base_position = Eigen::Vector3d::Zero());

// 以下は後方互換性のためのラッパー
// 以下は利便性のためのラッパー
std::unique_ptr<kinematics::KinematicChain>
createMultiArmKinematicChainFromModels(
    const RobotModel &model, const std::vector<std::string> &end_effector_names,
    const std::vector<std::string> &prefixes = {},
    const Eigen::Vector3d &base_position = Eigen::Vector3d::Zero(),
    const std::string &root_link_name = "");

void completeMissingBranchLinkTransforms(
    const RobotModel &model,
    const std::map<std::string, double> &joint_value_hints,
    std::map<std::string, Eigen::Isometry3d> &link_transforms);

/**
 * @brief Multi-arm adapter that exposes several independent KinematicChain
 * arms as a single chain-like interface.
 *
 * The primary arm is used as the representative EEF for the existing GNG/VLUT
 * pipeline, while all arms are updated and represented in the collision-aware
 * body transforms.
 */
class MultiArmKinematicAdapter : public kinematics::KinematicChain {
public:
  struct ArmEntry {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    kinematics::KinematicChain chain;
    std::string prefix;
    Eigen::Vector3d relative_base_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond relative_base_ori = Eigen::Quaterniond::Identity();
  };

  MultiArmKinematicAdapter() = default;
  explicit MultiArmKinematicAdapter(
      std::vector<ArmEntry, Eigen::aligned_allocator<ArmEntry>> arms,
      std::size_t primary_arm_index = 0);

  void setBase(const Eigen::Vector3d &position,
               const Eigen::Quaterniond &orientation =
                   Eigen::Quaterniond::Identity()) override;

  bool setJointValues(const std::vector<double> &values) override;
  void updateJointValuesByName(const std::vector<std::string> &names, const std::vector<double> &values) override;
  std::vector<double> getJointValues() const override;
  bool isWithinLimits(const std::vector<double> &values) const override;
  void clampToLimits(std::vector<double> &values) const override;

  void updateKinematics(const std::vector<double> &values) override;
  void forwardKinematicsAt(
      const std::vector<double> &values,
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          &out_positions,
      std::vector<Eigen::Quaterniond,
                  Eigen::aligned_allocator<Eigen::Quaterniond>>
          &out_orientations) const override;
  void forwardKinematicsAt(const std::vector<double> &values) override;
  void forwardKinematics() override;

  Eigen::MatrixXd calculateJacobianAt(int target_joint_index,
                                      const std::vector<double> &values) const override;
  Eigen::MatrixXd calculateJacobian(int target_joint_index) const override;

  int getNumJoints() const override;
  int getTotalDOF() const override;
  std::size_t getArmCount() const override;
  Eigen::Vector3d getJointPosition(int joint_index) const override;
  Eigen::Quaterniond getJointOrientation(int joint_index) const override;
  std::string getJointName(int joint_index) const override;
  int getJointDOF(int joint_index) const override;
  std::string getLinkName(int link_index) const override;
  Eigen::Vector3d getEEFPosition() const override;
  Eigen::Quaterniond getEEFOrientation() const override;
  Eigen::Vector3d getEEFPosition(std::size_t arm_index) const override;
  Eigen::Quaterniond getEEFOrientation(std::size_t arm_index) const override;
  const std::vector<Eigen::Vector3d,
                    Eigen::aligned_allocator<Eigen::Vector3d>> &
  getLinkPositions() const override;
  const std::vector<Eigen::Quaterniond,
                    Eigen::aligned_allocator<Eigen::Quaterniond>> &
  getLinkOrientations() const override;
  void setEEFOffset(const Eigen::Vector3d &offset) override;
  Eigen::Vector3d getEEFOffset() const override;
  void buildAllLinkTransforms(
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
      const std::vector<Eigen::Quaterniond,
                        Eigen::aligned_allocator<Eigen::Quaterniond>>
          &orientations,
      const std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
          &fixed_link_info,
      std::map<std::string, Eigen::Isometry3d> &link_transforms) const override;
  std::vector<double> sampleRandomJointValues() const override;
  void sampleRandomJointValues(std::vector<double> &out_values) const override;
  std::vector<double> sampleRandomJointValue(int joint_index) const override;
  void sampleRandomJointValue(int joint_index,
                              std::vector<double> &out_values) const override;
  std::vector<double>
  sampleRandomJointValues(const std::vector<int> &joint_indices) const override;
  void sampleRandomJointValues(const std::vector<int> &joint_indices,
                               std::vector<double> &out_values) const override;

private:
  void syncCachedState() const;
  std::size_t armOffset(std::size_t arm_index) const;
  std::size_t armPositionOffset(std::size_t arm_index) const;
  std::size_t armIndexForJoint(int joint_index) const;
  std::size_t armIndexForLink(int link_index) const;
  std::size_t armIndexForPositionIndex(int position_index) const;
  static bool startsWith(const std::string &value, const std::string &prefix);

  std::vector<ArmEntry, Eigen::aligned_allocator<ArmEntry>> arms_;
  std::size_t primary_arm_index_ = 0;
  Eigen::Vector3d base_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond base_orientation_ = Eigen::Quaterniond::Identity();

  mutable std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      cached_positions_;
  mutable std::vector<Eigen::Quaterniond,
                      Eigen::aligned_allocator<Eigen::Quaterniond>>
      cached_orientations_;
};

} // namespace simulation
