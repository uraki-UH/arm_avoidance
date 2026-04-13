#pragma once

#include "joint.hpp"
#include "utility.hpp"
#include <map>
#include <string>

namespace kinematics {

// リンクのパラメータを保持する構造体
struct Link {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string name = "";

  // 幾何学的関係（親ジョイントから子ジョイントへのオフセット）
  Eigen::Vector3d vector = Eigen::Vector3d::Zero();

  Link() = default;
};

// 複数のリンクと関節からなる運動連鎖を表現するクラス
class KinematicChain {
public:
  KinematicChain() = default;

  // --- 構造の定義 ---
  void setBase(
      const Eigen::Vector3d &position,
      const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity());
  void addSegment(const Link &link, const Joint &joint);

  // --- 関節変数の設定/取得 ---
  bool setJointValues(const std::vector<double> &values);
  std::vector<double> getJointValues() const;
  bool isWithinLimits(const std::vector<double> &values) const;
  void clampToLimits(std::vector<double> &values) const;

  // --- 計算メソッド ---
  // 内部状態を変更する共通API追加: Eigenベクターから直接関節値を設定しFKを実行
  template <typename Derived>
  void updateKinematics(const Eigen::MatrixBase<Derived> &q_eigen) {
    if (q_eigen.size() == 0)
      return;
    std::vector<double> q_vec(q_eigen.size());
    for (int i = 0; i < q_eigen.size(); ++i) {
      q_vec[i] = static_cast<double>(q_eigen[i]);
    }
    setJointValues(q_vec);
    forwardKinematics();
  }

  // 非破壊版：指定 joint 値ベクトルを使って FK を計算（内部状態は変更しない）
  template <typename Derived>
  void forwardKinematicsAt(
      const Eigen::MatrixBase<Derived> &q_eigen,
      std::vector<double> &q_vec_buffer,
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          &out_positions,
      std::vector<Eigen::Quaterniond,
                  Eigen::aligned_allocator<Eigen::Quaterniond>>
          &out_orientations) const {
    if (q_vec_buffer.size() != static_cast<size_t>(q_eigen.size())) {
      q_vec_buffer.resize(q_eigen.size());
    }
    for (int i = 0; i < q_eigen.size(); ++i) {
      q_vec_buffer[i] = static_cast<double>(q_eigen[i]);
    }
    forwardKinematicsAt(q_vec_buffer, out_positions, out_orientations);
  }

  template <typename Derived>
  void forwardKinematicsAt(
      const Eigen::MatrixBase<Derived> &q_eigen,
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          &out_positions,
      std::vector<Eigen::Quaterniond,
                  Eigen::aligned_allocator<Eigen::Quaterniond>>
          &out_orientations) const {
    std::vector<double> q_vec(q_eigen.size());
    for (int i = 0; i < q_eigen.size(); ++i) {
      q_vec[i] = static_cast<double>(q_eigen[i]);
    }
    forwardKinematicsAt(q_vec, out_positions, out_orientations);
  }
  void forwardKinematicsAt(
      const std::vector<double> &values,
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          &out_positions,
      std::vector<Eigen::Quaterniond,
                  Eigen::aligned_allocator<Eigen::Quaterniond>>
          &out_orientations) const;
  void forwardKinematicsAt(const std::vector<double> &values);

  // 順運動学 (FK)
  void forwardKinematics();

  // 非破壊版：指定 joint 値ベクトルを使って target_joint_index
  // に対するヤコビアンを計算
  Eigen::MatrixXd calculateJacobianAt(int target_joint_index,
                                      const std::vector<double> &values) const;
  // ヤコビアンの計算
  Eigen::MatrixXd calculateJacobian(int target_joint_index) const;
  // 逆運動学 (IK)
  /* 非破壊 IK (フル姿勢) */
  bool inverseKinematicsAt(
      int target_joint_index, const Eigen::Vector3d &target_position,
      const Eigen::Quaterniond &target_orientation,
      std::vector<double> initial_values, // if empty, use current joint values
      int max_iterations, double pos_tolerance, double ori_tolerance,
      std::vector<double> &out_solution) const;

  /* 非破壊 IK (位置のみ) */
  bool inverseKinematicsAt(int target_joint_index,
                           const Eigen::Vector3d &target_position,
                           std::vector<double> initial_values,
                           int max_iterations, double pos_tolerance,
                           std::vector<double> &out_solution) const;

  /* 非破壊 IK (PSO, 位置のみ) */
  bool inverseKinematicsPSOAt(int target_joint_index,
                              const Eigen::Vector3d &target_position,
                              int swarm_size, int max_iterations,
                              double pos_tolerance,
                              std::vector<double> &out_solution) const;

  bool inverseKinematics(int target_joint_index,
                         const Eigen::Vector3d &target_position,
                         const Eigen::Quaterniond &target_orientation,
                         int max_iterations = 100, double pos_tolerance = 1e-4,
                         double ori_tolerance = 1e-4);

  // 逆運動学 (IK) - 位置のみ
  bool inverseKinematics(int target_joint_index,
                         const Eigen::Vector3d &target_position,
                         int max_iterations = 100, double pos_tolerance = 1e-3);

  // 逆運動学 (IK) - PSOによる位置のみ
  bool inverseKinematicsPSO(int target_joint_index,
                            const Eigen::Vector3d &target_position,
                            int swarm_size = 100, int max_iterations = 100,
                            double pos_tolerance = 1e-3);

  // --- ゲッター (FK実行後に有効) ---
  int getNumJoints() const;
  int getTotalDOF() const;
  Eigen::Vector3d getJointPosition(int joint_index) const;
  Eigen::Quaterniond getJointOrientation(int joint_index) const;
  std::string getJointName(int joint_index) const;
  int getJointDOF(int joint_index) const;

  std::string getLinkName(int link_index) const;
  Eigen::Vector3d getLinkVector(int link_index) const {
    if (link_index >= 0 && link_index < (int)links_.size())
      return links_[link_index].vector;
    return Eigen::Vector3d::Zero();
  }

  const Link &getLink(int link_index) const { return links_.at(link_index); }

  Eigen::Vector3d getEEFPosition() const;
  Eigen::Quaterniond getEEFOrientation() const;

  // --- アクセサ追加 ---
  const std::vector<Eigen::Vector3d,
                    Eigen::aligned_allocator<Eigen::Vector3d>> &
  getLinkPositions() const {
    return joint_positions_world_;
  }
  const std::vector<Eigen::Quaterniond,
                    Eigen::aligned_allocator<Eigen::Quaterniond>> &
  getLinkOrientations() const {
    return joint_orientations_world_;
  }

  // --- 先端オフセットの設定 ---
  void setEEFOffset(const Eigen::Vector3d &offset);
  Eigen::Vector3d getEEFOffset() const;

  // --- 全リンク姿勢の構築（固定リンク対応） ---
  // forwardKinematicsAtの結果から、固定リンクを含む全リンクの姿勢を構築
  // RobotModelの情報を使って固定リンクの位置を計算
  void buildAllLinkTransforms(
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
      const std::vector<Eigen::Quaterniond,
                        Eigen::aligned_allocator<Eigen::Quaterniond>>
          &orientations,
      const std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
          &fixed_link_info,
      std::map<std::string, Eigen::Isometry3d> &link_transforms) const;

  // --- 追加：内部状態を変更しないサンプリングAPI ---
  std::vector<double> sampleRandomJointValues()
      const; // 全DOFをサンプリング（内部状態は変更しない）
  std::vector<double> sampleRandomJointValue(int joint_index)
      const; // 指定1関節のみサンプリングした全DOFベクトルを返す
  std::vector<double>
  sampleRandomJointValues(const std::vector<int> &joint_indices)
      const; // 指定複数関節のみサンプリング

private:
  // --- 構造定義 ---
  Eigen::Vector3d base_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond base_orientation_ = Eigen::Quaterniond::Identity();

  std::vector<Link, Eigen::aligned_allocator<Link>> links_;
  std::vector<Joint, Eigen::aligned_allocator<Joint>> joints_;
  int total_dof_ = 0;

  // 最終リンクの先端オフセット (最終関節の座標系における)
  Eigen::Vector3d eef_offset_ = Eigen::Vector3d::Zero();

  mutable std::mt19937 rng_{std::random_device{}()};

  // --- FK計算結果 (ワールド座標系) ---
  // i番目の要素は、i番目の関節の基点（＝i-1番目のリンクの終点）の位置・姿勢
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      joint_positions_world_;
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
      joint_orientations_world_;
};

} // namespace kinematics
