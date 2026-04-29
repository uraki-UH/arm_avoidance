#pragma once

#include "kinematics/kinematic_chain.hpp"
#include "robot_model/robot_model.hpp"
#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace dynamics {

/**
 * @brief ロボットの動力学計算を行うユーティリティクラス。
 * 運動学（KinematicChain）と物理パラメータ（RobotModel）を分離して管理する。
 */
class RobotDynamics {
public:
  static std::optional<Eigen::MatrixXd>
  calculateMassMatrix(const kinematics::KinematicChain &chain,
                      const simulation::RobotModel &model,
                      const std::vector<double> &joint_values) {
    // 1. パラメータの有効性チェック
    if (!hasInertialParameters(chain, model)) {
      return std::nullopt;
    }

    int n = chain.getTotalDOF();
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n, n);

    // 2. 現在の姿勢における各リンクの位置・姿勢を計算
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts;
    std::vector<Eigen::Quaterniond,
                Eigen::aligned_allocator<Eigen::Quaterniond>>
        orientations;
    chain.forwardKinematicsAt(joint_values, pts, orientations);

    // 3. 各リンクの重心（CoM）におけるヤコビアンを用いて質量行列を構築
    // M = sum( Jv_i^T * m_i * Jv_i + Jw_i^T * I_i * Jw_i )
    for (int i = 0; i < chain.getNumJoints(); ++i) {
      std::string link_name = chain.getLinkName(i);
      const auto *props = model.getLink(link_name);
      if (!props)
        continue;

      double mass = props->inertial.mass;
      if (mass <= 1e-6)
        continue;

      // Link i の原点 (Joint i-1 の終点、または Base) は pts[i]
      Eigen::Vector3d p_link = pts[i];
      Eigen::Quaterniond q_link = orientations[i];

      // inertial.origin は link frame から inertial frame (CoM) へのオフセット
      const Eigen::Isometry3d &inertial_tf = props->inertial.origin;
      Eigen::Vector3d p_com = p_link + q_link * inertial_tf.translation();

      // ワールド座標系での慣性テンソル
      Eigen::Matrix3d R_total =
          q_link.toRotationMatrix() * inertial_tf.linear();
      Eigen::Matrix3d I_world =
          R_total * props->inertial.inertia_tensor * R_total.transpose();

      // Link i の原点におけるヤコビアンを取得 (6 x n)
      Eigen::MatrixXd J_i = chain.calculateJacobianAt(i, joint_values);

      // 重心位置に対するヤコビアンに補正
      // Jv_com = Jv_i - skew(p_com - p_i) * Jw_i
      Eigen::MatrixXd Jv_com = J_i.topRows(3);
      Eigen::MatrixXd Jw_i = J_i.bottomRows(3);
      Eigen::Vector3d r = p_com - p_link;

      for (int k = 0; k < n; ++k) {
        Eigen::Vector3d w = Jw_i.col(k);
        Jv_com.col(k) += w.cross(-r);
      }

      M += mass * Jv_com.transpose() * Jv_com +
           Jw_i.transpose() * I_world * Jw_i;
    }

    return M;
  }

private:
  /**
   * @brief 全ての可動リンクに慣性パラメータが設定されているか確認する。
   */
  static bool hasInertialParameters(const kinematics::KinematicChain &chain,
                                    const simulation::RobotModel &model) {
    // 全てのリンクの名前を確認し、RobotModel に Inertial 情報があるかチェック
    for (int i = 0; i < chain.getNumJoints(); ++i) {
      std::string link_name = chain.getLinkName(i);
      const auto *props = model.getLink(link_name);
      // 質量が定義されていない、または 0 の場合は動力学計算不可とみなす
      if (!props || props->inertial.mass <= 1e-6)
        return false;
    }
    return true;
  }
};

} // namespace dynamics
