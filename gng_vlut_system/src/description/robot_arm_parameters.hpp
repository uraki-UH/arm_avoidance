#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry> // For Eigen::Quaterniond and Eigen::AngleAxisd
#include <string>
#include <vector>

#include "kinematics/joint.hpp" // For JointType enum and Joint struct
#include "kinematics/kinematic_chain.hpp"

// urdfを使わない場合のアームパラメータファイル

namespace robot_arm_params {

// 簡易慣性（ボックス／円柱／球）: 慣性テンソルをローカル原点中心で返す
inline Eigen::Matrix3d inertiaBox(double m, const Eigen::Vector3d &size) {
  double x = size.x(), y = size.y(), z = size.z();
  Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
  I(0, 0) = (1.0 / 12.0) * m * (y * y + z * z);
  I(1, 1) = (1.0 / 12.0) * m * (x * x + z * z);
  I(2, 2) = (1.0 / 12.0) * m * (x * x + y * y);
  return I;
}
inline Eigen::Matrix3d inertiaCylinderZ(double m, double r, double h) {
  Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
  I(0, 0) = (1.0 / 12.0) * m * (3 * r * r + h * h);
  I(1, 1) = I(0, 0);
  I(2, 2) = 0.5 * m * r * r;
  return I;
}
inline Eigen::Matrix3d inertiaSphere(double m, double r) {
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity() * (2.0 / 5.0) * m * r * r;
  return I;
}

// ロボットアームのリンクとジョイントの構成を定義する関数
inline std::vector<std::pair<kinematics::Link, kinematics::Joint>>
getRobotArmConfiguration() {
  std::vector<std::pair<kinematics::Link, kinematics::Joint>> config;

  // --- Segment 0 ---
  kinematics::Link link0;
  link0.name = "Link0";
  link0.vector = Eigen::Vector3d(0.0, 0.0, 0.5); // Z方向0.5m
  link0.radius = 0.05;

  kinematics::Joint joint0;
  joint0.name = "Joint0_Revolute";
  joint0.type = kinematics::JointType::Revolute;
  joint0.axis1 = Eigen::Vector3d::UnitZ(); // Z軸周り
  joint0.min_limits = {-M_PI / 2};
  joint0.max_limits = {M_PI / 2};

  config.push_back({link0, joint0});

  // --- Segment 1 ---
  kinematics::Link link1;
  link1.name = "Link1";
  link1.vector = Eigen::Vector3d(0.0, 0.0, 0.5); // Z方向0.5m
  link1.radius = 0.05;

  kinematics::Joint joint1;
  joint1.name = "Joint1_Revolute";
  joint1.type = kinematics::JointType::Revolute;
  joint1.axis1 = Eigen::Vector3d::UnitY(); // Y軸周り
  joint1.min_limits = {-M_PI / 2};
  joint1.max_limits = {M_PI / 2};

  config.push_back({link1, joint1});

  // --- Segment 2 ---
  kinematics::Link link2;
  link2.name = "Link2_EndEffector";
  link2.vector = Eigen::Vector3d(0.0, 0.0, 0.2);
  link2.radius = 0.03;

  kinematics::Joint joint2;
  joint2.name = "Joint2_Spherical";
  joint2.type = kinematics::JointType::Spherical;
  joint2.min_limits = {-M_PI / 4, -M_PI / 4, -M_PI / 4};
  joint2.max_limits = {M_PI / 4, M_PI / 4, M_PI / 4};

  config.push_back({link2, joint2});

  return config;
}

} // namespace robot_arm_params