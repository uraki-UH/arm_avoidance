#include "status/manipulability.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace Manipulability {

// ヤコビ行列の計算
Eigen::MatrixXd calculateJacobian(const std::vector<JointInfo> &joints,
                                  const Eigen::Vector3d &end_effector_pos) {
  int n_joints = joints.size();
  Eigen::MatrixXd J(3, n_joints);

  for (int i = 0; i < n_joints; ++i) {
    // r = 手先位置 - 関節位置
    Eigen::Vector3d r = end_effector_pos - joints[i].position;

    // J_i = z_i × r (外積)
    Eigen::Vector3d J_col = joints[i].axis.cross(r);

    J.col(i) = J_col;
  }

  return J;
}

// 部分ヤコビ行列の計算
Eigen::MatrixXd calculatePartialJacobian(const std::vector<JointInfo> &joints,
                                         int target_joint_index,
                                         const Eigen::Vector3d &target_pos) {
  if (target_joint_index < 0 ||
      target_joint_index > static_cast<int>(joints.size())) {
    return Eigen::MatrixXd::Zero(3, 0);
  }

  int n_joints = target_joint_index;
  Eigen::MatrixXd J(3, n_joints);

  for (int i = 0; i < n_joints; ++i) {
    Eigen::Vector3d r = target_pos - joints[i].position;
    Eigen::Vector3d J_col = joints[i].axis.cross(r);
    J.col(i) = J_col;
  }

  return J;
}

// 可操作性楕円体の計算
ManipulabilityEllipsoid
calculateManipulabilityEllipsoid(const Eigen::MatrixXd &J,
                                 ManipulabilityType type, double scale_factor) {
  ManipulabilityEllipsoid ellipsoid;

  if (J.rows() != 3 || J.cols() < 1) {
    ellipsoid.valid = false;
    return ellipsoid;
  }

  Eigen::MatrixXd A;

  switch (type) {
  case KINEMATIC:
    // 運動学的可操作性: A = J * J^T
    A = J * J.transpose();
    break;

  case DYNAMIC:
    // 動的可操作性（簡易版、慣性行列が必要な場合は別関数を使用）
    A = J * J.transpose();
    break;

  case PENALIZED:
    // ペナルティ付き可操作性（TODO: 実装）
    A = J * J.transpose();
    break;

  default:
    A = J * J.transpose();
    break;
  }

  // SVD分解
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);

  Eigen::VectorXd eigenvalues = svd.singularValues();

  // 特異値は固有値の平方根
  ellipsoid.singular_values = Eigen::Vector3d::Zero();
  int n_values = std::min(3, (int)eigenvalues.size());
  for (int i = 0; i < n_values; ++i) {
    ellipsoid.singular_values(i) = std::sqrt(std::max(0.0, eigenvalues(i))) *
                                   scale_factor; // Apply scale factor
  }
  ellipsoid.min_singular_value =
      (n_values > 0) ? ellipsoid.singular_values(n_values - 1) : 0.0;

  // 主軸方向
  ellipsoid.principal_directions = svd.matrixU().leftCols(3);

  // 可操作性尺度 (Yoshikawa's manipulability measure)
  double manip_product = 1.0;
  for (int i = 0; i < n_values; ++i) {
    manip_product *= ellipsoid.singular_values(i);
  }
  ellipsoid.manipulability =
      std::sqrt(std::max(0.0, svd.singularValues().prod())) * scale_factor;
  // NOTE: If n_values < 3, it's essentially 0 for a 3D volume, but Yoshikawa's
  // measure uses sqrt(det(JJT))

  // 楕円体の体積 (4/3 * PI * product of singular values)
  ellipsoid.volume = (4.0 / 3.0) * M_PI * manip_product;

  // 条件数
  if (ellipsoid.singular_values(0) > 1e-10 &&
      ellipsoid.singular_values(n_values - 1) > 1e-10) {
    ellipsoid.condition_number =
        ellipsoid.singular_values(0) / ellipsoid.singular_values(n_values - 1);
  } else {
    ellipsoid.condition_number = 1e10; // 非常に大きい値
  }

  ellipsoid.valid = true;

  return ellipsoid;
}

// 可操作性尺度の計算
double calculateManipulabilityMeasure(const Eigen::MatrixXd &J,
                                      ManipulabilityType type) {
  ManipulabilityEllipsoid ellipsoid = calculateManipulabilityEllipsoid(J, type);
  return ellipsoid.manipulability;
}

// 方向可操作性の評価
double
evaluateDirectionalManipulability(const ManipulabilityEllipsoid &ellipsoid,
                                  const Eigen::Vector3d &direction) {
  if (!ellipsoid.valid)
    return 0.0;

  Eigen::Vector3d d = direction.normalized();
  // y = U^T * d
  Eigen::Vector3d y = ellipsoid.principal_directions.transpose() * d;

  double sum_sq = 0.0;
  for (int i = 0; i < 3; ++i) {
    if (ellipsoid.singular_values(i) > 1e-10) {
      sum_sq += std::pow(y(i) / ellipsoid.singular_values(i), 2);
    } else {
      return 0.0; // その方向は特異
    }
  }

  return (sum_sq > 0) ? 1.0 / std::sqrt(sum_sq) : 0.0;
}

// 条件数の計算
double calculateConditionNumber(const Eigen::MatrixXd &J) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
  Eigen::VectorXd singular_values = svd.singularValues();

  if (singular_values.size() < 2)
    return 1.0;

  double max_sv = singular_values.maxCoeff();
  double min_sv = singular_values.minCoeff();

  if (min_sv < 1e-10)
    return 1e10;

  return max_sv / min_sv;
}

// 動的可操作性楕円体の計算
ManipulabilityEllipsoid calculateDynamicManipulabilityEllipsoid(
    const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, double scale_factor) {
  ManipulabilityEllipsoid ellipsoid;

  if (J.rows() != 3 || J.cols() < 1 || M.rows() != J.cols() ||
      M.cols() != J.cols()) {
    ellipsoid.valid = false;
    return ellipsoid;
  }

  // 動的可操作性: A = J * M^-1 * J^T
  Eigen::MatrixXd M_inv = M.inverse();
  Eigen::MatrixXd A = J * M_inv * J.transpose();

  // SVD分解
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);

  Eigen::VectorXd eigenvalues = svd.singularValues();

  ellipsoid.singular_values = Eigen::Vector3d::Zero();
  int n_values = std::min(3, (int)eigenvalues.size());
  for (int i = 0; i < n_values; ++i) {
    ellipsoid.singular_values(i) = std::sqrt(std::max(0.0, eigenvalues(i))) *
                                   scale_factor; // Apply scale factor
  }

  ellipsoid.principal_directions = svd.matrixU().leftCols(3);

  double manip_product = 1.0;
  for (int i = 0; i < n_values; ++i) {
    manip_product *= ellipsoid.singular_values(i);
  }
  ellipsoid.manipulability = std::pow(manip_product, 1.0 / n_values);

  if (ellipsoid.singular_values(n_values - 1) > 1e-6) {
    ellipsoid.condition_number =
        ellipsoid.singular_values(0) / ellipsoid.singular_values(n_values - 1);
  } else {
    ellipsoid.condition_number = 1e6;
  }

  ellipsoid.valid = true;

  return ellipsoid;
}

// 各関節の可操作性計算
std::vector<ManipulabilityEllipsoid>
calculateAllJointManipulabilities(const std::vector<JointInfo> &joints,
                                  double scale_factor) {
  std::vector<ManipulabilityEllipsoid> ellipsoids;

  for (size_t i = 0; i < joints.size(); ++i) {
    ManipulabilityEllipsoid ellipsoid;
    ellipsoid.center = joints[i].position;

    // その関節位置までの部分ヤコビ行列を計算
    Eigen::MatrixXd J =
        calculatePartialJacobian(joints, i + 1, joints[i].position);

    if (J.cols() >= 3) {
      ellipsoid = calculateManipulabilityEllipsoid(
          J, KINEMATIC, scale_factor); // Pass scale_factor
      ellipsoid.center = joints[i].position;
    } else {
      ellipsoid.valid = false;
    }

    ellipsoids.push_back(ellipsoid);
  }

  return ellipsoids;
}

// 零空間の計算
Eigen::MatrixXd calculateNullSpace(const Eigen::MatrixXd &J, double tolerance) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);

  Eigen::VectorXd singular_values = svd.singularValues();
  Eigen::MatrixXd V = svd.matrixV();

  // 特異値が閾値以下の成分を零空間とする
  std::vector<int> null_indices;
  for (int i = 0; i < singular_values.size(); ++i) {
    if (singular_values(i) < tolerance) {
      null_indices.push_back(i);
    }
  }

  if (null_indices.empty()) {
    return Eigen::MatrixXd::Zero(J.cols(), 0);
  }

  Eigen::MatrixXd null_space(J.cols(), null_indices.size());
  for (size_t i = 0; i < null_indices.size(); ++i) {
    null_space.col(i) = V.col(null_indices[i]);
  }

  return null_space;
}

// SVD計算
void computeSVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &U,
                Eigen::VectorXd &S, Eigen::MatrixXd &V) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);

  U = svd.matrixU();
  S = svd.singularValues();
  V = svd.matrixV();
}

ManipulabilityEllipsoid calculateFullMetrics(const Eigen::MatrixXd &J) {
  // 1. 運動学的可操作性楕円体を計算 (A = J*JT)
  ManipulabilityEllipsoid ellipsoid =
      calculateManipulabilityEllipsoid(J, KINEMATIC);

  if (ellipsoid.valid) {
    // 2. 各軸方向の可操作性を評価 (main.cppのロジックを統合)
    ellipsoid.manip_x =
        evaluateDirectionalManipulability(ellipsoid, Eigen::Vector3d::UnitX());
    ellipsoid.manip_y =
        evaluateDirectionalManipulability(ellipsoid, Eigen::Vector3d::UnitY());
    ellipsoid.manip_z =
        evaluateDirectionalManipulability(ellipsoid, Eigen::Vector3d::UnitZ());
  }

  return ellipsoid;
}

} // namespace Manipulability