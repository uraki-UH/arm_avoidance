#include "metrics/dynamic_manipulability.hpp"
#include <cmath>
#include <stdexcept>

DynamicManipulability::DynamicManipulability() {}

Eigen::MatrixXd
DynamicManipulability::computeInertiaInverse(const Eigen::MatrixXd &inertia) {
  if (inertia.rows() != inertia.cols()) {
    throw std::invalid_argument("Inertia matrix must be square");
  }

  // LU分解を使用して逆行列を計算
  Eigen::MatrixXd inertia_inv = inertia.inverse();
  return inertia_inv;
}

Eigen::MatrixXd DynamicManipulability::getDynamicManipulabilityMatrix(
    const Eigen::MatrixXd &jacobian, const Eigen::MatrixXd &inertia_matrix) {
  // 動的可操作性行列: W = J * M^(-1) * J^T
  Eigen::MatrixXd inertia_inv = computeInertiaInverse(inertia_matrix);
  Eigen::MatrixXd dynamic_manipulability_matrix =
      jacobian * inertia_inv * jacobian.transpose();

  return dynamic_manipulability_matrix;
}

double DynamicManipulability::computeDynamicManipulability(
    const Eigen::MatrixXd &jacobian, const Eigen::MatrixXd &inertia_matrix) {
  // 動的可操作性行列を取得
  Eigen::MatrixXd W = getDynamicManipulabilityMatrix(jacobian, inertia_matrix);

  // 特異値分解
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU |
                                               Eigen::ComputeThinV);

  singular_values_ = svd.singularValues();

  // 動的可操作性は特異値の積の平方根
  double manipulability = 1.0;
  for (int i = 0; i < singular_values_.size(); ++i) {
    manipulability *= singular_values_(i);
  }

  return std::sqrt(manipulability);
}

double DynamicManipulability::computeEllipsoidVolume(
    const Eigen::MatrixXd &jacobian, const Eigen::MatrixXd &inertia_matrix) {
  // 楕円体の体積は特異値の積に比例
  double manipulability =
      computeDynamicManipulability(jacobian, inertia_matrix);

  // n次元楕円体の体積係数
  int n = jacobian.rows();
  double volume_coefficient =
      std::pow(M_PI, n / 2.0) / std::tgamma(n / 2.0 + 1.0);

  return volume_coefficient * manipulability;
}

Eigen::VectorXd DynamicManipulability::getSingularValues() const {
  return singular_values_;
}