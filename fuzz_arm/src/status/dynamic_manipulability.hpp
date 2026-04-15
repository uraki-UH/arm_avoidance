#pragma once

#include <Eigen/Dense>

class DynamicManipulability {
public:
  // コンストラクタ
  DynamicManipulability();

  // 動的可操作性楕円体の計算
  double computeDynamicManipulability(const Eigen::MatrixXd &jacobian,
                                      const Eigen::MatrixXd &inertia_matrix);

  // 動的可操作性楕円体の体積を計算
  double computeEllipsoidVolume(const Eigen::MatrixXd &jacobian,
                                const Eigen::MatrixXd &inertia_matrix);

  // 動的可操作性行列を取得
  Eigen::MatrixXd
  getDynamicManipulabilityMatrix(const Eigen::MatrixXd &jacobian,
                                 const Eigen::MatrixXd &inertia_matrix);

  // 特異値を取得
  Eigen::VectorXd getSingularValues() const;

private:
  Eigen::VectorXd singular_values_;

  // 慣性行列の逆行列を計算
  Eigen::MatrixXd computeInertiaInverse(const Eigen::MatrixXd &inertia);
};