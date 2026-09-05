#pragma once

#include <fuzzrobo/libgng/api.h>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace fuzzrobo::node_support
{
using vector3 = Eigen::Vector3d;
using matrix3 = Eigen::Matrix3d;

struct options
{
  double sample_alpha = 0.01;
  double second_weight = 0.5;
  double min_axis_std = 0.002;
  double base_scale = 2.0;

  void validate() const
  {
    const std::array<double, 4> values{sample_alpha, second_weight,
      min_axis_std, base_scale};
    if (!std::all_of(values.begin(), values.end(), [](double x) {return std::isfinite(x);}) ||
      sample_alpha < 0 || sample_alpha > 1 || second_weight < 0 || second_weight > 1 ||
      min_axis_std <= 0 || base_scale <= 0)
    {
      throw std::invalid_argument("支持領域パラメータの範囲が不正です");
    }
  }
};

struct ellipsoid
{
  matrix3 moment = matrix3::Identity();
  matrix3 axes = matrix3::Identity();
  vector3 axis_std = vector3::Zero();
  bool has_support = false;
};

// 更新済み統計の描画用変換。入力イベント・ノード履歴の保持なし。
inline ellipsoid make_ellipsoid(const gng_node_statistics &stats, const options &config)
{
  ellipsoid shape;
  if (stats.support_weight_sum == 0) {return shape;}
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> moment(stats.support_moment);
  if (!moment.allFinite()) {return shape;}
  const Eigen::SelfAdjointEigenSolver<matrix3> solver(0.5 * (moment + moment.transpose()));
  if (solver.info() != Eigen::Success) {return shape;}
  const vector3 eigenvalues = solver.eigenvalues().cwiseMax(config.min_axis_std * config.min_axis_std);
  shape.axes = solver.eigenvectors();
  if (shape.axes.determinant() < 0) {shape.axes.col(0) *= -1;}
  shape.axis_std = eigenvalues.cwiseSqrt();
  shape.moment = shape.axes * eigenvalues.asDiagonal() * shape.axes.transpose();
  shape.has_support = true;
  return shape;
}
}  // 支持領域の名前空間
