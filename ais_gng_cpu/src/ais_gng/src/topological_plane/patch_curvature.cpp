#include "ais_gng/topological_plane/surface_model.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>

namespace fuzzrobo::surface_model
{
patch_curvature estimate_curvature(
  const local_patch &patch, const ais_gng_msgs::msg::TopologicalMap &map)
{
  using vec = Eigen::Vector3d;
  patch_curvature out;
  std::vector<vec> points, normals;
  vec center = vec::Zero(), mean_normal = vec::Zero();
  for (auto idx : patch.node_indices) {
    if (idx >= map.nodes.size()) continue;
    const auto &node = map.nodes[idx];
    vec p(node.pos.x, node.pos.y, node.pos.z), n(node.normal.x, node.normal.y, node.normal.z);
    if (!p.allFinite() || !n.allFinite() || n.norm() < 1e-8) continue;
    n.normalize();
    if (!normals.empty() && normals.front().dot(n) < 0) n = -n;
    points.push_back(p);
    normals.push_back(n);
    center += p;
    mean_normal += n;
  }
  out.sample_num = points.size();
  if (points.size() < 6 || mean_normal.norm() < 1e-8) return out;
  center /= static_cast<double>(points.size());
  out.normal = mean_normal.normalized();
  out.axis_u = out.normal.unitOrthogonal();
  out.axis_v = out.normal.cross(out.axis_u);
  Eigen::Matrix<double, 3, 2> basis;
  basis << out.axis_u, out.axis_v;
  Eigen::Matrix3d lhs = Eigen::Matrix3d::Zero();
  vec rhs = vec::Zero();
  double plane_error = 0;
  // 対称Kの3未知数を直接最小二乗推定。非対称解の事後平均とは別の制約付き最適化。
  for (std::size_t i = 0; i < points.size(); ++i) {
    const vec delta = points[i] - center;
    const Eigen::Vector2d q = basis.transpose() * delta;
    const Eigen::Vector2d r = basis.transpose() * (normals[i] - out.normal);
    const vec row_u(q.x(), q.y(), 0), row_v(0, q.x(), q.y());
    lhs.noalias() += row_u * row_u.transpose() + row_v * row_v.transpose();
    rhs.noalias() -= row_u * r.x() + row_v * r.y();
    out.support_cov.noalias() += q * q.transpose();
    out.normal_scatter.noalias() += normals[i] * normals[i].transpose();
    plane_error += std::pow(out.normal.dot(delta), 2);
  }
  out.support_cov /= static_cast<double>(points.size());
  out.normal_scatter /= static_cast<double>(points.size());
  out.plane_rms = std::sqrt(plane_error / points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> support(out.support_cov);
  if (support.info() != Eigen::Success || support.eigenvalues()(1) < 1e-12) return out;
  const double ratio = support.eigenvalues()(0) / support.eigenvalues()(1);
  if (ratio < 1e-4) return out;
  const vec k = lhs.ldlt().solve(rhs);
  if (!k.allFinite()) return out;
  out.tensor << k.x(), k.y(), k.y(), k.z();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(out.tensor);
  if (eigen.info() != Eigen::Success) return out;
  const int first = std::abs(eigen.eigenvalues()(0)) >= std::abs(eigen.eigenvalues()(1)) ? 0 : 1;
  out.kappa << eigen.eigenvalues()(first), eigen.eigenvalues()(1-first);
  out.directions_uv.col(0) = eigen.eigenvectors().col(first);
  out.directions_uv.col(1) = eigen.eigenvectors().col(1-first);
  double error = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector2d q = basis.transpose() * (points[i] - center);
    const Eigen::Vector2d r = basis.transpose() * (normals[i] - out.normal);
    error += (r + out.tensor * q).squaredNorm();
  }
  out.fit_error = std::sqrt(error / points.size());
  out.confidence = std::min(1.0, ratio / 0.05) * std::exp(-std::pow(out.fit_error / 0.087, 2));
  out.valid = true;
  return out;
}
}  // namespace fuzzrobo::surface_model
