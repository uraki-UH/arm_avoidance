#include "ais_gng/topological_plane/surface_model.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <map>

namespace fuzzrobo::surface_model
{
Eigen::Vector3d patch_normal_at(const patch_curvature &curvature, const Eigen::Vector3d &point)
{
  if (!curvature.valid || !point.allFinite()) return Eigen::Vector3d::Zero();
  const Eigen::Vector3d local = curvature.height_basis.transpose() *
    (point-curvature.height_origin) / curvature.height_scale;
  const auto &c = curvature.height_coeff;
  return (curvature.height_basis * Eigen::Vector3d(
    -(2*c(0)*local.x()+c(1)*local.y()+c(3)),
    -(c(1)*local.x()+2*c(2)*local.y()+c(4)), 1)).normalized();
}

patch_curvature estimate_curvature(
  const local_patch &patch, const ais_gng_msgs::msg::TopologicalMap &map)
{
  using vec = Eigen::Vector3d;
  patch_curvature out;
  std::vector<vec> points;
  for (auto idx : patch.node_indices) {
    if (idx >= map.nodes.size()) continue;
    const auto &p = map.nodes[idx].pos;
    const vec point(p.x,p.y,p.z);
    if (!point.allFinite()) continue;
    points.push_back(point);
    out.height_origin += point;
  }
  out.sample_num = points.size();
  // 6係数の補間ではなく、残差評価用の余剰支持を持つ近似。
  if (points.size() < 8) return out;
  out.height_origin /= static_cast<double>(points.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto &p : points) {
    const vec delta = p-out.height_origin;
    covariance.noalias() += delta*delta.transpose();
  }
  covariance /= static_cast<double>(points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> pca(covariance);
  if (pca.info()!=Eigen::Success || pca.eigenvalues()(2)<1e-12 ||
    pca.eigenvalues()(1)/pca.eigenvalues()(2)<1e-4) return out;
  vec reference_normal = pca.eigenvectors().col(0);
  // 入力法線に依存しない符号規約。絶対値最大の世界座標成分を正方向へ統一。
  Eigen::Index normal_idx;
  reference_normal.cwiseAbs().maxCoeff(&normal_idx);
  if (reference_normal(normal_idx)<0) reference_normal=-reference_normal;
  out.height_basis.col(0)=pca.eigenvectors().col(2);
  out.height_basis.col(1)=reference_normal.cross(out.height_basis.col(0));
  out.height_basis.col(2)=reference_normal;
  out.height_scale=std::sqrt(pca.eigenvalues()(2));
  out.plane_rms=std::sqrt(std::max(0.0,pca.eigenvalues()(0)));

  const auto num = points.size();
  Eigen::MatrixXd design(num,6);
  Eigen::VectorXd heights(num), density_weights(num), weights(num);
  std::vector<std::pair<int,int>> cells;
  std::map<std::pair<int,int>,std::size_t> counts;
  // 面内の同一寸法セルによる密集点の重複支持抑制。セル幅は代表広がりの半分。
  for (std::size_t i=0; i<num; ++i) {
    const vec p=out.height_basis.transpose()*(points[i]-out.height_origin)/out.height_scale;
    design.row(i) << p.x()*p.x(),p.x()*p.y(),p.y()*p.y(),p.x(),p.y(),1;
    heights(i)=p.z();
    cells.emplace_back(static_cast<int>(std::floor(2*p.x())),static_cast<int>(std::floor(2*p.y())));
    ++counts[cells.back()];
  }
  for (std::size_t i=0; i<num; ++i) density_weights(i)=1.0/counts[cells[i]];
  weights=density_weights;
  double condition_ratio=0;
  // 正規化座標の6係数SVDとHuber重みの反復。入力ノード法線の参照なし。
  for (int iter=0; iter<4; ++iter) {
    const Eigen::VectorXd root_weights=weights.array().sqrt();
    const Eigen::MatrixXd weighted_design=root_weights.asDiagonal()*design;
    Eigen::JacobiSVD<Eigen::MatrixXd> solver(weighted_design,Eigen::ComputeThinU|Eigen::ComputeThinV);
    condition_ratio=solver.singularValues()(5)/solver.singularValues()(0);
    if (condition_ratio<1e-5) return out;
    out.height_coeff=solver.solve(root_weights.cwiseProduct(heights));
    if (!out.height_coeff.allFinite()) return out;
    if (iter==3) break;
    const Eigen::VectorXd residual=(design*out.height_coeff-heights).cwiseAbs();
    std::vector<double> ordered(residual.data(),residual.data()+num);
    std::nth_element(ordered.begin(),ordered.begin()+num/2,ordered.end());
    const double huber_th=std::max(1e-6,1.345*1.4826*ordered[num/2]);
    for (std::size_t i=0; i<num; ++i) {
      weights(i)=density_weights(i)*std::min(1.0,huber_th/std::max(1e-15,residual(i)));
    }
  }
  const auto &c=out.height_coeff;
  out.normal=(out.height_basis*vec(-c(3),-c(4),1)).normalized();
  out.axis_u=out.normal.unitOrthogonal();
  out.axis_v=out.normal.cross(out.axis_u);
  Eigen::Matrix<double,3,2> tangent_basis;
  tangent_basis << out.axis_u,out.axis_v;
  const Eigen::Matrix2d projection=out.height_basis.leftCols<2>().transpose()*tangent_basis;
  Eigen::Matrix2d hessian;
  hessian << 2*c(0),c(1),c(1),2*c(2);
  // 実接平面の正規直交基底での第二基本形式。非零の一次係数も含む傾斜補正。
  out.tensor=projection.transpose()*hessian*projection /
    (out.height_scale*std::sqrt(1+c(3)*c(3)+c(4)*c(4)));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(out.tensor);
  if (eigen.info()!=Eigen::Success) return out;
  const int first=std::abs(eigen.eigenvalues()(0))>=std::abs(eigen.eigenvalues()(1)) ? 0:1;
  out.kappa << eigen.eigenvalues()(first),eigen.eigenvalues()(1-first);
  out.directions_uv.col(0)=eigen.eigenvectors().col(first);
  out.directions_uv.col(1)=eigen.eigenvectors().col(1-first);
  out.valid=true;
  double error=0;
  for (std::size_t i=0; i<num; ++i) {
    const Eigen::Vector2d q=tangent_basis.transpose()*(points[i]-out.height_origin);
    out.support_cov.noalias() += weights(i)*q*q.transpose();
    const vec n=patch_normal_at(out,points[i]);
    out.normal_scatter.noalias() += weights(i)*n*n.transpose();
    error+=weights(i)*std::pow(design.row(i).dot(c)-heights(i),2);
  }
  out.support_cov/=weights.sum();
  out.normal_scatter/=weights.sum();
  out.position_rms=out.height_scale*std::sqrt(error/weights.sum());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> support(out.support_cov);
  if (support.info()!=Eigen::Success || support.eigenvalues()(0)<1e-12) {
    out.valid=false;
    return out;
  }
  // 高さ残差から担当範囲の法線変化誤差への換算。確率ではない品質指標。
  out.fit_error=2*out.position_rms/std::sqrt(support.eigenvalues()(0));
  const double ratio=support.eigenvalues()(0)/support.eigenvalues()(1);
  out.confidence=std::min(1.0,ratio/0.05)*std::min(1.0,condition_ratio/0.01)*
    std::exp(-std::pow(out.fit_error/0.087,2));
  return out;
}
}  // 名前空間 fuzzrobo::surface_model
