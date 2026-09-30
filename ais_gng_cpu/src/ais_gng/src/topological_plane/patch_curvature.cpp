#include "ais_gng/topological_plane/surface_model.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/QR>
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
  Eigen::MatrixXd weighted_design(num,6);
  Eigen::VectorXd root_weights(num),weighted_heights(num),residual(num);
  std::vector<double> ordered(num);
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr;
  const auto update_weights=[&]() {
    residual=(design*out.height_coeff-heights).cwiseAbs();
    std::copy(residual.data(),residual.data()+num,ordered.begin());
    std::nth_element(ordered.begin(),ordered.begin()+num/2,ordered.end());
    const double huber_th=std::max(1e-6,1.345*1.4826*ordered[num/2]);
    for (std::size_t i=0; i<num; ++i) {
      weights(i)=density_weights(i)*std::min(1.0,huber_th/std::max(1e-15,residual(i)));
    }
  };
  // QRによる解と小さいR行列での条件検査。悪条件時のみ全体SVDへ切替。
  constexpr double min_condition_ratio=1e-5;
  constexpr double min_qr_condition_ratio=1e-3;
  constexpr double max_coeff_change=1e-5;
  for (int iter=0; iter<4; ++iter) {
    root_weights=weights.array().sqrt();
    weighted_design=design.array().colwise()*root_weights.array();
    weighted_heights=root_weights.cwiseProduct(heights);
    qr.compute(weighted_design);
    const Eigen::Matrix<double,6,6> triangular=qr.matrixR().topLeftCorner<6,6>().triangularView<Eigen::Upper>();
    const Eigen::JacobiSVD<Eigen::Matrix<double,6,6>> condition(triangular);
    condition_ratio=condition.singularValues()(5)/condition.singularValues()(0);
    if (!std::isfinite(condition_ratio) || condition_ratio<min_condition_ratio) return out;
    const auto last_coeff=out.height_coeff;
    if (condition_ratio<min_qr_condition_ratio) {
      const Eigen::JacobiSVD<Eigen::MatrixXd> solver(weighted_design,Eigen::ComputeThinU|Eigen::ComputeThinV);
      out.height_coeff=solver.solve(weighted_heights);
      out.has_svd_fallback=true;
    } else {
      out.height_coeff=qr.solve(weighted_heights);
    }
    ++out.fit_iter;
    if (!out.height_coeff.allFinite()) return out;
    if (iter==3 || (iter>0 && (out.height_coeff-last_coeff).norm()<=
      max_coeff_change*(1+out.height_coeff.norm()))) break;
    update_weights();
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

void update_patch_curvatures(result &surfaces,
  const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes,
  const options &config, patch_history *history)
{
  using vec = Eigen::Vector3d;
  const bool has_history = history && config.enable_patch_history;
  if (history && !has_history) history->clear();
  const auto clock = has_history ? ++history->clock : 0;
  const auto refresh = std::max<std::size_t>(1, config.max_patch_refresh_frames);
  surfaces.num_curvature_fits = 0;
  surfaces.num_curvature_reused = 0;
  surfaces.num_curvature_deferred = 0;
  if (!has_history) {
    for (std::size_t idx = 0; idx < surfaces.patches.size(); ++idx) {
      auto &patch = surfaces.patches[idx];
      patch.curvature = {};
      patch.is_curvature_deferred = false;
      patch.eval_priority = idx;
      if (patch.node_indices.size() < 8 || patch.plane_cluster_idx < 0) continue;
      patch.curvature = estimate_curvature(patch, map);
      ++surfaces.num_curvature_fits;
    }
    return;
  }
  std::map<std::uint32_t, std::size_t> plane_ids;
  if (has_history) for (const auto &plane : planes.clusters) ++plane_ids[plane.id];
  struct candidate
  {
    std::size_t idx = 0;
    patch_history::cache_entry *entry = nullptr;
    std::uint64_t wait = 0;
    int rank = 0;
  };
  std::vector<candidate> pending;
  for (std::size_t patch_idx = 0; patch_idx < surfaces.patches.size(); ++patch_idx) {
    auto &patch = surfaces.patches[patch_idx];
    patch.curvature = {};
    patch.is_curvature_deferred = false;
    patch.has_plane_features = false;
    patch.position_cov.setZero();
    patch.normal.setZero();
    patch.local_spacing = 0;
    patch.plane_residual_ratio = 0;
    patch.eval_priority = surfaces.patches.size() + patch_idx;
    if (patch.node_indices.size() < 8 || patch.plane_cluster_idx < 0 ||
      static_cast<std::size_t>(patch.plane_cluster_idx) >= planes.clusters.size()) continue;
    const auto &plane = planes.clusters[patch.plane_cluster_idx];
    // 元平面全体の統計のみ転送。部分パッチと未収録共分散は実点での集計。
    bool has_full_plane = patch.node_indices == plane.node_indices;
    if (!has_full_plane && patch.node_indices.size() == plane.node_indices.size()) {
      auto indices = patch.node_indices;
      auto plane_indices = plane.node_indices;
      std::sort(indices.begin(), indices.end());
      std::sort(plane_indices.begin(), plane_indices.end());
      has_full_plane = indices == plane_indices;
    }
    if (has_full_plane) {
      for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col) {
        patch.position_cov(row, col) = plane.position_covariance[3 * row + col];
      }
      const vec center(plane.centroid.x, plane.centroid.y, plane.centroid.z);
      if (center.allFinite() && patch.position_cov.allFinite() && patch.position_cov.trace() > 0) {
        patch.center = center;
        patch.normal = vec(plane.normal.x, plane.normal.y, plane.normal.z);
        patch.local_spacing = plane.local_spacing;
        patch.plane_residual_ratio = plane.residual_ratio;
        patch.has_plane_features = true;
      }
    }
    std::vector<patch_history::node_key> members;
    bool has_complete_support = true;
    std::size_t num_points = 0;
    vec center = vec::Zero();
    for (auto idx : patch.node_indices) {
      if (idx >= map.nodes.size()) { has_complete_support = false; continue; }
      const auto &node = map.nodes[idx];
      const vec point(node.pos.x, node.pos.y, node.pos.z);
      if (!point.allFinite()) { has_complete_support = false; continue; }
      center += point;
      ++num_points;
      if (has_history) members.emplace_back(node.id, node.frame);
    }
    if (!patch.has_plane_features) {
      patch.center = num_points ? (center / static_cast<double>(num_points)).eval() : vec::Zero();
      patch.position_cov.setZero();
      for (auto idx : patch.node_indices) {
        if (idx >= map.nodes.size()) continue;
        const auto &p = map.nodes[idx].pos;
        const vec delta = vec(p.x, p.y, p.z) - patch.center;
        if (delta.allFinite()) patch.position_cov.noalias() += delta * delta.transpose();
      }
      if (num_points) patch.position_cov /= static_cast<double>(num_points);
    }
    std::sort(members.begin(), members.end());
    has_complete_support = has_complete_support && !members.empty() &&
      std::adjacent_find(members.begin(), members.end()) == members.end();
    patch_history::cache_entry *entry = nullptr;
    bool has_old_fit = false;
    if (has_complete_support && plane_ids[plane.id] == 1) {
      const auto key = patch_history::patch_key{plane.id, members.front().first, members.front().second};
      entry = &history->entries[key];
      entry->last_seen = clock;
      has_old_fit = entry->curvature.valid;
      const bool has_same_members = entry->members == members;
      const auto &curve = entry->curvature;
      const double margin = std::max(0.0, config.max_point_residual);
      const double spread = std::sqrt(std::max(0.0, entry->position_cov.trace()));
      bool can_reuse = has_same_members && has_old_fit && clock - entry->last_eval < refresh &&
        curve.position_rms <= config.max_patch_rms &&
        (patch.center - entry->center).norm() <= margin &&
        (patch.position_cov - entry->position_cov).norm() <= margin * (2 * spread + margin);
      // 旧二次式への全点残差と旧支持域の確認。法線値だけの更新は失効条件外。
      double squared_error = 0;
      if (can_reuse) for (auto idx : patch.node_indices) {
        const auto &p = map.nodes[idx].pos;
        const vec local = curve.height_basis.transpose() *
          (vec(p.x, p.y, p.z) - curve.height_origin);
        if ((local.head<2>().array() < entry->min_support.array() - margin).any() ||
          (local.head<2>().array() > entry->max_support.array() + margin).any()) {
          can_reuse = false;
          break;
        }
        const vec q = local / curve.height_scale;
        const auto &c = curve.height_coeff;
        const double error = curve.height_scale *
          (q.z() - (c(0)*q.x()*q.x()+c(1)*q.x()*q.y()+c(2)*q.y()*q.y()+c(3)*q.x()+c(4)*q.y()+c(5)));
        squared_error += error * error;
        if (!std::isfinite(error) || std::abs(error) > margin) { can_reuse = false; break; }
      }
      can_reuse = can_reuse && std::sqrt(squared_error / num_points) <= config.max_patch_rms;
      if (can_reuse) {
        patch.curvature = curve;
        entry->first_pending = 0;
        ++surfaces.num_curvature_reused;
        continue;
      }
      entry->members = std::move(members);
      if (!has_same_members) entry->curvature = {};
      if (!entry->first_pending) entry->first_pending = clock;
    }
    const int rank = has_old_fit ? 0 : patch.has_retained_support ? 1 : patch.normal_change_hint > 0 ? 2 : 3;
    pending.push_back({patch_idx, entry, entry ? clock - entry->first_pending : 0, rank});
  }
  // 長期待機の繰上げ、逸脱、保持曲面周辺、法線変化候補の順。採用判定とは独立。
  std::stable_sort(pending.begin(), pending.end(), [&](const candidate &a, const candidate &b) {
    const bool is_old_a = a.wait >= refresh, is_old_b = b.wait >= refresh;
    if (is_old_a != is_old_b) return is_old_a;
    if (is_old_a && a.wait != b.wait) return a.wait > b.wait;
    if (a.rank != b.rank) return a.rank < b.rank;
    if (a.wait != b.wait) return a.wait > b.wait;
    return surfaces.patches[a.idx].normal_change_hint > surfaces.patches[b.idx].normal_change_hint;
  });
  for (std::size_t order = 0; order < pending.size(); ++order) {
    auto &patch = surfaces.patches[pending[order].idx];
    patch.eval_priority = order;
    if (surfaces.num_curvature_fits >= config.max_patch_fits) {
      patch.is_curvature_deferred = true;
      ++surfaces.num_curvature_deferred;
      continue;
    }
    patch.curvature = estimate_curvature(patch, map);
    ++surfaces.num_curvature_fits;
    if (auto *entry = pending[order].entry) {
      entry->curvature = patch.curvature;
      entry->center = patch.center;
      entry->position_cov = patch.position_cov;
      entry->last_eval = clock;
      entry->first_pending = 0;
      entry->min_support.setConstant(std::numeric_limits<double>::infinity());
      entry->max_support.setConstant(-std::numeric_limits<double>::infinity());
      for (auto idx : patch.node_indices) {
        const auto &p = map.nodes[idx].pos;
        const Eigen::Vector2d q = (patch.curvature.height_basis.transpose() *
          (vec(p.x, p.y, p.z) - patch.curvature.height_origin)).head<2>();
        entry->min_support = entry->min_support.cwiseMin(q);
        entry->max_support = entry->max_support.cwiseMax(q);
      }
    }
  }
  // 現フレームに存在しない平面・ノード世代の履歴解放。
  if (has_history) for (auto it = history->entries.begin(); it != history->entries.end();) {
    if (it->second.last_seen != clock) it = history->entries.erase(it);
    else ++it;
  }
}
}  // 名前空間 fuzzrobo::surface_model
