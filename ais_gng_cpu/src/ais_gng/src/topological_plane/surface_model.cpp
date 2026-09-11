#include "ais_gng/topological_plane/surface_model.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

namespace fuzzrobo::surface_model
{
namespace
{
using vec = Eigen::Vector3d;
using mat = Eigen::Matrix3d;
using coefficients = Eigen::Matrix<double, 10, 1>;
constexpr double pi = 3.14159265358979323846;

template<class point> vec position(const point &p) { return {p.x, p.y, p.z}; }
vec normal(const vec &n) { return n.allFinite() && n.norm() > 1e-8 ? vec(n.normalized()) : vec::Zero(); }

coefficients features(const vec &p)
{
  coefficients f;
  f << p.x()*p.x(), p.y()*p.y(), p.z()*p.z(), p.x()*p.y(), p.x()*p.z(),
    p.y()*p.z(), p.x(), p.y(), p.z(), 1.0;
  return f;
}

coefficients equation(const mat &a, const vec &b, double c)
{
  coefficients q;
  q << a(0,0), a(1,1), a(2,2), 2*a(0,1), 2*a(0,2), 2*a(1,2), b.x(), b.y(), b.z(), c;
  return q;
}

vec gradient(const coefficients &q, const vec &p)
{
  return {2*q(0)*p.x()+q(3)*p.y()+q(4)*p.z()+q(6),
    2*q(1)*p.y()+q(3)*p.x()+q(5)*p.z()+q(7),
    2*q(2)*p.z()+q(4)*p.x()+q(5)*p.y()+q(8)};
}

struct sample { vec p; vec n; std::uint32_t patch_idx; };

model fit_best(const std::vector<std::uint32_t> &patch_ids,
  const std::vector<local_patch> &patches, const std::vector<vec> &points,
  const std::vector<vec> &normals, const options &config)
{
  model best;
  std::vector<sample> samples;
  for (std::size_t i = 0; i < patch_ids.size(); ++i) {
    for (const auto node_idx : patches[patch_ids[i]].node_indices) {
      samples.push_back({points[node_idx], normals[node_idx], static_cast<std::uint32_t>(i)});
    }
  }
  if (samples.size() < config.min_fit_nodes) return best;
  vec origin = vec::Zero();
  for (const auto &s : samples) origin += s.p;
  origin /= static_cast<double>(samples.size());
  mat covariance = mat::Zero();
  mat normal_moment = mat::Zero();
  std::size_t normal_num = 0;
  for (const auto &s : samples) {
    covariance += (s.p-origin)*(s.p-origin).transpose();
    normal_moment += s.n*s.n.transpose();
    normal_num += s.n.squaredNorm() > 0.5;
  }
  covariance /= static_cast<double>(samples.size());
  Eigen::SelfAdjointEigenSolver<mat> pca(covariance);
  if (pca.info() != Eigen::Success || pca.eigenvalues()(1) < 1e-12) return best;
  const double scale = std::sqrt(covariance.trace());
  for (auto &s : samples) s.p = (s.p-origin)/scale;
  const std::size_t fit_num = std::min(samples.size(), config.max_fit_samples);
  const double min_normal_cos = std::cos(config.max_normal_deg*pi/180.0);
  bool has_dominant_flat_patch = false;
  if (config.protect_dominant_flat_patches) for (auto idx : patch_ids) {
    const auto &c = patches[idx].curvature;
    const double bend = std::sqrt(std::max(0.0, (c.tensor*c.support_cov*c.tensor.transpose()).trace()));
    if (c.valid && c.confidence >= 0.8 && c.plane_rms <= config.max_patch_rms &&
      c.support_cov.trace() >= covariance.trace() && bend <= config.max_curvature_normal_error) {
      has_dominant_flat_patch = true;
    }
  }

  // 係数推定は間引き、採否判定は全所属点・全パッチ。大きな平面による小領域の残差隠蔽の防止。
  const auto evaluate = [&](model candidate, int freedom) {
    if (has_dominant_flat_patch && candidate.type != "plane") return;
    if (!candidate.q.allFinite() || candidate.q.norm() < 1e-12) return;
    candidate.q.normalize();
    std::vector<double> patch_error(patch_ids.size(), 0.0), patch_angle(patch_ids.size(), 0.0);
    std::vector<std::size_t> patch_num(patch_ids.size(), 0), patch_normal_num(patch_ids.size(), 0);
    double sum = 0.0;
    for (const auto &s : samples) {
      const vec g = gradient(candidate.q, s.p);
      if (g.norm() < 1e-8) return;
      const double error = scale*std::abs(candidate.q.dot(features(s.p)))/g.norm();
      if (!std::isfinite(error) || error > config.max_point_residual) return;
      patch_error[s.patch_idx] += error*error;
      ++patch_num[s.patch_idx];
      sum += error*error;
      if (s.n.squaredNorm() > 0.5) {
        const double cos = std::clamp(std::abs(s.n.dot(g.normalized())), 0.0, 1.0);
        patch_angle[s.patch_idx] += cos;
        ++patch_normal_num[s.patch_idx];
      }
    }
    double worst = 0.0;
    for (std::size_t i = 0; i < patch_ids.size(); ++i) {
      worst = std::max(worst, std::sqrt(patch_error[i]/patch_num[i]));
      if (worst > config.max_patch_rms) return;
      if (patch_normal_num[i] && patch_angle[i]/patch_normal_num[i] < min_normal_cos) return;
      const auto &patch = patches[patch_ids[i]];
      const auto &curvature = patch.curvature;
      if (curvature.valid && curvature.confidence >= 0.5) {
        const vec g = gradient(candidate.q, (patch.center-origin)/scale);
        if (g.norm() < 1e-8) return;
        mat hessian;
        hessian << 2*candidate.q(0),candidate.q(3),candidate.q(4),
          candidate.q(3),2*candidate.q(1),candidate.q(5),
          candidate.q(4),candidate.q(5),2*candidate.q(2);
        Eigen::Matrix<double,3,2> basis;
        basis << curvature.axis_u,curvature.axis_v;
        const double sign = g.dot(curvature.normal) >= 0 ? 1.0 : -1.0;
        const mat tangent = mat::Identity()-g.normalized()*g.normalized().transpose();
        const Eigen::Matrix2d predicted = -sign*basis.transpose()*tangent*hessian*basis/(scale*g.norm());
        const Eigen::Matrix2d delta = predicted-curvature.tensor;
        // 曲率差を担当範囲内の法線変化差に換算。単位・パッチ寸法の違いの吸収。
        const double bend_error = std::sqrt(std::max(0.0,
          (delta*curvature.support_cov*delta.transpose()).trace()));
        if (bend_error > config.max_curvature_normal_error+curvature.fit_error) return;
      }
    }
    candidate.rms = std::sqrt(sum/samples.size());
    candidate.max_patch_rms = worst;
    candidate.score = sum/samples.size() + config.complexity_penalty*config.complexity_penalty*freedom;
    if (candidate.score < best.score) {
      candidate.origin = origin;
      candidate.scale = scale;
      best = std::move(candidate);
    }
  };

  model plane;
  plane.type = "plane";
  plane.axis = pca.eigenvectors().col(0);
  plane.center = origin;
  plane.q = equation(mat::Zero(), plane.axis, 0.0);
  evaluate(plane, 3);
  // 非平面モデルの最小ペナルティより既に良い平面解なら、追加推定での改善は不可能。
  if (best.type == "plane" && best.score <= 4*config.complexity_penalty*config.complexity_penalty) return best;
  if (has_dominant_flat_patch) return best;
  if (normal_num < config.min_fit_nodes/2) return best;

  Eigen::MatrixXd sphere_a(fit_num, 4);
  Eigen::VectorXd sphere_b(fit_num);
  for (std::size_t i = 0; i < fit_num; ++i) {
    const vec &p = samples[i*samples.size()/fit_num].p;
    sphere_a.row(i) << 2*p.x(), 2*p.y(), 2*p.z(), 1.0;
    sphere_b(i) = p.squaredNorm();
  }
  const auto sphere_qr = sphere_a.colPivHouseholderQr();
  if (sphere_qr.rank() == 4) {
    const Eigen::Vector4d c = sphere_qr.solve(sphere_b);
    const vec center = c.head<3>();
    const double radius2 = c(3)+center.squaredNorm();
    if (radius2 > 1e-10 && scale*std::sqrt(radius2) <= config.max_radius) {
      model sphere;
      sphere.type = "sphere";
      sphere.center = origin + scale*center;
      sphere.radii.setConstant(scale*std::sqrt(radius2));
      sphere.q = equation(mat::Identity(), -2*center, -c(3));
      evaluate(sphere, 4);
    }
  }

  Eigen::SelfAdjointEigenSolver<mat> normal_pca(normal_moment);
  if (normal_pca.info() == Eigen::Success && normal_pca.eigenvalues()(1) > 1e-6) {
    const vec axis = normal_pca.eigenvectors().col(0);
    const vec u = axis.unitOrthogonal(), v = axis.cross(u);
    Eigen::MatrixXd circle_a(fit_num, 3), ellipse_a(fit_num, 5);
    Eigen::VectorXd circle_b(fit_num);
    for (std::size_t i = 0; i < fit_num; ++i) {
      const vec &p = samples[i*samples.size()/fit_num].p;
      const double x = u.dot(p), y = v.dot(p);
      circle_a.row(i) << 2*x, 2*y, 1.0;
      circle_b(i) = x*x+y*y;
      ellipse_a.row(i) << x*x, 2*x*y, y*y, x, y;
    }
    const auto circle_qr = circle_a.colPivHouseholderQr();
    if (circle_qr.rank() == 3) {
      const vec c = circle_qr.solve(circle_b);
      const vec center = c.x()*u + c.y()*v;
      const double radius2 = c.z()+c.x()*c.x()+c.y()*c.y();
      if (radius2 > 1e-10 && scale*std::sqrt(radius2) <= config.max_radius) {
        model cylinder;
        cylinder.type = "cylinder";
        cylinder.axis = axis;
        cylinder.center = origin+scale*center;
        cylinder.radii.setConstant(scale*std::sqrt(radius2));
        cylinder.q = equation(mat::Identity()-axis*axis.transpose(), -2*center, -c.z());
        evaluate(cylinder, 5);
      }
    }
    const auto ellipse_qr = ellipse_a.colPivHouseholderQr();
    if (ellipse_qr.rank() == 5) {
      const Eigen::Matrix<double,5,1> c = ellipse_qr.solve(Eigen::VectorXd::Ones(fit_num));
      Eigen::Matrix2d a;
      a << c(0), c(1), c(1), c(2);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> axes(a);
      if (axes.info() == Eigen::Success && std::abs(a.determinant()) > 1e-10) {
        const Eigen::Vector2d center = -0.5*a.inverse()*c.tail<2>();
        const double level = 1.0+center.dot(a*center);
        if (level != 0 && (axes.eigenvalues()/level).minCoeff() > 1e-10) {
          Eigen::Vector2d radii = (axes.eigenvalues()/level).array().sqrt().inverse()*scale;
          if (radii.maxCoeff() <= config.max_radius) {
            model ellipse;
            ellipse.type = "elliptic_cylinder";
            ellipse.axis = axis;
            ellipse.center = origin+scale*(center.x()*u+center.y()*v);
            const int major_idx = radii.x() >= radii.y() ? 0 : 1;
            ellipse.major_direction = axes.eigenvectors()(0,major_idx)*u+axes.eigenvectors()(1,major_idx)*v;
            ellipse.radii << radii.maxCoeff(), radii.minCoeff();
            ellipse.q = equation(c(0)*u*u.transpose()+c(1)*(u*v.transpose()+v*u.transpose())+
              c(2)*v*v.transpose(), c(3)*u+c(4)*v, -1.0);
            evaluate(ellipse, 7);
          }
        }
      }
    }
  }

  Eigen::Matrix<double,10,10> moment = Eigen::Matrix<double,10,10>::Zero();
  for (std::size_t i = 0; i < fit_num; ++i) {
    const coefficients f = features(samples[i*samples.size()/fit_num].p);
    moment.noalias() += f*f.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,10,10>> solver(moment);
  // 多重零空間（平面・直線等）での任意の二次曲面選択の抑止。
  if (solver.info() == Eigen::Success && solver.eigenvalues()(1) > 1e-9*solver.eigenvalues()(9)) {
    model quadric;
    quadric.type = "quadric";
    quadric.center = origin;
    quadric.q = solver.eigenvectors().col(0);
    const auto &q=quadric.q;
    Eigen::Matrix4d implicit;
    implicit << q(0),q(3)/2,q(4)/2,q(6)/2,
      q(3)/2,q(1),q(5)/2,q(7)/2,
      q(4)/2,q(5)/2,q(2),q(8)/2,
      q(6)/2,q(7)/2,q(8)/2,q(9);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> rank(implicit,Eigen::EigenvaluesOnly);
    // Qの階数2以下は2平面の積等の退化モデル。入力パッチ数とは独立した曲面モデルの判定。
    if (rank.info()==Eigen::Success) {
      Eigen::Vector4d magnitudes=rank.eigenvalues().cwiseAbs();
      std::sort(magnitudes.data(),magnitudes.data()+4);
      if (magnitudes(1)>1e-6*magnitudes(3)) evaluate(quadric, 9);
    }
  }
  return best;
}
}

node_dev model_dev(const model &shape, const ais_gng_msgs::msg::TopologicalNode &node)
{
  if (!(shape.scale>0) || !std::isfinite(shape.scale) || !shape.q.allFinite()) return {};
  const vec p=(position(node.pos)-shape.origin)/shape.scale;
  if (!p.allFinite()) return {};
  const vec g=gradient(shape.q,p);
  if (!g.allFinite() || g.norm()<1e-8) return {};
  const double dist=shape.scale*std::abs(shape.q.dot(features(p)))/g.norm();
  if (!std::isfinite(dist)) return {};
  return {dist,std::abs(normal(position(node.normal)).dot(g.normalized()))};
}

std::size_t plane_patch_num(const result &surfaces, const region &surface)
{
  std::set<int> planes;
  for (auto idx:surface.patch_indices) {
    const int plane=surfaces.patches.at(idx).plane_cluster_idx;
    if (plane>=0) planes.insert(plane);
  }
  return planes.size();
}

result extract(const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes, const options &config,
  const std::vector<region> &retained)
{
  if (map.frame_number != planes.frame_number || map.header.frame_id != planes.header.frame_id) {
    throw std::invalid_argument("surface models require matching map/plane frame and coordinate frame");
  }
  if (!(config.max_link_length > 0) || !(config.max_patch_rms > 0) ||
    !(config.max_point_residual > 0) || !(config.complexity_penalty >= 0) ||
    !(config.max_radius > 0) || !(config.max_curvature_normal_error > 0) ||
    !std::isfinite(config.max_support_gap) || config.max_support_gap<0 ||
    !std::isfinite(config.max_support_spacing_ratio) || config.max_support_spacing_ratio<1 ||
    !std::isfinite(config.min_plane_usage_ratio) || config.min_plane_usage_ratio<0 || config.min_plane_usage_ratio>1 ||
    config.min_fit_nodes < 10 || config.max_fit_samples < config.min_fit_nodes ||
    config.max_model_fits == 0 || !(config.max_link_normal_deg > 0 && config.max_link_normal_deg < 90) ||
    !(config.max_normal_deg > 0 && config.max_normal_deg < 90)) {
    throw std::invalid_argument("invalid surface model options");
  }
  const auto begin = std::chrono::steady_clock::now();
  result output;
  std::vector<vec> points, normals;
  for (const auto &node : map.nodes) {
    points.push_back(position(node.pos));
    normals.push_back(normal(position(node.normal)));
  }
  std::vector<int> owner(points.size(), -1);
  std::vector<int> retained_owner(points.size(), -1);
  for (std::size_t i=0; i<retained.size(); ++i) for (auto idx:retained[i].node_indices) {
    if (idx<points.size() && retained_owner[idx]<0) retained_owner[idx]=static_cast<int>(i);
  }
  const auto add_patch = [&](local_patch patch) {
    if (patch.node_indices.empty()) return;
    for (auto idx : patch.node_indices) {
      owner[idx]=static_cast<int>(output.patches.size());
      patch.center += points[idx];
    }
    patch.center /= static_cast<double>(patch.node_indices.size());
    output.patches.push_back(std::move(patch));
  };
  for (std::size_t i = 0; i < planes.clusters.size(); ++i) {
    // 下位の平面クラスタは変更せず、維持所属と逸脱点を上位パッチ内だけで分離。
    std::map<int,local_patch> fragments;
    for (auto idx : planes.clusters[i].node_indices) {
      if (idx < points.size() && points[idx].allFinite() && owner[idx] < 0) {
        owner[idx] = 0;
        auto &patch=fragments[retained_owner[idx]];
        patch.plane_cluster_idx = static_cast<int>(i);
        patch.node_indices.push_back(idx);
      }
    }
    for (auto &entry:fragments) add_patch(std::move(entry.second));
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (owner[i] < 0 && points[i].allFinite()) {
      owner[i] = static_cast<int>(output.patches.size());
      local_patch patch;
      patch.node_indices.push_back(static_cast<std::uint32_t>(i));
      add_patch(std::move(patch));
    }
  }
  std::set<std::array<std::uint32_t,2>> edges, smooth, sharp, uncertain;
  const auto curvature_begin = std::chrono::steady_clock::now();
  for (auto &patch : output.patches) {
    if (patch.node_indices.size()<8 || patch.plane_cluster_idx<0) continue;
    patch.curvature=estimate_curvature(patch,map);
  }
  output.curvature_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - curvature_begin).count();
  const auto boundary_begin=std::chrono::steady_clock::now();
  struct boundary_support {
    vec center=vec::Zero();
    double link_length=0;
    std::size_t num=0;
    std::array<patch_curvature,2> local;
    std::array<const patch_curvature *,2> selected{nullptr,nullptr};
  };
  std::map<std::array<std::uint32_t,2>,boundary_support> boundaries;
  // 原GNGの短い境界エッジごとの集合。パッチ対・片側ごとに一度だけの追加推定。
  for (std::size_t i=0; i+1<map.edges.size(); i+=2) {
    const auto a=map.edges[i],b=map.edges[i+1];
    if (a>=owner.size() || b>=owner.size() || owner[a]<0 || owner[b]<0 || owner[a]==owner[b]) continue;
    const auto left=output.patches[owner[a]].plane_cluster_idx;
    const auto right=output.patches[owner[b]].plane_cluster_idx;
    const double dist=(points[a]-points[b]).norm();
    if (left<0 || right<0 || left==right || dist>config.max_link_length) continue;
    auto &support=boundaries[{static_cast<std::uint32_t>(std::min(owner[a],owner[b])),
      static_cast<std::uint32_t>(std::max(owner[a],owner[b]))}];
    support.center+=(points[a]+points[b])*0.5;
    support.link_length+=dist;
    ++support.num;
  }
  constexpr double min_boundary_confidence=0.5;
  for (auto &[pair,support]:boundaries) {
    support.center/=static_cast<double>(support.num);
    support.link_length/=static_cast<double>(support.num);
    for (std::size_t side=0; side<2; ++side) {
      const auto &patch=output.patches[pair[side]];
      const auto &c=patch.curvature;
      if (c.valid && c.confidence>=min_boundary_confidence) {
        support.selected[side]=&c;
        continue;
      }
      if (output.boundary_fit_num>=config.max_boundary_fits) continue;
      const double radius=std::max(2*support.link_length,0.75*std::sqrt(c.support_cov.trace()));
      local_patch local;
      for (auto idx:patch.node_indices) {
        if ((points[idx]-support.center).squaredNorm()<=radius*radius) local.node_indices.push_back(idx);
      }
      if (local.node_indices.size()<8 || local.node_indices.size()==patch.node_indices.size()) continue;
      ++output.boundary_fit_num;
      support.local[side]=estimate_curvature(local,map);
      if (support.local[side].valid && support.local[side].confidence>=min_boundary_confidence) {
        support.selected[side]=&support.local[side];
      }
    }
  }
  const auto can_evaluate_boundary=[](const patch_curvature &c,const vec &point) {
    // 面内支持範囲から離れた境界における、二次式の過大な外挿の抑止。
    constexpr double max_support_dev=16;
    const vec delta=point-c.height_origin;
    const double u=c.axis_u.dot(delta),v=c.axis_v.dot(delta);
    const auto &s=c.support_cov;
    const double determinant=s(0,0)*s(1,1)-s(0,1)*s(1,0);
    if (!(determinant>0)) return false;
    const double dev=(s(1,1)*u*u-2*s(0,1)*u*v+s(0,0)*v*v)/determinant;
    return std::isfinite(dev) && dev<=max_support_dev;
  };
  const double min_link_cos = std::cos(config.max_link_normal_deg*pi/180.0);
  enum class link_kind { none, smooth, sharp, uncertain };
  const auto classify_boundary = [&](const patch_curvature *a,const patch_curvature *b,const vec &point) {
    if (!a || !b || !a->valid || !b->valid ||
      !(a->confidence>=min_boundary_confidence) || !(b->confidence>=min_boundary_confidence) ||
      !can_evaluate_boundary(*a,point) || !can_evaluate_boundary(*b,point)) return link_kind::uncertain;
    return std::abs(patch_normal_at(*a,point).dot(patch_normal_at(*b,point)))<min_link_cos ?
      link_kind::sharp:link_kind::smooth;
  };
  // 初回構築・部分分割に共通の実エッジ走査と境界優先順位。分類規則だけを呼出側で指定。
  const auto rebuild_links = [&](const auto &classify) {
    edges.clear(); smooth.clear(); sharp.clear(); uncertain.clear();
    for (std::size_t i=0;i+1<map.edges.size();i+=2) {
      const auto a=map.edges[i],b=map.edges[i+1];
      if (a>=owner.size() || b>=owner.size() || owner[a]<0 || owner[b]<0 || owner[a]==owner[b]) continue;
      const std::array<std::uint32_t,2> pair={static_cast<std::uint32_t>(std::min(owner[a],owner[b])),
        static_cast<std::uint32_t>(std::max(owner[a],owner[b]))};
      edges.insert(pair);
      if ((points[a]-points[b]).norm()>config.max_link_length) continue;
      switch (classify(pair,a,b)) {
        case link_kind::smooth: smooth.insert(pair); break;
        case link_kind::sharp: sharp.insert(pair); break;
        case link_kind::uncertain: uncertain.insert(pair); break;
        case link_kind::none: break;
      }
    }
    for (const auto &pair:sharp) { smooth.erase(pair); uncertain.erase(pair); }
    for (const auto &pair:smooth) uncertain.erase(pair);
  };
  rebuild_links([&](const auto &pair,auto a,auto b) {
    const auto support=boundaries.find(pair);
    if (support!=boundaries.end()) {
      const auto &selected=support->second.selected;
      // 信頼できる位置由来の接平面同士の比較。入力法線による二重拒否なし。
      return classify_boundary(selected[0],selected[1],(points[a]+points[b])*0.5);
    }
    return std::abs(normals[a].dot(normals[b]))>=min_link_cos ? link_kind::smooth:link_kind::none;
  });
  output.boundary_ms=std::chrono::duration<double,std::milli>(
    std::chrono::steady_clock::now()-boundary_begin).count();
  std::vector<std::vector<std::uint32_t>> adjacency(output.patches.size());
  std::vector<std::vector<std::uint32_t>> conflicts(output.patches.size());
  const auto append_neighbors = [](auto &neighbors,const auto &links) {
    for (const auto &pair:links) {
      neighbors[pair[0]].push_back(pair[1]); neighbors[pair[1]].push_back(pair[0]);
    }
  };
  append_neighbors(adjacency,smooth);
  // 不明境界は探索候補のみ。新規統合には既存のモデル残差・法線・退化検査が必要。
  append_neighbors(adjacency,uncertain);
  std::vector<bool> has_owner(output.patches.size(), false), has_visited(output.patches.size(), false);
  append_neighbors(conflicts,sharp);
  std::vector<bool> is_proposed(output.patches.size(),false);
  const auto has_conflict = [&](const std::vector<std::uint32_t> &ids) {
    for (auto idx:ids) is_proposed[idx]=true;
    bool has_sharp_edge=false;
    for (auto idx:ids) for (auto neighbor:conflicts[idx]) if (is_proposed[neighbor]) has_sharp_edge=true;
    for (auto idx:ids) is_proposed[idx]=false;
    return has_sharp_edge;
  };
  const auto fit = [&](const std::vector<std::uint32_t> &ids) {
    // 新規・保持曲面に共通の鋭い境界の保護。迂回接続による境界横断の禁止。
    if (output.model_fits>=config.max_model_fits || has_conflict(ids)) return model{};
    ++output.model_fits;
    return fit_best(ids, output.patches, points, normals, config);
  };
  const auto finish = [&](const std::vector<std::uint32_t> &ids, model shape, const region *previous=nullptr) {
    region surface;
    surface.patch_indices = ids;
    surface.shape = std::move(shape);
    for (auto id : ids) {
      has_owner[id] = true;
      const auto &members = output.patches[id].node_indices;
      surface.node_indices.insert(surface.node_indices.end(),members.begin(),members.end());
    }
    // ノード配列添字の最小値をフレーム内IDに採用。永続追跡IDとは非互換。
    surface.id = *std::min_element(surface.node_indices.begin(),surface.node_indices.end());
    if (previous) {
      surface.id=previous->id;
      surface.is_retained=true;
      surface.seed_plane_patch_num=previous->seed_plane_patch_num;
      surface.rejected_node_num=previous->rejected_node_num;
      surface.shape.max_patch_rms=0;
      for (auto idx:ids) {
        double sum=0;
        for (auto node_idx:output.patches[idx].node_indices) {
          const double error=model_dev(surface.shape,map.nodes[node_idx]).dist;
          sum+=error*error;
        }
        surface.shape.max_patch_rms=std::max(surface.shape.max_patch_rms,
          std::sqrt(sum/output.patches[idx].node_indices.size()));
      }
    }
    output.regions.push_back(std::move(surface));
  };

  // 同じ元平面内で接続する領域の再評価。履歴だけによる分断の解消、全点検査による異曲面の保護。
  const auto merge_plane_fragments = [&]() {
    std::vector<std::size_t> region_owner(output.patches.size(),output.regions.size());
    for (std::size_t i=0;i<output.regions.size();++i)
      for (auto idx:output.regions[i].patch_indices) region_owner[idx]=i;
    std::set<std::array<std::size_t,2>> tested;
    for (const auto &pair:edges) {
      if (output.model_fits>=config.max_model_fits) break;
      const int plane=output.patches[pair[0]].plane_cluster_idx;
      if (plane<0 || plane!=output.patches[pair[1]].plane_cluster_idx ||
        (!smooth.count(pair) && !uncertain.count(pair))) continue;
      auto a=region_owner[pair[0]],b=region_owner[pair[1]];
      if (a==output.regions.size() || b==output.regions.size()) continue;
      if (a>b) std::swap(a,b);
      if (a==b || !tested.insert({a,b}).second) continue;
      auto ids=output.regions[a].patch_indices;
      ids.insert(ids.end(),output.regions[b].patch_indices.begin(),output.regions[b].patch_indices.end());
      auto shape=fit(ids);
      if (shape.type=="unknown") continue;
      const bool has_single_track=output.regions[a].is_retained!=output.regions[b].is_retained;
      const region previous=output.regions[output.regions[a].is_retained ? a:b];
      if (has_single_track) {
        // 既存核への追加では係数・参照集合の維持。新規IDによる追跡支持率の水増し防止。
        shape=previous.shape;
        double sum=0;
        std::size_t num=0;
        bool can_keep=true;
        shape.max_patch_rms=0;
        const double min_cos=std::cos(config.max_normal_deg*pi/180.0);
        for (auto idx:ids) {
          double patch_sum=0;
          for (auto node_idx:output.patches[idx].node_indices) {
            const auto dev=model_dev(shape,map.nodes[node_idx]);
            if (dev.dist>config.max_point_residual ||
              (normals[node_idx].squaredNorm()>0.5 && dev.normal_cos<min_cos)) can_keep=false;
            patch_sum+=dev.dist*dev.dist; ++num;
          }
          sum+=patch_sum;
          shape.max_patch_rms=std::max(shape.max_patch_rms,
            std::sqrt(patch_sum/output.patches[idx].node_indices.size()));
        }
        if (!can_keep || shape.max_patch_rms>config.max_patch_rms) continue;
        shape.score+=sum/num-shape.rms*shape.rms;
        shape.rms=std::sqrt(sum/num);
      }
      auto &merged=output.regions[a];
      merged.patch_indices=std::move(ids);
      merged.node_indices.insert(merged.node_indices.end(),
        output.regions[b].node_indices.begin(),output.regions[b].node_indices.end());
      merged.shape=std::move(shape);
      // 再推定後は新規追跡核。旧片側だけの参照集合による再分裂の防止。
      merged.is_retained=false;
      merged.id=*std::min_element(merged.node_indices.begin(),merged.node_indices.end());
      merged.seed_plane_patch_num=std::max(merged.seed_plane_patch_num,output.regions[b].seed_plane_patch_num);
      if (has_single_track) {
        merged.is_retained=true; merged.id=previous.id;
        merged.seed_plane_patch_num=previous.seed_plane_patch_num;
        merged.rejected_node_num=previous.rejected_node_num;
      }
      for (auto idx:output.regions[b].patch_indices) region_owner[idx]=a;
      output.regions[b].node_indices.clear();
      tested.clear();
    }
    output.regions.erase(std::remove_if(output.regions.begin(),output.regions.end(),
      [](const auto &surface) { return surface.node_indices.empty(); }),output.regions.end());
  };

  // 既存曲面は接続成分に依存せず維持。ただし現在フレームの明確な鋭い境界は維持対象外。
  std::vector<std::vector<std::uint32_t>> retained_patches(retained.size());
  for (std::size_t i=0; i<output.patches.size(); ++i) {
    const int idx=retained_owner[output.patches[i].node_indices.front()];
    if (idx>=0) retained_patches[idx].push_back(i);
  }
  for (std::size_t i=0; i<retained.size(); ++i) {
    const auto &ids=retained_patches[i];
    std::size_t node_num=0;
    for (auto idx:ids) node_num+=output.patches[idx].node_indices.size();
    if (!has_conflict(ids) && node_num>=config.min_fit_nodes) finish(ids,retained[i].shape,&retained[i]);
  }
  merge_plane_fragments();

  // 全体不適合の平面由来パッチだけを、成立済み曲面から到達可能な適合点と残余へ分離。
  std::vector<std::vector<std::uint32_t>> neighbors;
  const double min_node_cos=std::cos(config.max_normal_deg*pi/180.0);
  const auto can_attach_node = [&](const model &shape,std::uint32_t idx) {
    const auto dev=model_dev(shape,map.nodes[idx]);
    return dev.dist<=std::min(config.max_patch_rms,config.max_point_residual) &&
      (normals[idx].squaredNorm()<0.5 || dev.normal_cos>=min_node_cos);
  };
  const auto fit_partial_patch = [&](const std::vector<std::uint32_t> &members,
    std::uint32_t next,const model &current) -> std::pair<std::uint32_t,model> {
    const auto rejected=std::make_pair(next,model{});
    if (current.type=="plane" || current.type=="unknown" ||
      output.patches[next].plane_cluster_idx<0 || output.model_fits>=config.max_model_fits) return rejected;
    std::vector<bool> can_join(points.size(),false),is_reached(points.size(),false);
    for (auto idx:output.patches[next].node_indices) can_join[idx]=can_attach_node(current,idx);
    if (neighbors.empty()) {
      neighbors.resize(points.size());
      for (std::size_t i=0;i+1<map.edges.size();i+=2) {
        const auto a=map.edges[i],b=map.edges[i+1];
        if (a>=points.size() || b>=points.size() ||
          (points[a]-points[b]).norm()>config.max_link_length) continue;
        neighbors[a].push_back(b); neighbors[b].push_back(a);
      }
    }
    std::vector<std::uint32_t> reached;
    for (auto idx:members) for (auto node_idx:output.patches[idx].node_indices) reached.push_back(node_idx);
    for (std::size_t i=0;i<reached.size();++i) for (auto idx:neighbors[reached[i]]) {
      if (can_join[idx] && !is_reached[idx]) { is_reached[idx]=true; reached.push_back(idx); }
    }
    local_patch selected,remaining;
    selected.plane_cluster_idx=remaining.plane_cluster_idx=output.patches[next].plane_cluster_idx;
    for (auto idx:output.patches[next].node_indices)
      (is_reached[idx] ? selected:remaining).node_indices.push_back(idx);
    constexpr std::size_t min_partial_nodes=3;
    if (selected.node_indices.size()<min_partial_nodes || remaining.node_indices.empty()) return rejected;
    const auto update_patch = [&](local_patch &patch) {
      for (auto idx:patch.node_indices) patch.center+=points[idx];
      patch.center/=static_cast<double>(patch.node_indices.size());
      patch.curvature=estimate_curvature(patch,map);
    };
    const auto partial_curvature_begin=std::chrono::steady_clock::now();
    update_patch(selected); update_patch(remaining);
    output.curvature_ms+=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-partial_curvature_begin).count();
    const auto selected_idx=static_cast<std::uint32_t>(output.patches.size());
    output.patches.push_back(std::move(selected));
    auto ids=members; ids.push_back(selected_idx);
    // 混入部分除去後の実接続位置での境界再検査。元パッチ全体の誤った鋭角判定からの回復。
    const auto &selected_curvature=output.patches.back().curvature;
    for (auto idx:members) for (auto a:output.patches[idx].node_indices) for (auto b:neighbors[a]) {
      if (!is_reached[b]) continue;
      if (classify_boundary(&output.patches[idx].curvature,&selected_curvature,
        (points[a]+points[b])*0.5)==link_kind::sharp) {
        output.patches.pop_back(); return rejected;
      }
    }
    ++output.model_fits;
    auto shape=fit_best(ids,output.patches,points,normals,config);
    if (shape.type=="unknown") { output.patches.pop_back(); return rejected; }
    output.patches[next]=std::move(remaining);
    for (auto idx:output.patches[selected_idx].node_indices) owner[idx]=selected_idx;
    has_owner.push_back(false); has_visited.push_back(true); is_proposed.push_back(false);
    // 分割前の境界分類を継承し、実エッジの接続先だけを更新。存在しないパッチ接続の防止。
    const auto old_smooth=smooth,old_sharp=sharp,old_uncertain=uncertain;
    rebuild_links([&](const auto &pair,auto a,auto b) {
      if (pair[0]==selected_idx || pair[1]==selected_idx || pair[0]==next || pair[1]==next) {
        return classify_boundary(&output.patches[pair[0]].curvature,&output.patches[pair[1]].curvature,
          (points[a]+points[b])*0.5);
      }
      if (old_sharp.count(pair)) return link_kind::sharp;
      if (old_smooth.count(pair)) return link_kind::smooth;
      return old_uncertain.count(pair) ? link_kind::uncertain:link_kind::none;
    });
    adjacency.assign(output.patches.size(),{}); conflicts.assign(output.patches.size(),{});
    auto traversable=smooth; traversable.insert(uncertain.begin(),uncertain.end());
    append_neighbors(adjacency,traversable); append_neighbors(conflicts,sharp);
    return {selected_idx,std::move(shape)};
  };

  for (std::size_t start = 0; start < output.patches.size(); ++start) {
    if (has_visited[start] || has_owner[start]) continue;
    std::vector<std::uint32_t> connected{static_cast<std::uint32_t>(start)};
    has_visited[start] = true;
    for (std::size_t i = 0; i < connected.size(); ++i) {
      for (auto next : adjacency[connected[i]]) if (!has_visited[next] && !has_owner[next]) {
        has_visited[next] = true;
        connected.push_back(next);
      }
    }
    auto whole = fit(connected);
    if (whole.type != "unknown") { finish(connected, whole); continue; }

    // 全体不適合時の局所成長。平面パッチと未所属ノードに共通の追加・再推定判定。
    std::stable_sort(connected.begin(),connected.end(),[&](auto a, auto b) {
      // 混入で大きくなった低品質パッチより、位置支持の確かな核を優先。
      const auto &ca=output.patches[a].curvature,&cb=output.patches[b].curvature;
      const bool has_quality_a=ca.valid && ca.confidence>=0.5;
      const bool has_quality_b=cb.valid && cb.confidence>=0.5;
      if (has_quality_a!=has_quality_b) return has_quality_a;
      return output.patches[a].node_indices.size() > output.patches[b].node_indices.size();
    });
    for (auto seed : connected) {
      if (has_owner[seed] || output.model_fits >= config.max_model_fits) continue;
      std::vector<std::uint32_t> members{seed};
      std::set<std::uint32_t> included{seed};
      std::size_t sample_num = output.patches[seed].node_indices.size();
      for (std::size_t i = 0; i < members.size() && sample_num < config.min_fit_nodes; ++i) {
        for (auto next : adjacency[members[i]]) {
          if (has_owner[next] || !included.insert(next).second) continue;
          members.push_back(next);
          sample_num += output.patches[next].node_indices.size();
          if (sample_num >= config.min_fit_nodes) break;
        }
      }
      auto current = fit(members);
      if (current.type == "unknown") continue;
      bool has_growth = true;
      while (has_growth && output.model_fits < config.max_model_fits) {
        has_growth = false;
        std::set<std::uint32_t> candidates;
        for (auto member : members) for (auto next : adjacency[member]) {
          if (!has_owner[next] && !included.count(next)) candidates.insert(next);
        }
        for (auto member:members) for (auto next:conflicts[member]) {
          if (!has_owner[next] && !included.count(next)) candidates.insert(next);
        }
        // 現モデルに十分適合する隣接パッチの一括検査。ノード単位の再推定による予算消費の抑制。
        auto batch=members;
        for (auto next:candidates) {
          const auto &nodes=output.patches[next].node_indices;
          if (std::all_of(nodes.begin(),nodes.end(),[&](auto idx) {
            return can_attach_node(current,idx);
          })) batch.push_back(next);
        }
        if (batch.size()>members.size()+1) {
          auto shape=fit(batch);
          if (shape.type!="unknown") {
            members=std::move(batch); included.insert(members.begin(),members.end());
            current=std::move(shape); has_growth=true;
            continue;
          }
        }
        for (auto next : candidates) {
          auto proposal = members;
          proposal.push_back(next);
          auto shape = fit(proposal);
          auto accepted=next;
          if (shape.type=="unknown") {
            auto partial=fit_partial_patch(members,next,current);
            accepted=partial.first; shape=std::move(partial.second);
            proposal.back()=accepted;
          }
          if (shape.type != "unknown") {
            members = std::move(proposal);
            included.insert(accepted);
            current = std::move(shape);
            has_growth = true;
          }
          if (output.model_fits >= config.max_model_fits) break;
        }
      }
      finish(members, current);
    }
  }
  for (std::size_t i = 0; i < output.patches.size(); ++i) {
    if (!has_owner[i]) finish({static_cast<std::uint32_t>(i)}, model{});
  }
  merge_plane_fragments();
  output.patch_edges.assign(edges.begin(),edges.end());
  output.smooth_edges.assign(smooth.begin(),smooth.end());
  output.sharp_edges.assign(sharp.begin(),sharp.end());
  output.uncertain_edges.assign(uncertain.begin(),uncertain.end());
  split_support_regions(output,map,config);
  if (config.min_plane_usage_ratio>0) {
    // 分母は現在フレームの有効な元平面所属。分割パッチの使用数は同じ元平面ごとに合算。
    std::vector<std::size_t> plane_sizes(planes.clusters.size(),0);
    for (const auto &patch:output.patches) if (patch.plane_cluster_idx>=0)
      plane_sizes[patch.plane_cluster_idx]+=patch.node_indices.size();
    for (auto &surface:output.regions) {
      if (surface.shape.type=="unknown" || surface.shape.type=="plane") continue;
      std::map<int,std::size_t> used;
      for (auto idx:surface.patch_indices) {
        const auto &patch=output.patches[idx];
        if (patch.plane_cluster_idx>=0) used[patch.plane_cluster_idx]+=patch.node_indices.size();
      }
      const bool has_plane_core=std::any_of(used.begin(),used.end(),[&](const auto &entry) {
        return entry.second>=config.min_plane_usage_ratio*plane_sizes[entry.first];
      });
      if (!has_plane_core) {
        // ノード被覆は保持し、曲面仮説と追跡資格だけを棄却。
        surface.shape=model{}; surface.is_retained=false; surface.seed_plane_patch_num=0;
        surface.id=*std::min_element(surface.node_indices.begin(),surface.node_indices.end());
      }
    }
  }
  output.update_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  return output;
}
}  // namespace fuzzrobo::surface_model
