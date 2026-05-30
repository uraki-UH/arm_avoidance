#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

#include "collision/collision_detector.hpp"

namespace simulation {

struct SimplifiedPrimitive {
  enum class Type { Sphere, Cylinder };
  Type type = Type::Sphere;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
  double radius = 0.0;
  double length = 0.0;
};

namespace detail {

inline Eigen::Quaterniond axisToQuaternion(const Eigen::Vector3d &axis) {
  Eigen::Vector3d n = axis;
  if (n.norm() < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  n.normalize();
  return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), n);
}

inline Eigen::Vector3d centroid(const std::vector<Eigen::Vector3d> &points) {
  if (points.empty()) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto &p : points) {
    sum += p;
  }
  return sum / static_cast<double>(points.size());
}

inline Eigen::Vector3d principalAxis(const std::vector<Eigen::Vector3d> &points) {
  if (points.size() < 2) {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d c = centroid(points);
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto &p : points) {
    const Eigen::Vector3d d = p - c;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(points.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if (solver.info() != Eigen::Success) {
    return Eigen::Vector3d::Zero();
  }
  return solver.eigenvectors().col(2);
}

inline bool detectLinearCapsuleCandidate(
    const std::vector<collision::Sphere> &spheres,
    double voxel_size,
    std::size_t min_chain_spheres,
    double axis_ratio_threshold,
    double radius_cv_threshold,
    collision::Capsule &out_capsule,
    double &out_radius) {
  if (spheres.size() < min_chain_spheres) {
    return false;
  }

  std::vector<Eigen::Vector3d> centers;
  centers.reserve(spheres.size());
  std::vector<double> radii;
  radii.reserve(spheres.size());
  for (const auto &s : spheres) {
    centers.push_back(s.center);
    radii.push_back(s.radius);
  }

  const Eigen::Vector3d c = centroid(centers);
  const Eigen::Vector3d axis = principalAxis(centers);
  if (axis.norm() < 1e-12) {
    return false;
  }

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto &p : centers) {
    const Eigen::Vector3d d = p - c;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(centers.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if (solver.info() != Eigen::Success) {
    return false;
  }

  const auto eigenvalues = solver.eigenvalues();
  const double l0 = eigenvalues.z();
  const double l1 = eigenvalues.y();
  if (l0 < 1e-12 || l0 < axis_ratio_threshold * std::max(1e-12, l1)) {
    return false;
  }

  const double mean_r = std::accumulate(radii.begin(), radii.end(), 0.0) / static_cast<double>(radii.size());
  double var_r = 0.0;
  for (double r : radii) {
    const double dr = r - mean_r;
    var_r += dr * dr;
  }
  var_r /= static_cast<double>(radii.size());
  const double std_r = std::sqrt(std::max(0.0, var_r));
  const double cv = std_r / std::max(1e-12, mean_r);
  if (cv > radius_cv_threshold) {
    return false;
  }

  double min_t = std::numeric_limits<double>::max();
  double max_t = std::numeric_limits<double>::lowest();
  double max_orth_dist = 0.0;
  for (const auto &p : centers) {
    const Eigen::Vector3d d = p - c;
    const double t = d.dot(axis);
    const Eigen::Vector3d orth = d - axis * t;
    min_t = std::min(min_t, t);
    max_t = std::max(max_t, t);
    max_orth_dist = std::max(max_orth_dist, orth.norm());
  }

  const double span = max_t - min_t;
  if (span <= voxel_size * 2.0) {
    return false;
  }

  const double orth_tol = std::max(voxel_size * 2.5, mean_r * 0.75);
  if (max_orth_dist > orth_tol) {
    return false;
  }

  const Eigen::Vector3d start = c + axis * min_t;
  const Eigen::Vector3d end = c + axis * max_t;
  out_radius = std::max(mean_r, voxel_size * 0.5);
  out_capsule.a = start;
  out_capsule.b = end;
  out_capsule.radius = out_radius;
  return true;
}

} // namespace detail

inline std::vector<SimplifiedPrimitive>
convertSpheresToPrimitives(const std::vector<collision::Sphere> &spheres,
                           double voxel_size,
                           std::size_t min_chain_spheres,
                           double axis_ratio_threshold,
                           double radius_cv_threshold) {
  std::vector<SimplifiedPrimitive> primitives;
  if (spheres.empty()) {
    return primitives;
  }

  collision::Capsule capsule;
  double capsule_radius = 0.0;
  if (detail::detectLinearCapsuleCandidate(spheres, voxel_size, min_chain_spheres,
                                           axis_ratio_threshold, radius_cv_threshold,
                                           capsule, capsule_radius)) {
    const Eigen::Vector3d axis = capsule.b - capsule.a;
    const double len = axis.norm();
    if (len > 1e-12) {
      const Eigen::Quaterniond q = detail::axisToQuaternion(axis);
      const Eigen::Vector3d mid = 0.5 * (capsule.a + capsule.b);
      const double cyl_len = std::max(0.0, len - 2.0 * capsule_radius);

      SimplifiedPrimitive cylinder;
      cylinder.type = SimplifiedPrimitive::Type::Cylinder;
      cylinder.xyz = mid;
      cylinder.quat = q;
      cylinder.radius = capsule_radius;
      cylinder.length = cyl_len;
      primitives.push_back(cylinder);

      SimplifiedPrimitive sphere_a;
      sphere_a.type = SimplifiedPrimitive::Type::Sphere;
      sphere_a.xyz = capsule.a;
      sphere_a.radius = capsule_radius;
      primitives.push_back(sphere_a);

      SimplifiedPrimitive sphere_b;
      sphere_b.type = SimplifiedPrimitive::Type::Sphere;
      sphere_b.xyz = capsule.b;
      sphere_b.radius = capsule_radius;
      primitives.push_back(sphere_b);
      return primitives;
    }
  }

  for (const auto &sphere : spheres) {
    SimplifiedPrimitive primitive;
    primitive.type = SimplifiedPrimitive::Type::Sphere;
    primitive.xyz = sphere.center;
    primitive.radius = sphere.radius;
    primitives.push_back(primitive);
  }
  return primitives;
}

} // namespace simulation

