#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "collision/collision_detector.hpp"
#include "robot_model/robot_voxelizer.hpp"

namespace simulation {

struct VoxelSphereFitOptions {
  std::size_t max_spheres = 64;
  std::size_t min_points_per_sphere = 12;
  double min_gain_ratio = 0.15;
  std::size_t refine_iterations = 2;
  double containment_margin = 0.5;
  bool verbose = false;
};

struct VoxelSphereLinkResult {
  std::string link_name;
  std::size_t voxel_count = 0;
  std::vector<Eigen::Vector3d> voxel_centers;
  std::vector<collision::Sphere> spheres;
};

class VoxelSpherizer {
public:
  static std::vector<collision::Sphere>
  fit(const std::vector<Eigen::Vector3d> &voxel_centers,
      double voxel_size,
      const VoxelSphereFitOptions &options = {}) {
    std::vector<collision::Sphere> raw_spheres = fitRecursiveSpheres(
        voxel_centers, voxel_size, options,
        std::max<std::size_t>(1, options.max_spheres));
    std::vector<collision::Sphere> spheres = refineSphereSet(
        voxel_centers, voxel_size, raw_spheres, options.refine_iterations);
    const std::vector<collision::Sphere> pruned = pruneContainedSpheres(
        spheres, std::max(0.0, options.containment_margin) * std::max(1e-12, voxel_size));

    if (options.verbose && (raw_spheres.size() != pruned.size() || spheres.size() != pruned.size())) {
      std::cout << "[VoxelSpherizer] refine/prune: raw=" << raw_spheres.size()
                << " refined=" << spheres.size()
                << " pruned=" << pruned.size() << std::endl;
    }
    return pruned;
  }

  static std::vector<VoxelSphereLinkResult>
  fitLinks(const std::vector<LinkVoxelData> &link_voxel_data,
           double voxel_size,
           const VoxelSphereFitOptions &options = {}) {
    std::vector<VoxelSphereLinkResult> results;
    results.reserve(link_voxel_data.size());

    std::size_t total_voxels = 0;
    std::size_t total_spheres = 0;
    for (const auto &link : link_voxel_data) {
      VoxelSphereLinkResult result;
      result.link_name = link.name;
      result.voxel_count = link.local_voxel_centers.size();
      result.voxel_centers = link.local_voxel_centers;
      result.spheres = fit(link.local_voxel_centers, voxel_size, options);
      total_voxels += result.voxel_count;
      total_spheres += result.spheres.size();

      if (options.verbose) {
        std::cout << "[VoxelSpherizer] link=" << result.link_name
                  << " voxels=" << result.voxel_count
                  << " spheres=" << result.spheres.size() << std::endl;
      }

      results.push_back(std::move(result));
    }

    if (options.verbose) {
      std::cout << "[VoxelSpherizer] summary: links=" << results.size()
                << " voxels=" << total_voxels
                << " spheres=" << total_spheres
                << " max_spheres=" << options.max_spheres
                << " min_points_per_sphere=" << options.min_points_per_sphere
                << " min_gain_ratio=" << options.min_gain_ratio << std::endl;
    }

    return results;
  }

private:
  struct FitNode {
    std::vector<collision::Sphere> spheres;
    double cost = std::numeric_limits<double>::infinity();
  };

  static double sphereVolume(const collision::Sphere &sphere) {
    return (4.0 / 3.0) * M_PI * sphere.radius * sphere.radius * sphere.radius;
  }

  static collision::Sphere makeBoundingSphere(const std::vector<Eigen::Vector3d> &points,
                                              double voxel_size) {
    collision::Sphere sphere;
    if (points.empty()) {
      sphere.center.setZero();
      sphere.radius = 0.0;
      return sphere;
    }

    if (points.size() == 1) {
      sphere.center = points.front();
      sphere.radius = voxelHalfDiagonal(voxel_size);
      return sphere;
    }

    const Eigen::Vector3d p0 = points.front();
    const Eigen::Vector3d p1 = farthestPoint(points, p0);
    const Eigen::Vector3d p2 = farthestPoint(points, p1);

    sphere.center = 0.5 * (p1 + p2);
    sphere.radius = 0.5 * (p2 - p1).norm();

    for (const auto &p : points) {
      expandSphereToCoverPoint(sphere, p);
    }

    sphere.radius += voxelHalfDiagonal(voxel_size);
    return sphere;
  }

  static double voxelHalfDiagonal(double voxel_size) {
    return 0.5 * std::sqrt(3.0) * std::max(0.0, voxel_size);
  }

  static Eigen::Vector3d farthestPoint(const std::vector<Eigen::Vector3d> &points,
                                       const Eigen::Vector3d &reference) {
    double max_dist_sq = -1.0;
    Eigen::Vector3d farthest = reference;
    for (const auto &p : points) {
      const double dist_sq = (p - reference).squaredNorm();
      if (dist_sq > max_dist_sq) {
        max_dist_sq = dist_sq;
        farthest = p;
      }
    }
    return farthest;
  }

  static void expandSphereToCoverPoint(collision::Sphere &sphere,
                                       const Eigen::Vector3d &point) {
    const Eigen::Vector3d delta = point - sphere.center;
    const double dist = delta.norm();
    if (dist <= sphere.radius) {
      return;
    }

    if (dist < 1e-12) {
      sphere.radius = std::max(sphere.radius, dist);
      return;
    }

    const double new_radius = 0.5 * (sphere.radius + dist);
    const double shift = new_radius - sphere.radius;
    sphere.center += (delta / dist) * shift;
    sphere.radius = new_radius;
  }

  static bool splitOnAxis(const std::vector<Eigen::Vector3d> &points,
                          const Eigen::Vector3d &axis,
                          std::vector<Eigen::Vector3d> &left,
                          std::vector<Eigen::Vector3d> &right) {
    left.clear();
    right.clear();
    if (points.size() < 2 || axis.norm() < 1e-12) {
      return false;
    }

    const Eigen::Vector3d n = axis.normalized();
    std::vector<double> projections;
    projections.reserve(points.size());
    for (const auto &p : points) {
      projections.push_back(p.dot(n));
    }

    std::vector<double> sorted = projections;
    std::sort(sorted.begin(), sorted.end());
    double split_value = sorted[sorted.size() / 2];

    partitionPoints(points, n, split_value, left, right);
    if (left.empty() || right.empty()) {
      const auto [min_it, max_it] = std::minmax_element(sorted.begin(), sorted.end());
      if (min_it == sorted.end() || max_it == sorted.end()) {
        return false;
      }
      split_value = 0.5 * (*min_it + *max_it);
      partitionPoints(points, n, split_value, left, right);
    }

    if (left.empty() || right.empty()) {
      return false;
    }

    return true;
  }

  static void partitionPoints(const std::vector<Eigen::Vector3d> &points,
                              const Eigen::Vector3d &axis,
                              double split_value,
                              std::vector<Eigen::Vector3d> &left,
                              std::vector<Eigen::Vector3d> &right) {
    left.reserve(points.size());
    right.reserve(points.size());
    for (const auto &p : points) {
      if (p.dot(axis) <= split_value) {
        left.push_back(p);
      } else {
        right.push_back(p);
      }
    }
  }

  static Eigen::Vector3d principalAxis(const std::vector<Eigen::Vector3d> &points) {
    if (points.size() < 2) {
      return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto &p : points) {
      centroid += p;
    }
    centroid /= static_cast<double>(points.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto &p : points) {
      const Eigen::Vector3d d = p - centroid;
      cov += d * d.transpose();
    }
    cov /= static_cast<double>(points.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    if (solver.info() != Eigen::Success) {
      return Eigen::Vector3d::Zero();
    }

    const Eigen::Vector3d values = solver.eigenvalues();
    if (values.z() < 1e-12) {
      return Eigen::Vector3d::Zero();
    }
    return solver.eigenvectors().col(2);
  }

  static Eigen::Vector3d longestAabbAxis(const std::vector<Eigen::Vector3d> &points) {
    if (points.size() < 2) {
      return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d min_pt = points.front();
    Eigen::Vector3d max_pt = points.front();
    for (const auto &p : points) {
      min_pt = min_pt.cwiseMin(p);
      max_pt = max_pt.cwiseMax(p);
    }

    const Eigen::Vector3d extent = max_pt - min_pt;
    if (extent.x() >= extent.y() && extent.x() >= extent.z()) {
      return Eigen::Vector3d::UnitX();
    }
    if (extent.y() >= extent.x() && extent.y() >= extent.z()) {
      return Eigen::Vector3d::UnitY();
    }
    return Eigen::Vector3d::UnitZ();
  }

  static FitNode fitRecursiveNode(const std::vector<Eigen::Vector3d> &points,
                                  double voxel_size,
                                  const VoxelSphereFitOptions &options,
                                  std::size_t budget) {
    FitNode parent;
    parent.spheres.push_back(makeBoundingSphere(points, voxel_size));
    parent.cost = sphereVolume(parent.spheres.front());

    if (points.size() <= options.min_points_per_sphere || budget <= 1) {
      return parent;
    }

    struct Candidate {
      FitNode node;
      double cost = std::numeric_limits<double>::infinity();
    };

    Candidate best;
    const std::vector<Eigen::Vector3d> axes = {
        principalAxis(points),
        longestAabbAxis(points)};

    for (const auto &axis : axes) {
      std::vector<Eigen::Vector3d> left;
      std::vector<Eigen::Vector3d> right;
      if (!splitOnAxis(points, axis, left, right)) {
        continue;
      }
      if (left.size() < 2 || right.size() < 2) {
        continue;
      }

      std::size_t left_budget = std::max<std::size_t>(1, (budget * left.size()) / points.size());
      if (left_budget >= budget) {
        left_budget = budget - 1;
      }
      std::size_t right_budget = budget - left_budget;
      if (right_budget == 0) {
        right_budget = 1;
        if (left_budget > 1) {
          --left_budget;
        }
      }

      FitNode left_node = fitRecursiveNode(left, voxel_size, options, left_budget);
      FitNode right_node = fitRecursiveNode(right, voxel_size, options, right_budget);

      FitNode merged;
      merged.spheres.reserve(left_node.spheres.size() + right_node.spheres.size());
      merged.spheres.insert(merged.spheres.end(), left_node.spheres.begin(), left_node.spheres.end());
      merged.spheres.insert(merged.spheres.end(), right_node.spheres.begin(), right_node.spheres.end());
      merged.cost = left_node.cost + right_node.cost;

      if (merged.cost < best.cost) {
        best.node = std::move(merged);
        best.cost = best.node.cost;
      }
    }

    const double gain_threshold = parent.cost * (1.0 - std::max(0.0, options.min_gain_ratio));
    if (!best.node.spheres.empty() && best.cost < gain_threshold) {
      return best.node;
    }

    return parent;
  }

  static std::vector<collision::Sphere>
  fitRecursiveSpheres(const std::vector<Eigen::Vector3d> &points,
                      double voxel_size,
                      const VoxelSphereFitOptions &options,
                      std::size_t budget) {
    FitNode node = fitRecursiveNode(points, voxel_size, options, budget);
    return node.spheres;
  }

  static std::vector<collision::Sphere>
  refineSphereSet(const std::vector<Eigen::Vector3d> &points,
                  double voxel_size,
                  const std::vector<collision::Sphere> &spheres,
                  std::size_t refine_iterations) {
    if (spheres.size() < 2 || points.size() < 2 || refine_iterations == 0) {
      return spheres;
    }

    std::vector<collision::Sphere> current = spheres;
    for (std::size_t iter = 0; iter < refine_iterations; ++iter) {
      std::vector<std::vector<Eigen::Vector3d>> clusters(current.size());
      for (const auto &p : points) {
        std::size_t best_idx = 0;
        double best_dist_sq = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < current.size(); ++i) {
          const double dist_sq = (p - current[i].center).squaredNorm();
          if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx = i;
          }
        }
        clusters[best_idx].push_back(p);
      }

      std::vector<collision::Sphere> next;
      next.reserve(current.size());
      for (std::size_t i = 0; i < current.size(); ++i) {
        if (clusters[i].empty()) {
          next.push_back(current[i]);
          continue;
        }
        next.push_back(makeBoundingSphere(clusters[i], voxel_size));
      }

      current.swap(next);
    }

    return current;
  }

  static std::vector<collision::Sphere>
  pruneContainedSpheres(const std::vector<collision::Sphere> &spheres,
                        double margin) {
    if (spheres.size() < 2) {
      return spheres;
    }

    std::vector<std::size_t> order(spheres.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      return spheres[a].radius > spheres[b].radius;
    });

    std::vector<bool> keep(spheres.size(), true);
    for (std::size_t oi = 0; oi < order.size(); ++oi) {
      const std::size_t i = order[oi];
      if (!keep[i]) {
        continue;
      }
      for (std::size_t oj = 0; oj < oi; ++oj) {
        const std::size_t j = order[oj];
        if (!keep[j]) {
          continue;
        }
        const double center_dist = (spheres[i].center - spheres[j].center).norm();
        if (center_dist + spheres[i].radius <= spheres[j].radius + margin) {
          keep[i] = false;
          break;
        }
      }
    }

    std::vector<collision::Sphere> filtered;
    filtered.reserve(spheres.size());
    for (std::size_t i = 0; i < spheres.size(); ++i) {
      if (keep[i]) {
        filtered.push_back(spheres[i]);
      }
    }
    return filtered;
  }
};

} // namespace simulation
