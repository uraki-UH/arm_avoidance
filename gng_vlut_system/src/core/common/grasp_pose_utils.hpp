#pragma once

#include "common/voxel_utils.hpp"
#include "safety_engine/indexing/voxel_id_codec.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace robot_sim::common::grasp
{

struct VoxelObservation
{
  std::size_t count{0};
  Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
};

struct Candidate
{
  long voxel_id{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  double score{0.0};
  std::size_t count{0};
  std::size_t occupied_neighbors{0};
};

inline Eigen::Vector3d safeNormalize(
  const Eigen::Vector3d &value, const Eigen::Vector3d &fallback)
{
  if (value.squaredNorm() > 1e-12) {
    return value.normalized();
  }
  if (fallback.squaredNorm() > 1e-12) {
    return fallback.normalized();
  }
  return Eigen::Vector3d::UnitX();
}

inline Eigen::Quaterniond makeApproachOrientation(const Eigen::Vector3d &approach_dir)
{
  const Eigen::Vector3d x_axis = safeNormalize(approach_dir, Eigen::Vector3d::UnitX());
  Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  if (std::abs(x_axis.dot(up)) > 0.95) {
    up = Eigen::Vector3d::UnitY();
  }

  Eigen::Vector3d y_axis = up.cross(x_axis);
  if (y_axis.squaredNorm() <= 1e-12) {
    y_axis = Eigen::Vector3d::UnitY();
  } else {
    y_axis.normalize();
  }

  Eigen::Vector3d z_axis = x_axis.cross(y_axis);
  if (z_axis.squaredNorm() <= 1e-12) {
    z_axis = Eigen::Vector3d::UnitZ();
  } else {
    z_axis.normalize();
  }

  Eigen::Matrix3d basis;
  basis.col(0) = x_axis;
  basis.col(1) = y_axis;
  basis.col(2) = z_axis;
  return Eigen::Quaterniond(basis);
}

inline std::vector<Candidate> buildSimpleCandidates(
  const std::unordered_map<long, VoxelObservation> &voxel_stats,
  const ::robot_sim::analysis::VoxelIdCodec &codec,
  std::size_t min_points_per_voxel,
  std::size_t max_candidates,
  double approach_offset)
{
  std::vector<Candidate> candidates;
  candidates.reserve(voxel_stats.size());

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  std::size_t total_points = 0;
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  double max_radius = 0.0;

  for (const auto &[vid, stats] : voxel_stats) {
    (void)vid;
    centroid += stats.sum;
    total_points += stats.count;
  }
  if (total_points > 0) {
    centroid /= static_cast<double>(total_points);
  }

  for (const auto &[vid, stats] : voxel_stats) {
    if (stats.count < min_points_per_voxel) {
      continue;
    }
    const Eigen::Vector3d center = ::common::geometry::VoxelUtils::voxelToWorld(
      codec.toIndex(vid), static_cast<float>(codec.voxelSize())).cast<double>();
    min_z = std::min(min_z, center.z());
    max_z = std::max(max_z, center.z());
    max_radius = std::max(max_radius, (center - centroid).norm());
  }

  if (!std::isfinite(min_z)) {
    min_z = 0.0;
    max_z = 0.0;
  }

  const double z_span = std::max(1e-6, max_z - min_z);
  const double radius_span = std::max(1e-6, max_radius);
  std::size_t max_count = 0;
  for (const auto &[vid, stats] : voxel_stats) {
    (void)vid;
    if (stats.count >= min_points_per_voxel) {
      max_count = std::max(max_count, stats.count);
    }
  }
  const double count_span = static_cast<double>(std::max<std::size_t>(1U, max_count));

  const std::array<Eigen::Vector3i, 6> neighbor_offsets{{
    {1, 0, 0},
    {-1, 0, 0},
    {0, 1, 0},
    {0, -1, 0},
    {0, 0, 1},
    {0, 0, -1},
  }};

  for (const auto &[vid, stats] : voxel_stats) {
    if (stats.count < min_points_per_voxel) {
      continue;
    }

    const Eigen::Vector3i index = codec.toIndex(vid);
    const Eigen::Vector3d center = ::common::geometry::VoxelUtils::voxelToWorld(
      index, static_cast<float>(codec.voxelSize())).cast<double>();
    const Eigen::Vector3d outward_dir = safeNormalize(center - centroid, Eigen::Vector3d::UnitX());
    const Eigen::Vector3d approach_dir = -outward_dir;
    const Eigen::Vector3d position = center + outward_dir * approach_offset;

    std::size_t occupied_neighbors = 0;
    for (const auto &offset : neighbor_offsets) {
      const long neighbor_vid = codec.toFlatId(index + offset);
      if (voxel_stats.find(neighbor_vid) != voxel_stats.end()) {
        ++occupied_neighbors;
      }
    }

    const double density_score = static_cast<double>(stats.count) / count_span;
    const double boundary_score = 1.0 - static_cast<double>(occupied_neighbors) / 6.0;
    const double height_score = (center.z() - min_z) / z_span;
    const double radial_score = (center - centroid).norm() / radius_span;
    const double score = std::clamp(
      0.45 * density_score +
      0.25 * boundary_score +
      0.20 * height_score +
      0.10 * radial_score,
      0.0, 1.0);

    candidates.push_back(Candidate{
      vid,
      position,
      makeApproachOrientation(approach_dir),
      score,
      stats.count,
      occupied_neighbors
    });
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
    if (a.score != b.score) {
      return a.score > b.score;
    }
    if (a.count != b.count) {
      return a.count > b.count;
    }
    return a.voxel_id < b.voxel_id;
  });

  if (candidates.size() > max_candidates) {
    candidates.resize(max_candidates);
  }

  return candidates;
}

}  // namespace robot_sim::common::grasp
