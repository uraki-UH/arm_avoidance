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
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

struct PoseAxisMarkerOptions
{
  std::string marker_namespace{"grasp_pose_axes"};
  double primary_axis_length{0.12};
  double helper_axis_length_ratio{0.5};
  double shaft_diameter{0.01};
  double head_diameter{0.02};
  double alpha{1.0};
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

    // stats.count は「この voxel に落ちた入力点の数」。
    // occupied_neighbors は「6近傍のうち埋まっている voxel の数」。
    // 旧ヒューリスティックは一旦停止する。
    // const double density_score = static_cast<double>(stats.count) / count_span;
    // const double boundary_score = 1.0 - static_cast<double>(occupied_neighbors) / 6.0;
    // const double height_score = (center.z() - min_z) / z_span;
    // const double radial_score = (center - centroid).norm() / radius_span;
    // const double score = std::clamp(
    //   0.45 * density_score +
    //   0.25 * boundary_score +
    //   0.20 * height_score +
    //   0.10 * radial_score,
    //   0.0, 1.0);
    const double score = 0.0;

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

inline visualization_msgs::msg::MarkerArray buildPoseAxisMarkerArray(
  const geometry_msgs::msg::PoseArray &pose_array,
  const PoseAxisMarkerOptions &options = {})
{
  visualization_msgs::msg::MarkerArray out;
  out.markers.reserve(pose_array.poses.size() * 3);

  const std::array<std::array<double, 4>, 3> axis_colors{{
    {{1.0, 0.2, 0.2, 1.0}},
    {{0.2, 0.8, 0.2, 1.0}},
    {{0.2, 0.4, 1.0, 1.0}},
  }};
  const std::array<Eigen::Vector3d, 3> local_axes{{
    Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitZ(),
  }};
  const std::array<const char *, 3> axis_names{{"x", "y", "z"}};

  for (std::size_t pose_index = 0; pose_index < pose_array.poses.size(); ++pose_index) {
    const auto &pose = pose_array.poses[pose_index];
    const Eigen::Quaterniond q(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
    const Eigen::Vector3d origin(
      pose.position.x,
      pose.position.y,
      pose.position.z);

    for (std::size_t axis_index = 0; axis_index < local_axes.size(); ++axis_index) {
      const Eigen::Vector3d direction = (q * local_axes[axis_index]).normalized();
      const double length = options.primary_axis_length *
        (axis_index == 0 ? 1.0 : options.helper_axis_length_ratio);
      Eigen::Vector3d tip;
      if (axis_index == 0) {
        tip = origin + direction * length;
      } else {
        tip = local_axes[axis_index] * length;
      }

      visualization_msgs::msg::Marker marker;
      marker.header = pose_array.header;
      marker.ns = options.marker_namespace + "/" + axis_names[axis_index];
      marker.id = static_cast<int>(pose_index * 3 + axis_index);
      marker.type = axis_index == 0
        ? visualization_msgs::msg::Marker::ARROW
        : visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.points.resize(2);
      marker.points[0].x = axis_index == 0 ? origin.x() : 0.0;
      marker.points[0].y = axis_index == 0 ? origin.y() : 0.0;
      marker.points[0].z = axis_index == 0 ? origin.z() : 0.0;
      marker.points[1].x = tip.x();
      marker.points[1].y = tip.y();
      marker.points[1].z = tip.z();
      if (axis_index == 0) {
        marker.scale.x = length;
        marker.scale.y = options.shaft_diameter * 1.5;
        marker.scale.z = options.head_diameter * 1.5;
      } else {
        marker.scale.x = options.shaft_diameter * 1.2;
        marker.scale.y = options.shaft_diameter * 1.2;
        marker.scale.z = options.shaft_diameter * 1.2;
      }
      marker.color.r = static_cast<float>(axis_colors[axis_index][0]);
      marker.color.g = static_cast<float>(axis_colors[axis_index][1]);
      marker.color.b = static_cast<float>(axis_colors[axis_index][2]);
      marker.color.a = static_cast<float>(options.alpha);
      marker.frame_locked = false;
      if (axis_index != 0) {
        marker.pose.position.x = origin.x();
        marker.pose.position.y = origin.y();
        marker.pose.position.z = origin.z();
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
      }
      out.markers.push_back(std::move(marker));
    }
  }

  return out;
}

}  // namespace robot_sim::common::grasp
