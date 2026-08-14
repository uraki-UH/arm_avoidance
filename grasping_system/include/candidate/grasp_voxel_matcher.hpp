#pragma once

#include <voxel_idx.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace grasping_system::candidate
{

using VoxelIndex = voxel_idx::VoxelIndex;

struct VoxelIndexHash
{
  std::size_t operator()(const VoxelIndex &index) const noexcept
  {
    std::size_t seed = std::hash<int>{}(index.x);
    seed ^= std::hash<int>{}(index.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(index.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct VoxelGridGeometry
{
  double voxel_size = 0.0;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();

  bool isValid() const noexcept
  {
    return std::isfinite(voxel_size) && voxel_size > 0.0 && origin.allFinite();
  }

  bool matches(const VoxelGridGeometry &other, double tolerance = 1e-9) const noexcept
  {
    return std::abs(voxel_size - other.voxel_size) <= tolerance &&
           (origin - other.origin).cwiseAbs().maxCoeff() <= tolerance;
  }
};

class OccupiedVoxelGrid
{
public:
  explicit OccupiedVoxelGrid(VoxelGridGeometry geometry)
  : geometry_(std::move(geometry))
  {
    if (!geometry_.isValid()) {
      throw std::invalid_argument("occupied voxel grid geometry is invalid");
    }
  }

  bool add(const VoxelIndex &index)
  {
    if (!occupied_.insert(index).second) {
      return false;
    }
    cells_.push_back(index);
    return true;
  }

  bool contains(const VoxelIndex &index) const noexcept
  {
    return occupied_.find(index) != occupied_.end();
  }

  Eigen::Vector3d cellCenter(const VoxelIndex &index) const noexcept
  {
    return geometry_.origin + geometry_.voxel_size * Eigen::Vector3d(
      static_cast<double>(index.x) + 0.5,
      static_cast<double>(index.y) + 0.5,
      static_cast<double>(index.z) + 0.5);
  }

  const VoxelGridGeometry &geometry() const noexcept {return geometry_;}
  const std::vector<VoxelIndex> &cells() const noexcept {return cells_;}
  std::size_t size() const noexcept {return cells_.size();}

private:
  VoxelGridGeometry geometry_;
  std::vector<VoxelIndex> cells_;
  std::unordered_set<VoxelIndex, VoxelIndexHash> occupied_;
};

struct GraspVoxelTemplate
{
  std::vector<Eigen::Vector3d> required_occupied;
  std::vector<Eigen::Vector3d> optional_not_sole_support;
  std::vector<Eigen::Vector3d> required_empty;
};

struct CompiledGraspOrientation
{
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  std::vector<VoxelIndex> required_offsets;
  std::vector<VoxelIndex> outside_undersize_offsets;
  std::vector<VoxelIndex> forbidden_offsets;
};

struct CompiledGraspVoxelTemplate
{
  double voxel_size = 0.0;
  Eigen::Vector3d required_centroid = Eigen::Vector3d::Zero();
  bool has_undersize_region = false;
  std::vector<CompiledGraspOrientation> orientations;
};

struct GraspVoxelMatchConfig
{
  double minimum_required_occupancy_ratio = 0.1;
  std::size_t minimum_required_hits = 3;
  std::size_t minimum_outside_undersize_hits = 1;
  std::size_t maximum_forbidden_hits = 0;
  std::size_t maximum_anchor_voxels = 500;
  std::size_t maximum_candidates = 50;
};

struct GraspVoxelMatchCandidate
{
  Eigen::Vector3d tcp_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond tcp_orientation = Eigen::Quaterniond::Identity();
  VoxelIndex anchor;
  std::size_t orientation_index = 0;
  std::size_t required_hits = 0;
  std::size_t required_samples = 0;
  std::size_t outside_undersize_hits = 0;
  std::size_t forbidden_hits = 0;
  double required_occupancy_ratio = 0.0;
  double score = 0.0;
};

struct GraspVoxelMatchResult
{
  std::vector<GraspVoxelMatchCandidate> candidates;
  std::size_t anchors_considered = 0;
  std::size_t poses_evaluated = 0;
  std::size_t rejected_required_occupancy = 0;
  std::size_t rejected_undersize_only = 0;
  std::size_t rejected_forbidden_occupancy = 0;
};

class GraspVoxelMatcher
{
public:
  static CompiledGraspVoxelTemplate compile(
    const GraspVoxelTemplate &grasp_template,
    const std::vector<Eigen::Quaterniond> &orientations,
    double voxel_size)
  {
    if (grasp_template.required_occupied.empty()) {
      throw std::invalid_argument("required occupied grasp template must not be empty");
    }
    if (orientations.empty()) {
      throw std::invalid_argument("at least one grasp orientation is required");
    }
    if (!std::isfinite(voxel_size) || voxel_size <= 0.0) {
      throw std::invalid_argument("grasp matcher voxel size must be positive");
    }

    CompiledGraspVoxelTemplate compiled;
    compiled.voxel_size = voxel_size;
    compiled.has_undersize_region = !grasp_template.optional_not_sole_support.empty();
    for (const auto &point : grasp_template.required_occupied) {
      if (!point.allFinite()) {
        throw std::invalid_argument("grasp template contains a non-finite point");
      }
      compiled.required_centroid += point;
    }
    compiled.required_centroid /= static_cast<double>(grasp_template.required_occupied.size());
    validatePoints(grasp_template.optional_not_sole_support);
    validatePoints(grasp_template.required_empty);

    compiled.orientations.reserve(orientations.size());
    for (auto orientation : orientations) {
      if (!orientation.coeffs().allFinite() || orientation.squaredNorm() <= 1e-12) {
        throw std::invalid_argument("grasp orientation must be finite and non-zero");
      }
      orientation.normalize();

      CompiledGraspOrientation compiled_orientation;
      compiled_orientation.orientation = orientation;
      compiled_orientation.required_offsets = quantizeOffsets(
        grasp_template.required_occupied, compiled.required_centroid,
        orientation, voxel_size);
      const auto undersize_offsets = quantizeOffsets(
        grasp_template.optional_not_sole_support, compiled.required_centroid,
        orientation, voxel_size);
      compiled_orientation.forbidden_offsets = quantizeOffsets(
        grasp_template.required_empty, compiled.required_centroid,
        orientation, voxel_size);

      std::unordered_set<VoxelIndex, VoxelIndexHash> undersize_set(
        undersize_offsets.begin(), undersize_offsets.end());
      for (const auto &offset : compiled_orientation.required_offsets) {
        if (undersize_set.find(offset) == undersize_set.end()) {
          compiled_orientation.outside_undersize_offsets.push_back(offset);
        }
      }
      compiled.orientations.push_back(std::move(compiled_orientation));
    }
    return compiled;
  }

  static GraspVoxelMatchResult match(
    const OccupiedVoxelGrid &target_occupancy,
    const OccupiedVoxelGrid &collision_occupancy,
    const CompiledGraspVoxelTemplate &grasp_template,
    const GraspVoxelMatchConfig &config = {})
  {
    validateConfig(config);
    if (!target_occupancy.geometry().matches(collision_occupancy.geometry()) ||
      std::abs(target_occupancy.geometry().voxel_size - grasp_template.voxel_size) > 1e-9)
    {
      throw std::invalid_argument(
              "target, collision, and compiled grasp grids must use the same geometry");
    }

    GraspVoxelMatchResult result;
    if (target_occupancy.cells().empty() || grasp_template.orientations.empty()) {
      return result;
    }

    const std::size_t anchor_limit = std::min(
      target_occupancy.size(), config.maximum_anchor_voxels);
    const std::size_t anchor_stride = std::max<std::size_t>(
      1, (target_occupancy.size() + anchor_limit - 1) / anchor_limit);
    for (std::size_t anchor_index = 0;
      anchor_index < target_occupancy.size() && result.anchors_considered < anchor_limit;
      anchor_index += anchor_stride)
    {
      const auto &anchor = target_occupancy.cells()[anchor_index];
      ++result.anchors_considered;
      for (std::size_t orientation_index = 0;
        orientation_index < grasp_template.orientations.size(); ++orientation_index)
      {
        ++result.poses_evaluated;
        const auto &orientation = grasp_template.orientations[orientation_index];
        GraspVoxelMatchCandidate candidate;
        candidate.anchor = anchor;
        candidate.orientation_index = orientation_index;
        candidate.required_samples = orientation.required_offsets.size();
        candidate.tcp_orientation = orientation.orientation;
        candidate.tcp_position = target_occupancy.cellCenter(anchor) -
          orientation.orientation * grasp_template.required_centroid;

        candidate.required_hits = countOccupied(
          target_occupancy, anchor, orientation.required_offsets);
        candidate.required_occupancy_ratio = candidate.required_samples == 0 ? 0.0 :
          static_cast<double>(candidate.required_hits) /
          static_cast<double>(candidate.required_samples);
        const std::size_t required_threshold = std::max(
          config.minimum_required_hits,
          static_cast<std::size_t>(std::ceil(
            config.minimum_required_occupancy_ratio *
            static_cast<double>(candidate.required_samples))));
        if (candidate.required_hits < required_threshold) {
          ++result.rejected_required_occupancy;
          continue;
        }

        candidate.outside_undersize_hits = countOccupied(
          target_occupancy, anchor, orientation.outside_undersize_offsets);
        if (grasp_template.has_undersize_region &&
          candidate.outside_undersize_hits < config.minimum_outside_undersize_hits)
        {
          ++result.rejected_undersize_only;
          continue;
        }

        candidate.forbidden_hits = countOccupied(
          collision_occupancy, anchor, orientation.forbidden_offsets);
        if (candidate.forbidden_hits > config.maximum_forbidden_hits) {
          ++result.rejected_forbidden_occupancy;
          continue;
        }

        const double outside_ratio = orientation.outside_undersize_offsets.empty() ? 0.0 :
          static_cast<double>(candidate.outside_undersize_hits) /
          static_cast<double>(orientation.outside_undersize_offsets.size());
        candidate.score = 0.8 * candidate.required_occupancy_ratio + 0.2 * outside_ratio;
        result.candidates.push_back(std::move(candidate));
      }
    }

    std::sort(
      result.candidates.begin(), result.candidates.end(),
      [](const GraspVoxelMatchCandidate &lhs, const GraspVoxelMatchCandidate &rhs) {
        if (lhs.score != rhs.score) {
          return lhs.score > rhs.score;
        }
        if (lhs.required_hits != rhs.required_hits) {
          return lhs.required_hits > rhs.required_hits;
        }
        if (lhs.anchor.x != rhs.anchor.x) {
          return lhs.anchor.x < rhs.anchor.x;
        }
        if (lhs.anchor.y != rhs.anchor.y) {
          return lhs.anchor.y < rhs.anchor.y;
        }
        if (lhs.anchor.z != rhs.anchor.z) {
          return lhs.anchor.z < rhs.anchor.z;
        }
        return lhs.orientation_index < rhs.orientation_index;
      });
    if (result.candidates.size() > config.maximum_candidates) {
      result.candidates.resize(config.maximum_candidates);
    }
    return result;
  }

private:
  static void validatePoints(const std::vector<Eigen::Vector3d> &points)
  {
    for (const auto &point : points) {
      if (!point.allFinite()) {
        throw std::invalid_argument("grasp template contains a non-finite point");
      }
    }
  }

  static void validateConfig(const GraspVoxelMatchConfig &config)
  {
    if (!std::isfinite(config.minimum_required_occupancy_ratio) ||
      config.minimum_required_occupancy_ratio < 0.0 ||
      config.minimum_required_occupancy_ratio > 1.0)
    {
      throw std::invalid_argument("minimum required occupancy ratio must be in [0, 1]");
    }
    if (config.maximum_anchor_voxels == 0 || config.maximum_candidates == 0) {
      throw std::invalid_argument("matcher anchor and candidate limits must be positive");
    }
  }

  static VoxelIndex add(const VoxelIndex &lhs, const VoxelIndex &rhs) noexcept
  {
    return VoxelIndex{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
  }

  static std::size_t countOccupied(
    const OccupiedVoxelGrid &grid,
    const VoxelIndex &anchor,
    const std::vector<VoxelIndex> &offsets)
  {
    std::size_t hits = 0;
    for (const auto &offset : offsets) {
      if (grid.contains(add(anchor, offset))) {
        ++hits;
      }
    }
    return hits;
  }

  static std::vector<VoxelIndex> quantizeOffsets(
    const std::vector<Eigen::Vector3d> &points,
    const Eigen::Vector3d &center,
    const Eigen::Quaterniond &orientation,
    double voxel_size)
  {
    std::unordered_set<VoxelIndex, VoxelIndexHash> unique;
    unique.reserve(points.size());
    for (const auto &point : points) {
      const Eigen::Vector3d offset = orientation * (point - center) / voxel_size;
      unique.insert(VoxelIndex{
        static_cast<int>(std::floor(offset.x() + 0.5)),
        static_cast<int>(std::floor(offset.y() + 0.5)),
        static_cast<int>(std::floor(offset.z() + 0.5))});
    }
    std::vector<VoxelIndex> result(unique.begin(), unique.end());
    std::sort(
      result.begin(), result.end(),
      [](const VoxelIndex &lhs, const VoxelIndex &rhs) {
        if (lhs.x != rhs.x) {
          return lhs.x < rhs.x;
        }
        if (lhs.y != rhs.y) {
          return lhs.y < rhs.y;
        }
        return lhs.z < rhs.z;
      });
    return result;
  }
};

}  // namespace grasping_system::candidate
