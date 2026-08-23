#pragma once

#include <voxel_idx.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
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

struct GraspVoxelContactPair
{
  std::vector<Eigen::Vector3d> positive;
  std::vector<Eigen::Vector3d> negative;
};

struct GraspVoxelTemplate
{
  std::vector<Eigen::Vector3d> required_occupied;
  std::vector<Eigen::Vector3d> optional_not_sole_support;
  // Potential inner-finger contact locations for each sampled opening.  A
  // candidate must support both sides within one matching opening sample.
  std::vector<GraspVoxelContactPair> opposing_contact_pairs;
  // Boundary pairs orthogonal to finger closing.  Occupancy spanning both
  // sides of two such axes is a locally unbounded sheet/support, not a
  // detachable grasp target.
  std::vector<GraspVoxelContactPair> lateral_continuation_pairs;
  // Occupancy within this region belongs to the prospective grasped object,
  // so it must not be rejected even if a moving-gripper sweep overlaps it.
  std::vector<Eigen::Vector3d> collision_exempt;
  std::vector<Eigen::Vector3d> required_empty;
};

struct CompiledGraspVoxelContactPair
{
  std::vector<VoxelIndex> positive_offsets;
  std::vector<VoxelIndex> negative_offsets;
};

struct CompiledGraspOrientation
{
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  std::vector<VoxelIndex> required_offsets;
  std::vector<VoxelIndex> outside_undersize_offsets;
  std::vector<CompiledGraspVoxelContactPair> opposing_contact_pairs;
  std::vector<CompiledGraspVoxelContactPair> lateral_continuation_pairs;
  std::vector<VoxelIndex> forbidden_offsets;
};

struct CompiledGraspVoxelTemplate
{
  double voxel_size = 0.0;
  Eigen::Vector3d required_centroid = Eigen::Vector3d::Zero();
  bool has_undersize_region = false;
  bool has_opposing_contacts = false;
  std::vector<CompiledGraspOrientation> orientations;
};

struct GraspVoxelMatchConfig
{
  double minimum_required_occupancy_ratio = 0.1;
  std::size_t minimum_required_hits = 3;
  std::size_t minimum_outside_undersize_hits = 1;
  std::size_t minimum_contact_hits_per_side = 1;
  std::size_t maximum_lateral_continuation_axes = 1;
  std::size_t maximum_forbidden_hits = 0;
  std::size_t maximum_anchor_voxels = 500;
  std::size_t maximum_candidates = 50;
};

struct GraspVoxelContactPairHits
{
  std::size_t positive = 0;
  std::size_t negative = 0;
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
  std::size_t positive_contact_hits = 0;
  std::size_t negative_contact_hits = 0;
  std::size_t selected_contact_pair = std::numeric_limits<std::size_t>::max();
  std::vector<GraspVoxelContactPairHits> opposing_contact_pair_hits;
  std::vector<GraspVoxelContactPairHits> lateral_continuation_pair_hits;
  std::size_t lateral_continuation_axes = 0;
  std::size_t forbidden_hits = 0;
  double required_occupancy_ratio = 0.0;
  double score = 0.0;
};

struct GraspVoxelMatchResult
{
  std::vector<GraspVoxelMatchCandidate> candidates;
  std::size_t anchors_considered = 0;
  std::size_t poses_evaluated = 0;
  std::size_t candidate_states_tracked = 0;
  std::size_t candidate_states_updated = 0;
  std::size_t rejected_required_occupancy = 0;
  std::size_t rejected_undersize_only = 0;
  std::size_t rejected_missing_opposing_contact = 0;
  std::size_t rejected_lateral_continuation = 0;
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
    validatePoints(grasp_template.collision_exempt);
    validatePoints(grasp_template.required_empty);
    for (const auto &contact_pair : grasp_template.opposing_contact_pairs) {
      validatePoints(contact_pair.positive);
      validatePoints(contact_pair.negative);
      if (contact_pair.positive.empty() || contact_pair.negative.empty()) {
        throw std::invalid_argument(
                "each opposing grasp contact pair requires both non-empty sides");
      }
    }
    for (const auto &continuation_pair : grasp_template.lateral_continuation_pairs) {
      validatePoints(continuation_pair.positive);
      validatePoints(continuation_pair.negative);
      if (continuation_pair.positive.empty() || continuation_pair.negative.empty()) {
        throw std::invalid_argument(
                "each lateral continuation pair requires both non-empty sides");
      }
    }
    compiled.has_opposing_contacts = !grasp_template.opposing_contact_pairs.empty();

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
      compiled_orientation.opposing_contact_pairs.reserve(
        grasp_template.opposing_contact_pairs.size());
      for (const auto &contact_pair : grasp_template.opposing_contact_pairs) {
        CompiledGraspVoxelContactPair compiled_pair;
        compiled_pair.positive_offsets = quantizeOffsets(
          contact_pair.positive, compiled.required_centroid, orientation, voxel_size);
        compiled_pair.negative_offsets = quantizeOffsets(
          contact_pair.negative, compiled.required_centroid, orientation, voxel_size);
        compiled_orientation.opposing_contact_pairs.push_back(std::move(compiled_pair));
      }
      compiled_orientation.lateral_continuation_pairs.reserve(
        grasp_template.lateral_continuation_pairs.size());
      for (const auto &continuation_pair : grasp_template.lateral_continuation_pairs) {
        CompiledGraspVoxelContactPair compiled_pair;
        compiled_pair.positive_offsets = quantizeOffsets(
          continuation_pair.positive, compiled.required_centroid, orientation, voxel_size);
        compiled_pair.negative_offsets = quantizeOffsets(
          continuation_pair.negative, compiled.required_centroid, orientation, voxel_size);
        compiled_orientation.lateral_continuation_pairs.push_back(std::move(compiled_pair));
      }
      const auto collision_exempt_offsets = quantizeOffsets(
        grasp_template.collision_exempt, compiled.required_centroid,
        orientation, voxel_size);
      const auto forbidden_offsets = quantizeOffsets(
        grasp_template.required_empty, compiled.required_centroid,
        orientation, voxel_size);

      std::unordered_set<VoxelIndex, VoxelIndexHash> undersize_set(
        undersize_offsets.begin(), undersize_offsets.end());
      for (const auto &offset : compiled_orientation.required_offsets) {
        if (undersize_set.find(offset) == undersize_set.end()) {
          compiled_orientation.outside_undersize_offsets.push_back(offset);
        }
      }
      const std::unordered_set<VoxelIndex, VoxelIndexHash> collision_exempt_set(
        collision_exempt_offsets.begin(), collision_exempt_offsets.end());
      compiled_orientation.forbidden_offsets.reserve(forbidden_offsets.size());
      for (const auto &offset : forbidden_offsets) {
        if (collision_exempt_set.find(offset) == collision_exempt_set.end()) {
          compiled_orientation.forbidden_offsets.push_back(offset);
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

        if (grasp_template.has_opposing_contacts) {
          candidate.opposing_contact_pair_hits.reserve(
            orientation.opposing_contact_pairs.size());
          std::size_t best_pair_support = 0;
          for (std::size_t pair_index = 0;
            pair_index < orientation.opposing_contact_pairs.size(); ++pair_index)
          {
            const auto &contact_pair = orientation.opposing_contact_pairs[pair_index];
            GraspVoxelContactPairHits hits;
            hits.positive = countOccupied(
              target_occupancy, anchor, contact_pair.positive_offsets);
            hits.negative = countOccupied(
              target_occupancy, anchor, contact_pair.negative_offsets);
            candidate.opposing_contact_pair_hits.push_back(hits);
            const std::size_t pair_support = std::min(hits.positive, hits.negative);
            if (hits.positive >= config.minimum_contact_hits_per_side &&
              hits.negative >= config.minimum_contact_hits_per_side &&
              pair_support > best_pair_support)
            {
              best_pair_support = pair_support;
              candidate.selected_contact_pair = pair_index;
              candidate.positive_contact_hits = hits.positive;
              candidate.negative_contact_hits = hits.negative;
            }
          }
          if (candidate.selected_contact_pair == std::numeric_limits<std::size_t>::max()) {
            ++result.rejected_missing_opposing_contact;
            continue;
          }
        }

        candidate.lateral_continuation_pair_hits.reserve(
          orientation.lateral_continuation_pairs.size());
        for (const auto &continuation_pair : orientation.lateral_continuation_pairs) {
          GraspVoxelContactPairHits hits;
          hits.positive = countOccupied(
            target_occupancy, anchor, continuation_pair.positive_offsets);
          hits.negative = countOccupied(
            target_occupancy, anchor, continuation_pair.negative_offsets);
          candidate.lateral_continuation_pair_hits.push_back(hits);
          if (hits.positive > 0U && hits.negative > 0U) {
            ++candidate.lateral_continuation_axes;
          }
        }
        if (candidate.lateral_continuation_axes > config.maximum_lateral_continuation_axes) {
          ++result.rejected_lateral_continuation;
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
        double positive_contact_ratio = 0.0;
        double negative_contact_ratio = 0.0;
        if (candidate.selected_contact_pair != std::numeric_limits<std::size_t>::max()) {
          const auto &contact_pair = orientation.opposing_contact_pairs[
            candidate.selected_contact_pair];
          positive_contact_ratio = contact_pair.positive_offsets.empty() ? 0.0 :
            static_cast<double>(candidate.positive_contact_hits) /
            static_cast<double>(contact_pair.positive_offsets.size());
          negative_contact_ratio = contact_pair.negative_offsets.empty() ? 0.0 :
            static_cast<double>(candidate.negative_contact_hits) /
            static_cast<double>(contact_pair.negative_offsets.size());
        }
        candidate.score = 0.7 * candidate.required_occupancy_ratio + 0.2 * outside_ratio +
          0.1 * std::min(positive_contact_ratio, negative_contact_ratio);
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

class IncrementalGraspVoxelMatcher
{
public:
  IncrementalGraspVoxelMatcher(
    VoxelGridGeometry geometry,
    CompiledGraspVoxelTemplate grasp_template,
    GraspVoxelMatchConfig config = {})
  : geometry_(std::move(geometry)),
    grasp_template_(std::move(grasp_template)),
    config_(std::move(config))
  {
    if (!geometry_.isValid() ||
      std::abs(geometry_.voxel_size - grasp_template_.voxel_size) > 1e-9)
    {
      throw std::invalid_argument("incremental matcher geometry does not match grasp template");
    }
    if (config_.maximum_anchor_voxels == 0 || config_.maximum_candidates == 0) {
      throw std::invalid_argument("matcher anchor and candidate limits must be positive");
    }
    if (!std::isfinite(config_.minimum_required_occupancy_ratio) ||
      config_.minimum_required_occupancy_ratio < 0.0 ||
      config_.minimum_required_occupancy_ratio > 1.0)
    {
      throw std::invalid_argument("minimum required occupancy ratio must be in [0, 1]");
    }
  }

  void reset(
    const OccupiedVoxelGrid &target_occupancy,
    const OccupiedVoxelGrid &collision_occupancy)
  {
    if (!geometry_.matches(target_occupancy.geometry()) ||
      !geometry_.matches(collision_occupancy.geometry()))
    {
      throw std::invalid_argument("incremental matcher grids do not match configured geometry");
    }
    target_cells_.clear();
    collision_cells_.clear();
    target_cells_.insert(target_occupancy.cells().begin(), target_occupancy.cells().end());
    collision_cells_.insert(collision_occupancy.cells().begin(), collision_occupancy.cells().end());
    anchor_cells_.clear();
    candidate_states_.clear();
    updated_candidate_states_ = 0;

    if (target_occupancy.cells().empty()) {
      return;
    }

    const std::size_t anchor_limit = std::min(
      target_occupancy.size(), config_.maximum_anchor_voxels);
    const std::size_t anchor_stride = std::max<std::size_t>(
      1, (target_occupancy.size() + anchor_limit - 1) / anchor_limit);
    for (std::size_t index = 0;
      index < target_occupancy.size() && anchor_cells_.size() < anchor_limit;
      index += anchor_stride)
    {
      addAnchor(target_occupancy.cells()[index]);
    }
  }

  bool applyTargetDelta(
    const VoxelIndex &cell,
    bool old_occupied,
    bool new_occupied)
  {
    const bool was_occupied = target_cells_.find(cell) != target_cells_.end();
    if (was_occupied != old_occupied) {
      throw std::invalid_argument("target voxel delta does not match incremental state");
    }
    if (old_occupied == new_occupied) {
      return false;
    }
    if (new_occupied) {
      target_cells_.insert(cell);
    } else {
      target_cells_.erase(cell);
    }
    const int count_delta = new_occupied ? 1 : -1;
    updateTargetCounts(cell, count_delta);
    if (!new_occupied) {
      removeAnchor(cell);
    } else {
      addAnchor(cell);
    }
    return true;
  }

  bool applyCollisionDelta(
    const VoxelIndex &cell,
    bool old_occupied,
    bool new_occupied)
  {
    const bool was_occupied = collision_cells_.find(cell) != collision_cells_.end();
    if (was_occupied != old_occupied) {
      throw std::invalid_argument("collision voxel delta does not match incremental state");
    }
    if (old_occupied == new_occupied) {
      return false;
    }
    if (new_occupied) {
      collision_cells_.insert(cell);
    } else {
      collision_cells_.erase(cell);
    }
    const int count_delta = new_occupied ? 1 : -1;
    for (std::size_t orientation_index = 0;
      orientation_index < grasp_template_.orientations.size(); ++orientation_index)
    {
      updateCounts(
        cell, orientation_index,
        grasp_template_.orientations[orientation_index].forbidden_offsets,
        count_delta, &GraspVoxelMatchCandidate::forbidden_hits);
    }
    return true;
  }

  GraspVoxelMatchResult match()
  {
    GraspVoxelMatchResult result;
    result.anchors_considered = anchor_cells_.size();
    result.poses_evaluated = candidate_states_.size();
    result.candidate_states_tracked = candidate_states_.size();
    result.candidate_states_updated = updated_candidate_states_;
    for (const auto &[key, state] : candidate_states_) {
      (void)key;
      GraspVoxelMatchCandidate candidate = state;
      candidate.required_occupancy_ratio = candidate.required_samples == 0 ? 0.0 :
        static_cast<double>(candidate.required_hits) /
        static_cast<double>(candidate.required_samples);
      const std::size_t required_threshold = std::max(
        config_.minimum_required_hits,
        static_cast<std::size_t>(std::ceil(
          config_.minimum_required_occupancy_ratio *
          static_cast<double>(candidate.required_samples))));
      if (candidate.required_hits < required_threshold) {
        ++result.rejected_required_occupancy;
        continue;
      }
      if (grasp_template_.has_undersize_region &&
        candidate.outside_undersize_hits < config_.minimum_outside_undersize_hits)
      {
        ++result.rejected_undersize_only;
        continue;
      }
      const auto &orientation = grasp_template_.orientations[candidate.orientation_index];
      candidate.selected_contact_pair = std::numeric_limits<std::size_t>::max();
      candidate.positive_contact_hits = 0;
      candidate.negative_contact_hits = 0;
      if (grasp_template_.has_opposing_contacts) {
        std::size_t best_pair_support = 0;
        for (std::size_t pair_index = 0;
          pair_index < candidate.opposing_contact_pair_hits.size(); ++pair_index)
        {
          const auto &hits = candidate.opposing_contact_pair_hits[pair_index];
          const std::size_t pair_support = std::min(hits.positive, hits.negative);
          if (hits.positive >= config_.minimum_contact_hits_per_side &&
            hits.negative >= config_.minimum_contact_hits_per_side &&
            pair_support > best_pair_support)
          {
            best_pair_support = pair_support;
            candidate.selected_contact_pair = pair_index;
            candidate.positive_contact_hits = hits.positive;
            candidate.negative_contact_hits = hits.negative;
          }
        }
        if (candidate.selected_contact_pair == std::numeric_limits<std::size_t>::max()) {
          ++result.rejected_missing_opposing_contact;
          continue;
        }
      }
      candidate.lateral_continuation_axes = 0;
      for (const auto &hits : candidate.lateral_continuation_pair_hits) {
        if (hits.positive > 0U && hits.negative > 0U) {
          ++candidate.lateral_continuation_axes;
        }
      }
      if (candidate.lateral_continuation_axes > config_.maximum_lateral_continuation_axes) {
        ++result.rejected_lateral_continuation;
        continue;
      }
      if (candidate.forbidden_hits > config_.maximum_forbidden_hits) {
        ++result.rejected_forbidden_occupancy;
        continue;
      }
      const double outside_ratio = orientation.outside_undersize_offsets.empty() ? 0.0 :
        static_cast<double>(candidate.outside_undersize_hits) /
        static_cast<double>(orientation.outside_undersize_offsets.size());
      double positive_contact_ratio = 0.0;
      double negative_contact_ratio = 0.0;
      if (candidate.selected_contact_pair != std::numeric_limits<std::size_t>::max()) {
        const auto &contact_pair = orientation.opposing_contact_pairs[
          candidate.selected_contact_pair];
        positive_contact_ratio = contact_pair.positive_offsets.empty() ? 0.0 :
          static_cast<double>(candidate.positive_contact_hits) /
          static_cast<double>(contact_pair.positive_offsets.size());
        negative_contact_ratio = contact_pair.negative_offsets.empty() ? 0.0 :
          static_cast<double>(candidate.negative_contact_hits) /
          static_cast<double>(contact_pair.negative_offsets.size());
      }
      candidate.score = 0.7 * candidate.required_occupancy_ratio + 0.2 * outside_ratio +
        0.1 * std::min(positive_contact_ratio, negative_contact_ratio);
      result.candidates.push_back(std::move(candidate));
    }
    sortAndLimit(result);
    updated_candidate_states_ = 0;
    return result;
  }

  std::size_t targetVoxelCount() const noexcept {return target_cells_.size();}
  std::size_t collisionVoxelCount() const noexcept {return collision_cells_.size();}

private:
  struct CandidateKey
  {
    VoxelIndex anchor;
    std::size_t orientation_index = 0;

    bool operator==(const CandidateKey &other) const noexcept
    {
      return anchor == other.anchor && orientation_index == other.orientation_index;
    }
  };

  struct CandidateKeyHash
  {
    std::size_t operator()(const CandidateKey &key) const noexcept
    {
      std::size_t seed = VoxelIndexHash{}(key.anchor);
      seed ^= std::hash<std::size_t>{}(key.orientation_index) +
        0x9e3779b9U + (seed << 6U) + (seed >> 2U);
      return seed;
    }
  };

  static VoxelIndex add(const VoxelIndex &lhs, const VoxelIndex &rhs) noexcept
  {
    return VoxelIndex{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
  }

  static VoxelIndex subtract(const VoxelIndex &lhs, const VoxelIndex &rhs) noexcept
  {
    return VoxelIndex{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
  }

  static void adjust(std::size_t &count, int delta)
  {
    if (delta > 0) {
      ++count;
      return;
    }
    if (count == 0) {
      throw std::logic_error("incremental candidate count underflow");
    }
    --count;
  }

  std::size_t countOccupied(
    const std::unordered_set<VoxelIndex, VoxelIndexHash> &cells,
    const VoxelIndex &anchor,
    const std::vector<VoxelIndex> &offsets) const
  {
    std::size_t hits = 0;
    for (const auto &offset : offsets) {
      if (cells.find(add(anchor, offset)) != cells.end()) {
        ++hits;
      }
    }
    return hits;
  }

  void addAnchor(const VoxelIndex &anchor)
  {
    if (target_cells_.find(anchor) == target_cells_.end() ||
      anchor_cells_.find(anchor) != anchor_cells_.end() ||
      anchor_cells_.size() >= config_.maximum_anchor_voxels)
    {
      return;
    }
    anchor_cells_.insert(anchor);
    for (std::size_t orientation_index = 0;
      orientation_index < grasp_template_.orientations.size(); ++orientation_index)
    {
      const auto &orientation = grasp_template_.orientations[orientation_index];
      GraspVoxelMatchCandidate candidate;
      candidate.anchor = anchor;
      candidate.orientation_index = orientation_index;
      candidate.required_samples = orientation.required_offsets.size();
      candidate.tcp_orientation = orientation.orientation;
      candidate.tcp_position = geometry_.origin + geometry_.voxel_size * Eigen::Vector3d(
        static_cast<double>(anchor.x) + 0.5,
        static_cast<double>(anchor.y) + 0.5,
        static_cast<double>(anchor.z) + 0.5) -
        orientation.orientation * grasp_template_.required_centroid;
      candidate.required_hits = countOccupied(
        target_cells_, anchor, orientation.required_offsets);
      candidate.outside_undersize_hits = countOccupied(
        target_cells_, anchor, orientation.outside_undersize_offsets);
      candidate.opposing_contact_pair_hits.reserve(
        orientation.opposing_contact_pairs.size());
      for (const auto &contact_pair : orientation.opposing_contact_pairs) {
        candidate.opposing_contact_pair_hits.push_back({
          countOccupied(target_cells_, anchor, contact_pair.positive_offsets),
          countOccupied(target_cells_, anchor, contact_pair.negative_offsets)});
      }
      candidate.lateral_continuation_pair_hits.reserve(
        orientation.lateral_continuation_pairs.size());
      for (const auto &continuation_pair : orientation.lateral_continuation_pairs) {
        candidate.lateral_continuation_pair_hits.push_back({
          countOccupied(target_cells_, anchor, continuation_pair.positive_offsets),
          countOccupied(target_cells_, anchor, continuation_pair.negative_offsets)});
      }
      candidate.forbidden_hits = countOccupied(
        collision_cells_, anchor, orientation.forbidden_offsets);
      candidate_states_.emplace(
        CandidateKey{anchor, orientation_index}, std::move(candidate));
    }
  }

  void removeAnchor(const VoxelIndex &anchor)
  {
    if (anchor_cells_.erase(anchor) == 0) {
      return;
    }
    for (std::size_t orientation_index = 0;
      orientation_index < grasp_template_.orientations.size(); ++orientation_index)
    {
      candidate_states_.erase(CandidateKey{anchor, orientation_index});
    }
  }

  void updateTargetCounts(const VoxelIndex &cell, int delta)
  {
    for (std::size_t orientation_index = 0;
      orientation_index < grasp_template_.orientations.size(); ++orientation_index)
    {
      const auto &orientation = grasp_template_.orientations[orientation_index];
      updateCounts(
        cell, orientation_index, orientation.required_offsets,
        delta, &GraspVoxelMatchCandidate::required_hits);
      updateCounts(
        cell, orientation_index, orientation.outside_undersize_offsets,
        delta, &GraspVoxelMatchCandidate::outside_undersize_hits);
      for (std::size_t pair_index = 0;
        pair_index < orientation.opposing_contact_pairs.size(); ++pair_index)
      {
        const auto &contact_pair = orientation.opposing_contact_pairs[pair_index];
        updateContactPairCounts(
          cell, orientation_index, pair_index, contact_pair.positive_offsets, delta, true);
        updateContactPairCounts(
          cell, orientation_index, pair_index, contact_pair.negative_offsets, delta, false);
      }
      for (std::size_t pair_index = 0;
        pair_index < orientation.lateral_continuation_pairs.size(); ++pair_index)
      {
        const auto &continuation_pair = orientation.lateral_continuation_pairs[pair_index];
        updateLateralContinuationCounts(
          cell, orientation_index, pair_index, continuation_pair.positive_offsets, delta, true);
        updateLateralContinuationCounts(
          cell, orientation_index, pair_index, continuation_pair.negative_offsets, delta, false);
      }
    }
  }

  void updateCounts(
    const VoxelIndex &cell,
    std::size_t orientation_index,
    const std::vector<VoxelIndex> &offsets,
    int delta,
    std::size_t GraspVoxelMatchCandidate::*member)
  {
    for (const auto &offset : offsets) {
      const CandidateKey key{subtract(cell, offset), orientation_index};
      const auto state = candidate_states_.find(key);
      if (state == candidate_states_.end()) {
        continue;
      }
      adjust(state->second.*member, delta);
      ++updated_candidate_states_;
    }
  }

  void updateContactPairCounts(
    const VoxelIndex &cell,
    std::size_t orientation_index,
    std::size_t pair_index,
    const std::vector<VoxelIndex> &offsets,
    int delta,
    bool positive)
  {
    for (const auto &offset : offsets) {
      const CandidateKey key{subtract(cell, offset), orientation_index};
      const auto state = candidate_states_.find(key);
      if (state == candidate_states_.end() ||
        pair_index >= state->second.opposing_contact_pair_hits.size())
      {
        continue;
      }
      auto &hits = state->second.opposing_contact_pair_hits[pair_index];
      adjust(positive ? hits.positive : hits.negative, delta);
      ++updated_candidate_states_;
    }
  }

  void updateLateralContinuationCounts(
    const VoxelIndex &cell,
    std::size_t orientation_index,
    std::size_t pair_index,
    const std::vector<VoxelIndex> &offsets,
    int delta,
    bool positive)
  {
    for (const auto &offset : offsets) {
      const CandidateKey key{subtract(cell, offset), orientation_index};
      const auto state = candidate_states_.find(key);
      if (state == candidate_states_.end() ||
        pair_index >= state->second.lateral_continuation_pair_hits.size())
      {
        continue;
      }
      auto &hits = state->second.lateral_continuation_pair_hits[pair_index];
      adjust(positive ? hits.positive : hits.negative, delta);
      ++updated_candidate_states_;
    }
  }

  void sortAndLimit(GraspVoxelMatchResult &result) const
  {
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
    if (result.candidates.size() > config_.maximum_candidates) {
      result.candidates.resize(config_.maximum_candidates);
    }
  }

  VoxelGridGeometry geometry_;
  CompiledGraspVoxelTemplate grasp_template_;
  GraspVoxelMatchConfig config_;
  std::unordered_set<VoxelIndex, VoxelIndexHash> target_cells_;
  std::unordered_set<VoxelIndex, VoxelIndexHash> collision_cells_;
  std::unordered_set<VoxelIndex, VoxelIndexHash> anchor_cells_;
  std::unordered_map<CandidateKey, GraspVoxelMatchCandidate, CandidateKeyHash> candidate_states_;
  std::size_t updated_candidate_states_ = 0;
};

}  // namespace grasping_system::candidate
