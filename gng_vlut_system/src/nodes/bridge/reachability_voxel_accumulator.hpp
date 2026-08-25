#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "safety_engine/indexing/voxel_id_codec.hpp"

namespace robot_sim::bridge
{

struct reachability_bounds
{
  bool enable_filter{false};
  Eigen::Vector3d min_corner{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max_corner{Eigen::Vector3d::Zero()};
  Eigen::Vector3d margin{Eigen::Vector3d::Zero()};

  void validate() const
  {
    if (!enable_filter) {
      return;
    }
    if (!min_corner.allFinite() || !max_corner.allFinite() || !margin.allFinite() ||
      (margin.array() < 0.0).any() ||
      (min_corner.array() > max_corner.array()).any())
    {
      throw std::invalid_argument("reachability bounds are invalid");
    }
  }

  bool contains(const Eigen::Vector3d &point) const
  {
    return !enable_filter ||
      ((point.array() >= (min_corner - margin).array()).all() &&
      (point.array() <= (max_corner + margin).array()).all());
  }
};

struct reachability_voxelization_stats
{
  std::size_t input_point_count{0};
  std::size_t accepted_point_count{0};
  std::size_t nonfinite_point_count{0};
  std::size_t outside_point_count{0};
};

// 点群全体の中間コピーを持たない逐次ボクセル集約器
class reachability_voxel_accumulator
{
public:
  reachability_voxel_accumulator(
    const robot_sim::analysis::VoxelIdCodec &codec,
    reachability_bounds bounds,
    std::size_t max_dense_voxel_num)
  : codec_(codec), bounds_(std::move(bounds))
  {
    bounds_.validate();
    configure_dense_bitmap(max_dense_voxel_num);
  }

  void begin_frame(std::size_t input_point_count)
  {
    clear_frame_storage();
    stats_ = {};
    if (enable_dense_bitmap_) {
      voxel_ids_.reserve(std::min<std::size_t>(input_point_count, 16384U));
    } else {
      sparse_voxel_ids_.reserve(input_point_count);
    }
  }

  void add_point(
    const Eigen::Vector3d &source_point,
    const Eigen::Isometry3d &source_to_target)
  {
    ++stats_.input_point_count;
    if (!source_point.allFinite()) {
      ++stats_.nonfinite_point_count;
      return;
    }

    const Eigen::Vector3d target_point = source_to_target * source_point;
    if (!target_point.allFinite()) {
      ++stats_.nonfinite_point_count;
      return;
    }
    if (!bounds_.contains(target_point)) {
      ++stats_.outside_point_count;
      return;
    }

    const float voxel_size = static_cast<float>(codec_.voxelSize());
    const Eigen::Vector3i voxel_idx =
      ::common::geometry::VoxelUtils::worldToVoxel(
      target_point.cast<float>(), voxel_size);
    const long flat_voxel_id = codec_.toFlatId(voxel_idx);
    if (enable_dense_bitmap_) {
      const Eigen::Vector3i local_idx = voxel_idx - min_dense_idx_;
      const std::size_t local_flat_idx =
        static_cast<std::size_t>(local_idx.x()) +
        static_cast<std::size_t>(dense_dims_.x()) *
        (static_cast<std::size_t>(local_idx.y()) +
        static_cast<std::size_t>(dense_dims_.y()) * static_cast<std::size_t>(local_idx.z()));
      if (dense_occupancy_[local_flat_idx] == 0U) {
        dense_occupancy_[local_flat_idx] = 1U;
        touched_dense_indices_.push_back(local_flat_idx);
        voxel_ids_.push_back(flat_voxel_id);
      }
    } else {
      sparse_voxel_ids_.insert(flat_voxel_id);
    }
    ++stats_.accepted_point_count;
  }

  const std::vector<long> &finish_voxel_ids()
  {
    if (!enable_dense_bitmap_) {
      voxel_ids_.assign(sparse_voxel_ids_.begin(), sparse_voxel_ids_.end());
    }
    std::sort(voxel_ids_.begin(), voxel_ids_.end());
    return voxel_ids_;
  }

  const reachability_voxelization_stats &stats() const
  {
    return stats_;
  }

  bool uses_dense_bitmap() const
  {
    return enable_dense_bitmap_;
  }

private:
  void configure_dense_bitmap(std::size_t max_dense_voxel_num)
  {
    if (!bounds_.enable_filter || codec_.voxelSize() <= 0.0) {
      return;
    }

    const float voxel_size = static_cast<float>(codec_.voxelSize());
    min_dense_idx_ = ::common::geometry::VoxelUtils::worldToVoxel(
      (bounds_.min_corner - bounds_.margin).cast<float>(), voxel_size);
    const Eigen::Vector3i max_dense_idx = ::common::geometry::VoxelUtils::worldToVoxel(
      (bounds_.max_corner + bounds_.margin).cast<float>(), voxel_size);
    dense_dims_ = max_dense_idx - min_dense_idx_ + Eigen::Vector3i::Ones();
    if ((dense_dims_.array() <= 0).any()) {
      return;
    }

    const std::uint64_t dense_voxel_num =
      static_cast<std::uint64_t>(dense_dims_.x()) *
      static_cast<std::uint64_t>(dense_dims_.y()) *
      static_cast<std::uint64_t>(dense_dims_.z());
    if (dense_voxel_num == 0U || dense_voxel_num > max_dense_voxel_num ||
      dense_voxel_num > std::numeric_limits<std::size_t>::max())
    {
      return;
    }

    dense_occupancy_.assign(static_cast<std::size_t>(dense_voxel_num), 0U);
    touched_dense_indices_.reserve(16384U);
    voxel_ids_.reserve(16384U);
    enable_dense_bitmap_ = true;
  }

  void clear_frame_storage()
  {
    for (const std::size_t local_flat_idx : touched_dense_indices_) {
      dense_occupancy_[local_flat_idx] = 0U;
    }
    touched_dense_indices_.clear();
    voxel_ids_.clear();
    sparse_voxel_ids_.clear();
  }

  const robot_sim::analysis::VoxelIdCodec &codec_;
  reachability_bounds bounds_;
  bool enable_dense_bitmap_{false};
  Eigen::Vector3i min_dense_idx_{Eigen::Vector3i::Zero()};
  Eigen::Vector3i dense_dims_{Eigen::Vector3i::Zero()};
  std::vector<std::uint8_t> dense_occupancy_;
  std::vector<std::size_t> touched_dense_indices_;
  std::unordered_set<long> sparse_voxel_ids_;
  std::vector<long> voxel_ids_;
  reachability_voxelization_stats stats_;
};

}  // robot_sim::bridge namespace終端
