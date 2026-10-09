#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <point_cloud_store.hpp>

#include "safety_engine/indexing/voxel_id_codec.hpp"

namespace robot_sim::indexing
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
    std::size_t max_dense_voxel_num, bool enable_point_membership = false)
  : codec_(codec), bounds_(std::move(bounds)), enable_point_membership_(enable_point_membership)
  {
    bounds_.validate();
    configure_dense_bitmap(max_dense_voxel_num);
  }

  void begin_frame(std::size_t input_point_count)
  {
    clear_frame_storage();
    stats_ = {};
    if (enable_point_membership_) {
      // 読取中フレームの不変性と、解放済みバッファの容量再利用
      if (!roi_points_ || !roi_points_.unique()) {
        roi_points_.swap(spare_roi_points_);
        if (!roi_points_ || !roi_points_.unique()) {
          roi_points_ = std::make_shared<voxel_idx::roi_point_membership>();
        }
      }
      if (input_point_count > UINT32_MAX) {throw std::length_error("元点数の範囲外");}
      roi_points_->point_cells.assign(input_point_count, voxel_idx::roi_point_membership::no_cell);
      roi_points_->cells.clear();
      return;
    }
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

    add_target_point(source_to_target * source_point);
  }

  void add_point_in_target_frame(const Eigen::Vector3d &target_point)
  {
    ++stats_.input_point_count;
    add_target_point(target_point);
  }

  const std::vector<long> &finish_voxel_ids()
  {
    if (enable_point_membership_) {
      voxel_ids_.clear();
      std::sort(roi_points_->cells.begin(), roi_points_->cells.end(),
        [](const auto &left, const auto &right) {return left.id < right.id;});
      for (const auto &cell : roi_points_->cells) {voxel_ids_.push_back(cell.id);}
      return voxel_ids_;
    } else if (!enable_dense_bitmap_) {
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

  bool has_point_membership() const {return enable_point_membership_;}
  bool has_self_cells() const {return has_self_cells_;}

  // 自己姿勢更新時だけの既存ROI占有byteへの自己ラベル登録
  template<class CellIds>
  void set_self_cells(const CellIds &ids)
  {
    if (!enable_point_membership_) {throw std::logic_error("自己セル登録には共有ROI設定が必要");}
    for (const auto flat : self_dense_indices_) {dense_occupancy_[flat] &= ~std::uint8_t{2};}
    self_dense_indices_.clear();
    if (enable_dense_bitmap_) {
      for (const auto id : ids) {
        const Eigen::Vector3i local = codec_.toIndex(id) - min_dense_idx_;
        if ((local.array() < 0).any() || (local.array() >= dense_dims_.array()).any()) {continue;}
        const std::size_t flat = std::size_t(local.x()) + std::size_t(dense_dims_.x()) *
          (std::size_t(local.y()) + std::size_t(dense_dims_.y()) * std::size_t(local.z()));
        if ((dense_occupancy_[flat] & 2) == 0) {self_dense_indices_.push_back(flat);}
        dense_occupancy_[flat] |= 2;
      }
    }
    has_self_cells_ = true;
  }

  std::shared_ptr<const voxel_idx::roi_point_membership> point_membership() const
  {
    return roi_points_;
  }

  // ROI占有・元点対応・自己判定の同時登録。ROI外の自己領域のみ追加許可
  template<class CanIncludeOutside, class IsSelfCell>
  void add_shared_point(const Eigen::Vector3d &point, std::uint32_t source_idx,
    CanIncludeOutside can_include_outside, IsSelfCell is_self_cell)
  {
    if (!enable_point_membership_ || !roi_points_) {
      throw std::logic_error("共有ROI登録の初期化不足");
    }
    auto &point_cell = roi_points_->point_cells.at(source_idx);
    ++stats_.input_point_count;
    if (!point.allFinite()) {++stats_.nonfinite_point_count; return;}
    const bool is_roi = bounds_.contains(point);
    if (!is_roi && !can_include_outside()) {++stats_.outside_point_count; return;}
    const auto idx = ::common::geometry::VoxelUtils::worldToVoxel(
      point.cast<float>(), static_cast<float>(codec_.voxelSize()));
    const long id = codec_.toFlatId(idx);
    if (id < 0) {throw std::out_of_range("共有セルIDの符号ビット超過");}
    const Eigen::Vector3i local = idx - min_dense_idx_;
    const bool is_dense = enable_dense_bitmap_ && (is_roi ||
      ((local.array() >= 0).all() && (local.array() < dense_dims_.array()).all()));
    std::uint8_t *state = nullptr;
    if (is_dense) {
      const std::size_t flat = std::size_t(local.x()) + std::size_t(dense_dims_.x()) *
        (std::size_t(local.y()) + std::size_t(dense_dims_.y()) * std::size_t(local.z()));
      state = &dense_occupancy_[flat];
      if ((*state & 1) == 0) {touched_dense_indices_.push_back(flat);}
    } else {
      state = &sparse_cell_states_[id];
    }
    // 既存の占有byteへ判定済み・自己・ROI登録済みの3ビットを同居
    if ((*state & 1) == 0) {
      if (is_dense && has_self_cells_) {*state |= 1;}
      else {*state = is_self_cell(id) ? 3 : 1;}
    }
    const bool is_self = (*state & 2) != 0;
    if (is_roi && (*state & 4) == 0) {
      roi_points_->cells.push_back({id, is_self, true});
      *state |= 4;
    }
    point_cell = static_cast<std::uint64_t>(id) |
      (is_self ? voxel_idx::roi_point_membership::self_flag : 0);
    if (is_roi) {++stats_.accepted_point_count;}
    else {++stats_.outside_point_count;}
  }

private:
  void add_target_point(const Eigen::Vector3d &target_point)
  {
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
      dense_occupancy_[local_flat_idx] &= enable_point_membership_ && has_self_cells_ ? 2U : 0U;
    }
    touched_dense_indices_.clear();
    voxel_ids_.clear();
    sparse_voxel_ids_.clear();
    sparse_cell_states_.clear();
  }

  const robot_sim::analysis::VoxelIdCodec &codec_;
  reachability_bounds bounds_;
  bool enable_point_membership_{false};
  bool has_self_cells_{false};
  std::vector<std::size_t> self_dense_indices_;
  std::shared_ptr<voxel_idx::roi_point_membership> roi_points_, spare_roi_points_;
  std::unordered_map<long, std::uint8_t> sparse_cell_states_;
  bool enable_dense_bitmap_{false};
  Eigen::Vector3i min_dense_idx_{Eigen::Vector3i::Zero()};
  Eigen::Vector3i dense_dims_{Eigen::Vector3i::Zero()};
  std::vector<std::uint8_t> dense_occupancy_;
  std::vector<std::size_t> touched_dense_indices_;
  std::unordered_set<long> sparse_voxel_ids_;
  std::vector<long> voxel_ids_;
  reachability_voxelization_stats stats_;
};

}  // 名前空間robot_sim::indexingの終端
