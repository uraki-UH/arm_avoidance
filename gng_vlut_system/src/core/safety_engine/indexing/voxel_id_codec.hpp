// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "safety_engine/indexing/index_voxel_grid.hpp"

#include <algorithm>
#include <Eigen/Dense>
#include "common/voxel_utils.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <vector>

namespace robot_sim::analysis
{

// Flat voxel ID の生成と voxel メッセージ化をまとめた共通ユーティリティ。
// ここは「どうやって voxel_id を作るか」のルールだけを持ち、ノード側は入出力に集中できるようにする。
class VoxelIdCodec
{
public:
  explicit VoxelIdCodec(double voxel_size = 0.0)
  : grid_(voxel_size)
  {
  }

  void setVoxelSize(double voxel_size) { grid_.setVoxelSize(voxel_size); }
  void setIndexingParams(int x_shift, int y_shift, int z_shift, long offset)
  {
    grid_.setIndexingParams(x_shift, y_shift, z_shift, offset);
  }

  double voxelSize() const { return grid_.getVoxelSize(); }
  int xShift() const { return grid_.getXShift(); }
  int yShift() const { return grid_.getYShift(); }
  int zShift() const { return grid_.getZShift(); }
  long offset() const { return grid_.getOffset(); }

  long toFlatId(const Eigen::Vector3i &index) const
  {
    return grid_.getFlatVoxelId(index);
  }

  Eigen::Vector3i toIndex(long flat_id) const
  {
    return grid_.getIndexFromFlatId(flat_id);
  }

  std::vector<long> voxelize(const std::vector<Eigen::Vector3f> &points) const
  {
    std::vector<long> voxel_ids;
    voxel_ids.reserve(points.size());

    const float vs = static_cast<float>(grid_.getVoxelSize());
    if (vs <= 0.0f) {
      return {};
    }

    for (const auto &point : points) {
      const Eigen::Vector3i index =
        ::common::geometry::VoxelUtils::worldToVoxel(point, vs);
      voxel_ids.push_back(grid_.getFlatVoxelId(index));
    }

    return uniqueSorted(std::move(voxel_ids));
  }

  voxel_msgs::msg::Voxel makeMessage(
    const std_msgs::msg::Header &header,
    const std::vector<long> &voxel_ids) const
  {
    voxel_msgs::msg::Voxel msg;
    msg.header = header;
    msg.voxel_size = static_cast<float>(grid_.getVoxelSize());
    msg.x_shift = grid_.getXShift();
    msg.y_shift = grid_.getYShift();
    msg.z_shift = grid_.getZShift();
    msg.offset = grid_.getOffset();
    msg.data.reserve(voxel_ids.size());
    for (long id : voxel_ids) {
      msg.data.push_back(static_cast<int64_t>(id));
    }
    return msg;
  }

private:
  static std::vector<long> uniqueSorted(std::vector<long> values)
  {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
    return values;
  }

  ::GNG::Analysis::IndexVoxelGrid grid_;
};

}  // namespace robot_sim::analysis
