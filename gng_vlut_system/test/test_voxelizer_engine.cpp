#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <unordered_set>
#include <vector>

#include "core/common/voxelizer_engine.hpp"

namespace {

TEST(VoxelizerEngineTest, 境界三角形の両側セル収録) {
  GNG::Analysis::IndexVoxelGrid grid(0.02);
  std::unordered_set<long> voxel_ids;
  const std::vector<Eigen::Vector3d> triangle{
      Eigen::Vector3d(0.0, -0.015, -0.015),
      Eigen::Vector3d(0.0, 0.015, -0.015),
      Eigen::Vector3d(0.0, 0.0, 0.015)};

  robot_sim::common::VoxelizerEngine::voxelizeMeshTriangles(
      triangle, grid, voxel_ids);

  EXPECT_TRUE(voxel_ids.count(
      grid.getFlatVoxelId(Eigen::Vector3i(-1, 0, 0))));
  EXPECT_TRUE(voxel_ids.count(
      grid.getFlatVoxelId(Eigen::Vector3i(0, 0, 0))));
}

TEST(VoxelizerEngineTest, 非格子並進セルの出力グリッド被覆) {
  GNG::Analysis::IndexVoxelGrid grid(0.02);
  std::vector<long> voxel_ids;
  Eigen::Isometry3d local_to_target = Eigen::Isometry3d::Identity();
  local_to_target.translation().z() = 0.0015;

  robot_sim::common::VoxelizerEngine::appendTransformedVoxelCell(
      Eigen::Vector3d(0.01, 0.01, 0.01), local_to_target, grid, voxel_ids);

  EXPECT_NE(std::find(
                voxel_ids.begin(), voxel_ids.end(),
                grid.getFlatVoxelId(Eigen::Vector3i(0, 0, 0))),
            voxel_ids.end());
  EXPECT_NE(std::find(
                voxel_ids.begin(), voxel_ids.end(),
                grid.getFlatVoxelId(Eigen::Vector3i(0, 0, 1))),
            voxel_ids.end());
}

TEST(VoxelizerEngineTest, 回転セルの出力グリッド被覆) {
  GNG::Analysis::IndexVoxelGrid grid(0.02);
  std::vector<long> voxel_ids;
  Eigen::Isometry3d local_to_target = Eigen::Isometry3d::Identity();
  local_to_target.rotate(Eigen::AngleAxisd(
      std::acos(-1.0) / 4.0, Eigen::Vector3d::UnitZ()));

  robot_sim::common::VoxelizerEngine::appendTransformedVoxelCell(
      Eigen::Vector3d(0.01, 0.01, 0.01), local_to_target, grid, voxel_ids);

  EXPECT_GT(voxel_ids.size(), 1U);
  EXPECT_FALSE(robot_sim::common::VoxelizerEngine::
                   isVoxelGridAlignedTransform(local_to_target));
}

}  // 無名名前空間
