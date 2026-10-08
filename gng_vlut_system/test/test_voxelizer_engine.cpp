#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <unordered_set>
#include <vector>

#include "core/common/voxelizer_engine.hpp"

namespace {

TEST(VoxelizerEngineTest, カメラ直方体の端部セル収録) {
  GNG::Analysis::IndexVoxelGrid grid(0.019999999552965164);
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  // Longモデルのcamera_link衝突形状と同じ寸法・配置 [m]
  origin.translation() = Eigen::Vector3d(
      -0.0082262434959411625, 4.2556762695312306e-05, -4.2438507080050369e-08);
  const Eigen::Vector3d size(
      0.025052487373352052, 0.089856307983398442, 0.025000000953674318);
  std::unordered_set<long> voxel_ids;
  robot_sim::common::VoxelizerEngine::voxelizeBox(size, origin, grid, voxel_ids);

  // 各軸の外形が横切る3×6×2セルの被覆
  ASSERT_EQ(voxel_ids.size(), 36U);
  for (int x = -2; x <= 0; ++x) {
    for (int y = -3; y <= 2; ++y) {
      for (int z = -1; z <= 0; ++z) {
        EXPECT_EQ(voxel_ids.count(grid.getFlatVoxelId(Eigen::Vector3i(x, y, z))), 1U);
      }
    }
  }
}

TEST(VoxelizerEngineTest, セル中心を含まない薄い直方体の被覆) {
  GNG::Analysis::IndexVoxelGrid grid(0.02);
  std::unordered_set<long> voxel_ids;
  robot_sim::common::VoxelizerEngine::voxelizeBox(
      Eigen::Vector3d::Constant(0.002), Eigen::Isometry3d::Identity(), grid, voxel_ids);
  ASSERT_EQ(voxel_ids.size(), 8U);
  for (int x : {-1, 0}) for (int y : {-1, 0}) for (int z : {-1, 0}) {
    EXPECT_EQ(voxel_ids.count(grid.getFlatVoxelId(Eigen::Vector3i(x, y, z))), 1U);
  }
}

TEST(VoxelizerEngineTest, 直方体とセルの境界接触と非交差の区別) {
  GNG::Analysis::IndexVoxelGrid grid(0.02);
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translation().setConstant(0.01);
  std::unordered_set<long> voxel_ids;
  robot_sim::common::VoxelizerEngine::voxelizeBox(
      Eigen::Vector3d::Constant(0.02), origin, grid, voxel_ids);
  EXPECT_EQ(voxel_ids.count(grid.getFlatVoxelId(Eigen::Vector3i(-1, 0, 0))), 1U);
  EXPECT_EQ(voxel_ids.count(grid.getFlatVoxelId(Eigen::Vector3i(1, 0, 0))), 1U);
  EXPECT_EQ(voxel_ids.count(grid.getFlatVoxelId(Eigen::Vector3i(2, 0, 0))), 0U);
  EXPECT_EQ(voxel_ids.count(grid.getFlatVoxelId(Eigen::Vector3i(-2, 0, 0))), 0U);
}

TEST(VoxelizerEngineTest, 回転した薄い直方体の外形被覆と余剰セル除外) {
  GNG::Analysis::IndexVoxelGrid grid(0.02);
  const Eigen::Vector3d size(0.12, 0.006, 0.006);
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translation() = Eigen::Vector3d(0.003, -0.007, 0.013);
  origin.rotate(Eigen::AngleAxisd(std::acos(-1.0) / 4.0, Eigen::Vector3d::UnitZ()));
  origin.rotate(Eigen::AngleAxisd(0.31, Eigen::Vector3d::UnitY()));
  origin.rotate(Eigen::AngleAxisd(-0.27, Eigen::Vector3d::UnitX()));
  std::unordered_set<long> voxel_ids;
  robot_sim::common::VoxelizerEngine::voxelizeBox(size, origin, grid, voxel_ids);

  // 細長い形状の稜線と内部を含む独立サンプル
  for (int x = 0; x <= 100; ++x) {
    for (int y : {-1, 0, 1}) for (int z : {-1, 0, 1}) {
      const Eigen::Vector3d point = origin * Eigen::Vector3d(
          size.x() * (x / 100.0 - 0.5), y * size.y() / 2, z * size.z() / 2);
      const Eigen::Vector3i idx = common::geometry::VoxelUtils::worldToVoxel(
          point.cast<float>(), static_cast<float>(grid.getVoxelSize()));
      ASSERT_EQ(voxel_ids.count(grid.getFlatVoxelId(idx)), 1U) << point.transpose();
    }
  }
  // AABB内でも、細長い直方体と離れた隅セルの除外
  EXPECT_EQ(voxel_ids.count(grid.getFlatVoxelId(Eigen::Vector3i(-2, 1, 0))), 0U);
  EXPECT_EQ(voxel_ids.count(grid.getFlatVoxelId(Eigen::Vector3i(2, -2, 0))), 0U);
}

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
