#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <set>
#include <vector>

namespace
{

using ais_gng_msgs::msg::TopologicalMap;
using ais_gng_msgs::msg::TopologicalNode;
using fuzzrobo::topological_plane::incremental::ClusterOptions;
using fuzzrobo::topological_plane::incremental::ClusterResult;
using fuzzrobo::topological_plane::incremental::Clusterizer;

// 平面上の格子を地図へ足す。返り値は追加したノードの先頭添字。
//
// origin から axis_u / axis_v 方向へ spacing 間隔で並べ、4近傍をエッジでつなぐ。
std::size_t appendGrid(
  TopologicalMap &map, const std::size_t width, const std::size_t height,
  const double spacing, const double origin[3], const double axis_u[3],
  const double axis_v[3], const std::uint8_t label)
{
  const std::size_t base = map.nodes.size();
  double normal[3] = {
    axis_u[1] * axis_v[2] - axis_u[2] * axis_v[1],
    axis_u[2] * axis_v[0] - axis_u[0] * axis_v[2],
    axis_u[0] * axis_v[1] - axis_u[1] * axis_v[0]};
  const double length = std::sqrt(
    normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
  for (double &component : normal) {
    component /= length;
  }

  for (std::size_t v = 0U; v < height; ++v) {
    for (std::size_t u = 0U; u < width; ++u) {
      TopologicalNode node;
      node.id = static_cast<std::uint16_t>(map.nodes.size());
      node.pos.x = static_cast<float>(
        origin[0] + axis_u[0] * spacing * u + axis_v[0] * spacing * v);
      node.pos.y = static_cast<float>(
        origin[1] + axis_u[1] * spacing * u + axis_v[1] * spacing * v);
      node.pos.z = static_cast<float>(
        origin[2] + axis_u[2] * spacing * u + axis_v[2] * spacing * v);
      node.normal.x = static_cast<float>(normal[0]);
      node.normal.y = static_cast<float>(normal[1]);
      node.normal.z = static_cast<float>(normal[2]);
      node.label = label;
      map.nodes.push_back(node);
    }
  }

  const auto index_of = [base, width](const std::size_t u, const std::size_t v) {
      return static_cast<std::uint16_t>(base + v * width + u);
    };
  for (std::size_t v = 0U; v < height; ++v) {
    for (std::size_t u = 0U; u < width; ++u) {
      if (u + 1U < width) {
        map.edges.push_back(index_of(u, v));
        map.edges.push_back(index_of(u + 1U, v));
      }
      if (v + 1U < height) {
        map.edges.push_back(index_of(u, v));
        map.edges.push_back(index_of(u, v + 1U));
      }
    }
  }
  return base;
}

TopologicalMap makeSinglePlane()
{
  TopologicalMap map;
  const double origin[3] = {0.0, 0.0, 0.0};
  const double axis_u[3] = {1.0, 0.0, 0.0};
  const double axis_v[3] = {0.0, 1.0, 0.0};
  appendGrid(map, 6U, 6U, 0.05, origin, axis_u, axis_v, TopologicalMap::SAFE_TERRAIN);
  return map;
}

// 既定では新しいクラスタが出力に載るまで birth_confirm_frames ぶん掛かる。
// 同じ地図を規定回数入れて、確認済みの状態にしてから結果を見る。
ClusterResult warmUp(Clusterizer &clusterizer, const TopologicalMap &map)
{
  ClusterResult result;
  for (std::size_t i = 0; i < ClusterOptions{}.birth_confirm_frames + 1U; ++i) {
    result = clusterizer.update(map);
  }
  return result;
}

std::size_t totalChanges(const ClusterResult &result)
{
  return result.statistics.released_node_count +
         result.statistics.migrated_node_count +
         result.statistics.absorbed_node_count;
}

std::set<std::uint32_t> clusterIds(const ClusterResult &result)
{
  std::set<std::uint32_t> ids;
  for (const auto &cluster : result.clusters.clusters) {
    ids.insert(cluster.id);
  }
  return ids;
}

}  // 無名名前空間

// 平坦な格子ひとつが、ひとつのクラスタになる。
TEST(PlaneClusterIncremental, SinglePlaneBecomesOneCluster)
{
  Clusterizer clusterizer{ClusterOptions{}};
  const TopologicalMap map = makeSinglePlane();

  const ClusterResult result = warmUp(clusterizer, map);

  ASSERT_EQ(result.clusters.clusters.size(), 1U);
  EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
  EXPECT_EQ(result.clusters.clusters.front().source_label, TopologicalMap::SAFE_TERRAIN);
  EXPECT_NEAR(std::abs(result.clusters.clusters.front().normal.z), 1.0, 1.0e-3);
  const auto &covariance = result.clusters.clusters.front().position_covariance;
  EXPECT_NEAR(covariance[0], 0.007291667, 1.0e-7);
  EXPECT_NEAR(covariance[4], 0.007291667, 1.0e-7);
  EXPECT_NEAR(covariance[8], 0.0, 1.0e-9);
  EXPECT_NEAR(covariance[1], covariance[3], 1.0e-9);
  EXPECT_NEAR(covariance[2], covariance[6], 1.0e-9);
  EXPECT_NEAR(covariance[5], covariance[7], 1.0e-9);
}

// 同じ地図をもう一度入れたとき、所属がまったく動かない。
//
// 「クラスタ所属が定常状態にならない」という問題に対する、直接の回帰テスト。
TEST(PlaneClusterIncremental, ReachesSteadyStateOnRepeatedInput)
{
  Clusterizer clusterizer{ClusterOptions{}};
  const TopologicalMap map = makeSinglePlane();

  const ClusterResult first = warmUp(clusterizer, map);
  ASSERT_EQ(first.clusters.clusters.size(), 1U);

  for (int iteration = 0; iteration < 5; ++iteration) {
    const ClusterResult repeated = clusterizer.update(map);
    EXPECT_EQ(totalChanges(repeated), 0U) << "iteration " << iteration;
    EXPECT_EQ(repeated.statistics.born_cluster_count, 0U) << "iteration " << iteration;
    EXPECT_EQ(repeated.statistics.split_cluster_count, 0U) << "iteration " << iteration;
    EXPECT_EQ(repeated.statistics.merged_cluster_count, 0U) << "iteration " << iteration;
    EXPECT_EQ(clusterIds(repeated), clusterIds(first)) << "iteration " << iteration;
  }
}

// 直交する2面は、つながっていても別クラスタのままになる。
TEST(PlaneClusterIncremental, PerpendicularPlanesStaySeparate)
{
  TopologicalMap map;
  const double floor_origin[3] = {0.0, 0.0, 0.0};
  const double floor_u[3] = {1.0, 0.0, 0.0};
  const double floor_v[3] = {0.0, 1.0, 0.0};
  const std::size_t floor_base =
    appendGrid(map, 6U, 6U, 0.05, floor_origin, floor_u, floor_v, TopologicalMap::SAFE_TERRAIN);

  const double wall_origin[3] = {0.0, 0.30, 0.05};
  const double wall_u[3] = {1.0, 0.0, 0.0};
  const double wall_v[3] = {0.0, 0.0, 1.0};
  const std::size_t wall_base =
    appendGrid(map, 6U, 6U, 0.05, wall_origin, wall_u, wall_v, TopologicalMap::WALL);

  // 床の最終行と壁の最初の行をつないで、1つの連結グラフにする。
  for (std::size_t u = 0U; u < 6U; ++u) {
    map.edges.push_back(static_cast<std::uint16_t>(floor_base + 5U * 6U + u));
    map.edges.push_back(static_cast<std::uint16_t>(wall_base + u));
  }

  Clusterizer clusterizer{ClusterOptions{}};
  const ClusterResult result = warmUp(clusterizer, map);

  ASSERT_EQ(result.clusters.clusters.size(), 2U);
  std::set<std::uint8_t> labels;
  for (const auto &cluster : result.clusters.clusters) {
    labels.insert(cluster.source_label);
  }
  EXPECT_EQ(labels.count(TopologicalMap::SAFE_TERRAIN), 1U);
  EXPECT_EQ(labels.count(TopologicalMap::WALL), 1U);
}

// クラスタIDがフレームをまたいで持続する。
TEST(PlaneClusterIncremental, ClusterIdPersistsAcrossFrames)
{
  Clusterizer clusterizer{ClusterOptions{}};
  TopologicalMap map = makeSinglePlane();

  const ClusterResult first = warmUp(clusterizer, map);
  ASSERT_EQ(first.clusters.clusters.size(), 1U);
  const std::uint32_t original_id = first.clusters.clusters.front().id;

  // ノードをわずかに揺らしても、同じIDのまま追従する。
  for (auto &node : map.nodes) {
    node.pos.z += 0.0005F;
  }
  const ClusterResult second = warmUp(clusterizer, map);

  ASSERT_EQ(second.clusters.clusters.size(), 1U);
  EXPECT_EQ(second.clusters.clusters.front().id, original_id);
}

// reset() で所属を捨てると、次のフレームは新しいIDから作り直す。
TEST(PlaneClusterIncremental, ResetDiscardsOwnership)
{
  Clusterizer clusterizer{ClusterOptions{}};
  const TopologicalMap map = makeSinglePlane();

  const ClusterResult first = warmUp(clusterizer, map);
  ASSERT_EQ(first.clusters.clusters.size(), 1U);

  clusterizer.reset();
  const ClusterResult after_reset = warmUp(clusterizer, map);

  ASSERT_EQ(after_reset.clusters.clusters.size(), 1U);
  EXPECT_NE(after_reset.clusters.clusters.front().id, first.clusters.clusters.front().id);
  EXPECT_NE(after_reset.clusters.clusters.front().id, first.clusters.clusters.front().id);
}

// 一直線に並んだ鎖は、クラスタにならない。
//
// 共分散の第2固有値が第1固有値に対して小さすぎる形であり、平面として扱うと
// 法線が不定になる。育ちきる前に棄却されることを確認する。
TEST(PlaneClusterIncremental, StraightChainIsRejected)
{
  TopologicalMap map;
  const double origin[3] = {0.0, 0.0, 0.0};
  const double axis_u[3] = {1.0, 0.0, 0.0};
  const double axis_v[3] = {0.0, 1.0, 0.0};
  appendGrid(map, 20U, 1U, 0.05, origin, axis_u, axis_v, TopologicalMap::UNKNOWN_OBJECT);

  Clusterizer clusterizer{ClusterOptions{}};
  const ClusterResult result = warmUp(clusterizer, map);

  EXPECT_TRUE(result.clusters.clusters.empty());
  EXPECT_GT(result.statistics.chain_rejected_count, 0U);
  EXPECT_EQ(result.statistics.clustered_node_count, 0U);
}

// 細長い帯も鎖状として棄却する。第2固有値の比が小さいままだからである。
TEST(PlaneClusterIncremental, ThinStripIsRejected)
{
  TopologicalMap map;
  const double origin[3] = {0.0, 0.0, 0.0};
  const double axis_u[3] = {1.0, 0.0, 0.0};
  const double axis_v[3] = {0.0, 1.0, 0.0};
  appendGrid(map, 24U, 2U, 0.05, origin, axis_u, axis_v, TopologicalMap::WALL);

  Clusterizer clusterizer{ClusterOptions{}};
  const ClusterResult result = warmUp(clusterizer, map);

  EXPECT_TRUE(result.clusters.clusters.empty());
  EXPECT_GT(result.statistics.chain_rejected_count, 0U);
}

// 鎖を含む地図でも、同じ入力を繰り返せば処理量が増え続けない。
//
// 棄却したノードを毎フレーム同じように扱うため、結果がフレーム間で一致する。
TEST(PlaneClusterIncremental, ChainRejectionIsStableAcrossFrames)
{
  TopologicalMap map;
  const double plane_origin[3] = {0.0, 0.0, 0.0};
  const double axis_u[3] = {1.0, 0.0, 0.0};
  const double axis_v[3] = {0.0, 1.0, 0.0};
  appendGrid(map, 6U, 6U, 0.05, plane_origin, axis_u, axis_v, TopologicalMap::SAFE_TERRAIN);

  // 平面から離れた位置に、独立した鎖を置く。
  const double chain_origin[3] = {0.0, 0.0, 1.0};
  appendGrid(map, 20U, 1U, 0.05, chain_origin, axis_u, axis_v, TopologicalMap::UNKNOWN_OBJECT);

  Clusterizer clusterizer{ClusterOptions{}};
  const ClusterResult first = warmUp(clusterizer, map);
  ASSERT_EQ(first.clusters.clusters.size(), 1U);

  for (int iteration = 0; iteration < 4; ++iteration) {
    const ClusterResult repeated = clusterizer.update(map);
    EXPECT_EQ(repeated.clusters.clusters.size(), 1U) << "iteration " << iteration;
    EXPECT_EQ(totalChanges(repeated), 0U) << "iteration " << iteration;
    EXPECT_EQ(repeated.statistics.born_cluster_count, 0U) << "iteration " << iteration;
    EXPECT_EQ(
      repeated.statistics.chain_rejected_count,
      first.statistics.chain_rejected_count) << "iteration " << iteration;
  }
}
