#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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

// 分離した6x6平面パッチの配置。各パッチの生成後に接続を追加する併合試験用。
TopologicalMap make_plane_patches(const std::size_t num_columns, const std::size_t num_rows)
{
  TopologicalMap map;
  const double axis_u[3] = {1.0, 0.0, 0.0};
  const double axis_v[3] = {0.0, 1.0, 0.0};
  for (std::size_t row = 0U; row < num_rows; ++row) {
    for (std::size_t column = 0U; column < num_columns; ++column) {
      const double origin[3] = {0.30 * column, 0.30 * row, 0.0};
      appendGrid(map, 6U, 6U, 0.05, origin, axis_u, axis_v, TopologicalMap::WALL);
    }
  }
  return map;
}

// 隣接パッチの境界6点同士の接続。
void connect_plane_patches(
  TopologicalMap &map, const std::size_t first_idx, const std::size_t second_idx,
  const bool is_horizontal)
{
  for (std::size_t idx = 0U; idx < 6U; ++idx) {
    map.edges.push_back(static_cast<std::uint16_t>(
      first_idx * 36U + (is_horizontal ? idx * 6U + 5U : 30U + idx)));
    map.edges.push_back(static_cast<std::uint16_t>(
      second_idx * 36U + (is_horizontal ? idx * 6U : idx)));
  }
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

// CPU GNG直結経路と同じく計算済みrhoを種順序に使っても、
// 平面クラスタの結果が変わらない。
TEST(PlaneClusterIncremental, ReusesNodeRhoForSeedOrdering)
{
  ClusterOptions options;
  options.use_node_rho_for_seed_order = true;
  Clusterizer clusterizer{options};
  TopologicalMap map = makeSinglePlane();
  for (auto &node : map.nodes) {
    node.rho = 0.0F;
  }

  const ClusterResult result = warmUp(clusterizer, map);

  ASSERT_EQ(result.clusters.clusters.size(), 1U);
  EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
  EXPECT_NEAR(std::abs(result.clusters.clusters.front().normal.z), 1.0, 1.0e-3);

  clusterizer.setUseNodeRhoForSeedOrder(false);
  const ClusterResult switched = clusterizer.update(map);
  ASSERT_EQ(switched.clusters.clusters.size(), 1U);
  EXPECT_EQ(switched.statistics.clustered_node_count, map.nodes.size());
  EXPECT_EQ(switched.clusters.clusters.front().id, result.clusters.clusters.front().id);
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

// 各隣接対は平面でも、連鎖統合による全体の細長さを毎回再評価。
TEST(PlaneClusterIncremental, MergeChainChecksAccumulatedPlanarity)
{
  const ClusterOptions options;
  Clusterizer clusterizer{options};
  TopologicalMap map = make_plane_patches(6U, 1U);
  ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 6U);
  for (std::size_t idx = 0U; idx < 5U; ++idx) {
    connect_plane_patches(map, idx, idx + 1U, true);
  }

  for (std::size_t iter = 0U; iter < 5U; ++iter) {
    const ClusterResult result = clusterizer.update(map);
    EXPECT_GE(result.clusters.clusters.size(), 2U);
    EXPECT_LT(result.clusters.clusters.size(), 6U);
    EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
    for (const auto &cluster : result.clusters.clusters) {
      EXPECT_GE(cluster.planarity + 1.0e-6, options.merge_min_planarity);
    }
  }
}

// 線状でない2x2パッチの連鎖統合と、統合済み同士の二重計上防止。
TEST(PlaneClusterIncremental, CompactPatchesMergeWithoutNodeCovariance)
{
  Clusterizer clusterizer{ClusterOptions{}};
  TopologicalMap map = make_plane_patches(2U, 2U);
  for (auto &node : map.nodes) {
    node.winner_point_count = 0U;
    node.winner_point_covariance.fill(std::numeric_limits<float>::quiet_NaN());
  }
  ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 4U);
  connect_plane_patches(map, 0U, 1U, true);
  connect_plane_patches(map, 2U, 3U, true);
  connect_plane_patches(map, 0U, 2U, false);
  connect_plane_patches(map, 1U, 3U, false);

  const ClusterResult result = clusterizer.update(map);
  ASSERT_EQ(result.clusters.clusters.size(), 1U);
  EXPECT_EQ(result.statistics.merged_cluster_count, 3U);
  EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
  EXPECT_EQ(result.clusters.clusters.front().node_indices.size(), map.nodes.size());
  EXPECT_NEAR(result.clusters.clusters.front().planarity, 1.0, 1.0e-6);
  const ClusterResult repeated = clusterizer.update(map);
  EXPECT_EQ(clusterIds(repeated), clusterIds(result));
  EXPECT_EQ(repeated.statistics.merged_cluster_count, 0U);
}

// 隣接対では許容内でも、統合後全体では厚み超過となる段差の棄却。
TEST(PlaneClusterIncremental, MergeChainChecksAccumulatedResidual)
{
  ClusterOptions options;
  options.max_normalized_cluster_residual = 0.15;
  Clusterizer clusterizer{options};
  TopologicalMap map = make_plane_patches(3U, 1U);
  for (std::size_t idx = 36U; idx < 72U; ++idx) {
    map.nodes[idx].pos.z = 0.02F;
  }
  ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 3U);
  connect_plane_patches(map, 0U, 1U, true);
  connect_plane_patches(map, 1U, 2U, true);

  const ClusterResult result = clusterizer.update(map);
  EXPECT_EQ(result.clusters.clusters.size(), 2U);
  EXPECT_EQ(result.statistics.merged_cluster_count, 1U);
  EXPECT_GT(result.statistics.merge_absolute_residual_rejected_pair_count, 0U);
  EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
}

// 累積統計による少数側RMS判定。微小ずれの許容と、浮いた小面の誤吸収防止。
TEST(PlaneClusterIncremental, SmallerSideResidualUsesPositionStatistics)
{
  for (const double offset : {0.001, 0.03}) {
    SCOPED_TRACE(offset);
    ClusterOptions options;
    options.merge_residual_growth_min_th = 1.0;
    options.merge_smaller_side_residual_ratio = 0.10;
    Clusterizer clusterizer{options};
    TopologicalMap map;
    const double large_origin[3] = {0.0, 0.0, 0.0};
    const double small_origin[3] = {0.60, 0.0, offset};
    const double axis_u[3] = {1.0, 0.0, 0.0};
    const double axis_v[3] = {0.0, 1.0, 0.0};
    appendGrid(map, 12U, 12U, 0.05, large_origin, axis_u, axis_v, TopologicalMap::WALL);
    const std::size_t small_idx =
      appendGrid(map, 6U, 6U, 0.05, small_origin, axis_u, axis_v, TopologicalMap::WALL);
    ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
    for (std::size_t idx = 0U; idx < 6U; ++idx) {
      map.edges.push_back(static_cast<std::uint16_t>(idx * 12U + 11U));
      map.edges.push_back(static_cast<std::uint16_t>(small_idx + idx * 6U));
    }

    const ClusterResult result = clusterizer.update(map);
    if (offset == 0.001) {
      EXPECT_EQ(result.clusters.clusters.size(), 1U);
      EXPECT_EQ(result.statistics.merged_cluster_count, 1U);
    } else {
      EXPECT_EQ(result.clusters.clusters.size(), 2U);
      EXPECT_EQ(result.statistics.merged_cluster_count, 0U);
      EXPECT_EQ(result.statistics.merge_smaller_side_rejected_pair_count, 1U);
    }
    EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
  }
}

// 分割不要の早期終了経路でも、切断の確認待ちと再接続時のカウンタ初期化を維持。
TEST(PlaneClusterIncremental, SplitConfirmationSurvivesNoSplitFastPath)
{
  ClusterOptions options;
  options.split_confirm_frames = 2U;
  Clusterizer clusterizer{options};
  const TopologicalMap connected = makeSinglePlane();
  const auto original = warmUp(clusterizer, connected);
  ASSERT_EQ(original.clusters.clusters.size(), 1U);

  TopologicalMap disconnected = connected;
  disconnected.edges.clear();
  for (std::size_t idx = 0U; idx < connected.edges.size(); idx += 2U) {
    const auto first = connected.edges[idx];
    const auto second = connected.edges[idx + 1U];
    if (first % 6U == 2U && second == first + 1U) {
      continue;
    }
    disconnected.edges.push_back(first);
    disconnected.edges.push_back(second);
  }

  EXPECT_EQ(clusterizer.update(disconnected).statistics.split_cluster_count, 0U);
  EXPECT_EQ(clusterIds(clusterizer.update(connected)), clusterIds(original));
  for (std::size_t iter = 0U; iter < options.split_confirm_frames; ++iter) {
    const auto pending = clusterizer.update(disconnected);
    EXPECT_EQ(pending.statistics.split_cluster_count, 0U);
    EXPECT_EQ(clusterIds(pending), clusterIds(original));
  }
  const auto split = clusterizer.update(disconnected);
  EXPECT_EQ(split.statistics.split_cluster_count, 1U);
  ASSERT_EQ(split.clusters.clusters.size(), 2U);
  EXPECT_EQ(split.statistics.clustered_node_count, connected.nodes.size());
  EXPECT_EQ(clusterIds(clusterizer.update(connected)), clusterIds(original));
}

// 削除不要の早期終了経路の後でも、先頭クラスタ削除時の所属添字を正しく詰め直し。
TEST(PlaneClusterIncremental, RemovalRemapsAfterNoRemovalFastPath)
{
  Clusterizer clusterizer{ClusterOptions{}};
  const TopologicalMap complete = make_plane_patches(2U, 1U);
  const auto original = warmUp(clusterizer, complete);
  ASSERT_EQ(original.clusters.clusters.size(), 2U);
  std::uint32_t retained_id = 0U;
  for (const auto &cluster : original.clusters.clusters) {
    if (cluster.node_indices.front() >= 36U) {
      retained_id = cluster.id;
    }
  }
  ASSERT_NE(retained_id, 0U);

  TopologicalMap reduced = complete;
  reduced.nodes.erase(reduced.nodes.begin(), reduced.nodes.begin() + 36U);
  reduced.edges.clear();
  for (std::size_t idx = 0U; idx < complete.edges.size(); idx += 2U) {
    if (complete.edges[idx] >= 36U && complete.edges[idx + 1U] >= 36U) {
      reduced.edges.push_back(complete.edges[idx] - 36U);
      reduced.edges.push_back(complete.edges[idx + 1U] - 36U);
    }
  }
  const auto removed = clusterizer.update(reduced);
  EXPECT_EQ(removed.statistics.removed_cluster_count, 1U);
  ASSERT_EQ(removed.clusters.clusters.size(), 1U);
  EXPECT_EQ(removed.clusters.clusters.front().id, retained_id);
  EXPECT_EQ(removed.statistics.clustered_node_count, reduced.nodes.size());
  const auto repeated = clusterizer.update(reduced);
  EXPECT_EQ(clusterIds(repeated), clusterIds(removed));
  EXPECT_EQ(repeated.statistics.removed_cluster_count, 0U);
}

// 位置・法線の幾何更新と独立した、接続不変クラスタの探索省略。
TEST(PlaneClusterIncremental, ConnectivityTreeReusesMovingPlane)
{
  Clusterizer clusterizer{ClusterOptions{}};
  auto map = makeSinglePlane();
  const auto original = warmUp(clusterizer, map);
  for (auto &node : map.nodes) {
    node.pos.z += 0.001F;
  }
  const auto result = clusterizer.update(map);
  EXPECT_EQ(clusterIds(result), clusterIds(original));
  EXPECT_EQ(result.statistics.num_connectivity_reused_clusters, 1U);
  EXPECT_EQ(result.statistics.num_connectivity_scanned_nodes, 0U);
  EXPECT_NEAR(result.clusters.clusters.front().centroid.z, 0.001, 1.0e-6);
}

// 非木エッジの追加・削除による連結証明の維持。
TEST(PlaneClusterIncremental, ConnectivityTreeIgnoresNonTreeEdgeChanges)
{
  Clusterizer clusterizer{ClusterOptions{}};
  auto map = makeSinglePlane();
  warmUp(clusterizer, map);
  map.edges.push_back(0U);
  map.edges.push_back(35U);
  const auto added = clusterizer.update(map);
  EXPECT_EQ(added.statistics.num_connectivity_reused_clusters, 1U);
  EXPECT_EQ(added.statistics.num_connectivity_scanned_nodes, 0U);
  map.edges.resize(map.edges.size() - 2U);
  const auto removed = clusterizer.update(map);
  EXPECT_EQ(removed.statistics.num_connectivity_reused_clusters, 1U);
  EXPECT_EQ(removed.statistics.num_connectivity_scanned_nodes, 0U);
}

// 木エッジ消失時の該当クラスタ限定再探索と、代替経路での証明更新。
TEST(PlaneClusterIncremental, ConnectivityTreeDeletionOnlyRechecksAffectedCluster)
{
  Clusterizer clusterizer{ClusterOptions{}};
  auto map = make_plane_patches(2U, 1U);
  const auto original = warmUp(clusterizer, map);
  ASSERT_EQ(map.edges[0], 0U);
  ASSERT_EQ(map.edges[1], 1U);
  map.edges.erase(map.edges.begin(), map.edges.begin() + 2U);
  const auto changed = clusterizer.update(map);
  EXPECT_EQ(clusterIds(changed), clusterIds(original));
  EXPECT_EQ(changed.statistics.num_connectivity_reused_clusters, 1U);
  EXPECT_EQ(changed.statistics.num_connectivity_scanned_nodes, 36U);
  const auto repeated = clusterizer.update(map);
  EXPECT_EQ(repeated.statistics.num_connectivity_reused_clusters, 2U);
  EXPECT_EQ(repeated.statistics.num_connectivity_scanned_nodes, 0U);
}

// 入力ノード再配置時の親添字無効化と、ID・所属の維持。
TEST(PlaneClusterIncremental, ConnectivityTreeInvalidatesReorderedNodes)
{
  Clusterizer clusterizer{ClusterOptions{}};
  auto map = makeSinglePlane();
  const auto original = warmUp(clusterizer, map);
  std::reverse(map.nodes.begin(), map.nodes.end());
  for (auto &idx : map.edges) {
    idx = static_cast<std::uint16_t>(map.nodes.size() - 1U - idx);
  }
  const auto changed = clusterizer.update(map);
  EXPECT_EQ(clusterIds(changed), clusterIds(original));
  EXPECT_EQ(changed.statistics.num_connectivity_reused_clusters, 0U);
  EXPECT_EQ(changed.statistics.num_connectivity_scanned_nodes, map.nodes.size());
  EXPECT_EQ(clusterizer.update(map).statistics.num_connectivity_reused_clusters, 1U);
}

// CSR内の親位置が変わっても、同じ木エッジが残る場合の探索省略。
TEST(PlaneClusterIncremental, ConnectivityTreeReusesReorderedEdges)
{
  Clusterizer clusterizer{ClusterOptions{}};
  auto map = makeSinglePlane();
  const auto original = warmUp(clusterizer, map);
  std::reverse(map.edges.begin(), map.edges.end());
  const auto changed = clusterizer.update(map);
  EXPECT_EQ(clusterIds(changed), clusterIds(original));
  EXPECT_EQ(changed.statistics.num_connectivity_reused_clusters, 1U);
  EXPECT_EQ(changed.statistics.num_connectivity_scanned_nodes, 0U);
}

// 入力個数が同じでも、ノードID置換時には証明を再構築。
TEST(PlaneClusterIncremental, ConnectivityTreeInvalidatesReplacedNode)
{
  Clusterizer clusterizer{ClusterOptions{}};
  auto map = makeSinglePlane();
  const auto original = warmUp(clusterizer, map);
  map.nodes.back().id = 1000U;
  const auto changed = clusterizer.update(map);
  EXPECT_EQ(clusterIds(changed), clusterIds(original));
  EXPECT_EQ(changed.statistics.num_connectivity_reused_clusters, 0U);
  EXPECT_EQ(changed.statistics.num_connectivity_scanned_nodes, map.nodes.size());
  EXPECT_EQ(clusterizer.update(map).statistics.num_connectivity_reused_clusters, 1U);
}
