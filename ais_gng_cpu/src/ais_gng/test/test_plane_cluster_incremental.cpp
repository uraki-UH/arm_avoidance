#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <utility>
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

// 12x12の平面と5x5の小断片。1本接続試験用の独立した2成分。
std::size_t append_fragment_pair(
  TopologicalMap &map, const double scale = 1.0, const double gap_ratio = 1.0,
  const double offset_ratio = 0.0)
{
  const auto base = map.nodes.size();
  const double origin[3] = {0.0, 0.0, 0.0};
  const double small_origin[3] = {(0.55 + 0.05 * gap_ratio) * scale, 0.0, 0.05 * offset_ratio * scale};
  const double axis_u[3] = {1.0, 0.0, 0.0};
  const double axis_v[3] = {0.0, 1.0, 0.0};
  appendGrid(map, 12U, 12U, 0.05 * scale, origin, axis_u, axis_v, TopologicalMap::WALL);
  appendGrid(map, 5U, 5U, 0.05 * scale, small_origin, axis_u, axis_v, TopologicalMap::WALL);
  return base;
}

// 指定ノード群から面外へ伸びる接続。先端は元平面と直交する法線の未所属点。
void append_conflict_edges(
  TopologicalMap &map, const std::size_t base, const std::size_t num_nodes,
  const double scale = 1.0, const double horizontal_ratio = 0.0)
{
  for (std::size_t idx = 0U; idx < num_nodes; ++idx) {
    TopologicalNode node;
    node.id = static_cast<std::uint16_t>(map.nodes.size());
    node.pos = map.nodes[base + idx].pos;
    node.pos.z -= static_cast<float>(0.05 * scale);
    node.pos.x += static_cast<float>(0.05 * scale * horizontal_ratio);
    node.normal.x = 1.0;
    map.edges.insert(map.edges.end(), {static_cast<std::uint16_t>(base + idx), node.id});
    map.nodes.push_back(node);
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

// 面幅を保った同一平面の連鎖統合。長さだけを理由とする過剰分割の防止。
TEST(PlaneClusterIncremental, WidePlaneChainMergesDespiteAspectRatio)
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
    ASSERT_EQ(result.clusters.clusters.size(), 1U);
    EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
    EXPECT_LT(result.clusters.clusters.front().planarity, options.merge_min_planarity);
  }
}

// 長さに依存しない面幅判定。路面相当の長方形を初回から一面として生成。
TEST(PlaneClusterIncremental, LongWidePlaneIsNotRejectedAsChain)
{
  TopologicalMap map;
  const double origin[3] = {0.0, 0.0, 0.0};
  const double axis_u[3] = {1.0, 0.0, 0.0};
  const double axis_v[3] = {0.0, 1.0, 0.0};
  appendGrid(map, 120U, 6U, 0.05, origin, axis_u, axis_v, TopologicalMap::SAFE_TERRAIN);
  Clusterizer clusterizer{ClusterOptions{}};
  const auto result = warmUp(clusterizer, map);
  ASSERT_EQ(result.clusters.clusters.size(), 1U);
  EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
}

// 取り込み緩和のOFF時にも有効な保持ヒステリシスと、明確な逸脱の解放。
TEST(PlaneClusterIncremental, RetentionDoesNotDependOnMultiEdgeRelaxation)
{
  ClusterOptions options;
  options.enable_multi_edge_dist_relaxation = false;
  options.max_effective_spacing = 0.02;
  options.growth_residual_ratio = 0.70;
  options.max_normalized_cluster_residual = 0.70;
  options.retention_residual_ratio = 1.40;
  Clusterizer clusterizer{options};
  auto map = makeSinglePlane();
  const auto initial = warmUp(clusterizer, map);
  map.nodes[14].pos.z = 0.022F;
  const auto retained = clusterizer.update(map);
  EXPECT_EQ(retained.statistics.released_node_count, 0U);
  EXPECT_EQ(retained.statistics.clustered_node_count, map.nodes.size());
  EXPECT_EQ(clusterIds(retained), clusterIds(initial));
  map.nodes[14].pos.z = 0.10F;
  EXPECT_GT(clusterizer.update(map).statistics.released_node_count, 0U);
}

// 粗い点間隔での新規生成にも適用する厚み制約。生成直後の大量解放の防止。
TEST(PlaneClusterIncremental, BirthUsesBoundedSpacingForResidual)
{
  ClusterOptions options;
  options.birth_confirm_frames = 0U;
  options.max_effective_spacing = 0.02;
  auto map = makeSinglePlane();
  for (auto &node : map.nodes) {
    node.pos.x *= 10.0F;
    node.pos.y *= 10.0F;
    node.pos.z = node.id % 2U == 0U ? 0.05F : -0.05F;
  }
  Clusterizer clusterizer{options};
  const auto result = clusterizer.update(map);
  for (const auto &cluster : result.clusters.clusters) {
    double squared_sum = 0.0;
    for (const auto idx : cluster.node_indices) {
      const auto &p = map.nodes[idx].pos;
      const double dist = cluster.normal.x * (p.x - cluster.centroid.x) +
        cluster.normal.y * (p.y - cluster.centroid.y) +
        cluster.normal.z * (p.z - cluster.centroid.z);
      squared_sum += dist * dist;
    }
    EXPECT_LE(std::sqrt(squared_sum / cluster.node_indices.size()),
      options.max_normalized_cluster_residual * options.max_effective_spacing + 1.e-6);
  }
}

// 同じ相対ノイズ・変位に対する、卓上から屋外までの生成・保持・解放の一致。
TEST(PlaneClusterIncremental, MembershipScalesWithLocalEdges)
{
  for (const double scale : {0.1, 1.0, 10.0, 100.0}) {
    SCOPED_TRACE(scale);
    const double spacing = 0.02 * scale;
    TopologicalMap map;
    const double origin[3] = {0.0, 0.0, 0.0};
    const double axis_u[3] = {1.0, 0.0, 0.0};
    const double axis_v[3] = {0.0, 1.0, 0.0};
    appendGrid(map, 16U, 12U, spacing, origin, axis_u, axis_v, TopologicalMap::SAFE_TERRAIN);
    for (auto &node : map.nodes) {
      node.pos.z = static_cast<float>(spacing * 0.02 * std::sin(1.7 * node.id));
    }
    Clusterizer clusterizer{ClusterOptions{}};
    const auto original = warmUp(clusterizer, map);
    ASSERT_EQ(original.clusters.clusters.size(), 1U);
    EXPECT_EQ(original.statistics.clustered_node_count, map.nodes.size());
    map.nodes[88].pos.z = static_cast<float>(spacing * 0.22);
    const auto retained = clusterizer.update(map);
    EXPECT_EQ(retained.statistics.released_node_count, 0U);
    EXPECT_EQ(retained.statistics.clustered_node_count, map.nodes.size());
    EXPECT_EQ(clusterIds(retained), clusterIds(original));
    map.nodes[88].pos.z = static_cast<float>(spacing * 0.70);
    const auto released = clusterizer.update(map);
    EXPECT_EQ(released.statistics.released_node_count, 1U);
    EXPECT_EQ(released.statistics.clustered_node_count, map.nodes.size() - 1U);
  }
}

// 一本の異常に長いエッジによる距離許容幅の膨張防止。
TEST(PlaneClusterIncremental, LongBridgeDoesNotRelaxLocalRetention)
{
  auto map = makeSinglePlane();
  Clusterizer clusterizer{ClusterOptions{}};
  ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 1U);
  TopologicalNode far_node = map.nodes.front();
  far_node.id = static_cast<std::uint16_t>(map.nodes.size());
  far_node.pos.x = 100.0F;
  map.nodes.push_back(far_node);
  map.edges.insert(map.edges.end(), {14U, far_node.id});
  map.nodes[14].pos.z = 0.025F;
  const auto result = clusterizer.update(map);
  EXPECT_EQ(result.statistics.released_node_count, 1U);
  EXPECT_EQ(result.statistics.clustered_node_count, 35U);
}

// 点密度が異なる同数の面の統合判定。粗い側の間隔による小物体の吸収防止。
TEST(PlaneClusterIncremental, MixedDensityMergeUsesBothLocalScales)
{
  for (const double scale : {0.1, 1.0, 10.0}) {
    for (const double offset : {0.0, 0.04}) {
      SCOPED_TRACE(scale);
      SCOPED_TRACE(offset);
      TopologicalMap map;
      const double origin[3] = {0.0, 0.0, 0.0};
      const double small_origin[3] = {0.4 * scale, 0.4 * scale, offset * scale};
      const double axis_u[3] = {1.0, 0.0, 0.0};
      const double axis_v[3] = {0.0, 1.0, 0.0};
      appendGrid(map, 6U, 6U, scale, origin, axis_u, axis_v, TopologicalMap::SAFE_TERRAIN);
      const auto small_idx = appendGrid(
        map, 6U, 6U, 0.02 * scale, small_origin, axis_u, axis_v, TopologicalMap::UNKNOWN_OBJECT);
      Clusterizer clusterizer{ClusterOptions{}};
      ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
      map.edges.insert(map.edges.end(), {0U, static_cast<std::uint16_t>(small_idx),
        1U, static_cast<std::uint16_t>(small_idx + 5U)});
      const auto result = clusterizer.update(map);
      EXPECT_EQ(result.clusters.clusters.size(), offset == 0.0 ? 1U : 2U);
      EXPECT_EQ(result.statistics.clustered_node_count, 72U);
    }
  }
}

// 同じ法線を持つ隣接段差の分離。統合後の平面移動による段差隠蔽の防止。
TEST(PlaneClusterIncremental, ParallelStepStaysSeparateAcrossScale)
{
  for (const double scale : {0.1, 1.0, 10.0, 100.0}) {
    SCOPED_TRACE(scale);
    auto map = make_plane_patches(2U, 1U);
    for (auto &node : map.nodes) {
      node.pos.x *= scale;
      node.pos.y *= scale;
      node.pos.z = node.id < 36U ? 0.0F : static_cast<float>(0.01 * scale);
    }
    Clusterizer clusterizer{ClusterOptions{}};
    ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
    connect_plane_patches(map, 0U, 1U, true);
    for (std::size_t iter = 0U; iter < 8U; ++iter) {
      const auto result = clusterizer.update(map);
      EXPECT_EQ(result.clusters.clusters.size(), 2U);
      EXPECT_EQ(result.statistics.clustered_node_count, 72U);
    }
  }
}

// 既存平面への境界点の取り込みと、面から外れた追加点の拒否のスケール追従。
TEST(PlaneClusterIncremental, AbsorptionScalesWithLocalEdges)
{
  for (const double scale : {0.1, 1.0, 10.0, 100.0}) {
    SCOPED_TRACE(scale);
    auto map = makeSinglePlane();
    for (auto &node : map.nodes) {
      node.pos.x *= scale;
      node.pos.y *= scale;
    }
    Clusterizer clusterizer{ClusterOptions{}};
    ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 1U);
    TopologicalNode candidate = map.nodes.front();
    candidate.id = 36U;
    candidate.pos.x = static_cast<float>(0.30 * scale);
    candidate.pos.y = static_cast<float>(0.10 * scale);
    candidate.pos.z = static_cast<float>(0.005 * scale);
    map.nodes.push_back(candidate);
    map.edges.insert(map.edges.end(), {17U, 36U, 23U, 36U});
    const auto accepted = clusterizer.update(map);
    EXPECT_EQ(accepted.statistics.absorbed_node_count, 1U);
    EXPECT_EQ(accepted.statistics.clustered_node_count, 37U);
    candidate.id = 37U;
    candidate.pos.z = static_cast<float>(0.035 * scale);
    map.nodes.push_back(candidate);
    map.edges.insert(map.edges.end(), {17U, 37U, 23U, 37U});
    const auto rejected = clusterizer.update(map);
    EXPECT_EQ(rejected.statistics.absorbed_node_count, 0U);
    EXPECT_EQ(rejected.statistics.clustered_node_count, 37U);
  }
}

// 面内接続による未所属点の救済。単一接続・距離緩和・回転・スケールの確認。
TEST(PlaneClusterIncremental, coplanar_absorption_scales_and_preserves_id)
{
  for (const double scale : {0.1, 1.0, 10.0}) {
    for (const bool is_rotated : {false, true}) {
      for (const std::size_t num_contacts : {1U, 2U}) {
        SCOPED_TRACE(scale);
        SCOPED_TRACE(is_rotated);
        SCOPED_TRACE(num_contacts);
        auto map = makeSinglePlane();
        const auto transform = [scale, is_rotated](TopologicalNode &node) {
            node.pos.x *= scale;
            node.pos.y *= scale;
            node.pos.z *= scale;
            if (is_rotated) {
              std::swap(node.pos.x, node.pos.z);
              node.pos.x = -node.pos.x;
              std::swap(node.normal.x, node.normal.z);
              node.normal.x = -node.normal.x;
            }
          };
        TopologicalNode candidate = map.nodes.front();
        candidate.id = 36U;
        candidate.pos.x = 0.30F;
        candidate.pos.y = 0.10F;
        candidate.pos.z = num_contacts == 1U ? 0.005F : 0.010F;
        for (auto &node : map.nodes) {transform(node);}
        transform(candidate);
        ClusterOptions options;
        options.enable_multi_edge_dist_relaxation = false;
        Clusterizer clusterizer{options};
        const auto initial = warmUp(clusterizer, map);
        ASSERT_EQ(initial.clusters.clusters.size(), 1U);
        map.nodes.push_back(candidate);
        map.edges.insert(map.edges.end(), {17U, 36U});
        if (num_contacts == 2U) {map.edges.insert(map.edges.end(), {23U, 36U});}
        const auto accepted = clusterizer.update(map);
        EXPECT_EQ(accepted.statistics.num_coplanar_absorbed_nodes, 1U);
        EXPECT_EQ(accepted.statistics.absorbed_node_count, 1U);
        EXPECT_EQ(accepted.statistics.clustered_node_count, 37U);
        EXPECT_EQ(clusterIds(accepted), clusterIds(initial));
        const auto stable = clusterizer.update(map);
        EXPECT_EQ(stable.statistics.num_coplanar_absorbed_nodes, 0U);
        EXPECT_EQ(totalChanges(stable), 0U);
        EXPECT_EQ(stable.statistics.clustered_node_count, 37U);
      }
    }
  }
}

// OFF・接続要求・残差・法線・面外接続・橋長・角度設定・絶対上限の拒否条件。
TEST(PlaneClusterIncremental, coplanar_absorption_preserves_safety_gates)
{
  for (std::size_t case_idx = 0U; case_idx < 12U; ++case_idx) {
    SCOPED_TRACE(case_idx);
    auto map = makeSinglePlane();
    ClusterOptions options;
    options.enable_multi_edge_dist_relaxation = false;
    if (case_idx == 0U) {options.enable_coplanar_absorption = false;}
    if (case_idx == 1U) {options.connection_requirement = 3U;}
    if (case_idx == 6U) {options.max_absorption_edge_angle_deg_th = 0.0;}
    if (case_idx == 7U) {options.max_effective_spacing = 0.005;}
    if (case_idx == 10U) {options.max_absorption_edge_ratio_th = 0.5;}
    Clusterizer clusterizer{options};
    ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 1U);
    TopologicalNode candidate = map.nodes.front();
    candidate.id = 36U;
    candidate.pos.x = case_idx == 5U ? 1.0F : 0.30F;
    candidate.pos.y = 0.10F;
    candidate.pos.z = case_idx == 2U ? 0.035F :
      (case_idx == 9U || case_idx == 11U ? 0.010F : 0.005F);
    if (case_idx == 3U) {candidate.normal.x = 1.0F; candidate.normal.z = 0.0F;}
    map.nodes.push_back(candidate);
    if (case_idx != 8U) {map.edges.insert(map.edges.end(), {17U, 36U});}
    if (case_idx == 2U || case_idx == 11U) {map.edges.insert(map.edges.end(), {23U, 36U});}
    if (case_idx == 4U || case_idx == 11U) {
      auto protrusion = candidate;
      protrusion.id = 37U;
      protrusion.pos.z += 0.05F;
      map.nodes.push_back(protrusion);
      map.edges.insert(map.edges.end(), {36U, 37U});
    }
    for (std::size_t iter = 0U; iter < 4U; ++iter) {
      const auto rejected = clusterizer.update(map);
      EXPECT_EQ(rejected.statistics.num_coplanar_absorbed_nodes, 0U);
      EXPECT_EQ(rejected.statistics.absorbed_node_count, 0U);
      EXPECT_EQ(rejected.statistics.clustered_node_count, 36U);
    }
  }
}

// 複数の既存平面に触れる未所属点の救済禁止。平面境界の曖昧な所属の保護。
TEST(PlaneClusterIncremental, coplanar_absorption_rejects_competing_planes)
{
  auto map = make_plane_patches(2U, 1U);
  for (std::size_t idx = 36U; idx < map.nodes.size(); ++idx) {map.nodes[idx].pos.x += 1.0F;}
  ClusterOptions options;
  options.enable_multi_edge_dist_relaxation = false;
  Clusterizer clusterizer{options};
  ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
  auto candidate = map.nodes.front();
  candidate.id = 72U;
  candidate.pos.x = 0.30F;
  candidate.pos.y = 0.10F;
  candidate.pos.z = 0.005F;
  map.nodes.push_back(candidate);
  map.edges.insert(map.edges.end(), {17U, 72U, 48U, 72U});
  const auto rejected = clusterizer.update(map);
  EXPECT_EQ(rejected.statistics.num_coplanar_absorbed_nodes, 0U);
  EXPECT_EQ(rejected.statistics.clustered_node_count, 72U);
  EXPECT_EQ(rejected.clusters.clusters.size(), 2U);
}

// 既所属点から他平面への移動に対する救済の非適用。距離上限の保持。
TEST(PlaneClusterIncremental, coplanar_absorption_does_not_relax_migration)
{
  auto map = makeSinglePlane();
  const double origin[3] = {0.0, 0.0, 0.03};
  const double axis_u[3] = {1.0, 0.0, 0.0};
  const double axis_v[3] = {0.0, 1.0, 0.0};
  appendGrid(map, 6U, 6U, 0.05, origin, axis_u, axis_v, TopologicalMap::WALL);
  auto candidate = map.nodes.front();
  candidate.id = 72U;
  candidate.pos.x = 0.30F;
  candidate.pos.y = 0.10F;
  map.nodes.push_back(candidate);
  map.edges.insert(map.edges.end(), {17U, 72U, 23U, 72U});
  ClusterOptions options;
  options.enable_multi_edge_dist_relaxation = false;
  options.retention_residual_ratio = 0.60;
  options.migration_improvement_margin = 0.0;
  options.merge_connection_requirement = 1000U;
  Clusterizer clusterizer{options};
  const auto initial = warmUp(clusterizer, map);
  ASSERT_EQ(initial.clusters.clusters.size(), 2U);
  const auto owner = [](const ClusterResult &result) {
      for (const auto &cluster : result.clusters.clusters) {
        if (std::find(cluster.node_indices.begin(), cluster.node_indices.end(), 72U) !=
          cluster.node_indices.end()) {return cluster.id;}
      }
      return std::uint32_t{0U};
    };
  ASSERT_NE(owner(initial), 0U);
  map.nodes[72].pos.z = 0.020F;
  map.edges.resize(map.edges.size() - 4U);
  map.edges.insert(map.edges.end(), {53U, 72U, 59U, 72U});
  const auto retained = clusterizer.update(map);
  EXPECT_EQ(retained.statistics.released_node_count, 0U);
  EXPECT_EQ(retained.statistics.migrated_node_count, 0U);
  EXPECT_EQ(retained.statistics.num_coplanar_absorbed_nodes, 0U);
  EXPECT_EQ(owner(retained), owner(initial));
}

// 生成確認前の平面を用いた救済の禁止と、確認成立後の取り込み。
TEST(PlaneClusterIncremental, coplanar_absorption_requires_confirmed_plane)
{
  auto map = makeSinglePlane();
  ClusterOptions options;
  options.enable_multi_edge_dist_relaxation = false;
  Clusterizer clusterizer{options};
  clusterizer.update(map);
  auto candidate = map.nodes.front();
  candidate.id = 36U;
  candidate.pos.x = 0.30F;
  candidate.pos.y = 0.10F;
  map.nodes.push_back(candidate);
  map.edges.insert(map.edges.end(), {17U, 36U});
  for (std::size_t iter = 1U; iter < options.birth_confirm_frames; ++iter) {
    EXPECT_EQ(clusterizer.update(map).statistics.num_coplanar_absorbed_nodes, 0U);
  }
  const auto accepted = clusterizer.update(map);
  EXPECT_EQ(accepted.statistics.num_coplanar_absorbed_nodes, 1U);
  EXPECT_EQ(accepted.statistics.clustered_node_count, 37U);
}

// 絶対上限を明示した用途に限る距離制限。0の上限なし設定との区別。
TEST(PlaneClusterIncremental, ExplicitSpacingCapRemainsAvailable)
{
  for (const double cap : {0.0, 0.02}) {
    SCOPED_TRACE(cap);
    auto map = makeSinglePlane();
    for (auto &node : map.nodes) {
      node.pos.x *= 10.0F;
      node.pos.y *= 10.0F;
    }
    ClusterOptions options;
    options.max_effective_spacing = cap;
    Clusterizer clusterizer{options};
    ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 1U);
    map.nodes[14].pos.z = 0.03F;
    EXPECT_EQ(clusterizer.update(map).statistics.released_node_count, cap == 0.0 ? 0U : 1U);
  }
}

// 大平面内の別成分が接続された場合の一括統合。凸包包含だけによる誤吸収の防止。
TEST(PlaneClusterIncremental, InteriorPatchMergesOnlyWhenCoplanarAndConnected)
{
  for (const double offset : {0.0, 0.08}) {
    SCOPED_TRACE(offset);
    TopologicalMap map = make_plane_patches(3U, 3U);
    for (std::size_t idx = 4U * 36U; idx < 5U * 36U; ++idx) {
      map.nodes[idx].pos.z = static_cast<float>(offset);
    }
    for (const auto & pair : {std::pair{0U, 1U}, {1U, 2U}, {6U, 7U}, {7U, 8U}}) {
      connect_plane_patches(map, pair.first, pair.second, true);
    }
    for (const auto & pair : {std::pair{0U, 3U}, {3U, 6U}, {2U, 5U}, {5U, 8U}}) {
      connect_plane_patches(map, pair.first, pair.second, false);
    }
    Clusterizer clusterizer{ClusterOptions{}};
    ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
    connect_plane_patches(map, 3U, 4U, true);
    connect_plane_patches(map, 4U, 5U, true);
    const auto result = clusterizer.update(map);
    EXPECT_EQ(result.clusters.clusters.size(), offset == 0.0 ? 1U : 2U);
    EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
  }
}

// 小面の法線誤差の遠方外挿によらない内部パッチ統合。スケール・走査方向・段差の確認。
TEST(PlaneClusterIncremental, interior_patch_merge_uses_contact_region)
{
  for (const double scale : {0.1, 1.0, 10.0}) {
    for (const bool is_small_first : {false, true}) {
      for (const double offset : {0.0, 0.10}) {
        SCOPED_TRACE(scale);
        SCOPED_TRACE(is_small_first);
        SCOPED_TRACE(offset);
        TopologicalMap map;
        const double angle = 0.017453292519943295;
        const double large_origin[3] = {-10.0 * scale, -10.0 * scale, 0.0};
        const double small_origin[3] = {
          -std::cos(angle) * scale, -scale, (offset - std::sin(angle)) * scale};
        const double axis_u[3] = {1.0, 0.0, 0.0};
        const double axis_v[3] = {0.0, 1.0, 0.0};
        const double tilted_u[3] = {std::cos(angle), 0.0, std::sin(angle)};
        std::size_t large_idx = 0U, small_idx = 0U;
        for (const bool is_small : {is_small_first, !is_small_first}) {
          if (is_small) {
            small_idx = appendGrid(map, 5U, 5U, 0.5 * scale,
              small_origin, tilted_u, axis_v, TopologicalMap::WALL);
          } else {
            large_idx = appendGrid(map, 41U, 41U, 0.5 * scale,
              large_origin, axis_u, axis_v, TopologicalMap::WALL);
          }
        }
        ClusterOptions options;
        options.merge_connection_requirement = 2U;
        Clusterizer clusterizer{options};
        const auto disconnected = warmUp(clusterizer, map);
        ASSERT_EQ(disconnected.clusters.clusters.size(), 2U);
        for (std::size_t row = 0U; row < 5U; ++row) {
          for (const bool is_right : {false, true}) {
            map.edges.push_back(static_cast<std::uint16_t>(
              large_idx + (18U + row) * 41U + (is_right ? 23U : 17U)));
            map.edges.push_back(static_cast<std::uint16_t>(
              small_idx + row * 5U + (is_right ? 4U : 0U)));
          }
        }
        const auto result = clusterizer.update(map);
        const auto num_clusters = offset == 0.0 ? 1U : 2U;
        EXPECT_EQ(result.clusters.clusters.size(), num_clusters);
        EXPECT_EQ(result.statistics.merged_cluster_count, offset == 0.0 ? 1U : 0U);
        EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
        for (std::size_t iter = 0U; iter < 6U; ++iter) {
          const auto repeated = clusterizer.update(map);
          EXPECT_EQ(repeated.clusters.clusters.size(), num_clusters);
          EXPECT_EQ(repeated.statistics.clustered_node_count, map.nodes.size());
          EXPECT_EQ(clusterIds(repeated), clusterIds(result));
        }
      }
    }
  }
}

// 接触部が一致する傾斜面の分離。大面に埋もれる少数側の全体残差の検査。
TEST(PlaneClusterIncremental, contact_match_does_not_hide_tilted_small_plane)
{
  TopologicalMap map;
  const double large_origin[3] = {-10.0, -10.25, 0.0};
  const double small_origin[3] = {0.0, 0.0, 0.0};
  const double axis_u[3] = {1.0, 0.0, 0.0};
  const double axis_v[3] = {0.0, 1.0, 0.0};
  const double tilted_u[3] = {std::sqrt(0.75), 0.0, 0.5};
  appendGrid(map, 41U, 41U, 0.5, large_origin, axis_u, axis_v, TopologicalMap::WALL);
  const auto small_idx = appendGrid(
    map, 5U, 5U, 0.5, small_origin, tilted_u, axis_v, TopologicalMap::WALL);
  Clusterizer clusterizer{ClusterOptions{}};
  ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
  for (std::size_t row = 0U; row < 5U; ++row) {
    map.edges.push_back(static_cast<std::uint16_t>((21U + row) * 41U + 20U));
    map.edges.push_back(static_cast<std::uint16_t>(small_idx + row * 5U));
  }
  const auto result = clusterizer.update(map);
  EXPECT_EQ(result.clusters.clusters.size(), 2U);
  EXPECT_EQ(result.statistics.merged_cluster_count, 0U);
  EXPECT_GT(result.statistics.merge_smaller_side_rejected_pair_count, 0U);
  EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
}

// 同じ小断片対の連続適合による1本接続の統合。スケールとID安定性の確認。
TEST(PlaneClusterIncremental, fragment_merge_requires_consecutive_frames)
{
  for (const double scale : {0.1, 1.0, 10.0}) {
    SCOPED_TRACE(scale);
    TopologicalMap map;
    append_fragment_pair(map, scale);
    ClusterOptions options;
    options.merge_connection_requirement = 2U;
    Clusterizer clusterizer{options};
    ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
    map.edges.insert(map.edges.end(), {11U, 144U});
    for (std::size_t frame = 1U; frame < options.min_fragment_merge_frames; ++frame) {
      const auto pending = clusterizer.update(map);
      EXPECT_EQ(pending.clusters.clusters.size(), 2U);
      EXPECT_EQ(pending.statistics.num_fragment_pending_pairs, 1U);
    }
    const auto result = clusterizer.update(map);
    ASSERT_EQ(result.clusters.clusters.size(), 1U);
    EXPECT_EQ(result.statistics.num_fragment_merged_clusters, 1U);
    EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
    EXPECT_EQ(clusterIds(warmUp(clusterizer, map)), clusterIds(result));
  }
}

// 点数ではなく幾何と継続性による統合。大きい断片・同規模の平面・スケールの確認。
TEST(PlaneClusterIncremental, single_edge_merge_has_no_node_count_limit)
{
  for (const double scale : {0.1, 1.0, 10.0}) {
    for (const bool is_equal_size : {false, true}) {
      SCOPED_TRACE(scale);
      SCOPED_TRACE(is_equal_size);
      TopologicalMap map;
      const double spacing = 0.05 * scale;
      const double origin[3] = {0.0, 0.0, 0.0};
      const double next_origin[3] = {45.0 * spacing, 0.0, 0.0};
      const double axis_u[3] = {1.0, 0.0, 0.0};
      const double axis_v[3] = {0.0, 1.0, 0.0};
      appendGrid(map, 45U, 26U, spacing, origin, axis_u, axis_v, TopologicalMap::WALL);
      appendGrid(map, is_equal_size ? 45U : 11U, is_equal_size ? 26U : 9U,
        spacing, next_origin, axis_u, axis_v, TopologicalMap::WALL);
      ClusterOptions options;
      options.merge_connection_requirement = 2U;
      Clusterizer clusterizer{options};
      const auto initial = warmUp(clusterizer, map);
      ASSERT_EQ(initial.clusters.clusters.size(), 2U);
      map.edges.insert(map.edges.end(), {44U, 1170U});
      for (std::size_t frame = 1U; frame < options.min_fragment_merge_frames; ++frame) {
        const auto pending = clusterizer.update(map);
        EXPECT_EQ(pending.clusters.clusters.size(), 2U);
        EXPECT_EQ(pending.statistics.num_fragment_pending_pairs, 1U);
      }
      const auto merged = clusterizer.update(map);
      ASSERT_EQ(merged.clusters.clusters.size(), 1U);
      EXPECT_EQ(merged.statistics.num_fragment_merged_clusters, 1U);
      EXPECT_EQ(merged.statistics.clustered_node_count, map.nodes.size());
      const auto stable = warmUp(clusterizer, map);
      EXPECT_EQ(clusterIds(stable), clusterIds(merged));
      EXPECT_EQ(totalChanges(stable), 0U);
    }
  }
}

// 大平面に対する小さい側の段差・傾きの拒否。統合全体の低残差による隠蔽防止。
TEST(PlaneClusterIncremental, single_edge_merge_keeps_large_patch_geometry_guards)
{
  for (const double scale : {0.1, 1.0, 10.0}) {
    for (std::size_t case_idx = 0U; case_idx < 3U; ++case_idx) {
      SCOPED_TRACE(scale);
      SCOPED_TRACE(case_idx);
      TopologicalMap map;
      const double spacing = 0.05 * scale;
      const double origin[3] = {0.0, 0.0, 0.0};
      const double next_origin[3] = {45.0 * spacing, 0.0,
        (case_idx == 0U ? 0.12 : (case_idx == 1U ? 0.80 : 0.0)) * spacing};
      const double axis_u[3] = {1.0, 0.0, 0.0};
      const double axis_v[3] = {0.0, 1.0, 0.0};
      const double tilted_u[3] = {std::sqrt(0.75), 0.0, 0.5};
      appendGrid(map, 45U, 26U, spacing, origin, axis_u, axis_v, TopologicalMap::WALL);
      appendGrid(map, 11U, 9U, spacing, next_origin, case_idx == 2U ? tilted_u : axis_u,
        axis_v, TopologicalMap::WALL);
      ClusterOptions options;
      options.merge_connection_requirement = 2U;
      Clusterizer clusterizer{options};
      ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
      map.edges.insert(map.edges.end(), {44U, 1170U});
      for (std::size_t frame = 0U; frame < 8U; ++frame) {
        const auto rejected = clusterizer.update(map);
        EXPECT_EQ(rejected.clusters.clusters.size(), 2U);
        EXPECT_EQ(rejected.statistics.num_fragment_merged_clusters, 0U);
        EXPECT_EQ(rejected.statistics.clustered_node_count, map.nodes.size());
      }
    }
  }
}

// 長い橋・段差・緩い適合・接続なし・明示的な接続要求による救済対象の制限。
TEST(PlaneClusterIncremental, fragment_merge_preserves_safety_gates)
{
  for (const double scale : {0.1, 1.0, 10.0}) {
    for (std::size_t case_idx = 0U; case_idx < 6U; ++case_idx) {
      SCOPED_TRACE(scale);
      SCOPED_TRACE(case_idx);
      TopologicalMap map;
      append_fragment_pair(map, scale, case_idx == 0U ? 3.0 : 1.0,
        case_idx == 1U ? 0.8 : (case_idx == 2U ? 0.12 : 0.0));
      ClusterOptions options;
      options.merge_connection_requirement = case_idx == 5U ? 3U : 2U;
      options.enable_fragment_merge = case_idx != 4U;
      Clusterizer clusterizer{options};
      ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
      if (case_idx != 3U) {map.edges.insert(map.edges.end(), {11U, 144U});}
      for (std::size_t frame = 0U; frame < 8U; ++frame) {
        const auto result = clusterizer.update(map);
        EXPECT_EQ(result.clusters.clusters.size(), 2U);
        EXPECT_EQ(result.statistics.num_fragment_merged_clusters, 0U);
        EXPECT_EQ(result.statistics.clustered_node_count, map.nodes.size());
      }
    }
  }
}

// 接続消失・適合失敗・長い橋・reset・空入力による連続確認の初期化。
TEST(PlaneClusterIncremental, fragment_merge_restarts_after_interruption)
{
  for (std::size_t case_idx = 0U; case_idx < 5U; ++case_idx) {
    SCOPED_TRACE(case_idx);
    TopologicalMap disconnected;
    append_fragment_pair(disconnected);
    auto connected = disconnected;
    connected.edges.insert(connected.edges.end(), {11U, 144U});
    ClusterOptions options;
    options.merge_connection_requirement = 2U;
    Clusterizer clusterizer{options};
    ASSERT_EQ(warmUp(clusterizer, disconnected).clusters.clusters.size(), 2U);
    for (std::size_t frame = 0U; frame < 2U; ++frame) {
      EXPECT_EQ(clusterizer.update(connected).statistics.num_fragment_pending_pairs, 1U);
    }
    if (case_idx < 3U) {
      auto interrupted = case_idx == 0U ? disconnected : connected;
      for (std::size_t idx = 144U; idx < interrupted.nodes.size(); ++idx) {
        if (case_idx == 1U) {interrupted.nodes[idx].pos.z += 0.01F;}
        if (case_idx == 2U) {interrupted.nodes[idx].pos.x += 1.0F;}
      }
      EXPECT_EQ(clusterizer.update(interrupted).clusters.clusters.size(), 2U);
    } else {
      if (case_idx == 3U) {clusterizer.reset();}
      if (case_idx == 4U) {clusterizer.update(TopologicalMap{});}
      ASSERT_EQ(warmUp(clusterizer, disconnected).clusters.clusters.size(), 2U);
    }
    for (std::size_t frame = 0U; frame < 2U; ++frame) {
      const auto pending = clusterizer.update(connected);
      EXPECT_EQ(pending.clusters.clusters.size(), 2U);
      EXPECT_EQ(pending.statistics.num_fragment_pending_pairs, 1U);
    }
    EXPECT_EQ(clusterizer.update(connected).clusters.clusters.size(), 1U);
  }
}

// 別成分の削除による添字変更時も、同じ永続ID対の確認回数を保持。
TEST(PlaneClusterIncremental, fragment_merge_survives_cluster_compaction)
{
  auto map = makeSinglePlane();
  for (auto &node : map.nodes) {node.pos.x += 100.0F;}
  const auto base = append_fragment_pair(map);
  ClusterOptions options;
  options.merge_connection_requirement = 2U;
  options.weak_frame_allowance = 0U;
  Clusterizer clusterizer{options};
  ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 3U);
  map.edges.insert(map.edges.end(), {static_cast<std::uint16_t>(base + 11U),
    static_cast<std::uint16_t>(base + 144U)});
  EXPECT_EQ(clusterizer.update(map).statistics.num_fragment_pending_pairs, 1U);
  for (std::size_t idx = 0U; idx < base; ++idx) {
    map.nodes[idx].pos.x = std::numeric_limits<float>::quiet_NaN();
  }
  const auto compacted = clusterizer.update(map);
  EXPECT_EQ(compacted.statistics.removed_cluster_count, 1U);
  EXPECT_EQ(compacted.statistics.num_fragment_pending_pairs, 1U);
  const auto merged = clusterizer.update(map);
  EXPECT_EQ(merged.clusters.clusters.size(), 1U);
  EXPECT_EQ(merged.statistics.num_fragment_merged_clusters, 1U);
  EXPECT_EQ(merged.statistics.clustered_node_count, 169U);
}

// 救済OFF時の従来接続条件と、確認フレーム数設定の反映。
TEST(PlaneClusterIncremental, fragment_merge_options_and_regular_merge)
{
  for (const std::size_t num_frames : {1U, 5U}) {
    TopologicalMap map;
    append_fragment_pair(map);
    ClusterOptions options;
    options.merge_connection_requirement = 2U;
    options.min_fragment_merge_frames = num_frames;
    Clusterizer clusterizer{options};
    ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
    map.edges.insert(map.edges.end(), {11U, 144U});
    for (std::size_t frame = 1U; frame <= num_frames; ++frame) {
      EXPECT_EQ(clusterizer.update(map).clusters.clusters.size(), frame == num_frames ? 1U : 2U);
    }
  }
  TopologicalMap map;
  append_fragment_pair(map);
  ClusterOptions options;
  options.merge_connection_requirement = 2U;
  options.enable_fragment_merge = false;
  Clusterizer clusterizer{options};
  ASSERT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
  map.edges.insert(map.edges.end(), {11U, 144U});
  EXPECT_EQ(warmUp(clusterizer, map).clusters.clusters.size(), 2U);
  map.edges.insert(map.edges.end(), {23U, 149U});
  const auto result = clusterizer.update(map);
  EXPECT_EQ(result.clusters.clusters.size(), 1U);
  EXPECT_EQ(result.statistics.num_fragment_merged_clusters, 0U);
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
  // 全体RMS判定のみの検証用。各側の適合判定は別試験で確認。
  options.merge_smaller_side_residual_ratio = 1.0;
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
    options.max_normalized_cluster_residual = 1.0;
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

// 接続切れのみでは同一平面の所属を維持。最小生成数未満の断片も対象。
TEST(PlaneClusterIncremental, directional_split_preserves_coplanar_components)
{
  for (const auto min_nodes : {10U, 30U}) {
    TopologicalMap disconnected;
    append_fragment_pair(disconnected);
    auto connected = disconnected;
    connected.edges.insert(connected.edges.end(), {11U, 144U, 23U, 149U});
    ClusterOptions options;
    options.min_cluster_nodes = min_nodes;
    Clusterizer clusterizer{options};
    const auto original = warmUp(clusterizer, connected);
    ASSERT_EQ(original.clusters.clusters.size(), 1U);
    for (std::size_t frame = 0U; frame < 12U; ++frame) {
      const auto retained = clusterizer.update(disconnected);
      EXPECT_EQ(clusterIds(retained), clusterIds(original));
      EXPECT_EQ(retained.statistics.clustered_node_count, 169U);
      EXPECT_EQ(retained.statistics.split_cluster_count, 0U);
      EXPECT_EQ(retained.statistics.num_split_retained_components, 1U);
    }
    EXPECT_EQ(clusterIds(clusterizer.update(connected)), clusterIds(original));
  }
}

// 未所属点への面外接続による小領域の分割。拡大縮小・座標回転と連続確認の検証。
TEST(PlaneClusterIncremental, directional_split_confirms_external_edges)
{
  for (const double scale : {0.1, 1.0, 10.0}) {
    for (const bool is_rotated : {false, true}) {
      TopologicalMap disconnected;
      append_fragment_pair(disconnected, scale);
      auto connected = disconnected;
      connected.edges.insert(connected.edges.end(), {11U, 144U, 23U, 149U});
      append_conflict_edges(disconnected, 144U, 7U, scale);
      if (is_rotated) {
        for (auto *map : {&connected, &disconnected}) {
          for (auto &node : map->nodes) {
            std::swap(node.pos.y, node.pos.z);
            node.pos.z = -node.pos.z;
            std::swap(node.normal.y, node.normal.z);
            node.normal.z = -node.normal.z;
          }
        }
      }
      ClusterOptions options;
      options.min_cluster_nodes = 10U;
      Clusterizer clusterizer{options};
      ASSERT_EQ(warmUp(clusterizer, connected).clusters.clusters.size(), 1U);
      for (std::size_t frame = 0U; frame < options.split_confirm_frames; ++frame) {
        const auto pending = clusterizer.update(disconnected);
        EXPECT_EQ(pending.clusters.clusters.size(), 1U);
        EXPECT_EQ(pending.statistics.num_split_pending_components, 1U);
      }
      const auto split = clusterizer.update(disconnected);
      EXPECT_EQ(split.clusters.clusters.size(), 2U);
      EXPECT_EQ(split.statistics.split_cluster_count, 1U);
      EXPECT_EQ(split.statistics.clustered_node_count, 169U);
    }
  }
}

// 1本の異常接続・根拠割合不足・面内寄りの接続を分割根拠から除外。
TEST(PlaneClusterIncremental, directional_split_rejects_weak_evidence)
{
  for (std::size_t case_idx = 0U; case_idx < 5U; ++case_idx) {
    TopologicalMap disconnected;
    append_fragment_pair(disconnected);
    auto connected = disconnected;
    connected.edges.insert(connected.edges.end(), {11U, 144U, 23U, 149U});
    append_conflict_edges(disconnected, 144U, case_idx == 0U ? 1U : (case_idx == 1U ? 6U : 7U),
      1.0, case_idx == 2U ? 2.0 : (case_idx == 3U ? 1.0 : 0.0));
    ClusterOptions options;
    if (case_idx == 0U) {options.min_split_conflict_ratio_th = 0.0;}
    if (case_idx == 3U) {options.min_split_edge_angle_deg_th = 60.0;}
    if (case_idx == 4U) {options.min_split_conflict_nodes = 8U;}
    Clusterizer clusterizer{options};
    ASSERT_EQ(warmUp(clusterizer, connected).clusters.clusters.size(), 1U);
    for (std::size_t frame = 0U; frame < 8U; ++frame) {
      const auto retained = clusterizer.update(disconnected);
      EXPECT_EQ(retained.clusters.clusters.size(), 1U);
      EXPECT_EQ(retained.statistics.num_split_retained_components, 1U);
    }
  }
}

// 面外根拠が途切れた場合の確認回数初期化と、ノード配列並べ替え時のID追跡。
TEST(PlaneClusterIncremental, directional_split_tracks_evidence_by_id)
{
  TopologicalMap disconnected;
  append_fragment_pair(disconnected);
  auto connected = disconnected;
  connected.edges.insert(connected.edges.end(), {11U, 144U, 23U, 149U});
  auto conflict = disconnected;
  append_conflict_edges(conflict, 144U, 7U);
  ClusterOptions options;
  options.split_confirm_frames = 2U;
  Clusterizer clusterizer{options};
  ASSERT_EQ(warmUp(clusterizer, connected).clusters.clusters.size(), 1U);
  EXPECT_EQ(clusterizer.update(conflict).statistics.num_split_pending_components, 1U);
  EXPECT_EQ(clusterizer.update(disconnected).statistics.num_split_retained_components, 1U);
  EXPECT_EQ(clusterizer.update(conflict).statistics.num_split_pending_components, 1U);
  std::reverse(conflict.nodes.begin(), conflict.nodes.end());
  for (auto &idx : conflict.edges) {idx = static_cast<std::uint16_t>(conflict.nodes.size() - 1U - idx);}
  EXPECT_EQ(clusterizer.update(conflict).statistics.num_split_pending_components, 1U);
  EXPECT_EQ(clusterizer.update(conflict).statistics.split_cluster_count, 1U);
}

// 複数の非連結成分のうち、面外根拠のある成分だけを分割。
TEST(PlaneClusterIncremental, directional_split_keeps_component_evidence_independent)
{
  auto disconnected = make_plane_patches(3U, 1U);
  auto connected = disconnected;
  connect_plane_patches(connected, 0U, 1U, true);
  connect_plane_patches(connected, 1U, 2U, true);
  ClusterOptions options;
  Clusterizer clusterizer{options};
  ASSERT_EQ(warmUp(clusterizer, connected).clusters.clusters.size(), 1U);
  for (const auto base : {36U, 72U}) {
    auto conflict = disconnected;
    append_conflict_edges(conflict, base, 9U);
    for (std::size_t frame = 0U; frame < options.split_confirm_frames; ++frame) {
      const auto pending = clusterizer.update(conflict);
      EXPECT_EQ(pending.statistics.split_cluster_count, 0U);
      EXPECT_EQ(pending.statistics.num_split_pending_components, 1U);
    }
    const auto split = clusterizer.update(conflict);
    EXPECT_EQ(split.statistics.split_cluster_count, 1U);
    EXPECT_EQ(split.clusters.clusters.size(), base == 36U ? 2U : 3U);
    EXPECT_EQ(split.statistics.clustered_node_count, 108U);
  }
}

// 別平面クラスタへ伸びる面外接続も分断根拠の対象。
TEST(PlaneClusterIncremental, directional_split_accepts_edges_to_other_clusters)
{
  auto disconnected = make_plane_patches(3U, 1U);
  for (std::size_t idx = 72U; idx < disconnected.nodes.size(); ++idx) {
    disconnected.nodes[idx].pos.x -= 0.30F;
    disconnected.nodes[idx].pos.z = -0.05F;
    disconnected.edges.insert(disconnected.edges.end(), {
      static_cast<std::uint16_t>(idx - 36U), static_cast<std::uint16_t>(idx)});
  }
  auto connected = disconnected;
  connect_plane_patches(connected, 0U, 1U, true);
  ClusterOptions options;
  Clusterizer clusterizer{options};
  ASSERT_EQ(warmUp(clusterizer, connected).clusters.clusters.size(), 2U);
  for (std::size_t frame = 0U; frame < options.split_confirm_frames; ++frame) {
    EXPECT_EQ(clusterizer.update(disconnected).statistics.split_cluster_count, 0U);
  }
  const auto split = clusterizer.update(disconnected);
  EXPECT_EQ(split.statistics.split_cluster_count, 1U);
  EXPECT_EQ(split.clusters.clusters.size(), 3U);
  EXPECT_EQ(split.statistics.clustered_node_count, 108U);
}

// 全エッジ消失の短期保持、猶予切れと再接続による回復。
TEST(PlaneClusterIncremental, isolated_node_uses_bounded_previous_geometry)
{
  const auto connected = makeSinglePlane();
  auto isolated = connected;
  isolated.nodes[0].normal.z = 0.0;
  isolated.edges.clear();
  for (std::size_t idx = 0U; idx < connected.edges.size(); idx += 2U) {
    if (connected.edges[idx] != 0U && connected.edges[idx + 1U] != 0U) {
      isolated.edges.insert(isolated.edges.end(), {connected.edges[idx], connected.edges[idx + 1U]});
    }
  }
  ClusterOptions options;
  options.max_isolated_frames = 2U;
  Clusterizer clusterizer{options};
  const auto original = warmUp(clusterizer, connected);
  for (std::size_t frame = 0U; frame < options.max_isolated_frames; ++frame) {
    const auto retained = clusterizer.update(isolated);
    EXPECT_EQ(retained.statistics.clustered_node_count, 36U);
    EXPECT_EQ(retained.statistics.num_isolated_retained_nodes, 1U);
    EXPECT_EQ(clusterIds(retained), clusterIds(original));
  }
  EXPECT_EQ(clusterizer.update(isolated).statistics.clustered_node_count, 35U);
  EXPECT_EQ(warmUp(clusterizer, connected).statistics.clustered_node_count, 36U);
  EXPECT_EQ(clusterizer.update(isolated).statistics.clustered_node_count, 36U);
  clusterizer.reset();
  EXPECT_EQ(warmUp(clusterizer, isolated).statistics.clustered_node_count, 35U);
  warmUp(clusterizer, connected);
  clusterizer.update(TopologicalMap{});
  EXPECT_EQ(warmUp(clusterizer, isolated).statistics.clustered_node_count, 35U);
}

// 孤立猶予中も平面距離・法線の逸脱は解除。明示OFFと猶予0も即時解除。
TEST(PlaneClusterIncremental, isolated_node_preserves_geometry_guards)
{
  for (std::size_t case_idx = 0U; case_idx < 4U; ++case_idx) {
    const auto connected = makeSinglePlane();
    auto isolated = connected;
    isolated.edges.clear();
    for (std::size_t idx = 0U; idx < connected.edges.size(); idx += 2U) {
      if (connected.edges[idx] != 0U && connected.edges[idx + 1U] != 0U) {
        isolated.edges.insert(isolated.edges.end(), {connected.edges[idx], connected.edges[idx + 1U]});
      }
    }
    ClusterOptions options;
    options.normal_filter_alpha = 1.0;
    if (case_idx == 0U) {isolated.nodes[0].pos.z = 0.1F;}
    if (case_idx == 1U) {isolated.nodes[0].normal.x = 1.0; isolated.nodes[0].normal.z = 0.0;}
    if (case_idx == 2U) {options.enable_directional_split = false;}
    if (case_idx == 3U) {options.max_isolated_frames = 0U;}
    Clusterizer clusterizer{options};
    warmUp(clusterizer, connected);
    EXPECT_EQ(clusterizer.update(isolated).statistics.clustered_node_count, 35U);
  }
}

// 従来モードでの切断確認待ちと、再接続時のカウンタ初期化。
TEST(PlaneClusterIncremental, SplitConfirmationSurvivesNoSplitFastPath)
{
  ClusterOptions options;
  options.enable_directional_split = false;
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
