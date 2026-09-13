#include <candidate/top_grasp_surface_estimator.hpp>

#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{

using ais_gng_msgs::msg::PlaneCluster;
using ais_gng_msgs::msg::PlaneClusterArray;
using ais_gng_msgs::msg::TopologicalMap;
using grasping_system::candidate::TopGraspSurfaceConfig;
using grasping_system::candidate::TopGraspSurfaceEstimator;

void expect(bool condition, const char *message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::vector<std::uint32_t> addRectangle(
  TopologicalMap &map, double center_x, double center_y, double z,
  double extent_x, double extent_y)
{
  std::vector<std::uint32_t> indices;
  for (const double x : {-0.5 * extent_x, 0.5 * extent_x}) {
    for (const double y : {-0.5 * extent_y, 0.5 * extent_y}) {
      ais_gng_msgs::msg::TopologicalNode node;
      node.id = static_cast<std::uint16_t>(map.nodes.size());
      node.pos.x = static_cast<float>(center_x + x);
      node.pos.y = static_cast<float>(center_y + y);
      node.pos.z = static_cast<float>(z);
      map.nodes.push_back(node);
      indices.push_back(static_cast<std::uint32_t>(map.nodes.size() - 1U));
    }
  }
  return indices;
}

void addEdge(TopologicalMap &map, std::uint32_t first, std::uint32_t second)
{
  map.edges.push_back(static_cast<std::uint16_t>(first));
  map.edges.push_back(static_cast<std::uint16_t>(second));
}

PlaneCluster makeCluster(
  std::uint32_t id, const std::vector<std::uint32_t> &members,
  double x, double y, double z, const Eigen::Vector3d &normal)
{
  PlaneCluster cluster;
  cluster.id = id;
  cluster.node_indices = members;
  cluster.centroid.x = static_cast<float>(x);
  cluster.centroid.y = static_cast<float>(y);
  cluster.centroid.z = static_cast<float>(z);
  cluster.normal.x = normal.x();
  cluster.normal.y = normal.y();
  cluster.normal.z = normal.z();
  cluster.tangent_u.x = 1.0;
  cluster.tangent_v.y = 1.0;
  return cluster;
}

TopGraspSurfaceConfig makeConfig()
{
  TopGraspSurfaceConfig config;
  config.minimum_region_nodes = 4U;
  config.minimum_protrusion_distance = 0.01;
  config.grasp_size_x = 0.061;
  config.grasp_size_y = 0.074;
  return config;
}

}  // namespace

int main()
{
  {
    // 指定開口と観測幅の直接比較。旧余白による140 mm制限の除去
    TopologicalMap map;
    PlaneClusterArray clusters;
    const auto inside = addRectangle(map, 0, 0, 0.1, 0.145, 0.05);
    const auto outside = addRectangle(map, 1, 0, 0.1, 0.155, 0.05);
    clusters.clusters.push_back(makeCluster(1, inside, 0, 0, 0.1, {0, 0, 1}));
    clusters.clusters.push_back(makeCluster(2, outside, 1, 0, 0.1, {0, 0, 1}));
    TopGraspSurfaceConfig config;
    config.grasp_size_x = config.grasp_size_y = 0.15;
    config.enable_approach_check = false;
    const auto result = TopGraspSurfaceEstimator(config).estimate(map, clusters);
    expect(result.candidates.size() == 1 && result.candidates.front().cluster_id == 1,
      "145 mm must fit and 155 mm must exceed a 150 mm opening");
    expect(std::abs(result.candidates.front().extent_x - 0.145) < 1e-6,
      "reported extent must not include padding");
  }
  {
    // 複数平面候補の全構成ノード、各平面からの付属探索、候補外平面での停止
    TopologicalMap map;
    PlaneClusterArray clusters;
    const auto first = addRectangle(map, -0.025, 0, 0.10, 0.02, 0.02);
    const auto second = addRectangle(map, 0.025, 0, 0.10, 0.02, 0.02);
    const auto base = addRectangle(map, 0, 0, 0, 0.4, 0.4);
    const auto attached_first = addRectangle(map, -0.025, 0, 0.08, 0.02, 0.02);
    const auto attached_second = addRectangle(map, 0.025, 0, 0.08, 0.02, 0.02);
    clusters.clusters.push_back(makeCluster(1, first, -0.025, 0, 0.10, {0, 0, 1}));
    clusters.clusters.push_back(makeCluster(2, second, 0.025, 0, 0.10, {0, 0, 1}));
    clusters.clusters.push_back(makeCluster(3, base, 0, 0, 0, {0, 0, 1}));
    clusters.clusters[0].local_spacing = 0.1;
    clusters.clusters[1].local_spacing = 0.1;
    addEdge(map, first[0], attached_first[0]);
    addEdge(map, attached_first[0], second[0]);
    addEdge(map, second[0], attached_second[0]);
    addEdge(map, first[0], base[0]);
    for (std::size_t idx = 1; idx < 4; ++idx) {
      addEdge(map, attached_first[0], attached_first[idx]);
      addEdge(map, attached_second[0], attached_second[idx]);
    }
    TopGraspSurfaceConfig config;
    config.grasp_size_x = config.grasp_size_y = 0.15;
    config.enable_approach_check = false;
    config.enable_reference_plane_attachment = true;
    config.enable_plane_combinations = true;
    auto result = TopGraspSurfaceEstimator(config).estimate(map, clusters);
    const auto combined = std::find_if(result.candidates.begin(), result.candidates.end(),
      [](const auto &candidate) { return candidate.source_cluster_ids == std::vector<std::uint32_t>{1, 2}; });
    expect(combined != result.candidates.end(), "connected planes must form a combined candidate");
    expect(combined->node_indices.size() == 8, "combined candidate must contain all plane nodes");
    expect(combined->attached_node_indices.size() == 8, "attachments must start from every member plane");
    const auto single = std::find_if(result.candidates.begin(), result.candidates.end(),
      [](const auto &candidate) { return candidate.source_cluster_ids == std::vector<std::uint32_t>{1}; });
    expect(single != result.candidates.end(), "single candidates must remain available");
    expect(single->attached_node_indices.size() == 4, "nonmember plane must stop traversal");
    // 直接接続の第三平面からの外形拡張
    const auto third = addRectangle(map, 0, 0.035, 0.10, 0.02, 0.02);
    clusters.clusters.push_back(makeCluster(4, third, 0, 0.035, 0.10, {0, 0, 1}));
    clusters.clusters.back().local_spacing = 0.1;
    addEdge(map, second[0], third[0]);
    result = TopGraspSurfaceEstimator(config).estimate(map, clusters);
    expect(std::any_of(result.candidates.begin(), result.candidates.end(),
      [](const auto &candidate) { return candidate.source_cluster_ids == std::vector<std::uint32_t>{1, 2, 4} &&
        candidate.node_indices.size() == 12 && candidate.attached_node_indices.size() == 8; }),
      "three planes must form a complete combined candidate");
    clusters.clusters.pop_back();
    map.edges.resize(map.edges.size() - 2);
    config.enable_plane_combinations = false;
    result = TopGraspSurfaceEstimator(config).estimate(map, clusters);
    expect(result.candidates.size() == 2, "disabled combinations must retain only singles");
    config.enable_plane_combinations = true;
    config.grasp_size_x = config.grasp_size_y = 0.06;
    result = TopGraspSurfaceEstimator(config).estimate(map, clusters);
    expect(std::none_of(result.candidates.begin(), result.candidates.end(),
      [](const auto &candidate) { return candidate.source_cluster_ids.size() > 1; }),
      "oversized combinations must be rejected");
    config.grasp_size_x = config.grasp_size_y = 0.15;
    map.edges.clear();
    result = TopGraspSurfaceEstimator(config).estimate(map, clusters);
    expect(result.candidates.size() == 2, "disconnected planes must not form combinations");
  }
  {
    // 8平面の領域拡張。全組合せ列挙なしでの全ノード保持と重複排除
    TopologicalMap map;
    PlaneClusterArray clusters;
    for (std::uint32_t idx = 0; idx < 8; ++idx) {
      const double x = 0.008 * idx;
      const auto nodes = addRectangle(map, x, 0, 0.1, 0.006, 0.004);
      clusters.clusters.push_back(makeCluster(idx + 1, nodes, x, 0, 0.1, {0, 0, 1}));
      for (std::uint32_t previous = 0; previous < idx; ++previous) {
        addEdge(map, nodes[0], clusters.clusters[previous].node_indices[0]);
      }
    }
    TopGraspSurfaceConfig config;
    config.enable_plane_combinations = true;
    config.enable_reference_plane_attachment = true;
    config.enable_approach_check = false;
    config.grasp_size_x = config.grasp_size_y = 0.15;
    config.maximum_candidates = 1000;
    auto result = TopGraspSurfaceEstimator(config).estimate(map, clusters);
    expect(result.candidates.size() == 9, "eight singles and one grown region must remain");
    const auto full = std::find_if(result.candidates.begin(), result.candidates.end(),
      [](const auto &candidate) { return candidate.source_cluster_ids.size() == 8; });
    expect(full != result.candidates.end() && full->node_indices.size() == 32,
      "eight-plane candidate must retain all member nodes");
    expect(std::abs(full->extent_x - 0.062) < 1e-6,
      "projection bounds must expand in the seed plane orientation");
    config.grasp_size_x = config.grasp_size_y = 0.02;
    result = TopGraspSurfaceEstimator(config).estimate(map, clusters);
    expect(std::none_of(result.candidates.begin(), result.candidates.end(),
      [](const auto &candidate) { return candidate.source_cluster_ids.size() > 2; }),
      "size overflow must terminate further expansion");
  }
  {
    // 基準からの距離順による採用。左右を同時に含められない場合の近い側の優先
    TopologicalMap map;
    PlaneClusterArray clusters;
    for (const double x : {0.0, 0.012, -0.019}) {
      const auto nodes = addRectangle(map, x, 0, 0.1, 0.004, 0.002);
      clusters.clusters.push_back(makeCluster(clusters.clusters.size() + 1, nodes, x, 0, 0.1, {0, 0, 1}));
    }
    addEdge(map, 0, 4);
    addEdge(map, 0, 8);
    TopGraspSurfaceConfig config;
    config.enable_plane_combinations = true;
    config.enable_reference_plane_attachment = true;
    config.enable_approach_check = false;
    config.grasp_size_x = config.grasp_size_y = 0.03;
    const auto result = TopGraspSurfaceEstimator(config).estimate(map, clusters);
    expect(result.candidates.size() == 5, "greedy growth must retain singles and two feasible pairs");
    expect(std::any_of(result.candidates.begin(), result.candidates.end(),
      [](const auto &candidate) { return candidate.source_cluster_ids == std::vector<std::uint32_t>{1, 2}; }),
      "nearest adjacent plane must be accepted first");
    std::reverse(clusters.clusters.begin(), clusters.clusters.end());
    const auto reordered = TopGraspSurfaceEstimator(config).estimate(map, clusters);
    for (const auto &candidate : result.candidates) {
      expect(std::any_of(reordered.candidates.begin(), reordered.candidates.end(), [&](const auto &other) {
        return candidate.source_cluster_ids == other.source_cluster_ids;
      }), "cluster array order must not change grown membership");
    }
  }
  {
    // 候補平面を種とした非平面探索、土台近接帯での停止、外形超過の除外
    TopologicalMap reference_map;
    PlaneClusterArray reference_clusters;
    const auto seed = addRectangle(reference_map, 0, 0, 0.10, 0.02, 0.02);
    const auto base = addRectangle(reference_map, 0, 0, 0, 0.4, 0.4);
    const auto attached = addRectangle(reference_map, 0, 0, 0.06, 0.02, 0.02);
    reference_clusters.clusters.push_back(makeCluster(1, seed, 0, 0, 0.10, {0, 0, 1}));
    reference_clusters.clusters.front().local_spacing = 0.15;
    reference_clusters.clusters.push_back(makeCluster(2, base, 0, 0, 0, {0, 0, 1}));
    addEdge(reference_map, seed[0], base[0]);
    addEdge(reference_map, seed[0], attached[0]);
    for (std::size_t idx = 1; idx < attached.size(); ++idx)
      addEdge(reference_map, attached[0], attached[idx]);
    auto reference_config = makeConfig();
    reference_config.enable_reference_plane_attachment = true;
    reference_config.enable_approach_check = false;
    auto output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(!output.candidates.empty() && output.candidates.front().cluster_id == 1,
      "候補平面の保持");
    expect(output.candidates.front().attached_node_indices.size() == 4,
      "接続する非平面ノードの抽出");
    reference_clusters.clusters.push_back(makeCluster(3, attached, 0, 0, 0.06, {0, 0, 1}));
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(output.candidates.front().attached_node_indices.empty(), "他平面ノードの取り込み禁止");
    // 入口だけが別平面所属でも、その先の非平面ノードへの探索は禁止
    reference_clusters.clusters.back().node_indices = {attached[0]};
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(output.candidates.front().attached_node_indices.empty(), "他平面を経由した非平面探索の禁止");
    reference_clusters.clusters.pop_back();
    reference_map.nodes[attached[0]].pos.z = 0.005;
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(output.candidates.front().attached_node_indices.empty(), "土台近接帯での探索停止");
    reference_map.nodes[attached[0]].pos.z = 0.06;
    auto protruding_node = reference_map.nodes[attached[1]];
    protruding_node.pos.x = 0.06;
    reference_map.nodes.push_back(protruding_node);
    addEdge(reference_map, attached[0], reference_map.nodes.size() - 1);
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(output.rejected_attached_oversize > 0, "付属部分の開口超過による除外");
    reference_map.edges.clear();
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(output.candidates.front().attached_node_indices.empty(), "未接続領域の混入防止");
    // 非平面ノード列を介する参照面の検出。近接帯は参照探索で通過、対象抽出では停止
    auto bridge_node = reference_map.nodes[seed[0]];
    bridge_node.pos.z = 0.04;
    const auto bridge_idx = static_cast<std::uint32_t>(reference_map.nodes.size());
    reference_map.nodes.push_back(bridge_node);
    bridge_node.pos.z = 0.005;
    reference_map.nodes.push_back(bridge_node);
    addEdge(reference_map, seed[0], bridge_idx);
    addEdge(reference_map, bridge_idx, bridge_idx + 1);
    addEdge(reference_map, bridge_idx + 1, base[0]);
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(output.candidates.front().has_neighbor_plane_distance,
      "非平面ノード列を介した巨大平面の検出");
    expect(output.candidates.front().attached_node_indices == std::vector<std::uint32_t>{bridge_idx},
      "参照面から離れた接続ノードだけの抽出");
    // 内部平均をlocal_spacingより優先し、長い入口エッジだけの除外
    addEdge(reference_map, seed[0], seed[1]);
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(output.candidates.front().attached_node_indices.empty(), "内部平均より長い入口の除外");
    reference_map.nodes[bridge_idx].pos.z = 0.075;
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(output.candidates.front().attached_node_indices == std::vector<std::uint32_t>{bridge_idx},
      "内部平均より長く許容倍率内の入口の採用");
    reference_config.max_attachment_edge_length_ratio = 1.0;
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(output.candidates.front().attached_node_indices.empty(), "許容倍率変更の反映");
    reference_config.max_attachment_edge_length_ratio = 1.5;
    reference_map.edges.resize(reference_map.edges.size() - 2);
    reference_clusters.clusters.front().local_spacing = 0.0;
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(output.candidates.front().attached_node_indices.empty(), "基準長の未推定時の追加抑止");
    reference_clusters.clusters.front().local_spacing = 0.15;
    reference_map.nodes[bridge_idx].pos.z = 0.04;
    // 近接帯を飛び越えるエッジでも反対側ノードの取り込み禁止
    reference_map.nodes[bridge_idx + 1].pos.z = -0.02;
    for (const double normal_sign : {1.0, -1.0}) {
      reference_clusters.clusters[1].normal.z = normal_sign;
      output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
      expect(output.candidates.front().attached_node_indices == std::vector<std::uint32_t>{bridge_idx},
        "参照面の法線符号によらず候補側だけの抽出");
    }
    // 候補が参照面の下側にある場合の正方向反転
    for (auto &node : reference_map.nodes) node.pos.z = -node.pos.z;
    for (auto &cluster : reference_clusters.clusters) cluster.centroid.z = -cluster.centroid.z;
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    const auto seed_candidate = std::find_if(output.candidates.begin(), output.candidates.end(),
      [](const auto &candidate) { return candidate.cluster_id == 1; });
    expect(seed_candidate != output.candidates.end() &&
      seed_candidate->attached_node_indices == std::vector<std::uint32_t>{bridge_idx},
      "候補が下側でも候補側を正とした抽出");
    for (auto &node : reference_map.nodes) node.pos.z = -node.pos.z;
    for (auto &cluster : reference_clusters.clusters) cluster.centroid.z = -cluster.centroid.z;
    reference_clusters.clusters[1].normal.z = 1.0;
    reference_map.nodes[bridge_idx + 1].pos.z = 0.005;
    reference_map.edges.resize(reference_map.edges.size() - 2);
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(!output.candidates.front().has_neighbor_plane_distance &&
      output.candidates.front().attached_node_indices.empty(), "切断時の参照面の失効");
    addEdge(reference_map, bridge_idx + 1, base[0]);
    reference_map.nodes[bridge_idx + 1].boundary_evidence =
      ais_gng_msgs::msg::TopologicalNode::BOUNDARY_FREE_SPACE;
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(!output.candidates.front().has_neighbor_plane_distance, "自由空間境界をまたぐ接続の除外");
    // 150 mmを超える折返し経路と閉路でも、全接続ノードを一度ずつ抽出
    reference_map.nodes[bridge_idx + 1].boundary_evidence = 0;
    auto previous_idx = bridge_idx;
    for (std::uint32_t idx = 0; idx < 10; ++idx) {
      auto chain_node = reference_map.nodes[bridge_idx];
      chain_node.pos.z = idx % 2 == 0 ? 0.08 : 0.04;
      const auto next_idx = static_cast<std::uint32_t>(reference_map.nodes.size());
      reference_map.nodes.push_back(chain_node);
      addEdge(reference_map, previous_idx, next_idx);
      previous_idx = next_idx;
    }
    addEdge(reference_map, previous_idx, bridge_idx);
    output = TopGraspSurfaceEstimator(reference_config).estimate(reference_map, reference_clusters);
    expect(output.candidates.front().attached_node_indices.size() == 11,
      "累積距離による打ち切りと閉路による重複追加の防止");
  }
  TopologicalMap map;
  PlaneClusterArray clusters;

  const auto fitting_top = addRectangle(map, 0.0, 0.0, 0.10, 0.03, 0.04);
  const auto wall = addRectangle(map, -0.02, 0.0, 0.05, 0.20, 0.20);
  clusters.clusters.push_back(makeCluster(11U, fitting_top, 0.0, 0.0, 0.10, {
      0.0, 0.0, 1.0}));
  clusters.clusters.push_back(makeCluster(12U, wall, -0.02, 0.0, 0.05, {
      1.0, 0.0, 0.0}));
  addEdge(map, fitting_top.front(), wall.front());

  const auto flat_fragment = addRectangle(map, 0.20, 0.0, 0.0, 0.03, 0.02);
  const auto floor = addRectangle(map, 0.20, 0.0, 0.0, 0.20, 0.20);
  clusters.clusters.push_back(makeCluster(20U, flat_fragment, 0.20, 0.0, 0.0, {
      0.0, 0.0, 1.0}));
  clusters.clusters.push_back(makeCluster(21U, floor, 0.20, 0.0, 0.0, {
      0.0, 0.0, 1.0}));
  addEdge(map, flat_fragment.front(), floor.front());

  const auto isolated = addRectangle(map, 0.40, 0.0, 0.08, 0.03, 0.02);
  clusters.clusters.push_back(makeCluster(30U, isolated, 0.40, 0.0, 0.08, {
      0.0, 0.0, -1.0}));

  const TopGraspSurfaceEstimator estimator(makeConfig());
  const auto result = estimator.estimate(map, clusters);
  expect(result.region_count == 5U, "region count mismatch");
  expect(result.adjacent_region_pair_count == 2U, "region adjacency mismatch");
  expect(result.rejected_oversize_region == 1U, "large floor was not excluded");
  expect(result.rejected_surface_tilt == 1U, "wall normal was not excluded");
  expect(
    result.rejected_low_protrusion_region == 1U,
    "coplanar fragment was not rejected by protrusion distance");
  expect(result.candidates.size() == 2U, "expected wall-side and isolated candidates");

  const auto candidate_it = std::find_if(
    result.candidates.begin(), result.candidates.end(),
    [](const auto &candidate) {return candidate.cluster_id == 11U;});
  expect(candidate_it != result.candidates.end(), "wall-side protrusion was rejected");
  const auto &candidate = *candidate_it;
  expect(candidate.adjacent_region_count == 1U, "wall adjacency was not recorded");
  expect(candidate.has_neighbor_plane_distance, "wall plane distance was not computed");
  expect(
    std::abs(candidate.minimum_neighbor_plane_distance - 0.02) < 1.0e-6,
    "wall plane distance mismatch");
  expect(candidate.extent_x <= 0.061 + 1.0e-9, "candidate x extent exceeds grasp area");
  expect(candidate.extent_y <= 0.074 + 1.0e-9, "candidate y extent exceeds grasp area");
  expect(
    std::abs(candidate.tcp_position.z() - 0.10) < 1.0e-6,
    "TCP was not placed at the highest adjacent region");
  const Eigen::Vector3d approach = candidate.tcp_orientation * Eigen::Vector3d::UnitZ();
  expect((approach + Eigen::Vector3d::UnitZ()).norm() < 1.0e-9, "approach is not downward");

  const auto isolated_it = std::find_if(
    result.candidates.begin(), result.candidates.end(),
    [](const auto &surface) {return surface.cluster_id == 30U;});
  expect(isolated_it != result.candidates.end(), "isolated fitting region was rejected");
  expect(
    !isolated_it->has_neighbor_plane_distance,
    "isolated region unexpectedly has a neighbour distance");

  // 平面と付属部分の分離、開口包含、観測障害物による候補棄却
  TopologicalMap local_map;
  PlaneClusterArray local_clusters;
  const auto top = addRectangle(local_map, 0.0, 0.0, 0.10, 0.03, 0.02);
  local_clusters.clusters.push_back(makeCluster(1, top, 0, 0, 0.1, {0, 0, 1}));
  const auto base = estimator.estimate(local_map, local_clusters);
  expect(base.candidates.size() == 1, "base plane not accepted");
  ais_gng_msgs::msg::TopologicalNode attached;
  attached.id = 999;
  attached.nonplane_component_id = 0;
  attached.pos.x = 0.023;
  attached.pos.z = 0.08;
  local_map.nodes.push_back(attached);
  addEdge(local_map, top.back(), 4);
  auto evaluated = estimator.estimate(local_map, local_clusters);
  expect(evaluated.candidates.size() == 1, "fitting attachment rejected");
  expect(evaluated.candidates[0].attached_node_indices == std::vector<std::uint32_t>{4},
    "attachment must use array idx, not node id");
  expect(evaluated.candidates[0].attached_component_num == 1, "component 0 not counted");
  expect(evaluated.candidates[0].node_indices == top, "plane membership changed");
  expect((evaluated.candidates[0].tcp_position - base.candidates[0].tcp_position).norm() < 1e-9,
    "attachment moved seed TCP");
  expect(evaluated.candidates[0].extent_x == base.candidates[0].extent_x,
    "attachment changed plane OBB");
  expect(evaluated.candidates[0].target_extent_x > base.candidates[0].extent_x,
    "attachment missing from separate local bounds");

  local_map.nodes[4].pos.x = 0.04;
  evaluated = estimator.estimate(local_map, local_clusters);
  expect(evaluated.candidates.empty() && evaluated.rejected_attached_oversize == 1,
    "oversize attachment accepted");
  auto config = makeConfig();
  config.enable_nonplane_attachment = false;
  expect(TopGraspSurfaceEstimator(config).estimate(local_map, local_clusters).candidates.size() == 1,
    "attachment toggle has no effect");
  local_map.nodes[4].pos.x = 0.023;

  // 旧上下制限を超える付属ノードの採用。接近判定は独立して無効化
  config = makeConfig();
  config.enable_approach_check = false;
  for (const double height : {-0.1, 0.14}) {
    auto unrestricted_map = local_map;
    unrestricted_map.nodes[4].pos.z = height;
    const auto unrestricted_result = TopGraspSurfaceEstimator(config).estimate(
      unrestricted_map, local_clusters);
    expect(unrestricted_result.candidates.size() == 1 &&
      unrestricted_result.candidates.front().attached_node_indices.size() == 1,
      "上下位置による付属探索の打ち切りなし");
  }

  // 自由空間境界・未分類成分の付属対象からの除外
  local_map.nodes[4].boundary_evidence = attached.BOUNDARY_FREE_SPACE;
  expect(estimator.estimate(local_map, local_clusters).candidates[0].attached_node_indices.empty(),
    "free-space boundary admitted");
  local_map.nodes[4].boundary_evidence = 0;
  local_map.nodes[4].nonplane_component_id = attached.NONPLANE_COMPONENT_NONE;
  expect(estimator.estimate(local_map, local_clusters).candidates[0].attached_node_indices.empty(),
    "unclassified node admitted");
  local_map.nodes[4].nonplane_component_id = 0;

  // 複数エッジを経由した付属抽出と、世界座標の回転・並進に対する整合
  auto chain_map = local_map;
  auto chain_clusters = local_clusters;
  attached.pos.x = 0.02;
  attached.pos.z = 0.06;
  chain_map.nodes.push_back(attached);
  addEdge(chain_map, 4, 5);
  const auto chain_result = estimator.estimate(chain_map, chain_clusters);
  expect(chain_result.candidates[0].attached_node_indices.size() == 2,
    "multi-edge attachment missing");
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(
    0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  const Eigen::Vector3d translation(0.3, -0.2, 0.5);
  for (auto &node : chain_map.nodes) {
    const Eigen::Vector3d p = rotation * Eigen::Vector3d(node.pos.x, node.pos.y, node.pos.z) +
      translation;
    node.pos.x = p.x();
    node.pos.y = p.y();
    node.pos.z = p.z();
  }
  auto &plane = chain_clusters.clusters[0];
  const Eigen::Vector3d center = rotation * Eigen::Vector3d(0, 0, 0.1) + translation;
  const Eigen::Vector3d up = rotation * Eigen::Vector3d::UnitZ();
  plane.centroid.x = center.x();
  plane.centroid.y = center.y();
  plane.centroid.z = center.z();
  plane.normal.x = up.x();
  plane.normal.y = up.y();
  plane.normal.z = up.z();
  config = makeConfig();
  config.up_axis = up;
  const auto rotated = TopGraspSurfaceEstimator(config).estimate(chain_map, chain_clusters);
  expect(rotated.candidates.size() == 1 &&
    rotated.candidates[0].attached_node_indices.size() == 2, "rotated attachment changed");
  expect((rotated.candidates[0].tcp_position - center).norm() < 1e-6,
    "rotated TCP changed");

  // 同一IDでも非接続ノードは不採用。別平面への橋渡しは成分全体を付属対象外
  auto bridged_map = local_map;
  auto bridged_clusters = local_clusters;
  bridged_map.nodes.push_back(attached);
  expect(estimator.estimate(bridged_map, bridged_clusters).candidates[0]
    .attached_node_indices.size() == 1, "disconnected same-ID node attached");
  const auto other = addRectangle(bridged_map, 0.3, 0, 0, 0.03, 0.02);
  bridged_clusters.clusters.push_back(makeCluster(2, other, 0.3, 0, 0, {0, 0, 1}));
  addEdge(bridged_map, 4, other.front());
  expect(estimator.estimate(bridged_map, bridged_clusters).candidates[0]
    .attached_node_indices.empty(), "multi-plane component attached");

  // 接続なし・ラベルなしでも上方ノードは障害物。付属探索OFFでも同じ判定
  local_map.edges.clear();
  local_map.nodes[4].pos.x = 0.0;
  local_map.nodes[4].pos.z = 0.14;
  local_map.nodes[4].nonplane_component_id = attached.NONPLANE_COMPONENT_NONE;
  evaluated = estimator.estimate(local_map, local_clusters);
  expect(evaluated.candidates.empty() && evaluated.rejected_approach_obstacle == 1,
    "unconnected overhead obstacle accepted");
  local_map.nodes[4].pos.z = 0.105;
  expect(estimator.estimate(local_map, local_clusters).rejected_approach_obstacle == 1,
    "平面最高位置から5 mm上の接近障害物検出");
  local_map.nodes[4].pos.z = 0.14;
  config = makeConfig();
  config.enable_nonplane_attachment = false;
  expect(TopGraspSurfaceEstimator(config).estimate(local_map, local_clusters).candidates.empty(),
    "attachment toggle disabled obstacle check");
  config.enable_approach_check = false;
  expect(TopGraspSurfaceEstimator(config).estimate(local_map, local_clusters).candidates.size() == 1,
    "approach toggle has no effect");
  local_map.nodes[4].pos.z = 0.3;
  expect(estimator.estimate(local_map, local_clusters).candidates.size() == 1,
    "distant overhead point blocked finite approach");
  local_map.nodes[4].pos.z = 0.14;
  local_map.nodes[4].pos.x = 0.3;
  expect(estimator.estimate(local_map, local_clusters).candidates.size() == 1,
    "lateral distant point blocked approach");

  // 壁面・不正法線・入力フレーム不一致の除外
  local_clusters.clusters[0].normal.z = 0;
  local_clusters.clusters[0].normal.x = 1;
  expect(estimator.estimate(local_map, local_clusters).rejected_surface_tilt == 1,
    "small wall accepted");
  local_clusters.clusters[0].normal.x = 0;
  expect(estimator.estimate(local_map, local_clusters).rejected_invalid_region == 1,
    "zero normal accepted");
  local_clusters.clusters[0].normal.z = -1;
  expect(estimator.estimate(local_map, local_clusters).candidates.size() == 1,
    "normal sign flip changed eligibility");
  local_map.frame_number = 1;
  expect(estimator.estimate(local_map, local_clusters).candidates.empty(), "mixed frame numbers");
  local_clusters.frame_number = 1;
  local_map.header.frame_id = "other";
  expect(estimator.estimate(local_map, local_clusters).candidates.empty(), "mixed frames");

  config = makeConfig();
  config.max_attachment_edge_length_ratio = std::numeric_limits<double>::quiet_NaN();
  bool has_exception = false;
  try {
    TopGraspSurfaceEstimator invalid(config);
  } catch (const std::invalid_argument &) {
    has_exception = true;
  }
  expect(has_exception, "nonfinite config accepted");
  return 0;
}
