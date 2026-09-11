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
  config.footprint_margin = 0.0;
  config.footprint_padding = 0.005;
  return config;
}

}  // namespace

int main()
{
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

  // 上限外・自由空間境界・未分類成分は付属対象外
  config = makeConfig();
  config.max_nonplane_graph_dist = 0.001;
  expect(TopGraspSurfaceEstimator(config).estimate(local_map, local_clusters)
    .candidates[0].attached_node_indices.empty(), "graph dist bound ignored");
  config = makeConfig();
  config.max_nonplane_depth = 0.005;
  expect(TopGraspSurfaceEstimator(config).estimate(local_map, local_clusters)
    .candidates[0].attached_node_indices.empty(), "depth bound ignored");
  config = makeConfig();
  config.nonplane_margin = 0.0;
  expect(TopGraspSurfaceEstimator(config).estimate(local_map, local_clusters)
    .candidates[0].attached_node_indices.empty(), "footprint bound ignored");
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
  config = makeConfig();
  config.max_nonplane_graph_dist = 0.04;
  expect(TopGraspSurfaceEstimator(config).estimate(chain_map, chain_clusters)
    .candidates[0].attached_node_indices == std::vector<std::uint32_t>{4},
    "cumulative graph dist ignored");
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
  config.max_nonplane_graph_dist = std::numeric_limits<double>::quiet_NaN();
  bool has_exception = false;
  try {
    TopGraspSurfaceEstimator invalid(config);
  } catch (const std::invalid_argument &) {
    has_exception = true;
  }
  expect(has_exception, "nonfinite config accepted");
  return 0;
}
