#include <candidate/top_grasp_surface_estimator.hpp>

#include <ais_gng_msgs/msg/planar_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace
{

using ais_gng_msgs::msg::PlanarCluster;
using ais_gng_msgs::msg::PlanarClusterArray;
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

PlanarCluster makeCluster(
  std::uint32_t id, const std::vector<std::uint32_t> &members,
  double x, double y, double z, const Eigen::Vector3d &normal)
{
  PlanarCluster cluster;
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
  config.higher_neighbor_z_tolerance = 0.01;
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
  PlanarClusterArray clusters;

  const auto fitting_side = addRectangle(map, 0.0, 0.0, 0.05, 0.03, 0.02);
  const auto fitting_top = addRectangle(map, 0.0, 0.0, 0.10, 0.03, 0.04);
  clusters.clusters.push_back(makeCluster(10U, fitting_side, 0.0, 0.0, 0.05, {
      1.0, 0.0, 0.0}));
  clusters.clusters.push_back(makeCluster(11U, fitting_top, 0.0, 0.0, 0.10, {
      0.0, 0.0, 1.0}));
  addEdge(map, fitting_side.front(), fitting_top.front());

  const auto wide_lower = addRectangle(map, 0.15, 0.0, 0.05, 0.03, 0.02);
  const auto wide_upper = addRectangle(map, 0.23, 0.0, 0.10, 0.03, 0.02);
  clusters.clusters.push_back(makeCluster(20U, wide_lower, 0.15, 0.0, 0.05, {
      1.0, 0.0, 0.0}));
  clusters.clusters.push_back(makeCluster(21U, wide_upper, 0.23, 0.0, 0.10, {
      0.0, 0.0, 1.0}));
  addEdge(map, wide_lower.front(), wide_upper.front());

  const auto floor = addRectangle(map, 0.0, 0.0, 0.0, 0.20, 0.20);
  clusters.clusters.push_back(makeCluster(30U, floor, 0.0, 0.0, 0.0, {
      0.0, 0.0, 1.0}));
  addEdge(map, floor.front(), fitting_side.front());

  const TopGraspSurfaceEstimator estimator(makeConfig());
  const auto result = estimator.estimate(map, clusters);
  expect(result.region_count == 5U, "region count mismatch");
  expect(result.adjacent_region_pair_count == 3U, "region adjacency mismatch");
  expect(result.candidate_group_count == 2U, "candidate group count mismatch");
  expect(result.rejected_seed_oversize_region == 1U, "floor was not excluded as a seed");
  expect(result.rejected_group_oversize_region == 1U, "wide upper closure was not rejected");
  expect(result.candidates.size() == 1U, "fitting upper closure was not isolated");

  const auto &candidate = result.candidates.front();
  expect(candidate.cluster_id == 11U, "highest adjacent cluster was not selected as apex");
  expect(candidate.cluster_ids.size() == 2U, "adjacent side was not included");
  expect(
    std::find(candidate.cluster_ids.begin(), candidate.cluster_ids.end(), 10U) !=
    candidate.cluster_ids.end(), "vertical side cluster was filtered by its normal");
  expect(candidate.extent_x <= 0.061 + 1.0e-9, "candidate x extent exceeds grasp area");
  expect(candidate.extent_y <= 0.074 + 1.0e-9, "candidate y extent exceeds grasp area");
  expect(
    std::abs(candidate.tcp_position.z() - 0.10) < 1.0e-6,
    "TCP was not placed at the highest adjacent region");
  const Eigen::Vector3d approach = candidate.tcp_orientation * Eigen::Vector3d::UnitZ();
  expect((approach + Eigen::Vector3d::UnitZ()).norm() < 1.0e-9, "approach is not downward");
  return 0;
}
