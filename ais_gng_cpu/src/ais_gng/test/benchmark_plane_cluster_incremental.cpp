#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

namespace
{

using ais_gng_msgs::msg::TopologicalMap;
using ais_gng_msgs::msg::TopologicalNode;
using fuzzrobo::topological_plane::incremental::ClusterOptions;
using fuzzrobo::topological_plane::incremental::Clusterizer;

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
  const double normal_length = std::sqrt(
    normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
  for (double &component : normal) {
    component /= normal_length;
  }

  for (std::size_t v = 0U; v < height; ++v) {
    for (std::size_t u = 0U; u < width; ++u) {
      TopologicalNode node;
      node.id = static_cast<std::uint16_t>(map.nodes.size());
      node.pos.x = static_cast<float>(
        origin[0] + spacing * (axis_u[0] * u + axis_v[0] * v));
      node.pos.y = static_cast<float>(
        origin[1] + spacing * (axis_u[1] * u + axis_v[1] * v));
      node.pos.z = static_cast<float>(
        origin[2] + spacing * (axis_u[2] * u + axis_v[2] * v));
      node.normal.x = static_cast<float>(normal[0]);
      node.normal.y = static_cast<float>(normal[1]);
      node.normal.z = static_cast<float>(normal[2]);
      node.label = label;
      map.nodes.push_back(node);
    }
  }

  const auto indexOf = [base, width](const std::size_t u, const std::size_t v) {
      return static_cast<std::uint16_t>(base + v * width + u);
    };
  for (std::size_t v = 0U; v < height; ++v) {
    for (std::size_t u = 0U; u < width; ++u) {
      if (u + 1U < width) {
        map.edges.push_back(indexOf(u, v));
        map.edges.push_back(indexOf(u + 1U, v));
      }
      if (v + 1U < height) {
        map.edges.push_back(indexOf(u, v));
        map.edges.push_back(indexOf(u, v + 1U));
      }
    }
  }
  return base;
}

TopologicalMap makeBenchmarkMap()
{
  TopologicalMap map;
  const double x_axis[3] = {1.0, 0.0, 0.0};
  const double y_axis[3] = {0.0, 1.0, 0.0};
  const double z_axis[3] = {0.0, 0.0, 1.0};
  const double floor_origin[3] = {-0.50, -0.40, 0.0};
  const double wall_origin[3] = {-0.30, 0.40, 0.02};
  const double top_origin[3] = {-0.08, -0.08, 0.12};
  const double side_origin[3] = {-0.08, -0.08, 0.02};
  appendGrid(
    map, 50U, 40U, 0.02, floor_origin, x_axis, y_axis,
    TopologicalMap::SAFE_TERRAIN);
  appendGrid(map, 30U, 20U, 0.02, wall_origin, x_axis, z_axis, TopologicalMap::WALL);
  appendGrid(
    map, 10U, 8U, 0.02, top_origin, x_axis, y_axis,
    TopologicalMap::UNKNOWN_OBJECT);
  appendGrid(
    map, 10U, 8U, 0.02, side_origin, x_axis, z_axis,
    TopologicalMap::UNKNOWN_OBJECT);
  return map;
}

double percentile(const std::vector<double> &sorted, const double ratio)
{
  const std::size_t index = static_cast<std::size_t>(
    ratio * static_cast<double>(sorted.size() - 1U));
  return sorted[index];
}

}  // namespace

int main()
{
  constexpr std::size_t kWarmupIterations = 20U;
  constexpr std::size_t kMeasuredIterations = 2000U;
  TopologicalMap map = makeBenchmarkMap();
  ClusterOptions options;
  options.birth_confirm_frames = 0U;
  Clusterizer clusterizer(options);

  for (std::size_t iteration = 0U; iteration < kWarmupIterations; ++iteration) {
    map.frame_number = static_cast<std::uint32_t>(iteration + 1U);
    clusterizer.update(map);
  }

  std::vector<double> samples_ms;
  samples_ms.reserve(kMeasuredIterations);
  std::size_t checksum = 0U;
  for (std::size_t iteration = 0U; iteration < kMeasuredIterations; ++iteration) {
    map.frame_number = static_cast<std::uint32_t>(kWarmupIterations + iteration + 1U);
    const auto started = std::chrono::steady_clock::now();
    const auto result = clusterizer.update(map);
    const auto completed = std::chrono::steady_clock::now();
    samples_ms.push_back(
      std::chrono::duration<double, std::milli>(completed - started).count());
    checksum += result.statistics.clustered_node_count + result.statistics.cluster_count;
  }

  std::sort(samples_ms.begin(), samples_ms.end());
  const double mean_ms = std::accumulate(samples_ms.begin(), samples_ms.end(), 0.0) /
    static_cast<double>(samples_ms.size());
  std::cout << std::fixed << std::setprecision(6)
            << "nodes=" << map.nodes.size()
            << " edges=" << map.edges.size() / 2U
            << " iterations=" << samples_ms.size()
            << " mean_ms=" << mean_ms
            << " p50_ms=" << percentile(samples_ms, 0.50)
            << " p95_ms=" << percentile(samples_ms, 0.95)
            << " p99_ms=" << percentile(samples_ms, 0.99)
            << " checksum=" << checksum << '\n';
  return checksum == 0U ? 1 : 0;
}
