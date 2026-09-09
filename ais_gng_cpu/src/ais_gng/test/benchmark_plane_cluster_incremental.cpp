#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include "ais_gng/topological_plane/plane_cluster_parameters.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
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

// 他プロセスによる待ち時間を除いた、呼び出しスレッドのCPU時間[ms]。
double thread_time_ms()
{
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) {
    throw std::runtime_error("cannot read thread CPU clock");
  }
  return 1000.0 * value.tv_sec + 1.0e-6 * value.tv_nsec;
}

// ROSノードを起動せず、CPU直結経路の既定パラメータを共通定義から取得。
struct DefaultParameters
{
  template<typename T>
  T declare_parameter(const std::string &, const T &value) {return value;}
};

TopologicalMap make_repeated_map(const std::size_t num_copies)
{
  TopologicalMap map;
  const TopologicalMap source = makeBenchmarkMap();
  for (std::size_t copy_idx = 0U; copy_idx < num_copies; ++copy_idx) {
    const std::size_t base_idx = map.nodes.size();
    for (auto node : source.nodes) {
      node.id = static_cast<std::uint16_t>(map.nodes.size());
      node.pos.x += static_cast<float>(2U * copy_idx);
      map.nodes.push_back(node);
    }
    for (const auto idx : source.edges) {
      map.edges.push_back(static_cast<std::uint16_t>(base_idx + idx));
    }
  }
  return map;
}

// 微小変形、木エッジ削除、面の分断、ノード削除・再配置を含む決定的な入力列。
TopologicalMap make_dynamic_map(const TopologicalMap &source, const std::size_t iter)
{
  TopologicalMap map = source;
  const auto phase = iter % 32U;
  if (phase >= 24U && phase < 28U) {
    map.nodes.pop_back();
  }
  for (auto &node : map.nodes) {
    node.pos.z += static_cast<float>(0.0001 * std::sin(0.1 * iter + 0.01 * node.id));
  }
  map.edges.clear();
  for (std::size_t idx = 0U; idx < source.edges.size(); idx += 2U) {
    const auto first = source.edges[idx];
    const auto second = source.edges[idx + 1U];
    const auto local_idx = first % 2760U;
    if (first >= map.nodes.size() || second >= map.nodes.size() ||
      (idx == 0U && phase % 2U == 0U) ||
      (phase >= 8U && phase < 16U && local_idx < 2000U &&
      local_idx % 50U == 24U && second == first + 1U))
    {
      continue;
    }
    map.edges.push_back(first);
    map.edges.push_back(second);
  }
  if (phase >= 16U && phase < 24U) {
    std::reverse(map.nodes.begin(), map.nodes.end());
    for (auto &idx : map.edges) {
      idx = static_cast<std::uint16_t>(map.nodes.size() - 1U - idx);
    }
  }
  return map;
}

// 分離した6x6パッチと、境界エッジ追加後の同一入力の組。
std::pair<TopologicalMap, TopologicalMap> make_patch_maps(
  const std::size_t width, const bool is_chain, const bool has_step)
{
  TopologicalMap separated;
  const double axis_u[3] = {1.0, 0.0, 0.0};
  const double axis_v[3] = {0.0, 1.0, 0.0};
  const std::size_t height = is_chain ? 1U : width;
  for (std::size_t row = 0U; row < height; ++row) {
    for (std::size_t column = 0U; column < width; ++column) {
      const double origin[3] = {
        0.12 * column, 0.12 * row, has_step ? 0.1 * ((row + column) % 2U) : 0.0};
      appendGrid(separated, 6U, 6U, 0.02, origin, axis_u, axis_v, TopologicalMap::WALL);
    }
  }
  TopologicalMap connected = separated;
  for (std::size_t row = 0U; row < height; ++row) {
    for (std::size_t column = 0U; column < width; ++column) {
      const std::size_t base_idx = (row * width + column) * 36U;
      for (std::size_t idx = 0U; idx < 6U; ++idx) {
        if (column + 1U < width) {
          connected.edges.push_back(static_cast<std::uint16_t>(base_idx + idx * 6U + 5U));
          connected.edges.push_back(static_cast<std::uint16_t>(base_idx + 36U + idx * 6U));
        }
        if (row + 1U < height) {
          connected.edges.push_back(static_cast<std::uint16_t>(base_idx + 30U + idx));
          connected.edges.push_back(static_cast<std::uint16_t>(base_idx + width * 36U + idx));
        }
      }
    }
  }
  return {std::move(separated), std::move(connected)};
}

}  // 無名名前空間

int main(int argc, char **argv)
{
  const std::string mode = argc > 1 ? argv[1] : "steady";
  const std::size_t size = argc > 2 ? std::stoul(argv[2]) : 1U;
  const std::size_t num_samples = argc > 3 ? std::stoul(argv[3]) : 2000U;
  if (size == 0U || num_samples == 0U ||
    (mode != "steady" && mode != "birth" && mode != "merge" &&
    mode != "reject" && mode != "chain" && mode != "dynamic"))
  {
    throw std::invalid_argument("mode: steady|birth|merge|reject|chain|dynamic, size/samples: positive");
  }
  const bool has_patches = mode == "merge" || mode == "reject" || mode == "chain";
  // uint16ノードIDに収まる入力規模。
  if ((!has_patches && size > 23U) || (mode == "chain" && size > 1820U) ||
    (has_patches && mode != "chain" && size > 42U))
  {
    throw std::invalid_argument("node count exceeds uint16 capacity");
  }
  TopologicalMap separated;
  TopologicalMap map;
  if (has_patches) {
    auto maps = make_patch_maps(size, mode == "chain", mode == "reject");
    separated = std::move(maps.first);
    map = std::move(maps.second);
  } else {
    map = make_repeated_map(size);
  }
  DefaultParameters parameters;
  const ClusterOptions options =
    fuzzrobo::topological_plane::incremental::declareClusterOptions(parameters, "", true);
  Clusterizer clusterizer(options);

  const bool is_reset_required = mode == "birth" || mode == "merge" || mode == "chain";
  // 初期化・パッチ生成・確認待ちは測定対象外。対象フレームのupdate全体だけを計測。
  const auto prepare = [&]() {
      clusterizer.reset();
      if (mode != "birth") {
        for (std::size_t iter = 0U; iter < 4U; ++iter) {
          clusterizer.update(has_patches ? separated : map);
        }
      }
    };
  prepare();
  const TopologicalMap source = mode == "dynamic" ? map : TopologicalMap{};
  std::ofstream trace;
  if (argc > 4) {
    trace.open(argv[4]);
    if (!trace) {
      throw std::runtime_error("cannot open output trace");
    }
    trace << std::setprecision(17);
  }

  std::vector<double> samples_ms;
  samples_ms.reserve(num_samples);
  std::vector<double> cpu_samples_ms;
  cpu_samples_ms.reserve(num_samples);
  std::size_t checksum = 0U;
  std::size_t num_clusters = 0U;
  std::size_t num_merged = 0U;
  std::size_t num_pairs = 0U;
  std::size_t num_assigned = 0U;
  std::size_t num_reused_connectivity = 0U;
  std::size_t num_scanned_connectivity_nodes = 0U;
  double min_planarity = 1.0;
  for (std::size_t iter = 0U; iter < num_samples + 20U; ++iter) {
    if (is_reset_required) {
      prepare();
    }
    if (mode == "dynamic") {
      map = make_dynamic_map(source, iter);
    }
    map.frame_number = static_cast<std::uint32_t>(iter + 1U);
    const auto cpu_started_ms = thread_time_ms();
    const auto started = std::chrono::steady_clock::now();
    const auto result = clusterizer.update(map);
    const auto completed = std::chrono::steady_clock::now();
    const auto cpu_completed_ms = thread_time_ms();
    // 全出力フィールドの変更前後比較用。書き出し時間はupdate計測の対象外。
    if (trace.is_open()) {
      trace << "---\n";
      ais_gng_msgs::msg::to_block_style_yaml(result.clusters, trace);
    }
    if (iter < 20U) {
      continue;
    }
    samples_ms.push_back(
      std::chrono::duration<double, std::milli>(completed - started).count());
    cpu_samples_ms.push_back(cpu_completed_ms - cpu_started_ms);
    checksum += result.statistics.clustered_node_count + result.statistics.cluster_count;
    num_clusters += result.statistics.cluster_count;
    num_merged += result.statistics.merged_cluster_count;
    num_pairs += result.statistics.merge_adjacent_pair_count;
    num_assigned += result.statistics.clustered_node_count;
    num_reused_connectivity += result.statistics.num_connectivity_reused_clusters;
    num_scanned_connectivity_nodes += result.statistics.num_connectivity_scanned_nodes;
    for (const auto &cluster : result.clusters.clusters) {
      min_planarity = std::min(min_planarity, static_cast<double>(cluster.planarity));
    }
  }

  std::sort(samples_ms.begin(), samples_ms.end());
  std::sort(cpu_samples_ms.begin(), cpu_samples_ms.end());
  const double mean_ms = std::accumulate(samples_ms.begin(), samples_ms.end(), 0.0) /
    static_cast<double>(samples_ms.size());
  const double cpu_mean_ms =
    std::accumulate(cpu_samples_ms.begin(), cpu_samples_ms.end(), 0.0) / cpu_samples_ms.size();
  std::cout << std::fixed << std::setprecision(6)
            << "mode=" << mode << " size=" << size << " nodes=" << map.nodes.size()
            << " edges=" << map.edges.size() / 2U
            << " iterations=" << samples_ms.size()
            << " mean_ms=" << mean_ms
            << " p50_ms=" << percentile(samples_ms, 0.50)
            << " p95_ms=" << percentile(samples_ms, 0.95)
            << " p99_ms=" << percentile(samples_ms, 0.99)
            << " max_ms=" << samples_ms.back()
            << " cpu_mean_ms=" << cpu_mean_ms
            << " cpu_p95_ms=" << percentile(cpu_samples_ms, 0.95)
            << " clusters=" << static_cast<double>(num_clusters) / num_samples
            << " merged=" << static_cast<double>(num_merged) / num_samples
            << " adjacent_pairs=" << static_cast<double>(num_pairs) / num_samples
            << " assigned_nodes=" << static_cast<double>(num_assigned) / num_samples
            << " reused_connectivity=" << static_cast<double>(num_reused_connectivity) / num_samples
            << " scanned_connectivity_nodes=" << static_cast<double>(num_scanned_connectivity_nodes) / num_samples
            << " min_planarity=" << min_planarity
            << " checksum=" << checksum << '\n';
  if (trace.is_open() && !trace) {
    throw std::runtime_error("cannot write output trace");
  }
  return mode != "birth" && checksum == 0U ? 1 : 0;
}
