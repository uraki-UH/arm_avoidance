#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include "ais_gng/topological_plane/plane_cluster_parameters.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string_view>

// ROS起動なしの共有既定値と、共通YAMLと同じ種順序。
struct Parameters
{
  template<typename T>
  T declare_parameter(const std::string &, const T &value) {return value;}
};

template<typename T>
void read_value(std::istream &input, T &value)
{
  input.read(reinterpret_cast<char *>(&value), sizeof(value));
  if (!input) {throw std::runtime_error("truncated capture");}
}

double cpu_ms()
{
  timespec value{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value);
  return value.tv_sec * 1000.0 + value.tv_nsec * 1.e-6;
}

int main(int argc, char **argv)
{
  const std::string_view mode = argc == 3 ? argv[2] : "";
  if (argc < 2 || argc > 3 ||
    (!mode.empty() && mode != "--diagnose" && mode != "--disable-fragment-merge" &&
    mode != "--disable-directional-split" && mode != "--disable-coplanar-absorption")) {return 2;}
  const bool enable_diagnostics = mode == "--diagnose";
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) {return 2;}
  Parameters parameters;
  auto options = fuzzrobo::topological_plane::incremental::declareClusterOptions(parameters);
  if (mode == "--disable-fragment-merge") {options.enable_fragment_merge = false;}
  if (mode == "--disable-directional-split") {options.enable_directional_split = false;}
  if (mode == "--disable-coplanar-absorption") {options.enable_coplanar_absorption = false;}
  fuzzrobo::topological_plane::incremental::Clusterizer clusterizer(options);
  std::cout << "frame,nodes,clusters,assigned,released,born,merged,split,cpu_ms,wall_ms";
  if (enable_diagnostics) {
    std::cout << ",adjacent_pairs,insufficient_edges,invalid_fit,plane_extent,absolute_residual,"
      "residual_growth,side_residual,fragment_merged,fragment_pending,split_retained,split_pending,isolated_retained,coplanar_absorbed";
  }
  std::cout << '\n';
  std::size_t frame = 0U;
  while (input.peek() != EOF) {
    ais_gng_msgs::msg::TopologicalMap map;
    std::uint32_t num_nodes, num_edges;
    read_value(input, num_nodes);
    read_value(input, num_edges);
    if (num_nodes > 65536U || num_edges > 10000000U) {return 3;}
    map.nodes.resize(num_nodes);
    map.edges.resize(num_edges);
    for (auto &node : map.nodes) {
      read_value(input, node.id);
      read_value(input, node.label);
      read_value(input, node.pos.x);
      read_value(input, node.pos.y);
      read_value(input, node.pos.z);
      // 保存形式はfloat32、ROS法線の型とは独立。
      float x, y, z;
      read_value(input, x); read_value(input, y); read_value(input, z);
      node.normal.x = x; node.normal.y = y; node.normal.z = z;
      read_value(input, node.rho);
    }
    input.read(reinterpret_cast<char *>(map.edges.data()), num_edges * sizeof(std::uint16_t));
    if (!input) {return 3;}
    const auto start = std::chrono::steady_clock::now();
    const double start_cpu_ms = cpu_ms();
    const auto result = clusterizer.update(map);
    const double elapsed_cpu_ms = cpu_ms() - start_cpu_ms;
    const double wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    const auto &s = result.statistics;
    std::cout << frame++ << ',' << num_nodes << ',' << s.cluster_count << ','
              << s.clustered_node_count << ',' << s.released_node_count << ','
              << s.born_cluster_count << ',' << s.merged_cluster_count << ','
              << s.split_cluster_count << ',' << elapsed_cpu_ms << ',' << wall_ms;
    if (enable_diagnostics) {
      std::cout << ',' << s.merge_adjacent_pair_count << ',' << s.merge_insufficient_edge_pair_count
                << ',' << s.merge_invalid_fit_pair_count << ',' << s.merge_planarity_rejected_pair_count
                << ',' << s.merge_absolute_residual_rejected_pair_count
                << ',' << s.merge_residual_growth_rejected_pair_count
                << ',' << s.merge_smaller_side_rejected_pair_count
                << ',' << s.num_fragment_merged_clusters << ',' << s.num_fragment_pending_pairs
                << ',' << s.num_split_retained_components << ',' << s.num_split_pending_components
                << ',' << s.num_isolated_retained_nodes << ',' << s.num_coplanar_absorbed_nodes;
    }
    std::cout << '\n';
    // 最終フレームの観測点・接続・所属の診断用出力。本番の統合条件・ROS出力への変更なし。
    if (enable_diagnostics && input.peek() == EOF) {
      std::cerr << std::setprecision(9) << "{\"clusters\":[";
      bool is_first = true;
      for (const auto &cluster : result.clusters.clusters) {
        if (!is_first) {std::cerr << ',';}
        is_first = false;
        std::cerr << "{\"id\":" << cluster.id << ",\"spacing\":" << cluster.local_spacing
                  << ",\"nodes\":[";
        for (std::size_t idx = 0U; idx < cluster.node_indices.size(); ++idx) {
          if (idx != 0U) {std::cerr << ',';}
          std::cerr << cluster.node_indices[idx];
        }
        std::cerr << "]}";
      }
      std::cerr << "],\"nodes\":[";
      for (std::size_t idx = 0U; idx < map.nodes.size(); ++idx) {
        if (idx != 0U) {std::cerr << ',';}
        const auto &point = map.nodes[idx].pos;
        std::cerr << '[' << point.x << ',' << point.y << ',' << point.z << ']';
      }
      std::cerr << "],\"edges\":[";
      for (std::size_t idx = 0U; idx < map.edges.size(); ++idx) {
        if (idx != 0U) {std::cerr << ',';}
        std::cerr << map.edges[idx];
      }
      std::cerr << "]}\n";
    }
  }
}
