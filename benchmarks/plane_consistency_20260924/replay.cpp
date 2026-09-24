#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include "ais_gng/topological_plane/plane_cluster_parameters.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <stdexcept>

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
  if (argc != 2) {return 2;}
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) {return 2;}
  Parameters parameters;
  fuzzrobo::topological_plane::incremental::Clusterizer clusterizer(
    fuzzrobo::topological_plane::incremental::declareClusterOptions(parameters));
  std::cout << "frame,nodes,clusters,assigned,released,born,merged,split,cpu_ms,wall_ms\n";
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
              << s.split_cluster_count << ',' << elapsed_cpu_ms << ',' << wall_ms << '\n';
  }
}
