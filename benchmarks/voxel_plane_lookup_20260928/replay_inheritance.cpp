#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include "ais_gng/topological_plane/plane_cluster_parameters.hpp"
#include "../plane_consistency_20260924/output_fingerprint.hpp"
#include <fstream>
#include <map>
#include <string>
#include <ctime>
#include <iomanip>

namespace plane = fuzzrobo::topological_plane::incremental;
struct parameters {
  std::map<std::string, double> values;
  template<class T> T declare_parameter(const std::string &name, T value) {
    const auto it = values.find(name);
    return it == values.end() ? value : static_cast<T>(it->second);
  }
};

// 同一ビルド環境のnode_input保存列による世代付き固定入力。ROS・GNG学習なし。
int main(int argc, char **argv) {
  if (argc != 4) {return 2;}
  parameters params;
  std::ifstream config(std::string(argv[1])+"/plane_parameters.txt");
  std::string name; double value;
  while (config >> name >> value) {params.values[name] = value;}
  plane::Clusterizer clusterizer(plane::declareClusterOptions(params));
  std::ifstream input(argv[2], std::ios::binary);
  std::ofstream fingerprints(std::string(argv[3])+"/fingerprints.csv");
  std::ofstream timings(std::string(argv[3])+"/timings.csv");
  std::vector<plane::node_input> nodes;
  std::vector<std::uint16_t> edges;
  std::size_t frame = 0;
  double total_ms = 0;
  while (input.peek() != EOF) {
    uint32_t counts[2];
    input.read(reinterpret_cast<char *>(counts), sizeof(counts));
    if (!input || counts[0] > 65536 || counts[1] > 10000000) {return 3;}
    nodes.resize(counts[0]); edges.resize(counts[1]);
    input.read(reinterpret_cast<char *>(nodes.data()), nodes.size()*sizeof(nodes[0]));
    input.read(reinterpret_cast<char *>(edges.data()), edges.size()*sizeof(edges[0]));
    if (!input) {return 4;}
    plane::graph_view view{nodes.data(), nodes.size(), edges.data(), edges.size(),
      [](const void *data, std::size_t idx) {return static_cast<const plane::node_input *>(data)[idx];}};
    timespec start{}, end{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
    const std_msgs::msg::Header header;
    const auto result = clusterizer.update(view, header, frame);
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
    const double ms = (end.tv_sec-start.tv_sec)*1000.0+(end.tv_nsec-start.tv_nsec)*1.e-6;
    timings << frame << ',' << ms << '\n';
    if (frame >= 20) {total_ms += ms;}
    fingerprints << frame << ',' << output_fingerprint(result.clusters) << '\n';
    ++frame;
  }
  if (frame <= 20) {return 5;}
  std::ofstream metrics(std::string(argv[3])+"/metrics.json");
  metrics << std::setprecision(12) << "{\"plane_ms\":" << total_ms/(frame-20)
    << ",\"frames\":" << frame << "}\n";
}
