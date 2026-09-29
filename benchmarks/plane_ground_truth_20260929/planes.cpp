#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include "ais_gng/topological_plane/plane_cluster_parameters.hpp"
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <string>
#include <vector>

namespace plane = fuzzrobo::topological_plane::incremental;
struct parameters {
  std::map<std::string, double> values;
  template<class value_type> value_type declare_parameter(const std::string &name, value_type value) {
    const auto entry = values.find(name);
    return entry == values.end() ? value : static_cast<value_type>(entry->second);
  }
};

// 正解ラベルを読まない平面計算。学習済みグラフだけを変更前後へ共通入力。
int main(int argc, char **argv) {
  if (argc != 4) {return 2;}
  std::ifstream input(argv[1], std::ios::binary), config(argv[2]);
  std::ofstream output(std::string(argv[3]) + "/assignments.csv");
  std::ofstream timings(std::string(argv[3]) + "/plane_timings.csv");
  if (!input || !config || !output || !timings) {return 3;}
  parameters params;
  std::string name; double value;
  while (config >> name >> value) {params.values[name] = value;}
  if (!config.eof()) {return 4;}
  plane::Clusterizer clusterizer(plane::declareClusterOptions(params));
  output << std::setprecision(9) << "frame_idx,node_id,node_frame,x,y,z,pred_cluster\n";
  timings << "frame_idx,plane_ms,num_clusters\n";
  std::vector<plane::node_input> nodes;
  std::vector<std::uint16_t> edges;
  std::uint32_t frame_idx = 0;
  while (input.peek() != EOF) {
    std::uint32_t counts[2];
    input.read(reinterpret_cast<char *>(counts), sizeof(counts));
    if (!input || counts[0] > 65536U || counts[1] > 10000000U || counts[1] % 2U) {return 5;}
    nodes.resize(counts[0]); edges.resize(counts[1]);
    input.read(reinterpret_cast<char *>(nodes.data()), nodes.size() * sizeof(nodes[0]));
    input.read(reinterpret_cast<char *>(edges.data()), edges.size() * sizeof(edges[0]));
    if (!input) {return 6;}
    const plane::graph_view view{nodes.data(), nodes.size(), edges.data(), edges.size(),
      [](const void *data, std::size_t idx) {return static_cast<const plane::node_input *>(data)[idx];}};
    timespec start{}, end{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
    const auto result = clusterizer.update(view, std_msgs::msg::Header{}, frame_idx);
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
    const double elapsed_ms = (end.tv_sec-start.tv_sec)*1000.0 + (end.tv_nsec-start.tv_nsec)*1.e-6;
    timings << frame_idx << ',' << elapsed_ms << ',' << result.clusters.clusters.size() << '\n';
    std::vector<std::int64_t> predictions(nodes.size(), -1);
    for (const auto &cluster : result.clusters.clusters) {
      for (const auto idx : cluster.node_indices) {
        if (idx >= nodes.size() || predictions[idx] != -1) {return 7;}
        predictions[idx] = cluster.id;
      }
    }
    for (std::size_t idx = 0; idx < nodes.size(); ++idx) {
      const auto &node = nodes[idx];
      output << frame_idx << ',' << node.id << ',' << node.frame << ',' << node.pos.x << ','
        << node.pos.y << ',' << node.pos.z << ',' << predictions[idx] << '\n';
    }
    ++frame_idx;
  }
  output.close(); timings.close();
  return frame_idx > 0 && !input.bad() && output && timings ? 0 : 8;
}
