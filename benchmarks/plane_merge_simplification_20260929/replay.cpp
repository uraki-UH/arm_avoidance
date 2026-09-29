#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include "ais_gng/topological_plane/plane_cluster_parameters.hpp"
#include "../plane_consistency_20260924/output_fingerprint.hpp"

#include <algorithm>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace plane = fuzzrobo::topological_plane::incremental;

struct parameters
{
  std::map<std::string, double> values;

  template<class value_type>
  value_type declare_parameter(const std::string &name, value_type value)
  {
    const auto entry = values.find(name);
    return entry == values.end() ? value : static_cast<value_type>(entry->second);
  }
};

// 同一ビルド環境のnode_input保存列による固定入力。ROS・GNG学習なし。
int main(int argc, char **argv)
{
  if (argc != 4) {
    std::cerr << "Usage: replay parameter_dir graphs.bin output_dir\n";
    return 2;
  }
  constexpr std::size_t num_warm_frames = 20U;
  constexpr std::size_t num_expected_frames = 60U;
  parameters params;
  std::ifstream config(std::string(argv[1]) + "/plane_parameters.txt");
  if (!config) {return 3;}
  std::string name;
  double value;
  while (config >> name >> value) {params.values[name] = value;}
  if (!config.eof()) {return 4;}
  plane::Clusterizer clusterizer(plane::declareClusterOptions(params));
  std::ifstream input(argv[2], std::ios::binary);
  std::ofstream fingerprints(std::string(argv[3]) + "/fingerprints.csv");
  std::ofstream timings(std::string(argv[3]) + "/timings.csv");
  std::ofstream statistics(std::string(argv[3]) + "/statistics.csv");
  std::ofstream metrics(std::string(argv[3]) + "/metrics.json");
  if (!input || !fingerprints || !timings || !statistics || !metrics) {return 5;}
  timings << std::setprecision(12);
  statistics << std::setprecision(12)
    << "frame_idx,num_valid_nodes,num_usable_nodes,num_clusters,num_clustered_nodes,"
    << "clustered_valid_node_ratio,num_output_clusters,num_output_nodes,"
    << "max_output_residual_ratio,num_merged_clusters,num_fragment_merged_clusters,"
    << "num_split_clusters,num_merge_adjacent_pairs,num_merge_insufficient_edge_pairs,"
    << "num_merge_invalid_fit_pairs,num_merge_planarity_rejected_pairs,"
    << "num_merge_absolute_residual_rejected_pairs,num_merge_smaller_side_rejected_pairs,"
    << "num_merge_residual_growth_rejected_pairs\n";
  std::vector<plane::node_input> nodes;
  std::vector<std::uint16_t> edges;
  std::size_t frame_idx = 0U;
  double total_ms = 0.0;
  double total_num_clusters = 0.0;
  double total_num_clustered_nodes = 0.0;
  double total_clustered_valid_node_ratio = 0.0;
  double total_num_output_clusters = 0.0;
  double total_num_output_nodes = 0.0;
  std::size_t total_num_merged_clusters = 0U;
  std::size_t total_num_fragment_merged_clusters = 0U;
  std::size_t total_num_split_clusters = 0U;
  std::size_t total_num_merge_absolute_residual_rejected_pairs = 0U;
  std::size_t total_num_merge_smaller_side_rejected_pairs = 0U;
#ifdef plane_merge_benchmark_before
  std::size_t total_num_merge_residual_growth_rejected_pairs = 0U;
#endif
  while (input.peek() != EOF) {
    std::uint32_t counts[2];
    input.read(reinterpret_cast<char *>(counts), sizeof(counts));
    if (!input || counts[0] > 65536U || counts[1] > 10000000U || counts[1] % 2U != 0U) {
      return 6;
    }
    nodes.resize(counts[0]);
    edges.resize(counts[1]);
    input.read(reinterpret_cast<char *>(nodes.data()), nodes.size() * sizeof(nodes[0]));
    input.read(reinterpret_cast<char *>(edges.data()), edges.size() * sizeof(edges[0]));
    if (!input) {return 7;}
    plane::graph_view view{nodes.data(), nodes.size(), edges.data(), edges.size(),
      [](const void *data, std::size_t idx) {
        return static_cast<const plane::node_input *>(data)[idx];
      }};
    timespec start{}, end{};
    const std_msgs::msg::Header header;
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start) != 0) {return 8;}
    const auto result = clusterizer.update(view, header, frame_idx);
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end) != 0) {return 8;}
    const double plane_ms = (end.tv_sec - start.tv_sec) * 1000.0 +
      (end.tv_nsec - start.tv_nsec) * 1.e-6;
    const auto &stats = result.statistics;
    const double clustered_valid_node_ratio = stats.valid_node_count == 0U ? 0.0 :
      static_cast<double>(stats.clustered_node_count) / stats.valid_node_count;
    std::size_t num_output_nodes = 0U;
    double max_output_residual_ratio = 0.0;
    for (const auto &cluster : result.clusters.clusters) {
      num_output_nodes += cluster.node_indices.size();
      max_output_residual_ratio = std::max(
        max_output_residual_ratio, static_cast<double>(cluster.residual_ratio));
    }
    timings << frame_idx << ',' << plane_ms << '\n';
    fingerprints << frame_idx << ',' << output_fingerprint(result.clusters) << '\n';
    statistics << frame_idx << ',' << stats.valid_node_count << ',' << stats.usable_node_count
      << ',' << stats.cluster_count << ',' << stats.clustered_node_count
      << ',' << clustered_valid_node_ratio << ',' << result.clusters.clusters.size()
      << ',' << num_output_nodes << ',';
    // 出力平面なしの残差最大値は未定義として空欄。
    if (!result.clusters.clusters.empty()) {statistics << max_output_residual_ratio;}
    statistics << ',' << stats.merged_cluster_count << ',' << stats.num_fragment_merged_clusters
      << ',' << stats.split_cluster_count << ',' << stats.merge_adjacent_pair_count
      << ',' << stats.merge_insufficient_edge_pair_count << ',' << stats.merge_invalid_fit_pair_count
      << ',' << stats.merge_planarity_rejected_pair_count
      << ',' << stats.merge_absolute_residual_rejected_pair_count
      << ',' << stats.merge_smaller_side_rejected_pair_count << ',';
#ifdef plane_merge_benchmark_before
    statistics << stats.merge_residual_growth_rejected_pair_count;
#endif
    statistics << '\n';
    if (frame_idx >= num_warm_frames) {
      total_ms += plane_ms;
      total_num_clusters += stats.cluster_count;
      total_num_clustered_nodes += stats.clustered_node_count;
      total_clustered_valid_node_ratio += clustered_valid_node_ratio;
      total_num_output_clusters += result.clusters.clusters.size();
      total_num_output_nodes += num_output_nodes;
      total_num_merged_clusters += stats.merged_cluster_count;
      total_num_fragment_merged_clusters += stats.num_fragment_merged_clusters;
      total_num_split_clusters += stats.split_cluster_count;
      total_num_merge_absolute_residual_rejected_pairs += stats.merge_absolute_residual_rejected_pair_count;
      total_num_merge_smaller_side_rejected_pairs += stats.merge_smaller_side_rejected_pair_count;
#ifdef plane_merge_benchmark_before
      total_num_merge_residual_growth_rejected_pairs += stats.merge_residual_growth_rejected_pair_count;
#endif
    }
    ++frame_idx;
  }
  if (input.bad() || frame_idx != num_expected_frames) {return 9;}
  const auto num_measured_frames = frame_idx - num_warm_frames;
  // 所属率は有効ノード数に対する割合のフレーム平均。正解ラベルとの精度評価なし。
  metrics << std::setprecision(12) << "{\"plane_ms\":" << total_ms / num_measured_frames
    << ",\"frames\":" << frame_idx << ",\"measured_frames\":" << num_measured_frames
    << ",\"mean_num_clusters\":" << total_num_clusters / num_measured_frames
    << ",\"mean_num_clustered_nodes\":" << total_num_clustered_nodes / num_measured_frames
    << ",\"mean_clustered_valid_node_ratio\":" << total_clustered_valid_node_ratio / num_measured_frames
    << ",\"mean_num_output_clusters\":" << total_num_output_clusters / num_measured_frames
    << ",\"mean_num_output_nodes\":" << total_num_output_nodes / num_measured_frames
    << ",\"num_merged_clusters\":" << total_num_merged_clusters
    << ",\"num_fragment_merged_clusters\":" << total_num_fragment_merged_clusters
    << ",\"num_split_clusters\":" << total_num_split_clusters
    << ",\"num_merge_absolute_residual_rejected_pairs\":" << total_num_merge_absolute_residual_rejected_pairs
    << ",\"num_merge_smaller_side_rejected_pairs\":" << total_num_merge_smaller_side_rejected_pairs;
#ifdef plane_merge_benchmark_before
  metrics << ",\"num_merge_residual_growth_rejected_pairs\":" << total_num_merge_residual_growth_rejected_pairs;
#endif
  metrics << "}\n";
  fingerprints.close();
  timings.close();
  statistics.close();
  metrics.close();
  return fingerprints && timings && statistics && metrics ? 0 : 10;
}
