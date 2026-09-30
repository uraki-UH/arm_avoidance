// 保存済み関節角度による探索単体の比較。学習全体の計測対象外。
#include <SpatialTree/MovingBSPTree.hpp>
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

struct saved_node { int id; Eigen::VectorXf weight_angle; };
template<class value_type> value_type read_value(std::ifstream &input) {
  value_type value{};
  input.read(reinterpret_cast<char *>(&value), sizeof(value));
  if (!input) throw std::runtime_error("truncated model");
  return value;
}
Eigen::VectorXf read_vector(std::ifstream &input) {
  const auto rows = read_value<std::int64_t>(input);
  const auto cols = read_value<std::int64_t>(input);
  if (rows < 0 || cols < 0 || rows > 1000 || cols > 1000 || rows * cols > 10000)
    throw std::runtime_error("invalid matrix dimensions");
  Eigen::VectorXf values(rows * cols);
  input.read(reinterpret_cast<char *>(values.data()), values.size() * sizeof(float));
  if (!input || !values.allFinite()) throw std::runtime_error("invalid vector payload");
  return values;
}
std::vector<saved_node> read_model(const std::string &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("model open failed");
  if (read_value<std::uint32_t>(input) != 9) throw std::runtime_error("unsupported model version");
  const int num_layers = read_value<int>(input);
  const int num_nodes = read_value<int>(input);
  if (num_layers < 1 || num_layers > 100 || num_nodes < 4 || num_nodes > 1000000)
    throw std::runtime_error("invalid model header");
  std::vector<saved_node> nodes;
  nodes.reserve(num_nodes);
  for (int idx = 0; idx < num_nodes; ++idx) {
    saved_node node;
    node.id = read_value<int>(input);
    read_value<float>(input);
    read_value<float>(input);
    node.weight_angle = read_vector(input);
    read_vector(input);
    const int num_coords = read_value<int>(input);
    if (num_coords < 0 || num_coords > 100) throw std::runtime_error("invalid coord count");
    for (int coord_idx = 0; coord_idx < num_coords; ++coord_idx) read_vector(input);
    read_value<int>(input);
    for (int status_idx = 0; status_idx < 5; ++status_idx) read_value<std::uint8_t>(input);
    read_vector(input);
    for (int score_idx = 0; score_idx < 3; ++score_idx) read_value<float>(input);
    read_value<std::uint8_t>(input);
    read_value<float>(input);
    read_value<float>(input);
    read_value<std::uint8_t>(input);
    if (node.id < 0) throw std::runtime_error("invalid node id");
    nodes.push_back(std::move(node));
  }
  // 保存ノード列終端の境界検証。
  const int num_edges = read_value<int>(input);
  if (num_edges < 0 || num_edges > 100000000) throw std::runtime_error("invalid edge count");
  return nodes;
}
template<int num_dims> struct tree_node {
  SpatialTree::Point<float, num_dims> position;
  void *spatial_handle = nullptr;
  int leaf_idx = -1;
  int id = -1;
};
template<int num_dims> struct tree_traits {
  using node_type = tree_node<num_dims>;
  using point_type = SpatialTree::Point<float, num_dims>;
  static const point_type &getPosition(const node_type *node) { return node->position; }
  static void setPosition(node_type *node, const point_type &point) { node->position = point; }
  static const void *getHandle(const node_type *node) { return node->spatial_handle; }
  static void setHandle(node_type *node, const void *handle) { node->spatial_handle = const_cast<void *>(handle); }
  static int getIndex(const node_type *node) { return node->leaf_idx; }
  static void setIndex(node_type *node, int leaf_idx) { node->leaf_idx = leaf_idx; }
};
using result_type = std::array<std::pair<float, int>, 4>;
using clock_type = std::chrono::steady_clock;
std::map<std::string, double> metrics;
volatile std::uint64_t result_checksum = 0;
result_type brute_search(const std::vector<saved_node> &nodes, const Eigen::VectorXf &query,
                         std::vector<std::pair<float, int>> &candidates) {
  candidates.clear();
  for (const auto &node : nodes) candidates.push_back({(query - node.weight_angle).squaredNorm(), node.id});
  std::partial_sort(candidates.begin(), candidates.begin() + 4, candidates.end());
  result_type result;
  std::copy_n(candidates.begin(), 4, result.begin());
  return result;
}
void check_results(const std::string &prefix, const std::vector<result_type> &baseline,
                   const std::vector<result_type> &tree_results) {
  int num_id_mismatches = 0, num_near_ties = 0, num_dist_mismatches = 0;
  double max_dist_error = 0;
  for (std::size_t query_idx = 0; query_idx < baseline.size(); ++query_idx) {
    bool is_id_match = true, is_dist_match = true;
    for (int rank = 0; rank < 4; ++rank) {
      const auto &expected = baseline[query_idx][rank];
      const auto &actual = tree_results[query_idx][rank];
      is_id_match &= expected.second == actual.second;
      const double diff = std::abs(double(expected.first) - actual.first);
      max_dist_error = std::max(max_dist_error, diff);
      is_dist_match &= diff <= 2e-6 * std::max(1.0, double(expected.first));
    }
    num_id_mismatches += !is_id_match;
    num_near_ties += !is_id_match && is_dist_match;
    num_dist_mismatches += !is_dist_match;
  }
  metrics[prefix + "_num_id_mismatches"] = num_id_mismatches;
  metrics[prefix + "_num_near_ties"] = num_near_ties;
  metrics[prefix + "_num_dist_mismatches"] = num_dist_mismatches;
  metrics[prefix + "_max_squared_dist_error"] = max_dist_error;
}
template<int num_dims> void measure_dataset(const std::vector<saved_node> &model, int max_nodes,
                                            int seed, int trial) {
  std::vector<saved_node> nodes(model.begin(), model.begin() + std::min<int>(max_nodes, model.size()));
  using point_type = SpatialTree::Point<float, num_dims>;
  using tree_type = SpatialTree::MovingBSPTree<tree_node<num_dims>, float, num_dims, tree_traits<num_dims>, SpatialTree::NoHysteresis>;
  std::vector<tree_node<num_dims>> tree_nodes(nodes.size());
  std::map<int, std::size_t> node_offsets;
  for (std::size_t idx = 0; idx < nodes.size(); ++idx) {
    if (nodes[idx].weight_angle.size() != num_dims) throw std::runtime_error("unexpected joint dimension");
    tree_nodes[idx].id = nodes[idx].id;
    node_offsets[nodes[idx].id] = idx;
    for (int dim_idx = 0; dim_idx < num_dims; ++dim_idx) tree_nodes[idx].position[dim_idx] = nodes[idx].weight_angle[dim_idx];
  }
  SpatialTree::MovingBSPParams<float> params;
  params.approx_eps = 0;
  tree_type tree(params);
  const auto build_start = clock_type::now();
  for (auto &node : tree_nodes) tree.add(&node);
  const double build_ms = std::chrono::duration<double, std::milli>(clock_type::now() - build_start).count();
  if (!tree.checkInvariants().empty()) throw std::runtime_error(tree.checkInvariants());
  Eigen::VectorXf min_angles = nodes.front().weight_angle, max_angles = min_angles;
  for (const auto &node : nodes) {
    min_angles = min_angles.cwiseMin(node.weight_angle);
    max_angles = max_angles.cwiseMax(node.weight_angle);
  }
  constexpr int num_queries = 1000;
  for (int query_kind = 0; query_kind < 2; ++query_kind) {
    const std::string prefix = "d" + std::to_string(num_dims) + "_n" + std::to_string(nodes.size()) + (query_kind ? "_near" : "_uniform");
    std::mt19937 generator(seed + query_kind);
    std::uniform_real_distribution<float> uniform(0, 1);
    std::normal_distribution<float> gaussian(0, 0.05f);
    std::uniform_int_distribution<int> choose_node(0, nodes.size() - 1);
    std::vector<Eigen::VectorXf> queries;
    std::vector<point_type> tree_queries(num_queries);
    for (int query_idx = 0; query_idx < num_queries; ++query_idx) {
      Eigen::VectorXf query(num_dims);
      const int node_idx = choose_node(generator);
      for (int dim_idx = 0; dim_idx < num_dims; ++dim_idx) {
        query[dim_idx] = query_kind ? nodes[node_idx].weight_angle[dim_idx] + gaussian(generator)
                                   : min_angles[dim_idx] + uniform(generator) * (max_angles[dim_idx] - min_angles[dim_idx]);
        tree_queries[query_idx][dim_idx] = query[dim_idx];
      }
      queries.push_back(std::move(query));
    }
    std::vector<std::pair<float, int>> candidates;
    candidates.reserve(nodes.size());
    std::vector<result_type> baseline(num_queries), tree_results(num_queries);
    auto run_baseline = [&]() {
      const auto start = clock_type::now();
      for (int query_idx = 0; query_idx < num_queries; ++query_idx) baseline[query_idx] = brute_search(nodes, queries[query_idx], candidates);
      return std::chrono::duration<double, std::micro>(clock_type::now() - start).count() / num_queries;
    };
    auto run_tree = [&]() {
      const auto start = clock_type::now();
      for (int query_idx = 0; query_idx < num_queries; ++query_idx) {
        const auto found = tree.findNBest(tree_queries[query_idx], 4);
        if (found.size() != 4) throw std::runtime_error("short tree result");
        for (int rank = 0; rank < 4; ++rank) tree_results[query_idx][rank] = {found[rank].distance_sq, found[rank].element->id};
      }
      return std::chrono::duration<double, std::micro>(clock_type::now() - start).count() / num_queries;
    };
    // 少数クエリによるページ・命令キャッシュの事前接触。
    for (int query_idx = 0; query_idx < 32; ++query_idx) {
      result_checksum += brute_search(nodes, queries[query_idx], candidates)[0].second;
      result_checksum += tree.findNBest(tree_queries[query_idx], 4)[0].element->id;
    }
    double baseline_us, tree_us;
    if (trial % 2) { tree_us = run_tree(); baseline_us = run_baseline(); }
    else { baseline_us = run_baseline(); tree_us = run_tree(); }
    metrics[prefix + "_num_queries"] = num_queries;
    metrics[prefix + "_build_ms"] = build_ms;
    metrics[prefix + "_baseline_us"] = baseline_us;
    metrics[prefix + "_tree_us"] = tree_us;
    metrics[prefix + "_speedup"] = baseline_us / tree_us;
    check_results(prefix, baseline, tree_results);
    std::cout << prefix << " baseline_us=" << baseline_us << " tree_us=" << tree_us
              << " speedup=" << baseline_us / tree_us << " id_mismatches="
              << metrics[prefix + "_num_id_mismatches"] << " near_ties="
              << metrics[prefix + "_num_near_ties"] << " dist_mismatches="
              << metrics[prefix + "_num_dist_mismatches"] << '\n';
  }
}

// ランダムな点移動を用いた更新API単体の計測。実際の隣接グラフ更新の再現対象外。
template<int num_dims> void measure_updates(const std::vector<saved_node> &model, int seed) {
  using point_type = SpatialTree::Point<float, num_dims>;
  using tree_type = SpatialTree::MovingBSPTree<tree_node<num_dims>, float, num_dims, tree_traits<num_dims>, SpatialTree::NoHysteresis>;
  std::vector<tree_node<num_dims>> tree_nodes(model.size());
  Eigen::VectorXf min_angles = model.front().weight_angle, max_angles = min_angles;
  for (std::size_t idx = 0; idx < model.size(); ++idx) {
    min_angles = min_angles.cwiseMin(model[idx].weight_angle);
    max_angles = max_angles.cwiseMax(model[idx].weight_angle);
    tree_nodes[idx].id = model[idx].id;
    for (int dim_idx = 0; dim_idx < num_dims; ++dim_idx)
      tree_nodes[idx].position[dim_idx] = model[idx].weight_angle[dim_idx];
  }
  tree_type tree;
  for (auto &node : tree_nodes) tree.add(&node);
  constexpr int num_updates = 100000;
  std::mt19937 generator(seed);
  std::uniform_real_distribution<float> uniform(0, 1);
  std::uniform_int_distribution<int> choose_node(0, model.size() - 1);
  std::vector<int> update_nodes(num_updates);
  std::vector<point_type> update_targets(num_updates);
  for (int update_idx = 0; update_idx < num_updates; ++update_idx) {
    update_nodes[update_idx] = choose_node(generator);
    for (int dim_idx = 0; dim_idx < num_dims; ++dim_idx)
      update_targets[update_idx][dim_idx] = min_angles[dim_idx] + uniform(generator) * (max_angles[dim_idx] - min_angles[dim_idx]);
  }
  tree.resetStats();
  const auto start = clock_type::now();
  for (int update_idx = 0; update_idx < num_updates; ++update_idx) {
    auto &node = tree_nodes[update_nodes[update_idx]];
    const auto next_position = node.position + (update_targets[update_idx] - node.position) * 0.008f;
    tree.updatePosition(&node, next_position);
  }
  const double update_us = std::chrono::duration<double, std::micro>(clock_type::now() - start).count() / num_updates;
  const std::string prefix = "d" + std::to_string(num_dims) + "_update";
  const auto &stats = tree.getStats();
  metrics[prefix + "_us"] = update_us;
  metrics[prefix + "_num_updates"] = stats.updates;
  metrics[prefix + "_num_crossings"] = stats.crossings;
  metrics[prefix + "_num_stayed_in_leaf"] = stats.stayed_in_leaf;
  metrics[prefix + "_num_rebuilds"] = stats.rebuilds;
  metrics[prefix + "_num_splits"] = stats.splits;
  metrics[prefix + "_num_collapses"] = stats.collapses;
  const auto error = tree.checkInvariants();
  if (!error.empty()) throw std::runtime_error(error);
  // 更新後の保存座標と木内座標の一致確認。
  auto current_nodes = model;
  for (std::size_t idx = 0; idx < model.size(); ++idx)
    for (int dim_idx = 0; dim_idx < num_dims; ++dim_idx)
      current_nodes[idx].weight_angle[dim_idx] = tree_nodes[idx].position[dim_idx];
  std::vector<std::pair<float, int>> candidates;
  candidates.reserve(model.size());
  std::vector<result_type> baseline(100), tree_results(100);
  for (int query_idx = 0; query_idx < 100; ++query_idx) {
    Eigen::VectorXf query(num_dims);
    point_type tree_query = update_targets[query_idx];
    for (int dim_idx = 0; dim_idx < num_dims; ++dim_idx) query[dim_idx] = tree_query[dim_idx];
    baseline[query_idx] = brute_search(current_nodes, query, candidates);
    const auto found = tree.findNBest(tree_query, 4);
    if (found.size() != 4) throw std::runtime_error("short updated tree result");
    for (int rank = 0; rank < 4; ++rank) tree_results[query_idx][rank] = {found[rank].distance_sq, found[rank].element->id};
  }
  check_results(prefix, baseline, tree_results);
  std::cout << prefix << " us=" << update_us << " crossings=" << stats.crossings
            << " rebuilds=" << stats.rebuilds << " splits=" << stats.splits
            << " collapses=" << stats.collapses << " id_mismatches="
            << metrics[prefix + "_num_id_mismatches"] << '\n';
}

int main(int argc, char **argv) {
  try {
    if (argc != 4 && argc != 5) throw std::runtime_error("usage: measure seed trial metrics_path");
    const int seed = std::stoi(argv[1]), trial = std::stoi(argv[2]);
    const auto model_7 = read_model("/home/uraki/uraki_ws/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin");
    const auto model_14 = read_model("/home/uraki/uraki_ws/gng_vlut_system/gng_results/topo_dual_arm_max_long/gng.bin");
    if (argc == 5 && std::string(argv[4]) == "updates") {
      measure_updates<7>(model_7, seed);
      measure_updates<14>(model_14, seed);
    } else {
    measure_dataset<7>(model_7, model_7.size(), seed, trial);
    measure_dataset<7>(model_7, 1000, seed, trial);
    measure_dataset<14>(model_14, model_14.size(), seed, trial);
    measure_dataset<14>(model_14, 1000, seed, trial);
    }
    std::ofstream output(argv[3]);
    output << std::setprecision(12) << "{\n";
    bool is_first = true;
    for (const auto &metric : metrics) {
      if (!is_first) output << ",\n";
      output << '"' << metric.first << "\":" << metric.second;
      is_first = false;
    }
    output << "\n}\n";
    return output ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
