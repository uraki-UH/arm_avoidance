#include "common.hpp"
#include "collision/joint_segment_collision.hpp"
#include <atomic>
#include <future>
#include <mutex>

struct options {
  std::string mode, gng, urdf, resource_root, mesh_root, output;
  std::string references, heldout, safe_node_ids, extra_nodes, exclusion_pairs;
  size_t max_nodes = std::numeric_limits<size_t>::max();
  size_t max_reference_poses = std::numeric_limits<size_t>::max();
  int32_t min_node_id = 0;
  size_t num_candidates = 8;
  size_t max_added_neighbors = 2;
  size_t num_workers = 1;
  double voxel_size = 0.001;
};
static options read_options(int argc, char **argv) {
  options values;
  for (int idx = 1; idx < argc; ++idx) {
    const std::string name = argv[idx];
    require(idx + 1 < argc, "Missing option value: " + name);
    const std::string value = argv[++idx];
    if (name == "--mode") values.mode = value;
    else if (name == "--gng") values.gng = value;
    else if (name == "--urdf") values.urdf = value;
    else if (name == "--resource-root") values.resource_root = value;
    else if (name == "--mesh-root") values.mesh_root = value;
    else if (name == "--output") values.output = value;
    else if (name == "--references") values.references = value;
    else if (name == "--heldout") values.heldout = value;
    else if (name == "--safe-node-ids") values.safe_node_ids = value;
    else if (name == "--extra-nodes") values.extra_nodes = value;
    else if (name == "--exclusion-pairs") values.exclusion_pairs = value;
    else if (name == "--voxel-size") values.voxel_size = std::stod(value);
    else if (name == "--max-nodes") values.max_nodes = std::stoull(value);
    else if (name == "--max-reference-poses") values.max_reference_poses = std::stoull(value);
    else if (name == "--min-node-id") values.min_node_id = std::stoi(value);
    else if (name == "--num-candidates") values.num_candidates = std::stoull(value);
    else if (name == "--max-added-neighbors") values.max_added_neighbors = std::stoull(value);
    else if (name == "--num-workers") values.num_workers = std::stoull(value);
    else throw std::runtime_error("Unknown option " + name);
  }
  require(values.mode == "audit" || values.mode == "rebuild", "--mode must be audit or rebuild");
  require(std::isfinite(values.voxel_size) && values.voxel_size > 0, "Invalid voxel size");
  require(values.num_workers > 0 && values.num_workers <= 64, "Invalid worker count");
  require(!values.gng.empty() && !values.urdf.empty() && !values.resource_root.empty() &&
          !values.mesh_root.empty() && !values.output.empty(), "Missing required path option");
  require(values.num_candidates > 0 && values.max_added_neighbors > 0 &&
          values.max_added_neighbors <= values.num_candidates, "Invalid neighbor counts");
  require(values.mode != "rebuild" || !values.safe_node_ids.empty(), "Rebuild requires --safe-node-ids");
  // 共通一時ファイルへの xacro 並列展開の防止
  if (values.num_workers > 1) {
    const std::filesystem::path urdf_path(values.urdf);
    require(urdf_path.is_absolute() && std::filesystem::is_regular_file(urdf_path) &&
            urdf_path.extension() == ".urdf" && values.urdf.find(".xacro") == std::string::npos,
            "Parallel workers require an existing absolute expanded .urdf path without .xacro");
  }
  return values;
}
static std::vector<std::string> split_csv(std::string line) {
  if (!line.empty() && line.back() == '\r') line.pop_back();
  std::stringstream stream(line);
  std::vector<std::string> values;
  std::string value;
  while (std::getline(stream, value, ',')) values.push_back(value);
  return values;
}
static std::vector<std::pair<std::string, std::string>> read_exclusion_pairs(const std::string &path) {
  if (path.empty()) return {{"L_finger_left", "L_finger_right"}, {"R_finger_left", "R_finger_right"}};
  std::ifstream input(path);
  require(bool(input), "Cannot read exclusion CSV");
  std::string line;
  require(bool(std::getline(input, line)) && split_csv(line) == std::vector<std::string>{"first", "second"},
          "Exclusion CSV requires first,second header");
  std::vector<std::pair<std::string, std::string>> pairs;
  std::set<std::pair<std::string, std::string>> seen;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto values = split_csv(line);
    require(values.size() == 2 && !values[0].empty() && !values[1].empty() && values[0] != values[1],
            "Invalid exclusion CSV row");
    const auto ordered = std::minmax(values[0], values[1]);
    require(seen.emplace(ordered.first, ordered.second).second, "Duplicate exclusion pair");
    pairs.emplace_back(values[0], values[1]);
  }
  return pairs;
}
static std::vector<node_record> read_poses(const std::string &path) {
  std::ifstream input(path);
  require(bool(input), "Cannot read pose CSV: " + path);
  std::string line;
  require(bool(std::getline(input, line)), "Missing pose CSV header");
  const auto columns = split_csv(line);
  std::map<std::string, size_t> by_name;
  for (size_t idx = 0; idx < columns.size(); ++idx) require(by_name.emplace(columns[idx], idx).second, "Duplicate CSV column");
  for (const auto *name : {"id", "source_arm", "source_idx"}) require(by_name.count(name), "Missing CSV column");
  for (int idx = 0; idx < 14; ++idx) require(by_name.count("q" + std::to_string(idx)), "Missing joint column");
  for (int layer_idx = 0; layer_idx < 2; ++layer_idx) for (const auto *axis : {"x", "y", "z"}) {
    require(by_name.count("tcp" + std::to_string(layer_idx) + "_" + axis), "Missing TCP column");
  }
  std::vector<node_record> nodes;
  std::set<int32_t> ids;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto values = split_csv(line);
    require(values.size() == columns.size(), "Unexpected pose CSV row size");
    node_record node;
    node.id = std::stoi(values[by_name.at("id")]);
    require(node.id >= 0 && ids.insert(node.id).second, "Invalid pose CSV ID");
    for (size_t idx = 0; idx < 14; ++idx) node.q[idx] = std::stof(values[by_name.at("q" + std::to_string(idx))]);
    size_t coord_idx = 0;
    for (int layer_idx = 0; layer_idx < 2; ++layer_idx) for (const auto *axis : {"x", "y", "z"}) {
      node.tcp[coord_idx++] = std::stof(values[by_name.at("tcp" + std::to_string(layer_idx) + "_" + axis)]);
    }
    node.source_arm = values[by_name.at("source_arm")];
    node.source_idx = std::stoll(values[by_name.at("source_idx")]);
    for (const auto value : node.q) require(std::isfinite(value), "Nonfinite CSV angle");
    for (const auto value : node.tcp) require(std::isfinite(value), "Nonfinite CSV TCP");
    nodes.push_back(node);
  }
  return nodes;
}
static void pose_header(std::ostream &output) {
  output << "id";
  for (int idx = 0; idx < 14; ++idx) output << ",q" << idx;
  output << ",tcp0_x,tcp0_y,tcp0_z,tcp1_x,tcp1_y,tcp1_z,source_arm,source_idx\n";
  output << std::setprecision(std::numeric_limits<float>::max_digits10);
}
static void pose_row(std::ostream &output, const node_record &node) {
  output << node.id;
  for (const auto value : node.q) output << ',' << value;
  for (const auto value : node.tcp) output << ',' << value;
  output << ',' << node.source_arm << ',' << node.source_idx << '\n';
}
static void verify_tcp(collision_context &context, const node_record &node) {
  const auto actual = context.tcp();
  for (size_t idx = 0; idx < actual.size(); ++idx) require(std::abs(double(actual[idx]) - node.tcp[idx]) <= 2e-5, "CSV/GNG TCP differs from FK");
}
static json audit_poses(collision_context &context, const std::vector<node_record> &nodes,
                        const std::string &prefix, size_t max_nodes, int32_t min_node_id) {
  std::ofstream report(prefix + "_audit.csv"), kept(prefix + "_kept.csv"), rejected(prefix + "_rejected.csv");
  require(bool(report) && bool(kept) && bool(rejected), "Cannot create audit output");
  report << "id,is_safe,reason,source_arm,source_idx\n";
  pose_header(kept); pose_header(rejected);
  size_t num_checked = 0, num_safe = 0, num_limit_rejected = 0;
  json first_rejected_samples = json::array();
  double safe_pose_check_sec = 0, rejected_pose_check_sec = 0, collision_pair_collection_sec = 0;
  const auto started = clock_type::now();
  for (const auto &node : nodes) {
    if (node.id < min_node_id) continue;
    if (num_checked >= max_nodes) break;
    const bool is_within_limits = context.is_within_limits(node.q);
    const auto pose_started = clock_type::now();
    const bool is_safe = context.is_free(node.q);
    const double pose_check_sec = elapsed_sec(pose_started);
    if (is_safe) safe_pose_check_sec += pose_check_sec; else rejected_pose_check_sec += pose_check_sec;
    if (is_within_limits) verify_tcp(context, node);
    ++num_checked;
    num_safe += is_safe ? 1 : 0;
    num_limit_rejected += is_within_limits ? 0 : 1;
    if (!is_safe && first_rejected_samples.size() < 5) {
      json sample{{"id", node.id}, {"q", node.q}, {"source_arm", node.source_arm},
                  {"source_idx", node.source_idx}, {"reason", is_within_limits ? "self_collision" : "joint_limit"}};
      if (is_within_limits) {
        const auto pairs_started = clock_type::now();
        sample["collision_pairs"] = context.checker->collectSelfCollisionPairs();
        collision_pair_collection_sec += elapsed_sec(pairs_started);
      }
      first_rejected_samples.push_back(std::move(sample));
    }
    report << node.id << ',' << is_safe << ',' << (is_safe ? "safe" : (is_within_limits ? "self_collision" : "joint_limit"))
           << ',' << node.source_arm << ',' << node.source_idx << '\n';
    pose_row(is_safe ? kept : rejected, node);
    if (num_checked % 1000 == 0) std::cout << prefix << " checked=" << num_checked << " safe=" << num_safe << std::endl;
  }
  report.close(); kept.close(); rejected.close();
  require(bool(report) && bool(kept) && bool(rejected), "Audit output write failed");
  return {{"num_input", nodes.size()}, {"num_checked", num_checked}, {"num_safe", num_safe},
      {"num_rejected", num_checked - num_safe}, {"num_limit_rejected", num_limit_rejected},
      {"is_complete", num_checked == nodes.size()}, {"first_rejected_samples", first_rejected_samples},
      {"safe_pose_check_sec", safe_pose_check_sec}, {"rejected_pose_check_sec", rejected_pose_check_sec},
      {"pose_check_sec", safe_pose_check_sec + rejected_pose_check_sec},
      {"collision_pair_collection_sec", collision_pair_collection_sec},
      {"elapsed_sec", elapsed_sec(started)}};
}
static std::set<int32_t> read_safe_ids(const std::string &path) {
  std::ifstream input(path);
  require(bool(input), "Cannot read safe node IDs");
  std::string line;
  require(bool(std::getline(input, line)), "Missing safe node ID header");
  const auto columns = split_csv(line);
  std::map<std::string, size_t> by_name;
  for (size_t idx = 0; idx < columns.size(); ++idx) require(by_name.emplace(columns[idx], idx).second, "Duplicate safe ID column");
  require(by_name.count("id"), "Missing safe node ID column");
  std::set<int32_t> ids, seen_ids;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto values = split_csv(line);
    require(values.size() == columns.size(), "Unexpected safe ID CSV row size");
    const int32_t id = std::stoi(values[by_name.at("id")]);
    require(id >= 0 && seen_ids.insert(id).second, "Invalid or duplicate safe ID");
    if (by_name.count("is_safe")) {
      const auto &is_safe = values[by_name.at("is_safe")];
      require(is_safe == "0" || is_safe == "1", "Invalid safe flag");
      if (is_safe == "0") continue;
    }
    ids.insert(id);
  }
  return ids;
}
static json rebuild(collision_context &context, graph_data original, const options &values,
                    const std::vector<std::pair<std::string, std::string>> &exclusion_pairs) {
  const auto started = clock_type::now();
  const auto safe_ids = read_safe_ids(values.safe_node_ids);
  graph_data graph;
  int32_t max_input_id = -1;
  std::set<int32_t> input_ids;
  for (auto &node : original.nodes) {
    max_input_id = std::max(max_input_id, node.id); input_ids.insert(node.id);
    if (!safe_ids.count(node.id)) continue;
    if (graph.nodes.size() >= values.max_nodes) continue;
    require(context.is_free(node.q), "Selected saved node failed corrected checker: " + std::to_string(node.id));
    verify_tcp(context, node);
    require(node.is_active && node.is_collision_free, "Selected input node has inactive status");
    graph.nodes.push_back(std::move(node));
  }
  for (const auto id : safe_ids) require(input_ids.count(id), "Safe ID absent in source GNG");
  const size_t num_kept_nodes = graph.nodes.size();
  if (!values.extra_nodes.empty()) {
    auto additions = read_poses(values.extra_nodes);
    int32_t next_id = max_input_id + 1;
    for (auto &node : additions) {
      require(node.id == next_id++, "New witness IDs must follow input maximum ID");
      require(context.is_free(node.q), "New witness failed corrected checker: " + std::to_string(node.id));
      verify_tcp(context, node);
      make_node_bytes(node, (context.chain->getEEFOrientation(0) * Eigen::Vector3d::UnitX()).cast<float>());
      graph.nodes.push_back(std::move(node));
    }
  }
  require(!graph.nodes.empty(), "No safe graph nodes remain");
  // 旧辺の全破棄。各出力辺は修正済み判定器による検査結果だけに限定。
  const size_t num_nodes = graph.nodes.size();
  disjoint_set connectivity(num_nodes, 1);
  std::unordered_set<uint64_t> checked_pairs;
  size_t num_candidates_checked = 0, num_rejected_edges = 0, num_added_edges = 0;
  uint64_t num_intermediate_samples = 0;
  const double max_joint_step_rad = 0.025;
  std::ofstream edge_report(values.output + "/edge_audit.csv");
  edge_report << "first_id,second_id,is_safe,num_segments,num_checked_intermediate,phase\n";
  const auto pair_key = [](size_t first, size_t second) {
    if (first > second) std::swap(first, second);
    return (uint64_t(first) << 32) | uint64_t(second);
  };
  struct edge_evaluation {
    size_t first_idx, second_idx, num_segments, num_checked;
    bool is_safe;
  };
  const auto evaluate_edge = [&](collision_context &worker_context, size_t first_idx, size_t second_idx) {
    const auto &first = graph.nodes[first_idx]; const auto &second = graph.nodes[second_idx];
    Eigen::VectorXf first_angles(14), second_angles(14);
    for (size_t idx = 0; idx < 14; ++idx) { first_angles[idx] = first.q[idx]; second_angles[idx] = second.q[idx]; }
    const double span = (second_angles - first_angles).cwiseAbs().maxCoeff();
    const size_t num_segments = std::max<size_t>(1, std::ceil(span / max_joint_step_rad));
    size_t sample_idx = 0, num_checked = 0;
    const bool is_safe = !simulation::has_joint_segment_collision(first_angles, second_angles, max_joint_step_rad,
        [&](const Eigen::VectorXf &angles) {
          q_type q{};
          for (size_t idx = 0; idx < 14; ++idx) q[idx] = angles[idx];
          if (sample_idx > 0 && sample_idx < num_segments) ++num_checked;
          ++sample_idx;
          return !worker_context.is_free(q);
        });
    return edge_evaluation{first_idx, second_idx, num_segments, num_checked, is_safe};
  };
  const auto merge_edge = [&](const edge_evaluation &evaluation, const char *phase) {
    require(checked_pairs.insert(pair_key(evaluation.first_idx, evaluation.second_idx)).second, "Duplicate evaluated edge");
    ++num_candidates_checked; num_intermediate_samples += evaluation.num_checked;
    const auto &first = graph.nodes[evaluation.first_idx]; const auto &second = graph.nodes[evaluation.second_idx];
    edge_report << first.id << ',' << second.id << ',' << evaluation.is_safe << ',' << evaluation.num_segments
                << ',' << evaluation.num_checked << ',' << phase << '\n';
    if (!evaluation.is_safe) { ++num_rejected_edges; return false; }
    const auto edge = make_edge(first.id, second.id);
    for (auto &edges : graph.edges) edges.push_back(edge);
    connectivity.join(evaluation.first_idx, evaluation.second_idx); ++num_added_edges;
    return true;
  };
  const auto try_connect = [&](size_t first_idx, size_t second_idx, const char *phase) {
    if (checked_pairs.count(pair_key(first_idx, second_idx))) return false;
    return merge_edge(evaluate_edge(context, first_idx, second_idx), phase);
  };
  const auto squared_joint_dist = [&](size_t first_idx, size_t second_idx) {
    double value = 0;
    for (size_t idx = 0; idx < 14; ++idx) {
      const double diff = double(graph.nodes[first_idx].q[idx]) - graph.nodes[second_idx].q[idx]; value += diff * diff;
    }
    return value;
  };
  // 各worker専用のFK・FCL・キャッシュ。主contextはworker 0で再利用。
  const size_t num_workers = std::min(values.num_workers, std::max<size_t>(1, num_nodes - 1));
  const auto context_started = clock_type::now();
  std::vector<std::unique_ptr<collision_context>> owned_contexts;
  std::vector<collision_context *> worker_contexts(num_workers, nullptr);
  worker_contexts[0] = &context;
  std::vector<std::future<std::unique_ptr<collision_context>>> context_tasks;
  for (size_t worker_idx = 1; worker_idx < num_workers; ++worker_idx) {
    context_tasks.push_back(std::async(std::launch::async, [&]() {
      return std::make_unique<collision_context>(values.urdf, values.resource_root, values.mesh_root,
                                                 exclusion_pairs, values.voxel_size);
    }));
  }
  for (size_t worker_idx = 1; worker_idx < num_workers; ++worker_idx) {
    owned_contexts.push_back(context_tasks[worker_idx - 1].get());
    worker_contexts[worker_idx] = owned_contexts.back().get();
  }
  const double worker_context_init_sec = elapsed_sec(context_started);
  std::vector<std::pair<uint64_t, uint64_t>> worker_counts_before;
  for (const auto *worker : worker_contexts) worker_counts_before.emplace_back(worker->num_collision_checks, worker->num_cache_hits);
  std::vector<std::vector<edge_evaluation>> node_evaluations(num_nodes);
  std::atomic<size_t> next_node_idx{1}, num_processed_nodes{0};
  std::atomic<bool> has_worker_failed{false};
  std::mutex progress_mutex;
  const auto edge_started = clock_type::now();
  const auto process_nodes = [&](size_t worker_idx) {
    try {
      while (!has_worker_failed.load()) {
        const size_t node_idx = next_node_idx.fetch_add(1);
        if (node_idx >= num_nodes) break;
        std::vector<std::pair<double, size_t>> candidates;
        candidates.reserve(node_idx);
        for (size_t target_idx = 0; target_idx < node_idx; ++target_idx) {
          candidates.emplace_back(squared_joint_dist(node_idx, target_idx), target_idx);
        }
        const size_t num_candidates = std::min(values.num_candidates, candidates.size());
        std::partial_sort(candidates.begin(), candidates.begin() + num_candidates, candidates.end());
        size_t num_neighbors = 0;
        auto &evaluations = node_evaluations[node_idx];
        evaluations.reserve(num_candidates);
        for (size_t idx = 0; idx < num_candidates; ++idx) {
          evaluations.push_back(evaluate_edge(*worker_contexts[worker_idx], node_idx, candidates[idx].second));
          if (evaluations.back().is_safe) ++num_neighbors;
          if (num_neighbors >= values.max_added_neighbors || has_worker_failed.load()) break;
        }
        const size_t num_processed = num_processed_nodes.fetch_add(1) + 1;
        if (num_processed % 1000 == 0) {
          std::lock_guard<std::mutex> guard(progress_mutex);
          std::cout << "rebuilt_nodes=" << num_processed << '/' << num_nodes
                    << " elapsed_sec=" << elapsed_sec(edge_started) << std::endl;
        }
      }
    } catch (...) {
      has_worker_failed.store(true);
      throw;
    }
  };
  if (num_workers == 1) {
    process_nodes(0);
  } else {
    std::vector<std::future<void>> worker_tasks;
    try {
      for (size_t worker_idx = 0; worker_idx < num_workers; ++worker_idx) {
        worker_tasks.push_back(std::async(std::launch::async, process_nodes, worker_idx));
      }
      for (auto &task : worker_tasks) task.get();
    } catch (...) {
      has_worker_failed.store(true);
      throw;
    }
  }
  // node_idx順の統合による逐次版と同一の辺・候補ログ・連結成分。
  for (size_t node_idx = 1; node_idx < num_nodes; ++node_idx) {
    require(!node_evaluations[node_idx].empty(), "Missing initial node evaluation");
    for (const auto &evaluation : node_evaluations[node_idx]) merge_edge(evaluation, "initial");
  }
  json initial_worker_metrics = json::array();
  for (size_t worker_idx = 0; worker_idx < num_workers; ++worker_idx) {
    const auto &worker = *worker_contexts[worker_idx];
    initial_worker_metrics.push_back({{"worker_idx", worker_idx},
        {"num_collision_checks", worker.num_collision_checks - worker_counts_before[worker_idx].first},
        {"num_cache_hits", worker.num_cache_hits - worker_counts_before[worker_idx].second},
        {"num_cached_poses", worker.pose_cache.size()}});
    if (worker_idx != 0) {
      context.num_collision_checks += worker.num_collision_checks;
      context.num_cache_hits += worker.num_cache_hits;
      context.num_limit_failures += worker.num_limit_failures;
    }
  }
  owned_contexts.clear();
  const double initial_edge_sec = elapsed_sec(edge_started);
  const size_t num_initial_edges = num_added_edges;
  const auto fallback_started = clock_type::now();
  size_t num_fallback_passes = 0;
  // 別成分への候補は全ノードから探索。改善があるpassだけの反復。
  while (true) {
    bool has_improved = false, has_unconnected = false;
    std::set<size_t> visited;
    for (size_t member_idx = 0; member_idx < num_nodes; ++member_idx) {
      const auto source_root = connectivity.find(member_idx);
      if (connectivity.has_original[source_root] || !visited.insert(source_root).second) continue;
      has_unconnected = true;
      using candidate_type = std::tuple<double, size_t, size_t>;
      std::priority_queue<candidate_type> nearest;
      for (size_t source_idx = 0; source_idx < num_nodes; ++source_idx) {
        if (connectivity.find(source_idx) != source_root) continue;
        for (size_t target_idx = 0; target_idx < num_nodes; ++target_idx) {
          if (connectivity.find(target_idx) == source_root || checked_pairs.count(pair_key(source_idx, target_idx))) continue;
          candidate_type candidate{squared_joint_dist(source_idx, target_idx), source_idx, target_idx};
          if (nearest.size() < 64) nearest.push(candidate);
          else if (candidate < nearest.top()) { nearest.pop(); nearest.push(candidate); }
        }
      }
      std::vector<candidate_type> candidates;
      while (!nearest.empty()) { candidates.push_back(nearest.top()); nearest.pop(); }
      std::reverse(candidates.begin(), candidates.end());
      for (const auto &[dist, source_idx, target_idx] : candidates) {
        static_cast<void>(dist);
        if (connectivity.find(source_idx) == connectivity.find(target_idx)) continue;
        if (try_connect(source_idx, target_idx, "fallback")) {
          has_improved = true; visited.insert(connectivity.find(source_idx)); break;
        }
      }
    }
    if (!has_unconnected) break;
    ++num_fallback_passes;
    std::cout << "fallback_pass=" << num_fallback_passes << " added_edges=" << num_added_edges - num_initial_edges << std::endl;
    if (!has_improved) break;
  }
  const double fallback_edge_sec = elapsed_sec(fallback_started);
  edge_report.close(); require(bool(edge_report), "Edge audit write failed");
  const std::string output_path = values.output + "/gng.bin";
  write_graph(graph, output_path); verify_output(graph, output_path);
  std::ofstream unconnected(values.output + "/unconnected_nodes.csv");
  unconnected << "id\n";
  for (size_t idx = 0; idx < num_nodes; ++idx) if (!connectivity.has_original[connectivity.find(idx)]) unconnected << graph.nodes[idx].id << '\n';
  unconnected.close(); require(bool(unconnected), "Unconnected report write failed");
  return {{"num_input_nodes", original.nodes.size()}, {"num_kept_nodes", num_kept_nodes},
      {"num_added_witness_nodes", num_nodes - num_kept_nodes}, {"num_output_nodes", num_nodes},
      {"num_old_edges_discarded", {original.edges[0].size(), original.edges[1].size(), original.edges[2].size()}},
      {"num_unique_candidates_checked", num_candidates_checked}, {"num_rejected_edges", num_rejected_edges},
      {"num_validated_edges_per_layer", num_added_edges}, {"num_intermediate_samples", num_intermediate_samples},
      {"num_workers", num_workers}, {"num_requested_workers", values.num_workers},
      {"worker_context_init_sec", worker_context_init_sec}, {"initial_worker_metrics", initial_worker_metrics},
      {"initial_edge_sec", initial_edge_sec}, {"fallback_edge_sec", fallback_edge_sec},
      {"num_fallback_passes", num_fallback_passes}, {"num_fallback_added_edges", num_added_edges - num_initial_edges},
      {"graph", graph_metrics(graph, num_kept_nodes)}, {"is_roundtrip_verified", true},
      {"elapsed_sec", elapsed_sec(started)}};
}
int main(int argc, char **argv) {
  try {
    const auto values = read_options(argc, argv);
    require(!std::filesystem::exists(values.output) && !std::filesystem::is_symlink(values.output), "Output already exists");
    const auto started = clock_type::now();
    auto graph = read_graph(values.gng);
    const auto exclusion_pairs = read_exclusion_pairs(values.exclusion_pairs);
    collision_context context(values.urdf, values.resource_root, values.mesh_root, exclusion_pairs, values.voxel_size);
    require(std::filesystem::create_directories(values.output), "Output appeared during initialization");
    json metrics{{"mode", values.mode}, {"gng", values.gng}, {"urdf", values.urdf},
        {"collision_method", "mesh_surface_and_component_voxel_containment"},
        {"collision_voxel_size_m", values.voxel_size}, {"max_joint_step_rad", 0.025},
        {"manual_exclusions", exclusion_pairs}, {"exclusion_pairs_csv", values.exclusion_pairs},
        {"edge_guarantee", "all retained edges checked at endpoints and discrete interpolation samples; no continuous certification"}};
    if (values.mode == "audit") {
      // 判定器の初期機構接触の記録。ゼロ姿勢だけを理由とした自動除外なし。
      const q_type zero_q{};
      const bool is_zero_within_limits = context.is_within_limits(zero_q);
      const bool is_zero_safe = context.is_free(zero_q);
      metrics["zero_pose"] = {{"q", zero_q}, {"is_safe", is_zero_safe},
          {"reason", is_zero_safe ? "safe" : (is_zero_within_limits ? "self_collision" : "joint_limit")}};
      if (is_zero_within_limits) metrics["zero_pose"]["collision_pairs"] = context.checker->collectSelfCollisionPairs();
      for (auto &node : graph.nodes) { node.source_arm = node.id < 10000 ? "original" : "added"; node.source_idx = node.id; }
      metrics["nodes"] = audit_poses(context, graph.nodes, values.output + "/nodes", values.max_nodes, values.min_node_id);
      if (!values.references.empty()) metrics["references"] = audit_poses(context, read_poses(values.references), values.output + "/references", values.max_reference_poses, 0);
      if (!values.heldout.empty()) metrics["heldout"] = audit_poses(context, read_poses(values.heldout), values.output + "/heldout", values.max_reference_poses, 0);
    } else {
      metrics["rebuild"] = rebuild(context, std::move(graph), values, exclusion_pairs);
    }
    metrics["num_collision_checks"] = context.num_collision_checks;
    metrics["num_cache_hits"] = context.num_cache_hits;
    metrics["num_cached_poses"] = context.pose_cache.size();
    metrics["num_limit_failures"] = context.num_limit_failures;
    metrics["elapsed_sec"] = elapsed_sec(started);
    std::ofstream output(values.output + "/metrics.json");
    output << metrics.dump(2) << '\n'; output.close(); require(bool(output), "Metrics write failed");
    std::cout << metrics.dump() << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "safe_graph: " << error.what() << std::endl;
    return 1;
  }
}
