#include "collision/geometric_self_collision_checker.hpp"
#include "gng/GrowingNeuralGas.hpp"
#include "reachability/joint_sampling.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <queue>
#include <tuple>
#include <unordered_set>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using json = nlohmann::json;
using clock_type = std::chrono::steady_clock;
using q_type = std::array<float, 14>;
using tcp_type = std::array<float, 6>;
using graph_type = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

static void require(bool is_valid, const std::string &message) {
  if (!is_valid) throw std::runtime_error(message);
}
static double elapsed_sec(clock_type::time_point started) {
  return std::chrono::duration<double>(clock_type::now() - started).count();
}
struct node_record {
  int32_t id = -1;
  q_type q{};
  tcp_type tcp{};
  bool is_active = false;
  bool is_collision_free = false;
  std::string source_arm;
  int64_t source_idx = -1;
  std::vector<char> bytes;
};
struct edge_record {
  int32_t first_id;
  int32_t second_id;
  int32_t age;
  bool is_active;
  std::array<char, 13> bytes;
};
struct graph_data {
  std::vector<node_record> nodes;
  std::array<std::vector<edge_record>, 3> edges;
};
struct reader {
  std::vector<char> bytes;
  size_t offset = 0;
  explicit reader(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    require(bool(input), "Cannot read " + path);
    bytes.assign(std::istreambuf_iterator<char>(input), {});
  }
  template<class value_type> value_type read() {
    require(offset + sizeof(value_type) <= bytes.size(), "Unexpected binary end");
    value_type value;
    std::memcpy(&value, bytes.data() + offset, sizeof(value));
    offset += sizeof(value);
    return value;
  }
  std::vector<float> read_vector() {
    const auto rows = read<int64_t>();
    const auto cols = read<int64_t>();
    require(rows > 0 && rows < 4096 && cols > 0 && cols < 4096 && rows * cols < 4096,
            "Invalid Eigen vector dimensions");
    std::vector<float> values(rows * cols);
    for (auto &value : values) value = read<float>();
    return values;
  }
};
static graph_data read_graph(const std::string &path) {
  reader input(path);
  require(input.read<uint32_t>() == 9, "GNG version must be 9");
  require(input.read<int32_t>() == 2, "GNG must have two coordinate layers");
  const int32_t num_nodes = input.read<int32_t>();
  require(num_nodes >= 0 && num_nodes <= 1000000, "Invalid GNG node count");
  graph_data graph;
  std::set<int32_t> ids;
  for (int32_t idx = 0; idx < num_nodes; ++idx) {
    const size_t begin = input.offset;
    node_record node;
    node.id = input.read<int32_t>();
    require(node.id >= 0 && ids.insert(node.id).second, "Invalid or duplicate GNG node ID");
    input.read<float>(); input.read<float>();
    const auto q = input.read_vector();
    require(q.size() == 14, "GNG angle dimension must be 14");
    std::copy(q.begin(), q.end(), node.q.begin());
    require(input.read_vector().size() == 3, "Invalid primary coordinate dimension");
    require(input.read<int32_t>() == 2, "Invalid node coordinate layer count");
    for (size_t layer_idx = 0; layer_idx < 2; ++layer_idx) {
      const auto tcp = input.read_vector();
      require(tcp.size() == 3, "Invalid coordinate dimension");
      std::copy(tcp.begin(), tcp.end(), node.tcp.begin() + 3 * layer_idx);
    }
    input.read<int32_t>();
    input.read<uint8_t>(); input.read<uint8_t>();
    node.is_collision_free = input.read<uint8_t>() != 0;
    node.is_active = input.read<uint8_t>() != 0;
    input.read<uint8_t>();
    require(input.read_vector().size() == 3, "Invalid EE direction dimension");
    input.read<float>(); input.read<float>(); input.read<float>();
    input.read<uint8_t>(); input.read<float>(); input.read<float>(); input.read<uint8_t>();
    for (const auto value : node.q) require(std::isfinite(value), "Nonfinite GNG joint value");
    for (const auto value : node.tcp) require(std::isfinite(value), "Nonfinite GNG coordinate");
    node.bytes.assign(input.bytes.begin() + begin, input.bytes.begin() + input.offset);
    graph.nodes.push_back(std::move(node));
  }
  for (auto &edges : graph.edges) {
    const int32_t num_edges = input.read<int32_t>();
    require(num_edges >= 0 && static_cast<size_t>(num_edges) <= (input.bytes.size() - input.offset) / 13,
            "Invalid edge count");
    for (int32_t idx = 0; idx < num_edges; ++idx) {
      const auto begin = input.offset;
      edge_record edge;
      edge.first_id = input.read<int32_t>(); edge.second_id = input.read<int32_t>();
      edge.age = input.read<int32_t>(); edge.is_active = input.read<uint8_t>() != 0;
      require(ids.count(edge.first_id) && ids.count(edge.second_id) && edge.first_id != edge.second_id,
              "Invalid edge endpoints");
      std::copy_n(input.bytes.begin() + begin, 13, edge.bytes.begin());
      edges.push_back(edge);
    }
  }
  require(input.offset == input.bytes.size(), "Unexpected GNG trailing bytes");
  return graph;
}
template<class value_type>
static void append(std::vector<char> &bytes, const value_type &value) {
  const auto *begin = reinterpret_cast<const char *>(&value);
  bytes.insert(bytes.end(), begin, begin + sizeof(value));
}
static void append_vector(std::vector<char> &bytes, const float *values, int64_t num_values) {
  append(bytes, num_values); append(bytes, int64_t(1));
  for (int64_t idx = 0; idx < num_values; ++idx) append(bytes, values[idx]);
}
static void make_node_bytes(node_record &node, const Eigen::Vector3f &direction) {
  // 関節角・両TCPの実測証拠保存。未測定の可操作性は無効状態。
  auto &bytes = node.bytes;
  append(bytes, node.id); append(bytes, 0.0f); append(bytes, 0.0f);
  append_vector(bytes, node.q.data(), 14); append_vector(bytes, node.tcp.data(), 3);
  append(bytes, int32_t(2));
  append_vector(bytes, node.tcp.data(), 3); append_vector(bytes, node.tcp.data() + 3, 3);
  append(bytes, int32_t(0));
  for (uint8_t value : {uint8_t(0), uint8_t(0), uint8_t(1), uint8_t(1), uint8_t(0)}) append(bytes, value);
  append_vector(bytes, direction.data(), 3);
  append(bytes, 0.0f); append(bytes, 0.0f); append(bytes, 1.0f); append(bytes, uint8_t(0));
  append(bytes, 0.0f); append(bytes, 0.0f); append(bytes, uint8_t(0));
  node.is_active = true; node.is_collision_free = true;
}
static edge_record make_edge(int32_t first_id, int32_t second_id) {
  if (first_id > second_id) std::swap(first_id, second_id);
  edge_record edge{first_id, second_id, 1, true, {}};
  std::vector<char> bytes;
  append(bytes, first_id); append(bytes, second_id); append(bytes, int32_t(1)); append(bytes, uint8_t(1));
  std::copy(bytes.begin(), bytes.end(), edge.bytes.begin());
  return edge;
}
static void write_graph(const graph_data &graph, const std::string &path) {
  std::ofstream output(path, std::ios::binary);
  require(bool(output), "Cannot create GNG output");
  const uint32_t version = 9;
  const int32_t layers = 2;
  const int32_t num_nodes = static_cast<int32_t>(graph.nodes.size());
  output.write(reinterpret_cast<const char *>(&version), 4);
  output.write(reinterpret_cast<const char *>(&layers), 4);
  output.write(reinterpret_cast<const char *>(&num_nodes), 4);
  for (const auto &node : graph.nodes) output.write(node.bytes.data(), node.bytes.size());
  for (const auto &edges : graph.edges) {
    const int32_t num_edges = static_cast<int32_t>(edges.size());
    output.write(reinterpret_cast<const char *>(&num_edges), 4);
    for (const auto &edge : edges) output.write(edge.bytes.data(), edge.bytes.size());
  }
  output.close();
  require(bool(output), "GNG output write failed");
}
static std::vector<std::string> split_csv(std::string line) {
  if (!line.empty() && line.back() == '\r') line.pop_back();
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) values.push_back(value);
  return values;
}
static std::vector<node_record> read_new_nodes(const std::string &path, int32_t max_original_id,
                                              size_t max_new_nodes) {
  std::ifstream input(path);
  require(bool(input), "Cannot read new-node CSV");
  std::string line;
  require(bool(std::getline(input, line)), "Missing CSV header");
  auto names = split_csv(line);
  std::vector<std::string> expected{"id"};
  for (int idx = 0; idx < 14; ++idx) expected.push_back("q" + std::to_string(idx));
  for (int layer_idx = 0; layer_idx < 2; ++layer_idx) {
    for (const auto *axis : {"x", "y", "z"}) expected.push_back("tcp" + std::to_string(layer_idx) + "_" + axis);
  }
  expected.push_back("source_arm"); expected.push_back("source_idx");
  require(names == expected, "Unexpected new-node CSV columns");
  std::vector<node_record> nodes;
  int64_t next_id = static_cast<int64_t>(max_original_id) + 1;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto values = split_csv(line);
    require(values.size() == expected.size(), "Unexpected CSV row size");
    node_record node;
    const int64_t id = std::stoll(values[0]);
    require(id == next_id++ && id < 1000000, "CSV IDs must append consecutively");
    node.id = static_cast<int32_t>(id);
    for (size_t idx = 0; idx < 14; ++idx) node.q[idx] = std::stof(values[1 + idx]);
    for (size_t idx = 0; idx < 6; ++idx) node.tcp[idx] = std::stof(values[15 + idx]);
    node.source_arm = values[21]; node.source_idx = std::stoll(values[22]);
    for (const auto value : node.q) require(std::isfinite(value), "Nonfinite CSV angle");
    for (const auto value : node.tcp) require(std::isfinite(value), "Nonfinite CSV TCP");
    if (nodes.size() < max_new_nodes) nodes.push_back(std::move(node));
  }
  return nodes;
}
struct collision_context {
  simulation::RobotModel model;
  std::unique_ptr<kinematics::KinematicChain> chain;
  std::unique_ptr<simulation::GeometricSelfCollisionChecker> checker;
  std::vector<std::pair<double, double>> limits;
  uint64_t num_collision_checks = 0;
  uint64_t num_limit_failures = 0;
  collision_context(const std::string &urdf, const std::string &resource_root, const std::string &mesh_root)
      : model(simulation::loadRobotFromUrdf(urdf, resource_root, mesh_root)) {
    chain = simulation::createMultiArmKinematicChain(model,
        {{"L_shoulder_mount", "L_tcp", ""}, {"R_shoulder_mount", "R_tcp", ""}});
    require(chain->getTotalDOF() == 14 && chain->getArmCount() == 2, "Unexpected robot dimensions");
    std::vector<std::string> joint_names;
    for (int idx = 0; idx < chain->getNumJoints(); ++idx) {
      if (chain->getJointDOF(idx) > 0) joint_names.push_back(chain->getJointName(idx));
    }
    std::vector<std::string> expected_names;
    for (const auto *prefix : {"L_joint", "R_joint"}) {
      for (int idx = 1; idx <= 7; ++idx) expected_names.push_back(prefix + std::to_string(idx));
    }
    require(joint_names == expected_names, "Unexpected joint order");
    limits = robot_sim::reachability::collect_joint_limits(model, *chain);
    checker = std::make_unique<simulation::GeometricSelfCollisionChecker>(model, *chain);
    checker->setStrictMode(true);
    size_t num_expected_meshes = 0;
    for (const auto &entry : model.getLinks()) {
      if (entry.first == model.getRootLinkName()) continue;
      for (const auto &shape : entry.second.collisions) {
        if (shape.geometry.type == simulation::GeometryType::MESH) ++num_expected_meshes;
      }
    }
    size_t num_actual_meshes = 0;
    const auto &objects = checker->getCollisionObjects();
    require(!objects.empty(), "No collision geometry");
    for (size_t idx = 0; idx < objects.size(); ++idx) {
      require(checker->getFCLObject(static_cast<int>(idx)) != nullptr, "Missing FCL object");
      if (objects[idx].type == collision::SelfCollisionChecker::ShapeType::MESH) ++num_actual_meshes;
    }
    require(num_expected_meshes == num_actual_meshes, "Mesh fallback or missing collision mesh");
  }
  bool is_free(const std::vector<double> &q) {
    for (size_t idx = 0; idx < q.size(); ++idx) {
      if (!std::isfinite(q[idx]) || q[idx] < limits[idx].first || q[idx] > limits[idx].second) {
        ++num_limit_failures; return false;
      }
    }
    chain->updateKinematics(q);
    const auto actual = chain->getJointValues();
    require(actual.size() == q.size(), "Kinematic joint size changed");
    for (size_t idx = 0; idx < q.size(); ++idx) require(std::abs(actual[idx] - q[idx]) < 1e-10, "Kinematics clamped joint value");
    checker->updateBodyPoses(chain->getLinkPositions(), chain->getLinkOrientations());
    ++num_collision_checks;
    return !checker->checkCollision();
  }
  bool is_free(const q_type &q) { return is_free(std::vector<double>(q.begin(), q.end())); }
  tcp_type tcp() const {
    tcp_type values{};
    for (size_t layer_idx = 0; layer_idx < 2; ++layer_idx) {
      const auto point = chain->getEEFPosition(layer_idx);
      for (size_t idx = 0; idx < 3; ++idx) values[3 * layer_idx + idx] = static_cast<float>(point[idx]);
    }
    return values;
  }
};
static json describe_checker(simulation::GeometricSelfCollisionChecker &checker) {
  json result;
  result["is_colliding"] = checker.checkCollision();
  result["collision_pairs"] = checker.collectSelfCollisionPairs();
  result["finger_exclusions"] = {
      checker.shouldSkipCollision("L_finger_left", "L_finger_right"),
      checker.shouldSkipCollision("R_finger_left", "R_finger_right")};
  result["objects"] = json::array();
  for (size_t idx = 0; idx < checker.getCollisionObjects().size(); ++idx) {
    const auto object = checker.getFCLObject(static_cast<int>(idx));
    require(object != nullptr, "Missing diagnostic FCL object");
    json item{{"idx", idx}, {"link", checker.getLinkNameForObject(static_cast<int>(idx))}};
    const auto transform = object->getTransform();
    item["matrix"] = json::array();
    for (int row = 0; row < 4; ++row) {
      item["matrix"].push_back({transform.matrix()(row, 0), transform.matrix()(row, 1),
                                 transform.matrix()(row, 2), transform.matrix()(row, 3)});
    }
    result["objects"].push_back(item);
  }
  return result;
}
static json diagnose_node(collision_context &context, const node_record &node) {
  json result{{"id", node.id}, {"source_arm", node.source_arm},
              {"source_idx", node.source_idx}, {"q", node.q}, {"input_tcp", node.tcp}};
  result["limit_violations"] = json::array();
  for (size_t idx = 0; idx < node.q.size(); ++idx) {
    if (node.q[idx] < context.limits[idx].first || node.q[idx] > context.limits[idx].second) {
      result["limit_violations"].push_back({{"idx", idx}, {"value", node.q[idx]},
          {"min", context.limits[idx].first}, {"max", context.limits[idx].second}});
    }
  }
  context.chain->updateKinematics(std::vector<double>(node.q.begin(), node.q.end()));
  context.checker->updateBodyPoses(context.chain->getLinkPositions(), context.chain->getLinkOrientations());
  result["actual_tcp"] = context.tcp();
  result["dual"] = describe_checker(*context.checker);
  const size_t arm_idx = (node.source_arm == "R" || node.source_arm == "right") ? 1 : 0;
  const std::string prefix = arm_idx == 0 ? "L" : "R";
  auto single_chain = simulation::createMultiArmKinematicChain(context.model,
      {{prefix + "_shoulder_mount", prefix + "_tcp", ""}});
  simulation::GeometricSelfCollisionChecker single_checker(context.model, *single_chain);
  single_checker.setStrictMode(true);
  single_chain->updateKinematics(std::vector<double>(node.q.begin() + arm_idx * 7, node.q.begin() + (arm_idx + 1) * 7));
  single_checker.updateBodyPoses(single_chain->getLinkPositions(), single_chain->getLinkOrientations());
  result["single_arm"] = describe_checker(single_checker);
  const auto point = single_chain->getEEFPosition();
  result["single_arm_tcp"] = {point.x(), point.y(), point.z()};
  auto root_chain = simulation::createMultiArmKinematicChain(context.model,
      {{context.model.getRootLinkName(), prefix + "_tcp", ""}});
  simulation::GeometricSelfCollisionChecker root_checker(context.model, *root_chain);
  root_checker.setStrictMode(true);
  root_chain->updateKinematics(std::vector<double>(node.q.begin() + arm_idx * 7, node.q.begin() + (arm_idx + 1) * 7));
  root_checker.updateBodyPoses(root_chain->getLinkPositions(), root_chain->getLinkOrientations());
  result["single_arm_urdf_root"] = describe_checker(root_checker);
  result["urdf_root"] = context.model.getRootLinkName();
  return result;
}

struct disjoint_set {
  std::vector<size_t> parent;
  std::vector<bool> has_original;
  explicit disjoint_set(size_t size, size_t num_original_nodes = 0)
      : parent(size), has_original(size, false) {
    std::iota(parent.begin(), parent.end(), 0);
    for (size_t idx = 0; idx < num_original_nodes; ++idx) has_original[idx] = true;
  }
  size_t find(size_t idx) { return parent[idx] == idx ? idx : parent[idx] = find(parent[idx]); }
  void join(size_t first, size_t second) {
    const size_t first_root = find(first), second_root = find(second);
    has_original[second_root] = has_original[first_root] || has_original[second_root];
    parent[first_root] = second_root;
  }
};
static json graph_metrics(const graph_data &graph, size_t num_original_nodes) {
  std::unordered_map<int32_t, size_t> by_id;
  for (size_t idx = 0; idx < graph.nodes.size(); ++idx) by_id[graph.nodes[idx].id] = idx;
  json output = json::array();
  for (const auto &edges : graph.edges) {
    disjoint_set components(graph.nodes.size());
    std::vector<size_t> degrees(graph.nodes.size());
    size_t num_active_edges = 0;
    for (const auto &edge : edges) {
      const size_t first_idx = by_id.at(edge.first_id), second_idx = by_id.at(edge.second_id);
      if (!edge.is_active || !graph.nodes[first_idx].is_active || !graph.nodes[second_idx].is_active ||
          !graph.nodes[first_idx].is_collision_free || !graph.nodes[second_idx].is_collision_free) continue;
      components.join(first_idx, second_idx);
      ++degrees[first_idx]; ++degrees[second_idx]; ++num_active_edges;
    }
    std::map<size_t, size_t> counts;
    std::set<size_t> original_components;
    for (size_t idx = 0; idx < graph.nodes.size(); ++idx) ++counts[components.find(idx)];
    for (size_t idx = 0; idx < num_original_nodes; ++idx) original_components.insert(components.find(idx));
    size_t num_isolated_nodes = 0, num_isolated_new_nodes = 0, num_new_nodes_connected_to_original = 0;
    for (size_t idx = 0; idx < graph.nodes.size(); ++idx) {
      if (!degrees[idx]) { ++num_isolated_nodes; if (idx >= num_original_nodes) ++num_isolated_new_nodes; }
      if (idx >= num_original_nodes && original_components.count(components.find(idx))) ++num_new_nodes_connected_to_original;
    }
    size_t max_component_size = 0;
    for (const auto &item : counts) max_component_size = std::max(max_component_size, item.second);
    output.push_back({{"num_edges", edges.size()}, {"num_active_edges", num_active_edges},
        {"mean_degree", graph.nodes.empty() ? 0.0 : 2.0 * num_active_edges / graph.nodes.size()},
        {"num_isolated_nodes", num_isolated_nodes}, {"num_isolated_new_nodes", num_isolated_new_nodes},
        {"num_components", counts.size()}, {"max_component_size", max_component_size},
        {"num_new_nodes_connected_to_original", num_new_nodes_connected_to_original}});
  }
  return output;
}
static void verify_output(const graph_data &expected, const std::string &path) {
  const auto actual = read_graph(path);
  require(actual.nodes.size() == expected.nodes.size(), "Roundtrip node count mismatch");
  for (size_t idx = 0; idx < expected.nodes.size(); ++idx) {
    require(actual.nodes[idx].bytes == expected.nodes[idx].bytes, "Roundtrip node bytes mismatch");
  }
  for (size_t layer_idx = 0; layer_idx < 3; ++layer_idx) {
    require(actual.edges[layer_idx].size() == expected.edges[layer_idx].size(), "Roundtrip edge count mismatch");
    for (size_t idx = 0; idx < expected.edges[layer_idx].size(); ++idx) {
      require(actual.edges[layer_idx][idx].bytes == expected.edges[layer_idx][idx].bytes, "Roundtrip edge bytes mismatch");
    }
  }
  // 実際の既存 GNG ローダーによる全姿勢・全隣接リストの照合。
  graph_type loaded(14, 3, nullptr);
  require(loaded.load(path), "Existing GNG library load failed");
  require(loaded.getCoordLayerCount() == 2, "Existing GNG library layer mismatch");
  for (const auto &node : expected.nodes) {
    const auto &value = loaded.nodeAt(node.id);
    require(value.id == node.id && value.weight_angle.size() == 14 && value.weight_coords.size() == 2,
            "Existing GNG library node mismatch");
    for (size_t idx = 0; idx < 14; ++idx) require(value.weight_angle[idx] == node.q[idx], "GNG joint mismatch");
    for (size_t idx = 0; idx < 6; ++idx) require(value.weight_coords[idx / 3][idx % 3] == node.tcp[idx], "GNG TCP mismatch");
  }
  for (size_t layer_idx = 0; layer_idx < 3; ++layer_idx) {
    std::unordered_map<int32_t, std::set<int32_t>> expected_neighbors;
    for (const auto &edge : expected.edges[layer_idx]) {
      expected_neighbors[edge.first_id].insert(edge.second_id);
      expected_neighbors[edge.second_id].insert(edge.first_id);
    }
    for (const auto &node : expected.nodes) {
      const auto &neighbors = layer_idx == 0 ? loaded.getNeighborsAngle(node.id) : loaded.getNeighborsCoord(node.id, layer_idx - 1);
      require(std::set<int32_t>(neighbors.begin(), neighbors.end()) == expected_neighbors[node.id], "GNG neighbor roundtrip mismatch");
    }
  }
}
struct options {
  std::string gng, new_nodes, urdf, resource_root, mesh_root, output;
  size_t max_new_nodes = std::numeric_limits<size_t>::max();
  size_t num_candidates = 8;
  size_t max_added_neighbors = 2;
  double max_joint_step_rad = 0.05;
  size_t sample_count = 0;
  uint64_t sample_seed = 20261001;
  bool is_sample_only = false;
};
static options parse_options(int argc, char **argv) {
  options values;
  for (int idx = 1; idx < argc; ++idx) {
    const std::string name = argv[idx];
    if (name == "--sample-only") { values.is_sample_only = true; continue; }
    require(idx + 1 < argc, "Missing argument for " + name);
    const std::string value = argv[++idx];
    if (name == "--gng") values.gng = value;
    else if (name == "--new-nodes") values.new_nodes = value;
    else if (name == "--urdf") values.urdf = value;
    else if (name == "--resource-root") values.resource_root = value;
    else if (name == "--mesh-root") values.mesh_root = value;
    else if (name == "--output") values.output = value;
    else if (name == "--max-new-nodes") values.max_new_nodes = std::stoull(value);
    else if (name == "--num-candidates") values.num_candidates = std::stoull(value);
    else if (name == "--max-added-neighbors") values.max_added_neighbors = std::stoull(value);
    else if (name == "--max-joint-step-rad") values.max_joint_step_rad = std::stod(value);
    else if (name == "--sample-count") values.sample_count = std::stoull(value);
    else if (name == "--sample-seed") values.sample_seed = std::stoull(value);
    else throw std::runtime_error("Unknown option " + name);
  }
  require(!values.urdf.empty() && !values.resource_root.empty() && !values.mesh_root.empty() && !values.output.empty(),
          "Required: --urdf --resource-root --mesh-root --output");
  require(values.is_sample_only || (!values.gng.empty() && !values.new_nodes.empty()), "Required: --gng --new-nodes");
  require(values.num_candidates > 0 && values.max_added_neighbors > 0 &&
          values.max_added_neighbors <= values.num_candidates, "Invalid neighbor counts");
  require(std::isfinite(values.max_joint_step_rad) && values.max_joint_step_rad > 0 && values.max_joint_step_rad <= 0.05,
          "Joint interpolation step must be in (0, 0.05]");
  return values;
}
static json write_heldout(collision_context &context, const options &values) {
  const auto started = clock_type::now();
  std::ofstream output(values.output + "/heldout.csv");
  require(bool(output), "Cannot write heldout CSV");
  output << "id";
  for (int idx = 0; idx < 14; ++idx) output << ",q" << idx;
  output << ",tcp0_x,tcp0_y,tcp0_z,tcp1_x,tcp1_y,tcp1_z,source_arm,source_idx\n";
  output << std::setprecision(std::numeric_limits<float>::max_digits10);
  std::mt19937_64 generator(values.sample_seed);
  std::array<size_t, 2> num_accepted{};
  size_t output_id = 0;
  for (size_t arm_idx = 0; arm_idx < 2; ++arm_idx) {
    for (size_t sample_idx = 0; sample_idx < values.sample_count; ++sample_idx) {
      q_type q{};
      for (size_t idx = arm_idx * 7; idx < (arm_idx + 1) * 7; ++idx) {
        std::uniform_real_distribution<double> distribution(context.limits[idx].first, context.limits[idx].second);
        q[idx] = static_cast<float>(distribution(generator));
      }
      if (!context.is_free(q)) continue;
      output << output_id++;
      for (const auto value : q) output << ',' << value;
      for (const auto value : context.tcp()) output << ',' << value;
      output << ',' << (arm_idx == 0 ? "L" : "R") << ',' << sample_idx << '\n';
      ++num_accepted[arm_idx];
    }
  }
  output.close(); require(bool(output), "Heldout CSV write failed");
  return {{"num_trials_per_arm", values.sample_count}, {"num_accepted_per_arm", num_accepted},
          {"seed", values.sample_seed}, {"elapsed_sec", elapsed_sec(started)}};
}
int main(int argc, char **argv) {
  try {
    static_assert(sizeof(float) == 4 && sizeof(int32_t) == 4, "Unexpected binary types");
    const uint32_t endian_test = 1;
    require(*reinterpret_cast<const uint8_t *>(&endian_test) == 1, "Little-endian machine required");
    const auto values = parse_options(argc, argv);
    require(!std::filesystem::exists(values.output) && !std::filesystem::is_symlink(values.output), "Output directory already exists");
    const auto started = clock_type::now();
    collision_context context(values.urdf, values.resource_root, values.mesh_root);
    require(std::filesystem::create_directories(values.output), "Output directory appeared during initialization");
    json metrics{{"collision_scope", "full URDF model with existing checker exclusions; strict FCL; no environment obstacles"},
        {"edge_guarantee", "discrete interpolation samples only; not continuous collision certification"},
        {"original_edges_rechecked", false}, {"edge_layer_order", {"angle", "coord_0", "coord_1"}},
        {"max_joint_step_rad", values.max_joint_step_rad}, {"num_candidates", values.num_candidates},
        {"max_added_neighbors", values.max_added_neighbors}};
    if (!values.is_sample_only) {
      auto graph = read_graph(values.gng);
      const size_t num_original_nodes = graph.nodes.size();
      int32_t max_original_id = -1;
      for (const auto &node : graph.nodes) max_original_id = std::max(max_original_id, node.id);
      auto additional_nodes = read_new_nodes(values.new_nodes, max_original_id, values.max_new_nodes);
      metrics["original_graph"] = graph_metrics(graph, num_original_nodes);
      // 元ノードの状態・生レコードは不変更。追加辺端点の衝突検証キャッシュ兼監査。
      const auto original_audit_start = clock_type::now();
      std::vector<int8_t> endpoint_state(num_original_nodes, -1);
      size_t num_original_collision_free = 0;
      size_t num_original_limit_rejected = 0;
      std::ofstream original_report(values.output + "/original_node_collision.csv");
      original_report << "id,is_collision_free,has_limit_violation\n";
      for (size_t idx = 0; idx < num_original_nodes; ++idx) {
        const auto previous_failures = context.num_limit_failures;
        const bool is_free = context.is_free(graph.nodes[idx].q);
        const bool has_limit_violation = context.num_limit_failures != previous_failures;
        endpoint_state[idx] = is_free ? 1 : 0;
        num_original_collision_free += is_free ? 1 : 0;
        num_original_limit_rejected += has_limit_violation ? 1 : 0;
        original_report << graph.nodes[idx].id << ',' << (is_free ? 1 : 0) << ',' << (has_limit_violation ? 1 : 0) << '\n';
      }
      original_report.close(); require(bool(original_report), "Original collision audit write failed");
      metrics["num_original_collision_free"] = num_original_collision_free;
      metrics["num_original_collision_rejected"] = num_original_nodes - num_original_collision_free;
      metrics["num_original_limit_rejected"] = num_original_limit_rejected;
      metrics["original_collision_audit_sec"] = elapsed_sec(original_audit_start);
      std::cout << "Original node audit free=" << num_original_collision_free << " rejected="
                << num_original_nodes - num_original_collision_free << std::endl;
      const auto validate_start = clock_type::now();
      double max_tcp_dev_m = 0.0;
      for (auto &node : additional_nodes) {
        if (!context.is_free(node.q)) {
          const auto diagnostic = diagnose_node(context, node);
          std::ofstream failure(values.output + "/failed_node.json");
          failure << diagnostic.dump(2) << '\n';
          failure.close();
          std::cerr << "NODE_DIAGNOSTIC " << diagnostic.dump() << std::endl;
          throw std::runtime_error("New node failed strict collision/limits: " + std::to_string(node.id));
        }
        const auto actual_tcp = context.tcp();
        for (size_t idx = 0; idx < 6; ++idx) max_tcp_dev_m = std::max(max_tcp_dev_m, std::abs(double(actual_tcp[idx]) - node.tcp[idx]));
        require(max_tcp_dev_m <= 2e-5, "CSV TCP does not match URDF FK");
        const Eigen::Vector3f direction = (context.chain->getEEFOrientation(0) * Eigen::Vector3d::UnitX()).cast<float>();
        make_node_bytes(node, direction);
        graph.nodes.push_back(std::move(node));
      }
      metrics["node_validation_sec"] = elapsed_sec(validate_start);
      metrics["max_tcp_component_dev_m"] = max_tcp_dev_m;
      endpoint_state.resize(graph.nodes.size(), 1);
      const auto connect_start = clock_type::now();
      size_t num_edge_candidates_checked = 0, num_rejected_edges = 0, num_added_edges = 0;
      uint64_t num_intermediate_samples = 0;
      double max_actual_joint_step_rad = 0.0;
      std::ofstream report(values.output + "/added_edges.csv");
      report << "first_id,second_id,num_segments,max_joint_step_rad,joint_dist_rad,tcp0_dist_m,tcp1_dist_m\n";
      report << std::setprecision(17);
      disjoint_set connectivity(graph.nodes.size(), num_original_nodes);
      std::unordered_map<int32_t, size_t> node_by_id;
      for (size_t idx = 0; idx < graph.nodes.size(); ++idx) node_by_id[graph.nodes[idx].id] = idx;
      for (const auto &edge : graph.edges[0]) {
        const size_t first_idx = node_by_id.at(edge.first_id), second_idx = node_by_id.at(edge.second_id);
        if (edge.is_active && graph.nodes[first_idx].is_active && graph.nodes[second_idx].is_active &&
            graph.nodes[first_idx].is_collision_free && graph.nodes[second_idx].is_collision_free) {
          connectivity.join(first_idx, second_idx);
        }
      }
      const auto pair_key = [](size_t first, size_t second) {
        if (first > second) std::swap(first, second);
        return (uint64_t(first) << 32) | uint64_t(second);
      };
      std::unordered_set<uint64_t> checked_pairs;
      const auto try_connect = [&](size_t node_idx, size_t target_idx, double squared_joint_dist) {
        if (!checked_pairs.insert(pair_key(node_idx, target_idx)).second) return false;
        ++num_edge_candidates_checked;
        const auto &node = graph.nodes[node_idx];
        const auto &target = graph.nodes[target_idx];
        if (endpoint_state[target_idx] < 0) endpoint_state[target_idx] = context.is_free(target.q) ? 1 : 0;
        if (endpoint_state[target_idx] == 0) { ++num_rejected_edges; return false; }
        double max_delta = 0;
        for (size_t idx = 0; idx < 14; ++idx) max_delta = std::max(max_delta, std::abs(double(node.q[idx]) - target.q[idx]));
        const size_t num_segments = std::max<size_t>(1, std::ceil(max_delta / values.max_joint_step_rad));
        for (size_t sample_idx = 1; sample_idx < num_segments; ++sample_idx) {
          const double ratio = double(sample_idx) / num_segments;
          std::vector<double> q(14);
          for (size_t idx = 0; idx < 14; ++idx) q[idx] = double(node.q[idx]) + ratio * (double(target.q[idx]) - node.q[idx]);
          ++num_intermediate_samples;
          if (!context.is_free(q)) { ++num_rejected_edges; return false; }
        }
        const auto edge = make_edge(node.id, target.id);
        for (auto &edges : graph.edges) edges.push_back(edge);
        connectivity.join(node_idx, target_idx);
        ++num_added_edges;
        const double actual_step = max_delta / num_segments;
        max_actual_joint_step_rad = std::max(max_actual_joint_step_rad, actual_step);
        report << edge.first_id << ',' << edge.second_id << ',' << num_segments << ',' << actual_step << ',' << std::sqrt(squared_joint_dist);
        for (size_t layer_idx = 0; layer_idx < 2; ++layer_idx) {
          double squared_dist = 0;
          for (size_t axis_idx = 0; axis_idx < 3; ++axis_idx) {
            const size_t idx = layer_idx * 3 + axis_idx;
            const double diff = double(node.tcp[idx]) - target.tcp[idx]; squared_dist += diff * diff;
          }
          report << ',' << std::sqrt(squared_dist);
        }
        report << '\n';
        return true;
      };
      // 新ノードから既に存在する姿勢への接続。元ノード間の辺は不変更。
      for (size_t node_idx = num_original_nodes; node_idx < graph.nodes.size(); ++node_idx) {
        const auto &node = graph.nodes[node_idx];
        std::vector<std::pair<double, size_t>> candidates;
        for (size_t target_idx = 0; target_idx < node_idx; ++target_idx) {
          const auto &target = graph.nodes[target_idx];
          if (!target.is_active || !target.is_collision_free) continue;
          double squared_dist = 0;
          for (size_t idx = 0; idx < 14; ++idx) { const double diff = double(node.q[idx]) - target.q[idx]; squared_dist += diff * diff; }
          candidates.emplace_back(squared_dist, target_idx);
        }
        const size_t num_candidates = std::min(values.num_candidates, candidates.size());
        std::partial_sort(candidates.begin(), candidates.begin() + num_candidates, candidates.end());
        size_t num_added_neighbors = 0;
        for (size_t candidate_idx = 0; candidate_idx < num_candidates; ++candidate_idx) {
          const size_t target_idx = candidates[candidate_idx].second;
          if (try_connect(node_idx, target_idx, candidates[candidate_idx].first)) ++num_added_neighbors;
          if (num_added_neighbors >= values.max_added_neighbors) break;
        }
        if ((node_idx - num_original_nodes + 1) % 100 == 0) {
          std::cout << "Connected new nodes " << node_idx - num_original_nodes + 1 << '/' << additional_nodes.size()
                    << " elapsed_sec=" << elapsed_sec(connect_start) << std::endl;
        }
      }
      metrics["initial_connection_sec"] = elapsed_sec(connect_start);
      const auto fallback_start = clock_type::now();
      const size_t num_initial_added_edges = num_added_edges;
      const size_t num_initial_candidates_checked = num_edge_candidates_checked;
      size_t num_fallback_passes = 0;
      const size_t max_fallback_candidates = 64;
      // 別成分への橋の追加。後続の新姿勢も含む全ノードからの候補探索。
      while (true) {
        std::set<size_t> visited_components;
        bool has_improved = false;
        bool has_unconnected_component = false;
        for (size_t member_idx = num_original_nodes; member_idx < graph.nodes.size(); ++member_idx) {
          const size_t source_root = connectivity.find(member_idx);
          if (connectivity.has_original[source_root] || !visited_components.insert(source_root).second) continue;
          has_unconnected_component = true;
          using candidate_type = std::tuple<double, size_t, size_t>;
          std::priority_queue<candidate_type> nearest;
          for (size_t source_idx = num_original_nodes; source_idx < graph.nodes.size(); ++source_idx) {
            if (connectivity.find(source_idx) != source_root) continue;
            for (size_t target_idx = 0; target_idx < graph.nodes.size(); ++target_idx) {
              const auto &target = graph.nodes[target_idx];
              if (connectivity.find(target_idx) == source_root || endpoint_state[target_idx] == 0 ||
                  !target.is_active || !target.is_collision_free ||
                  checked_pairs.count(pair_key(source_idx, target_idx))) continue;
              double squared_dist = 0;
              for (size_t idx = 0; idx < 14; ++idx) {
                const double diff = double(graph.nodes[source_idx].q[idx]) - target.q[idx];
                squared_dist += diff * diff;
              }
              const candidate_type candidate{squared_dist, source_idx, target_idx};
              if (nearest.size() < max_fallback_candidates) nearest.push(candidate);
              else if (candidate < nearest.top()) { nearest.pop(); nearest.push(candidate); }
            }
          }
          std::vector<candidate_type> candidates;
          while (!nearest.empty()) { candidates.push_back(nearest.top()); nearest.pop(); }
          std::reverse(candidates.begin(), candidates.end());
          for (const auto &[squared_dist, source_idx, target_idx] : candidates) {
            if (connectivity.find(source_idx) == connectivity.find(target_idx)) continue;
            if (try_connect(source_idx, target_idx, squared_dist)) {
              has_improved = true;
              visited_components.insert(connectivity.find(source_idx));
              break;
            }
          }
        }
        if (!has_unconnected_component) break;
        ++num_fallback_passes;
        size_t num_unconnected_nodes = 0;
        for (size_t idx = num_original_nodes; idx < graph.nodes.size(); ++idx) {
          if (!connectivity.has_original[connectivity.find(idx)]) ++num_unconnected_nodes;
        }
        std::cout << "Fallback pass=" << num_fallback_passes << " unconnected_new_nodes=" << num_unconnected_nodes
                  << " added_edges=" << num_added_edges - num_initial_added_edges << std::endl;
        if (!has_improved) break;
      }
      metrics["fallback_connection_sec"] = elapsed_sec(fallback_start);
      metrics["num_fallback_passes"] = num_fallback_passes;
      metrics["max_fallback_candidates_per_component_pass"] = max_fallback_candidates;
      metrics["num_fallback_added_edges_per_layer"] = num_added_edges - num_initial_added_edges;
      metrics["num_fallback_candidates_checked"] = num_edge_candidates_checked - num_initial_candidates_checked;
      report.close(); require(bool(report), "Edge audit write failed");
      metrics["connection_sec"] = elapsed_sec(connect_start);
      metrics["num_original_nodes"] = num_original_nodes;
      metrics["num_added_nodes"] = additional_nodes.size();
      metrics["num_output_nodes"] = graph.nodes.size();
      metrics["num_edge_candidates_checked"] = num_edge_candidates_checked;
      metrics["num_rejected_edges"] = num_rejected_edges;
      metrics["num_added_edges_per_layer"] = num_added_edges;
      metrics["num_intermediate_samples"] = num_intermediate_samples;
      metrics["max_actual_joint_step_rad"] = max_actual_joint_step_rad;
      metrics["output_graph"] = graph_metrics(graph, num_original_nodes);
      const std::string output_path = values.output + "/gng.bin";
      write_graph(graph, output_path);
      verify_output(graph, output_path);
      metrics["is_binary_roundtrip_verified"] = true;
      metrics["is_existing_gng_loader_verified"] = true;
      std::ofstream node_report(values.output + "/added_nodes.csv");
      node_report << "id,source_arm,source_idx\n";
      for (size_t idx = num_original_nodes; idx < graph.nodes.size(); ++idx) {
        const auto &node = graph.nodes[idx];
        node_report << node.id << ',' << node.source_arm << ',' << node.source_idx << '\n';
      }
      node_report.close(); require(bool(node_report), "Node audit write failed");
    }
    if (values.sample_count > 0) metrics["heldout"] = write_heldout(context, values);
    metrics["num_collision_checks"] = context.num_collision_checks;
    metrics["num_limit_failures"] = context.num_limit_failures;
    metrics["elapsed_sec"] = elapsed_sec(started);
    std::ofstream output(values.output + "/repair_metrics.json");
    output << metrics.dump(2) << '\n'; output.close(); require(bool(output), "Metrics write failed");
    std::cout << metrics.dump() << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "repair_graph: " << error.what() << std::endl;
    return 1;
  }
}
