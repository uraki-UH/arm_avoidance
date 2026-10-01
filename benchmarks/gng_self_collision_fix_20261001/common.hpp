#pragma once
// GNG v9保存と既存ローダー照合、修正済み自己干渉判定の共通処理。
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
struct collision_context {
  simulation::RobotModel model;
  std::unique_ptr<kinematics::KinematicChain> chain;
  std::unique_ptr<simulation::GeometricSelfCollisionChecker> checker;
  std::vector<std::pair<double, double>> limits;
  uint64_t num_collision_checks = 0;
  uint64_t num_limit_failures = 0;
  uint64_t num_cache_hits = 0;
  std::unordered_map<std::string, bool> pose_cache;
  collision_context(const std::string &urdf, const std::string &resource_root, const std::string &mesh_root,
                    const std::vector<std::pair<std::string, std::string>> &exclusion_pairs, double voxel_size)
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
    checker = std::make_unique<simulation::GeometricSelfCollisionChecker>(model, *chain, true, voxel_size);
    checker->setStrictMode(true);
    for (const auto &pair : exclusion_pairs) {
      require(pair.first != pair.second && model.getLink(pair.first) && model.getLink(pair.second),
              "Unknown or identical explicit exclusion links");
      checker->addCollisionExclusion(pair.first, pair.second);
    }
    size_t num_expected_meshes = 0;
    for (const auto &entry : model.getLinks()) {
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
  bool is_within_limits(const q_type &q) const {
    for (size_t idx = 0; idx < q.size(); ++idx) {
      if (!std::isfinite(q[idx]) || q[idx] < limits[idx].first || q[idx] > limits[idx].second) return false;
    }
    return true;
  }
  bool is_free(const q_type &q) {
    // float32関節角の全ビット一致による同一姿勢だけの再利用。
    const std::string key(reinterpret_cast<const char *>(q.data()), sizeof(q));
    const auto found = pose_cache.find(key);
    if (found != pose_cache.end()) {
      ++num_cache_hits;
      if (is_within_limits(q)) {
        chain->updateKinematics(std::vector<double>(q.begin(), q.end()));
        checker->updateBodyPoses(chain->getLinkPositions(), chain->getLinkOrientations());
      }
      return found->second;
    }
    const bool is_safe = is_free(std::vector<double>(q.begin(), q.end()));
    if (pose_cache.size() < 250000) pose_cache.emplace(key, is_safe);
    return is_safe;
  }
  tcp_type tcp() const {
    tcp_type values{};
    for (size_t layer_idx = 0; layer_idx < 2; ++layer_idx) {
      const auto point = chain->getEEFPosition(layer_idx);
      for (size_t idx = 0; idx < 3; ++idx) values[3 * layer_idx + idx] = static_cast<float>(point[idx]);
    }
    return values;
  }
};
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
