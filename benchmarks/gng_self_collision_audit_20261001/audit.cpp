// 保存GNGの全身自己衝突と胴体対腕の直接FCL照合。元データへの書込みなし。
#include "collision/geometric_self_collision_checker.hpp"
#include "reachability/joint_sampling.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include <nlohmann/json.hpp>
#include <fcl/narrowphase/collision.h>
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
struct pair_audit {
  std::string body_link;
  std::string arm_link;
  std::vector<std::pair<size_t, size_t>> object_pairs;
  bool is_normally_excluded = false;
  bool is_colliding_at_zero = false;
  std::array<size_t, 2> num_colliding{};
  std::array<json, 2> first_collision{nullptr, nullptr};
};
struct audit_options {
  std::string gng, urdf, resource_root, mesh_root, output;
  size_t max_nodes = std::numeric_limits<size_t>::max();
  int32_t min_node_id = 0;
};
static audit_options read_options(int argc, char **argv) {
  audit_options values;
  for (int idx = 1; idx < argc; ++idx) {
    const std::string name = argv[idx];
    require(idx + 1 < argc, "Missing option value: " + name);
    const std::string value = argv[++idx];
    if (name == "--gng") values.gng = value;
    else if (name == "--urdf") values.urdf = value;
    else if (name == "--resource-root") values.resource_root = value;
    else if (name == "--mesh-root") values.mesh_root = value;
    else if (name == "--output") values.output = value;
    else if (name == "--max-nodes") values.max_nodes = std::stoull(value);
    else if (name == "--min-node-id") values.min_node_id = std::stoi(value);
    else throw std::runtime_error("Unknown option: " + name);
  }
  require(!values.gng.empty() && !values.urdf.empty() && !values.resource_root.empty() &&
          !values.mesh_root.empty() && !values.output.empty(),
          "Required: --gng --urdf --resource-root --mesh-root --output");
  require(values.min_node_id >= 0, "Invalid min-node-id");
  return values;
}
static bool is_pair_colliding(const pair_audit &pair, collision_context &context) {
  // 通常除外フラグによらない、実FCL形状の全組合せ照合。
  for (const auto &[body_idx, arm_idx] : pair.object_pairs) {
    const auto body = context.checker->getFCLObject(static_cast<int>(body_idx));
    const auto arm = context.checker->getFCLObject(static_cast<int>(arm_idx));
    fcl::CollisionRequest<double> request;
    request.num_max_contacts = 1;
    request.enable_contact = false;
    fcl::CollisionResult<double> result;
    fcl::collide(body.get(), arm.get(), request, result);
    if (result.isCollision()) return true;
  }
  return false;
}
static bool apply_pose(collision_context &context, const q_type &q) {
  bool is_within_limits = true;
  for (size_t idx = 0; idx < q.size(); ++idx) {
    if (q[idx] < context.limits[idx].first || q[idx] > context.limits[idx].second) is_within_limits = false;
  }
  context.chain->updateKinematics(std::vector<double>(q.begin(), q.end()));
  const auto actual = context.chain->getJointValues();
  require(actual.size() == q.size(), "Applied q dimension mismatch");
  for (size_t idx = 0; idx < q.size(); ++idx) {
    require(actual[idx] == static_cast<double>(q[idx]), "Joint clamp prevents exact saved-pose audit");
  }
  context.checker->updateBodyPoses(context.chain->getLinkPositions(), context.chain->getLinkOrientations());
  return is_within_limits;
}
int main(int argc, char **argv) {
  try {
    const auto values = read_options(argc, argv);
    const uint32_t endian_test = 1;
    require(*reinterpret_cast<const uint8_t *>(&endian_test) == 1, "Little-endian machine required");
    require(!std::filesystem::exists(values.output) && !std::filesystem::is_symlink(values.output), "Output already exists");
    const auto started = clock_type::now();
    const auto graph = read_graph(values.gng);
    collision_context context(values.urdf, values.resource_root, values.mesh_root);
    const std::set<std::string> body_names{
        "base_link", "torso_link", "neck_pan_link", "neck_tilt_link", "waist_cover_link",
        "L_shoulder_cover_link", "R_shoulder_cover_link", "neck_tilt_cover_link", "realsense_mount_link"};
    std::map<std::string, std::vector<size_t>> objects_by_link;
    for (size_t idx = 0; idx < context.checker->getCollisionObjects().size(); ++idx) {
      objects_by_link[context.checker->getLinkNameForObject(static_cast<int>(idx))].push_back(idx);
    }
    std::vector<std::string> missing_body_links;
    std::vector<pair_audit> pairs;
    for (const auto &body_name : body_names) {
      const auto body = objects_by_link.find(body_name);
      if (body == objects_by_link.end()) { missing_body_links.push_back(body_name); continue; }
      for (const auto &[arm_name, arm_objects] : objects_by_link) {
        const bool is_arm = arm_name.rfind("L_", 0) == 0 || arm_name.rfind("R_", 0) == 0;
        if (!is_arm || body_names.count(arm_name)) continue;
        pair_audit pair;
        pair.body_link = body_name; pair.arm_link = arm_name;
        pair.is_normally_excluded = context.checker->shouldSkipCollision(body_name, arm_name);
        for (const auto body_idx : body->second) {
          for (const auto arm_idx : arm_objects) pair.object_pairs.emplace_back(body_idx, arm_idx);
        }
        pairs.push_back(std::move(pair));
      }
    }
    require(!pairs.empty(), "No body-arm collision object pairs");
    const q_type zero_q{};
    const bool is_zero_within_limits = apply_pose(context, zero_q);
    const bool is_zero_normally_colliding = context.checker->checkCollision();
    const auto zero_normal_pairs = context.checker->collectSelfCollisionPairs();
    for (auto &pair : pairs) pair.is_colliding_at_zero = is_pair_colliding(pair, context);
    require(std::filesystem::create_directories(values.output), "Output appeared during initialization");
    std::ofstream node_output(values.output + "/node_audit.csv");
    require(bool(node_output), "Cannot write node audit");
    node_output << "node_id,is_original,is_stored_active,is_stored_collision_free,is_within_limits,is_normally_colliding,num_direct_colliding_body_pairs,num_excluded_colliding_body_pairs,num_nonexcluded_colliding_body_pairs,num_colliding_body_pairs_without_zero_contact\n";
    std::array<size_t, 2> num_checked{}, num_normal_colliding{}, num_direct_colliding{}, num_excluded_colliding{}, num_nonexcluded_colliding{}, num_without_zero_contact{}, num_limit_violations{};
    std::array<json, 2> first_normal_collision{nullptr, nullptr};
    size_t num_checked_nodes = 0;
    size_t num_stored_free_normal_colliding = 0;
    const auto audit_started = clock_type::now();
    for (const auto &node : graph.nodes) {
      if (node.id < values.min_node_id) continue;
      if (num_checked_nodes >= values.max_nodes) break;
      const bool is_original = node.id < 10000;
      const size_t category_idx = is_original ? 0 : 1;
      const bool is_within_limits = apply_pose(context, node.q);
      const bool is_normally_colliding = context.checker->checkCollision();
      ++num_checked_nodes; ++num_checked[category_idx];
      if (!is_within_limits) ++num_limit_violations[category_idx];
      if (is_normally_colliding) {
        ++num_normal_colliding[category_idx];
        if (node.is_collision_free) ++num_stored_free_normal_colliding;
        if (first_normal_collision[category_idx].is_null()) {
          first_normal_collision[category_idx] = {{"id", node.id}, {"q", node.q},
              {"collision_pairs", context.checker->collectSelfCollisionPairs()}};
        }
      }
      size_t num_direct_pairs = 0, num_excluded_pairs = 0, num_pairs_without_zero_contact = 0;
      for (auto &pair : pairs) {
        if (!is_pair_colliding(pair, context)) continue;
        ++num_direct_pairs;
        if (pair.is_normally_excluded) ++num_excluded_pairs;
        if (!pair.is_colliding_at_zero) ++num_pairs_without_zero_contact;
        ++pair.num_colliding[category_idx];
        if (pair.first_collision[category_idx].is_null()) {
          pair.first_collision[category_idx] = {{"id", node.id}, {"q", node.q}};
        }
      }
      if (num_direct_pairs) ++num_direct_colliding[category_idx];
      if (num_excluded_pairs) ++num_excluded_colliding[category_idx];
      if (num_direct_pairs > num_excluded_pairs) ++num_nonexcluded_colliding[category_idx];
      if (num_pairs_without_zero_contact) ++num_without_zero_contact[category_idx];
      node_output << node.id << ',' << is_original << ',' << node.is_active << ',' << node.is_collision_free << ','
                  << is_within_limits << ',' << is_normally_colliding << ',' << num_direct_pairs << ','
                  << num_excluded_pairs << ',' << num_direct_pairs - num_excluded_pairs << ',' << num_pairs_without_zero_contact << '\n';
      if (num_checked_nodes % 1000 == 0) {
        std::cout << "audited_nodes=" << num_checked_nodes << " elapsed_sec=" << elapsed_sec(audit_started) << std::endl;
      }
    }
    node_output.close(); require(bool(node_output), "Node audit write failed");
    std::ofstream pair_output(values.output + "/body_pair_summary.csv");
    std::ofstream sample_output(values.output + "/first_collision_samples.csv");
    require(bool(pair_output) && bool(sample_output), "Cannot write pair audit");
    pair_output << "body_link,arm_link,is_normally_excluded,is_colliding_at_zero,num_original_colliding,num_added_colliding,first_original_node_id,first_added_node_id\n";
    sample_output << "body_link,arm_link,is_normally_excluded,is_colliding_at_zero,is_original,node_id";
    for (int idx = 0; idx < 14; ++idx) sample_output << ",q" << idx;
    sample_output << '\n' << std::setprecision(std::numeric_limits<float>::max_digits10);
    json pair_results = json::array();
    for (const auto &pair : pairs) {
      std::array<int32_t, 2> first_ids{-1, -1};
      for (size_t category_idx = 0; category_idx < 2; ++category_idx) {
        if (pair.first_collision[category_idx].is_null()) continue;
        first_ids[category_idx] = pair.first_collision[category_idx].at("id").get<int32_t>();
        sample_output << pair.body_link << ',' << pair.arm_link << ',' << pair.is_normally_excluded << ','
                      << pair.is_colliding_at_zero << ',' << (category_idx == 0) << ',' << first_ids[category_idx];
        for (const auto &value : pair.first_collision[category_idx].at("q")) sample_output << ',' << value.get<float>();
        sample_output << '\n';
      }
      pair_output << pair.body_link << ',' << pair.arm_link << ',' << pair.is_normally_excluded << ','
                  << pair.is_colliding_at_zero << ',' << pair.num_colliding[0] << ',' << pair.num_colliding[1] << ','
                  << first_ids[0] << ',' << first_ids[1] << '\n';
      pair_results.push_back({{"body_link", pair.body_link}, {"arm_link", pair.arm_link},
          {"num_object_pairs", pair.object_pairs.size()}, {"is_normally_excluded", pair.is_normally_excluded},
          {"is_colliding_at_zero", pair.is_colliding_at_zero}, {"num_original_colliding", pair.num_colliding[0]},
          {"num_added_colliding", pair.num_colliding[1]}, {"first_original_collision", pair.first_collision[0]},
          {"first_added_collision", pair.first_collision[1]}});
    }
    pair_output.close(); sample_output.close();
    require(bool(pair_output) && bool(sample_output), "Pair audit write failed");
    json summary{{"gng", values.gng}, {"urdf", values.urdf}, {"resource_root", values.resource_root},
        {"mesh_root", values.mesh_root}, {"num_file_nodes", graph.nodes.size()}, {"num_checked_nodes", num_checked_nodes},
        {"min_node_id", values.min_node_id}, {"category_order", {"original", "added"}},
        {"original_id_rule", "node_id < 10000"}, {"num_checked_by_category", num_checked},
        {"num_normal_colliding_by_category", num_normal_colliding},
        {"num_direct_body_colliding_by_category", num_direct_colliding},
        {"num_excluded_body_colliding_by_category", num_excluded_colliding},
        {"num_nonexcluded_body_colliding_by_category", num_nonexcluded_colliding},
        {"num_body_colliding_without_zero_contact_by_category", num_without_zero_contact},
        {"num_limit_violations_by_category", num_limit_violations},
        {"num_stored_free_normal_colliding", num_stored_free_normal_colliding},
        {"first_normal_collision_by_category", first_normal_collision},
        {"body_links", body_names}, {"missing_body_links", missing_body_links},
        {"num_body_arm_link_pairs", pairs.size()}, {"is_zero_within_limits", is_zero_within_limits},
        {"is_zero_normally_colliding", is_zero_normally_colliding}, {"zero_normal_collision_pairs", zero_normal_pairs},
        {"pairs", pair_results}, {"audit_sec", elapsed_sec(audit_started)}, {"elapsed_sec", elapsed_sec(started)},
        {"scope", "stored poses; strict existing checker plus direct body-arm FCL without pair exclusions; no environment obstacles"},
        {"limitation", "direct intersection may include intentional adjacent-link contact; zero-pose results recorded separately; no collision geometry added by this tool"}};
    std::ofstream output(values.output + "/audit_metrics.json");
    output << summary.dump(2) << '\n'; output.close(); require(bool(output), "Metrics write failed");
    std::cout << "completed_nodes=" << num_checked_nodes << " normal_colliding=" << num_normal_colliding[0] + num_normal_colliding[1]
              << " direct_body_colliding=" << num_direct_colliding[0] + num_direct_colliding[1]
              << " elapsed_sec=" << elapsed_sec(started) << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "gng_self_collision_audit: " << error.what() << std::endl;
    return 1;
  }
}
