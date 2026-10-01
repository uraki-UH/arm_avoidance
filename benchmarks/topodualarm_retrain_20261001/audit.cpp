// 保存GNGを再読込した全姿勢・全層辺の自己干渉監査
#include "collision/geometric_self_collision_checker.hpp"
#include "gng/GrowingNeuralGas.hpp"
#include "reachability/joint_sampling.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>

using json = nlohmann::json;
using graph_type = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

int main(int argc, char **argv) {
  try {
    if (argc != 2) throw std::runtime_error("Usage: audit config.json");
    const auto started = std::chrono::steady_clock::now();
    std::ifstream input(argv[1]);
    const json config = json::parse(input);
    const std::string output_path = config.at("output");
    if (std::filesystem::exists(output_path)) throw std::runtime_error("Output exists");
    const auto model = simulation::loadRobotFromUrdf(
        config.at("urdf"), config.at("resource_root"), config.at("mesh_root"));
    auto chain = simulation::createMultiArmKinematicChain(model,
        {{config.at("root"), config.at("eef"), ""}});
    simulation::GeometricSelfCollisionChecker checker(model, *chain, true, config.at("voxel_size"));
    checker.setStrictMode(true);
    for (const auto &pair : config.at("exclusions")) checker.addCollisionExclusion(pair[0], pair[1]);
    const auto limits = robot_sim::reachability::collect_joint_limits(model, *chain);
    graph_type graph(chain->getTotalDOF(), 3, nullptr);
    if (!graph.load(config.at("gng_path"))) throw std::runtime_error("GNG load failed");
    if (graph.getCoordLayerCount() != 1) throw std::runtime_error("Expected one coordinate layer");
    std::size_t num_checks = 0, num_nodes = 0, num_colliding = 0, num_invalid = 0;
    std::size_t num_stored_unsafe = 0;
    double max_fk_error_m = 0;
    std::map<std::string, std::size_t> pairs;
    std::map<int, bool> is_node_safe;
    json examples = json::array();
    json edge_examples = json::array();
    json states = json::object();
    const bool enable_geometry_checks = config.value("enable_geometry_checks", true);
    std::set<std::pair<int, int>> selected_edges;
    for (const auto &pair : config.value("edge_pairs", json::array()))
      selected_edges.emplace(pair[0].get<int>(), pair[1].get<int>());
    const auto is_valid = [&](const Eigen::VectorXf &q) {
      if (q.size() != static_cast<int>(limits.size()) || !q.allFinite()) return false;
      for (int idx = 0; idx < q.size(); ++idx) {
        if (q[idx] < limits[idx].first || q[idx] > limits[idx].second) return false;
      }
      return true;
    };
    const auto is_free = [&](const Eigen::VectorXf &q) {
      std::vector<double> values(q.data(), q.data() + q.size());
      chain->updateKinematics(values);
      if (chain->getJointValues() != values) throw std::runtime_error("Unexpected joint clamping");
      if (!enable_geometry_checks) return true;
      if (config.value("enable_pure_fk", false)) {
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
        chain->forwardKinematicsAt(values, positions, orientations);
        checker.updateBodyPoses(positions, orientations);
      } else {
        checker.updateBodyPoses(chain->getLinkPositions(), chain->getLinkOrientations());
      }
      ++num_checks;
      return !checker.checkCollision();
    };
    for (const auto &node : graph.get_nodes()) {
      if (node.id < 0) continue;
      ++num_nodes;
      if (!node.status.self_collision_free || !node.status.active) ++num_stored_unsafe;
      if (!is_valid(node.weight_angle)) {
        ++num_invalid; is_node_safe[node.id] = false; continue;
      }
      const bool is_safe = is_free(node.weight_angle);
      is_node_safe[node.id] = is_safe;
      if (node.weight_coords.size() != 1 || !node.weight_coords[0].allFinite())
        throw std::runtime_error("Invalid stored TCP");
      max_fk_error_m = std::max(max_fk_error_m,
          (chain->getEEFPosition() - node.weight_coords[0].cast<double>()).norm());
      states[std::to_string(node.id)] = {
          {"q", std::vector<float>(node.weight_angle.data(), node.weight_angle.data() + node.weight_angle.size())},
          {"pos", std::vector<float>(node.weight_coords[0].data(), node.weight_coords[0].data() + 3)}};
      if (!is_safe) {
        ++num_colliding;
        const auto collisions = checker.collectSelfCollisionPairs();
        json names = json::array();
        for (const auto &pair : collisions) {
          const std::string name = pair.first + "|" + pair.second;
          ++pairs[name]; names.push_back(name);
        }
        if (examples.size() < 20) examples.push_back({{"node_id", node.id}, {"pairs", names},
            {"q", std::vector<float>(node.weight_angle.data(), node.weight_angle.data() + node.weight_angle.size())}});
      }
    }
    std::cout << "Audited nodes=" << num_nodes << " colliding=" << num_colliding << std::endl;
    std::map<std::pair<int, int>, bool> edge_safety;
    json layers = json::array();
    for (int layer_idx = 0; layer_idx <= graph.getCoordLayerCount(); ++layer_idx) {
      std::size_t num_edges = 0, num_unsafe_edges = 0, num_isolated_nodes = 0;
      std::set<int> remaining;
      for (const auto &[id, unused] : is_node_safe) remaining.insert(id);
      const auto neighbors = [&](int id) -> const std::vector<int> & {
        return layer_idx == 0 ? graph.getNeighborsAngle(id) : graph.getNeighborsCoord(id, layer_idx - 1);
      };
      for (const auto &[id, is_safe] : is_node_safe) {
        if (neighbors(id).empty()) ++num_isolated_nodes;
        for (int other : neighbors(id)) {
          if (!is_node_safe.count(other)) throw std::runtime_error("Missing edge endpoint");
          if (id >= other) continue;
          if (!selected_edges.empty() && !selected_edges.count({id, other})) continue;
          ++num_edges;
          if (!config.at("enable_edges").get<bool>()) continue;
          const auto key = std::make_pair(id, other);
          auto found = edge_safety.find(key);
          if (found == edge_safety.end()) {
            bool is_edge_safe = is_safe && is_node_safe.at(other);
            const auto &first = graph.nodeAt(id).weight_angle;
            const auto &second = graph.nodeAt(other).weight_angle;
            const int num_steps = std::max(1, static_cast<int>(std::ceil((second-first).cwiseAbs().maxCoeff() / 0.025)));
            for (int idx = 1; is_edge_safe && idx < num_steps; ++idx) {
              const Eigen::VectorXf q = first + (second-first) * (static_cast<float>(idx) / num_steps);
              const bool is_joint_valid = is_valid(q);
              is_edge_safe = is_joint_valid && is_free(q);
              if (!is_edge_safe && edge_examples.size() < 20) {
                json names = json::array();
                if (is_joint_valid && enable_geometry_checks)
                  for (const auto &pair : checker.collectSelfCollisionPairs())
                    names.push_back({pair.first, pair.second});
                json example = {{"ids", {id, other}}, {"sample_idx", idx}, {"num_steps", num_steps},
                    {"is_joint_valid", is_joint_valid}, {"pairs", names},
                    {"q", std::vector<float>(q.data(), q.data() + q.size())}};
                edge_examples.push_back(example);
                std::cout << "UNSAFE_EDGE " << example.dump() << std::endl;
              }
            }
            found = edge_safety.emplace(key, is_edge_safe).first;
          }
          if (!found->second) ++num_unsafe_edges;
        }
      }
      std::size_t num_components = 0, max_component_nodes = 0;
      while (!remaining.empty()) {
        std::vector<int> stack{*remaining.begin()}; remaining.erase(stack[0]);
        std::size_t num_component_nodes = 0; ++num_components;
        while (!stack.empty()) {
          const int id = stack.back(); stack.pop_back(); ++num_component_nodes;
          for (int other : neighbors(id)) if (remaining.erase(other)) stack.push_back(other);
        }
        max_component_nodes = std::max(max_component_nodes, num_component_nodes);
      }
      layers.push_back({{"layer_idx", layer_idx}, {"num_edges", num_edges},
          {"num_unsafe_edges", config.at("enable_edges").get<bool>() ? json(num_unsafe_edges) : json(nullptr)},
          {"num_components", num_components}, {"max_component_nodes", max_component_nodes},
          {"num_isolated_nodes", num_isolated_nodes}});
    }
    const json result = {{"config", config}, {"num_nodes", num_nodes}, {"num_colliding", num_colliding},
        {"num_limit_failures", num_invalid}, {"num_stored_unsafe", num_stored_unsafe},
        {"max_fk_error_m", max_fk_error_m}, {"num_collision_checks", num_checks},
        {"collision_pairs", pairs}, {"examples", examples}, {"edge_examples", edge_examples}, {"layers", layers},
        {"elapsed_sec", std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()}};
    std::ofstream output(output_path);
    output << result.dump(2) << '\n'; output.close();
    if (!output) throw std::runtime_error("Cannot save audit result");
    if (config.at("enable_edges").get<bool>()) {
      std::ofstream state_output(output_path + ".states.json");
      state_output << states.dump() << '\n'; state_output.close();
      if (!state_output) throw std::runtime_error("Cannot save audit states");
    }
    std::cout << "AUDIT " << output_path << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
