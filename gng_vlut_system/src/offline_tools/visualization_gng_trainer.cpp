#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "common/resource_utils.hpp"
#include "gng/GrowingNeuralGas.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include "visualization/visualization_gng.hpp"

namespace {

using SourceGng = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

struct Options {
  std::filesystem::path input;
  std::filesystem::path output_prefix;
  robot_sim::visualization::VisualizationGngTrainingParams training;
  robot_sim::visualization::VisualizationGngInterpolationParams interpolation;
  float default_joint_max_velocity = 0.6f;
  int layer = -1;
};

struct FkContext {
  std::unique_ptr<kinematics::KinematicChain> chain;
  std::vector<std::pair<std::size_t, int>> selected_dofs;
  std::vector<float> selected_joint_max_velocities;
  int selected_dof_count = 0;

  Eigen::Vector3f position(const Eigen::VectorXf &selected_values,
                           std::uint32_t layer) {
    if (!chain || selected_values.size() != selected_dof_count ||
        layer >= chain->getArmCount()) {
      throw std::runtime_error("FK input does not match the selected GNG joints");
    }
    std::vector<double> full_values(
        static_cast<std::size_t>(chain->getTotalDOF()), 0.0);
    std::size_t selected_cursor = 0;
    for (const auto &[full_offset, dof] : selected_dofs) {
      for (int d = 0; d < dof; ++d) {
        full_values[full_offset + static_cast<std::size_t>(d)] =
            selected_values[static_cast<int>(selected_cursor++)];
      }
    }
    chain->updateKinematics(full_values);
    return chain->getEEFPosition(layer).cast<float>();
  }
};

struct GraphStats {
  std::size_t connected_components = 0;
  std::size_t isolated_nodes = 0;
};

bool transitionsEqual(
    const robot_sim::visualization::VisualizationGngModel &lhs,
    const robot_sim::visualization::VisualizationGngModel &rhs) {
  if (lhs.transition_paths.size() != rhs.transition_paths.size() ||
      lhs.transition_path_nodes != rhs.transition_path_nodes) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.transition_paths.size(); ++i) {
    const auto &lhs_path = lhs.transition_paths[i];
    const auto &rhs_path = rhs.transition_paths[i];
    if (lhs_path.source_node_id != rhs_path.source_node_id ||
        lhs_path.target_node_id != rhs_path.target_node_id ||
        lhs_path.path_offset != rhs_path.path_offset ||
        lhs_path.path_size != rhs_path.path_size ||
        lhs_path.motion_time_sec != rhs_path.motion_time_sec ||
        lhs_path.has_visual_connection != rhs_path.has_visual_connection) {
      return false;
    }
  }
  return true;
}

bool nodesEqual(
    const std::vector<robot_sim::visualization::VisualizationGngNode> &lhs,
    const std::vector<robot_sim::visualization::VisualizationGngNode> &rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    if (!(lhs[index].position.array() == rhs[index].position.array()).all() ||
        !(lhs[index].normal.array() == rhs[index].normal.array()).all() ||
        lhs[index].label != rhs[index].label ||
        lhs[index].representative_source_node_id !=
            rhs[index].representative_source_node_id ||
        lhs[index].representative_joint_angle.size() !=
            rhs[index].representative_joint_angle.size() ||
        (lhs[index].representative_joint_angle.array() !=
         rhs[index].representative_joint_angle.array()).any() ||
        lhs[index].source_node_ids != rhs[index].source_node_ids) {
      return false;
    }
  }
  return true;
}

bool staticNodesEqual(
    const std::vector<robot_sim::visualization::VisualizationGngStaticNode> &lhs,
    const std::vector<robot_sim::visualization::VisualizationGngStaticNode> &rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    if (!(lhs[index].position.array() == rhs[index].position.array()).all() ||
        !(lhs[index].normal.array() == rhs[index].normal.array()).all() ||
        lhs[index].label != rhs[index].label ||
        lhs[index].representative_joint_angle.size() !=
            rhs[index].representative_joint_angle.size() ||
        (lhs[index].representative_joint_angle.array() !=
         rhs[index].representative_joint_angle.array()).any()) {
      return false;
    }
  }
  return true;
}

GraphStats computeGraphStats(
    std::size_t node_count,
    const std::vector<std::pair<std::uint32_t, std::uint32_t>> &edges) {
  std::vector<std::vector<std::uint32_t>> neighbors(node_count);
  for (const auto &[source, target] : edges) {
    neighbors[source].push_back(target);
    neighbors[target].push_back(source);
  }

  GraphStats stats;
  std::vector<bool> visited(node_count, false);
  std::vector<std::uint32_t> pending;
  for (std::uint32_t start = 0; start < node_count; ++start) {
    if (neighbors[start].empty()) {
      ++stats.isolated_nodes;
    }
    if (visited[start]) {
      continue;
    }
    ++stats.connected_components;
    pending.push_back(start);
    visited[start] = true;
    while (!pending.empty()) {
      const std::uint32_t current = pending.back();
      pending.pop_back();
      for (const std::uint32_t neighbor : neighbors[current]) {
        if (!visited[neighbor]) {
          visited[neighbor] = true;
          pending.push_back(neighbor);
        }
      }
    }
  }
  return stats;
}

void printUsage(const char *program) {
  std::cerr
      << "Usage: " << program
      << " --input <gng.bin> [--output-prefix <path>]"
         " [--target-nodes <n>] [--iterations <n>]"
         " [--insertion-interval <n>] [--max-edge-age <n>]"
         " [--seed <n>] [--layer <n>]"
         " [--interpolation-joint-step <rad>]"
         " [--max-interpolation-samples <n>]"
         " [--edge-attachment-knn <n>]"
         " [--edge-attachment-radius-scale <scale>]"
         " [--edge-min-attachment-radius <m>]"
         " [--edge-max-neighbors <n>]"
         " [--joint-motion-weight <weight>]"
         " [--workspace-motion-sec-per-m <sec_per_m>]"
         " [--workspace-sample-resolution <m>]"
         " [--default-joint-max-velocity <rad_per_sec>]"
         " --ros-args --params-file <robot.yaml>\n";
}

int parseInt(const std::string &value, const std::string &name) {
  std::size_t consumed = 0;
  const long parsed = std::stol(value, &consumed);
  if (consumed != value.size() || parsed < std::numeric_limits<int>::min() ||
      parsed > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("invalid " + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

std::uint32_t parseUint32(const std::string &value,
                          const std::string &name) {
  std::size_t consumed = 0;
  const unsigned long parsed = std::stoul(value, &consumed);
  if (consumed != value.size() ||
      parsed > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("invalid " + name + ": " + value);
  }
  return static_cast<std::uint32_t>(parsed);
}

float parseFloat(const std::string &value, const std::string &name) {
  std::size_t consumed = 0;
  const float parsed = std::stof(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument("invalid " + name + ": " + value);
  }
  return parsed;
}

Options parseOptions(const std::vector<std::string> &arguments) {
  Options options;
  for (std::size_t i = 1; i < arguments.size(); ++i) {
    const std::string &argument = arguments[i];
    if (argument == "--help" || argument == "-h") {
      printUsage(arguments[0].c_str());
      std::exit(0);
    }
    if (i + 1 >= arguments.size()) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const std::string &value = arguments[++i];
    if (argument == "--input") {
      options.input = value;
    } else if (argument == "--output-prefix") {
      options.output_prefix = value;
    } else if (argument == "--target-nodes") {
      options.training.target_nodes = parseInt(value, argument);
    } else if (argument == "--iterations") {
      options.training.iterations = parseInt(value, argument);
    } else if (argument == "--insertion-interval") {
      options.training.insertion_interval = parseInt(value, argument);
    } else if (argument == "--max-edge-age") {
      options.training.max_edge_age = parseInt(value, argument);
    } else if (argument == "--seed") {
      options.training.seed = parseUint32(value, argument);
    } else if (argument == "--layer") {
      options.layer = parseInt(value, argument);
    } else if (argument == "--interpolation-joint-step") {
      options.interpolation.max_joint_step = parseFloat(value, argument);
    } else if (argument == "--max-interpolation-samples") {
      options.interpolation.max_samples_per_edge = parseInt(value, argument);
    } else if (argument == "--edge-attachment-knn") {
      options.interpolation.attachment_knn = parseInt(value, argument);
    } else if (argument == "--edge-attachment-radius-scale") {
      options.interpolation.attachment_radius_scale = parseFloat(value, argument);
    } else if (argument == "--edge-min-attachment-radius") {
      options.interpolation.min_attachment_radius = parseFloat(value, argument);
    } else if (argument == "--edge-max-neighbors") {
      options.interpolation.max_edge_neighbors = parseInt(value, argument);
    } else if (argument == "--joint-motion-weight") {
      options.training.joint_motion_weight = parseFloat(value, argument);
    } else if (argument == "--workspace-motion-sec-per-m") {
      options.training.workspace_motion_sec_per_m = parseFloat(value, argument);
    } else if (argument == "--workspace-sample-resolution") {
      options.training.workspace_sample_resolution = parseFloat(value, argument);
    } else if (argument == "--default-joint-max-velocity") {
      options.default_joint_max_velocity = parseFloat(value, argument);
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.input.empty()) {
    throw std::invalid_argument("--input is required");
  }
  if (options.output_prefix.empty()) {
    options.output_prefix = options.input.parent_path() / "vis_gng";
  }
  if (options.training.target_nodes < 2 || options.training.iterations < 1 ||
      options.training.insertion_interval < 1 ||
      options.training.max_edge_age < 1 || options.layer < -1) {
    throw std::invalid_argument("numeric options are outside the valid range");
  }
  if (options.interpolation.max_joint_step <= 0.0f ||
      options.interpolation.max_samples_per_edge < 2 ||
      options.interpolation.attachment_knn < 1 ||
      options.interpolation.attachment_radius_scale <= 0.0f ||
      options.interpolation.min_attachment_radius < 0.0f ||
      options.interpolation.max_edge_neighbors < 1 ||
      options.training.joint_motion_weight <= 0.0f ||
      options.training.workspace_motion_sec_per_m <= 0.0f ||
      options.training.workspace_sample_resolution < 0.0f ||
      options.default_joint_max_velocity <= 0.0f) {
    throw std::invalid_argument(
        "interpolation options are outside the valid range");
  }
  return options;
}

std::vector<std::string> splitCommaSeparated(const std::string &text) {
  std::vector<std::string> values;
  std::stringstream stream(text);
  std::string value;
  while (std::getline(stream, value, ',')) {
    const auto begin = value.find_first_not_of(" \t");
    if (begin == std::string::npos) {
      continue;
    }
    const auto end = value.find_last_not_of(" \t");
    values.push_back(value.substr(begin, end - begin + 1));
  }
  return values;
}

std::vector<std::string> getStringList(const rclcpp::Node &node,
                                       const std::string &name) {
  if (!node.has_parameter(name)) {
    return {};
  }
  const auto parameter = node.get_parameter(name);
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    return parameter.as_string_array();
  }
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    return splitCommaSeparated(parameter.as_string());
  }
  return {};
}

std::string getString(const rclcpp::Node &node, const std::string &name) {
  if (!node.has_parameter(name)) {
    return {};
  }
  const auto parameter = node.get_parameter(name);
  return parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING
             ? parameter.as_string()
             : std::string{};
}

void appendUnique(std::vector<std::string> &target,
                  const std::vector<std::string> &values) {
  std::unordered_set<std::string> existing(target.begin(), target.end());
  for (const auto &value : values) {
    if (existing.insert(value).second) {
      target.push_back(value);
    }
  }
}

FkContext buildFkContext(const rclcpp::Node &node,
                         float default_joint_max_velocity) {
  std::string urdf_path = getString(node, "urdf_path");
  if (urdf_path.empty()) {
    urdf_path = getString(node, "robot_urdf_path");
  }
  urdf_path = robot_sim::common::resolvePath(urdf_path);
  if (urdf_path.empty() || !std::filesystem::exists(urdf_path)) {
    throw std::runtime_error(
        "urdf_path is required to precompute transition interpolation");
  }
  const auto model = simulation::loadRobotFromUrdf(
      urdf_path, getString(node, "resource_root_dir"),
      getString(node, "mesh_root_dir"));

  auto profile_names = getStringList(node, "gng.profile_names");
  if (profile_names.empty()) {
    profile_names = getStringList(node, "gng.profile_name");
  }
  if (profile_names.empty()) {
    throw std::runtime_error("gng.profile_names is required for FK setup");
  }

  std::vector<simulation::ArmConfig> arm_configs;
  std::vector<std::string> include_joint_names;
  std::vector<std::string> exclude_joint_names;
  for (const auto &profile_name : profile_names) {
    const std::string prefix = "gng.profiles." + profile_name + ".";
    auto leaf_names = splitCommaSeparated(getString(node, prefix + "eef"));
    if (leaf_names.empty()) {
      leaf_names =
          splitCommaSeparated(getString(node, prefix + "eef_link_names"));
    }
    if (leaf_names.empty()) {
      leaf_names = splitCommaSeparated(
          getString(node, prefix + "arm_leaf_link_names"));
    }
    if (leaf_names.empty()) {
      throw std::runtime_error("profile " + profile_name +
                               " does not define an end-effector link");
    }
    const std::string root = getString(node, prefix + "root");
    for (const auto &leaf : leaf_names) {
      arm_configs.push_back({root, leaf, ""});
    }
    appendUnique(include_joint_names,
                 getStringList(node, prefix + "include_joint_names"));
    appendUnique(exclude_joint_names,
                 getStringList(node, prefix + "exclude_joint_names"));
    appendUnique(exclude_joint_names,
                 getStringList(node, prefix + "dof_exclude"));
    appendUnique(exclude_joint_names,
                 getStringList(node, prefix + "dof_exclude_joint_names"));
  }
  appendUnique(include_joint_names,
               getStringList(node, "gng.include_joint_names"));
  appendUnique(exclude_joint_names,
               getStringList(node, "gng.exclude_joint_names"));
  appendUnique(exclude_joint_names,
               getStringList(node, "gng.dof_exclude_joint_names"));

  FkContext context;
  context.chain = simulation::createMultiArmKinematicChain(model, arm_configs);
  const std::unordered_set<std::string> include_set(include_joint_names.begin(),
                                                    include_joint_names.end());
  const std::unordered_set<std::string> exclude_set(exclude_joint_names.begin(),
                                                    exclude_joint_names.end());
  std::size_t full_offset = 0;
  for (int joint = 0; joint < context.chain->getNumJoints(); ++joint) {
    const int dof = context.chain->getJointDOF(joint);
    if (dof <= 0) {
      continue;
    }
    const std::string name = context.chain->getJointName(joint);
    const bool included = include_set.empty() || include_set.count(name) > 0;
    if (included && exclude_set.count(name) == 0) {
      context.selected_dofs.emplace_back(full_offset, dof);
      context.selected_dof_count += dof;
      const auto *joint_props = model.getJoint(name);
      const float max_velocity =
          joint_props && std::isfinite(joint_props->limits.velocity) &&
                  joint_props->limits.velocity > 0.0
              ? static_cast<float>(joint_props->limits.velocity)
              : default_joint_max_velocity;
      for (int d = 0; d < dof; ++d) {
        context.selected_joint_max_velocities.push_back(max_velocity);
      }
    }
    full_offset += static_cast<std::size_t>(dof);
  }
  if (context.selected_dof_count <= 0) {
    throw std::runtime_error("FK joint selection is empty");
  }
  if (context.selected_joint_max_velocities.size() !=
      static_cast<std::size_t>(context.selected_dof_count)) {
    throw std::runtime_error("FK joint velocity setup is inconsistent");
  }
  return context;
}

void trainLayer(const SourceGng &source, const Options &options, int layer,
                FkContext &fk_context) {
  const auto source_points =
      robot_sim::visualization::collectVisualizationGngSourcePoints(source,
                                                                    layer);
  const auto started = std::chrono::steady_clock::now();
  auto model = robot_sim::visualization::trainVisualizationGng(
      source_points, static_cast<std::uint32_t>(layer), options.training);
  if (source_points.empty() ||
      source_points.front().weight_angle.size() !=
          fk_context.selected_dof_count) {
    throw std::runtime_error(
        "source GNG angle dimension does not match the selected FK joints");
  }
  robot_sim::visualization::precomputeVisualizationGngTransitionPaths(
      source_points, model,
      [&fk_context](const Eigen::VectorXf &angle, std::uint32_t coord_layer) {
        return fk_context.position(angle, coord_layer);
      },
      options.interpolation);
  const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started);
  const auto output = robot_sim::visualization::visualizationGngLayerPath(
      options.output_prefix, static_cast<std::uint32_t>(layer));

  std::string error;
  if (!model.save(output, &error)) {
    throw std::runtime_error(error);
  }
  robot_sim::visualization::VisualizationGngModel loaded;
  if (!loaded.load(output, &error)) {
    throw std::runtime_error("saved file verification failed: " + error);
  }
  std::size_t mapped_source_count = 0;
  std::size_t empty_node_count = 0;
  std::unordered_set<int> mapped_source_ids;
  for (const auto &node : loaded.nodes) {
    mapped_source_count += node.source_node_ids.size();
    empty_node_count += node.source_node_ids.empty() ? 1 : 0;
    mapped_source_ids.insert(node.source_node_ids.begin(),
                             node.source_node_ids.end());
  }
  std::unordered_set<int> expected_source_ids;
  for (const auto &source_point : source_points) {
    expected_source_ids.insert(source_point.source_node_id);
  }
  if (loaded.source_signature != model.source_signature ||
      loaded.coord_layer != model.coord_layer ||
      loaded.joint_angle_dimension != model.joint_angle_dimension ||
      !nodesEqual(loaded.nodes, model.nodes) ||
      loaded.edges != model.edges ||
      !transitionsEqual(loaded, model) ||
      empty_node_count != 0 || mapped_source_count != source_points.size() ||
      mapped_source_ids.size() != mapped_source_count ||
      mapped_source_ids != expected_source_ids) {
    throw std::runtime_error("saved file verification found inconsistent data");
  }
  const auto static_output =
      robot_sim::visualization::visualizationGngStaticLayerPath(
          options.output_prefix, static_cast<std::uint32_t>(layer));
  const auto static_model =
      robot_sim::visualization::makeVisualizationGngStaticModel(loaded);
  if (!static_model.save(static_output, &error)) {
    throw std::runtime_error(error);
  }
  robot_sim::visualization::VisualizationGngStaticModel loaded_static_model;
  if (!loaded_static_model.load(static_output, &error) ||
      loaded_static_model.coord_layer != static_model.coord_layer ||
      loaded_static_model.joint_angle_dimension !=
          static_model.joint_angle_dimension ||
      !staticNodesEqual(loaded_static_model.nodes, static_model.nodes) ||
      loaded_static_model.edges != static_model.edges) {
    throw std::runtime_error("static GNG file verification found inconsistent data");
  }
  const GraphStats graph_stats =
      computeGraphStats(loaded.nodes.size(), loaded.edges);

  std::cout << "layer=" << layer << " source_nodes=" << source_points.size()
            << " visual_nodes=" << loaded.nodes.size()
            << " visual_edges=" << loaded.edges.size()
            << " transition_overrides=" << loaded.transition_paths.size()
            << " empty_nodes=" << empty_node_count
            << " connected_components=" << graph_stats.connected_components
            << " isolated_nodes=" << graph_stats.isolated_nodes
            << " elapsed_sec=" << elapsed.count()
            << " output=" << output
            << " static_output=" << static_output << '\n';
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    const auto non_ros_arguments = rclcpp::remove_ros_arguments(argc, argv);
    Options options = parseOptions(non_ros_arguments);
    const auto node = std::make_shared<rclcpp::Node>(
        "visualization_gng_trainer", rclcpp::NodeOptions()
                                             .allow_undeclared_parameters(true)
                                             .automatically_declare_parameters_from_overrides(true));
    auto fk_context = buildFkContext(*node, options.default_joint_max_velocity);
    options.training.joint_max_velocities =
        fk_context.selected_joint_max_velocities;
    options.interpolation.joint_max_velocities =
        fk_context.selected_joint_max_velocities;
    SourceGng source(fk_context.selected_dof_count, 3, nullptr);
    if (!source.load(options.input.string())) {
      throw std::runtime_error("failed to load source GNG: " +
                               options.input.string());
    }

    const int layer_count = source.getCoordLayerCount();
    if (options.layer >= layer_count) {
      throw std::invalid_argument("requested layer does not exist");
    }
    if (options.layer >= 0) {
      trainLayer(source, options, options.layer, fk_context);
    } else {
      for (int layer = 0; layer < layer_count; ++layer) {
        trainLayer(source, options, layer, fk_context);
      }
    }
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "visualization_gng_trainer: " << error.what() << '\n';
    printUsage(argv[0]);
    rclcpp::shutdown();
    return 1;
  }
}
