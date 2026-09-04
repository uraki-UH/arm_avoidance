#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include "visualization/visualization_gng.hpp"

namespace {

struct Options {
  std::filesystem::path input;
  std::filesystem::path output;
  robot_sim::visualization::VisualizationGngTrainingParams training;
  robot_sim::visualization::VisualizationGngInterpolationParams interpolation;
  float default_joint_max_velocity = 0.6f;
  std::string urdf_path;
  std::string resource_root_dir;
  std::string mesh_root_dir;
  std::string root_link;
  std::string eef_link;
};

struct FkContext {
  std::unique_ptr<kinematics::KinematicChain> chain;

  Eigen::Vector3f position(const Eigen::VectorXf &joint_angle,
                           std::uint32_t coord_layer) {
    if (!chain || coord_layer != 0 ||
        joint_angle.size() != chain->getTotalDOF()) {
      throw std::runtime_error("reachability voxel FK input is invalid");
    }
    chain->updateKinematics(
        std::vector<double>(joint_angle.data(),
                            joint_angle.data() + joint_angle.size()));
    return chain->getEEFPosition().cast<float>();
  }
};

int parseInt(const std::string &value, const std::string &name) {
  std::size_t consumed = 0;
  const long parsed = std::stol(value, &consumed);
  if (consumed != value.size() || parsed < 1 || parsed > 10000000) {
    throw std::invalid_argument("invalid " + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

float parseFloat(const std::string &value, const std::string &name) {
  std::size_t consumed = 0;
  const float parsed = std::stof(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed) || parsed <= 0.0f) {
    throw std::invalid_argument("invalid " + name + ": " + value);
  }
  return parsed;
}

float parseNonnegativeFloat(const std::string &value, const std::string &name) {
  std::size_t consumed = 0;
  const float parsed = std::stof(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed) || parsed < 0.0f) {
    throw std::invalid_argument("invalid " + name + ": " + value);
  }
  return parsed;
}

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (index + 1 >= argc) {
      throw std::invalid_argument("value is required for " + argument);
    }
    const std::string value = argv[++index];
    if (argument == "--input") {
      options.input = value;
    } else if (argument == "--output") {
      options.output = value;
    } else if (argument == "--target-nodes") {
      options.training.target_nodes = parseInt(value, argument);
    } else if (argument == "--iterations") {
      options.training.iterations = parseInt(value, argument);
    } else if (argument == "--seed") {
      options.training.seed = static_cast<std::uint32_t>(parseInt(value, argument));
    } else if (argument == "--joint-motion-weight") {
      options.training.joint_motion_weight = parseFloat(value, argument);
    } else if (argument == "--workspace-motion-sec-per-m") {
      options.training.workspace_motion_sec_per_m = parseFloat(value, argument);
    } else if (argument == "--workspace-sample-resolution") {
      options.training.workspace_sample_resolution =
          parseNonnegativeFloat(value, argument);
    } else if (argument == "--default-joint-max-velocity") {
      options.default_joint_max_velocity = parseFloat(value, argument);
    } else if (argument == "--urdf-path") {
      options.urdf_path = value;
    } else if (argument == "--resource-root-dir") {
      options.resource_root_dir = value;
    } else if (argument == "--mesh-root-dir") {
      options.mesh_root_dir = value;
    } else if (argument == "--root-link") {
      options.root_link = value;
    } else if (argument == "--eef-link") {
      options.eef_link = value;
    } else if (argument == "--interpolation-joint-step") {
      options.interpolation.max_joint_step = parseFloat(value, argument);
    } else if (argument == "--max-interpolation-samples") {
      options.interpolation.max_samples_per_edge = parseInt(value, argument);
    } else if (argument == "--edge-attachment-knn") {
      options.interpolation.attachment_knn = parseInt(value, argument);
    } else if (argument == "--edge-attachment-radius-scale") {
      options.interpolation.attachment_radius_scale = parseFloat(value, argument);
    } else if (argument == "--edge-min-attachment-radius") {
      options.interpolation.min_attachment_radius = parseNonnegativeFloat(value, argument);
    } else if (argument == "--edge-max-neighbors") {
      options.interpolation.max_edge_neighbors = parseInt(value, argument);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.input.empty()) {
    throw std::invalid_argument("--input is required");
  }
  if (options.output.empty()) {
    options.output = options.input.parent_path() /
        (options.input.stem().string() + "_vis_gng.bin");
  }
  if (options.urdf_path.empty() || options.eef_link.empty() ||
      options.interpolation.max_joint_step <= 0.0f ||
      options.interpolation.max_samples_per_edge < 2 ||
      options.interpolation.attachment_knn < 1 ||
      options.interpolation.attachment_radius_scale <= 0.0f ||
      options.interpolation.min_attachment_radius < 0.0f ||
      options.interpolation.max_edge_neighbors < 1) {
    throw std::invalid_argument("reachability voxel visualization option is invalid");
  }
  return options;
}

void printUsage(const char *program) {
  std::cerr
      << "Usage: " << program
      << " --input <reachability_voxel_map.bin> [--output <vis_gng.bin>]"
         " [--target-nodes <n>] [--iterations <n>] [--seed <n>]"
         " [--joint-motion-weight <weight>]"
         " [--workspace-motion-sec-per-m <sec_per_m>]"
         " [--workspace-sample-resolution <m>]"
         " [--default-joint-max-velocity <rad_per_sec>]"
         " --urdf-path <robot.urdf> [--resource-root-dir <dir>]"
         " [--mesh-root-dir <dir>] [--root-link <link>] --eef-link <link>"
         " [--interpolation-joint-step <rad>]"
         " [--max-interpolation-samples <n>]"
         " [--edge-attachment-knn <n>]"
         " [--edge-attachment-radius-scale <scale>]"
         " [--edge-min-attachment-radius <m>]"
         " [--edge-max-neighbors <n>]\n";
}

std::vector<robot_sim::visualization::VisualizationGngSourcePoint>
makeSourcePoints(const robot_sim::visualization::VisualizationGngStaticModel &input) {
  std::vector<robot_sim::visualization::VisualizationGngSourcePoint> points;
  points.reserve(input.nodes.size());
  for (std::size_t index = 0; index < input.nodes.size(); ++index) {
    const auto &node = input.nodes[index];
    points.push_back({static_cast<int>(index), node.position, node.normal,
                      node.label, node.representative_joint_angle, {}, {}});
  }
  for (const auto &[source, target] : input.edges) {
    if (source >= points.size() || target >= points.size() || source == target) {
      continue;
    }
    points[source].angle_neighbor_source_node_ids.push_back(
        static_cast<int>(target));
    points[target].angle_neighbor_source_node_ids.push_back(
        static_cast<int>(source));
  }
  return points;
}

FkContext makeFkContext(const Options &options,
                        std::uint32_t joint_angle_dimension) {
  const simulation::RobotModel model = simulation::loadRobotFromUrdf(
      options.urdf_path, options.resource_root_dir, options.mesh_root_dir);
  const simulation::ArmConfig arm_config{options.root_link, options.eef_link,
                                          "reachability_voxel"};
  FkContext context;
  context.chain = simulation::createMultiArmKinematicChain(model, {arm_config});
  if (context.chain->getTotalDOF() !=
      static_cast<int>(joint_angle_dimension)) {
    throw std::runtime_error(
        "reachability voxel joint angle dimension does not match URDF chain");
  }
  return context;
}

}  // 無名名前空間

int main(int argc, char **argv) {
  try {
    const Options options = parseOptions(argc, argv);
    robot_sim::visualization::VisualizationGngStaticModel input;
    std::string error;
    if (!input.load(options.input, &error)) {
      throw std::runtime_error(error);
    }
    if (input.nodes.size() < 2 || input.joint_angle_dimension == 0) {
      throw std::invalid_argument("reachability voxel map has too few nodes");
    }
    auto fk_context = makeFkContext(options, input.joint_angle_dimension);
    auto training = options.training;
    training.joint_max_velocities.assign(input.joint_angle_dimension,
                                         options.default_joint_max_velocity);
    auto interpolation = options.interpolation;
    interpolation.joint_max_velocities = training.joint_max_velocities;
    const auto source_points = makeSourcePoints(input);
    auto model = robot_sim::visualization::trainVisualizationGng(
        source_points, input.coord_layer, training);
    robot_sim::visualization::precomputeVisualizationGngTransitionPaths(
        source_points, model,
        [&fk_context](const Eigen::VectorXf &joint_angle,
                      std::uint32_t coord_layer) {
          return fk_context.position(joint_angle, coord_layer);
        },
        interpolation);
    const auto static_model =
        robot_sim::visualization::makeVisualizationGngStaticModel(model);
    if (!static_model.save(options.output, &error)) {
      throw std::runtime_error(error);
    }
    robot_sim::visualization::VisualizationGngStaticModel verified;
    if (!verified.load(options.output, &error) ||
        verified.nodes.size() != static_model.nodes.size() ||
        verified.edges != static_model.edges) {
      throw std::runtime_error("visualization GNG save verification failed: " + error);
    }
    std::cout << "input_voxels=" << input.nodes.size()
              << " visual_nodes=" << verified.nodes.size()
              << " visual_edges=" << verified.edges.size()
              << " output=" << options.output << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "reachability_voxel_visualization_gng_trainer: "
              << error.what() << '\n';
    printUsage(argv[0]);
    return 1;
  }
}
