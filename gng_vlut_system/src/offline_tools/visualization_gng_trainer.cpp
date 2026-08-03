#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "gng/GrowingNeuralGas.hpp"
#include "visualization/visualization_gng.hpp"

namespace {

using SourceGng = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

struct Options {
  std::filesystem::path input;
  std::filesystem::path output_prefix;
  robot_sim::visualization::VisualizationGngTrainingParams training;
  int layer = -1;
};

struct GraphStats {
  std::size_t connected_components = 0;
  std::size_t isolated_nodes = 0;
};

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
         " [--seed <n>] [--layer <n>]\n";
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

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const std::string value = argv[++i];
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
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.input.empty()) {
    throw std::invalid_argument("--input is required");
  }
  if (options.output_prefix.empty()) {
    options.output_prefix = options.input.parent_path() / "visualization_gng";
  }
  if (options.training.target_nodes < 2 || options.training.iterations < 1 ||
      options.training.insertion_interval < 1 ||
      options.training.max_edge_age < 1 || options.layer < -1) {
    throw std::invalid_argument("numeric options are outside the valid range");
  }
  return options;
}

void trainLayer(const SourceGng &source, const Options &options, int layer) {
  const auto source_points =
      robot_sim::visualization::collectVisualizationGngSourcePoints(source,
                                                                    layer);
  const auto started = std::chrono::steady_clock::now();
  const auto model = robot_sim::visualization::trainVisualizationGng(
      source_points, static_cast<std::uint32_t>(layer), options.training);
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
  const auto expected_edges =
      robot_sim::visualization::contractVisualizationGngEdges(source_points,
                                                               loaded.nodes);
  if (loaded.source_signature != model.source_signature ||
      loaded.coord_layer != model.coord_layer ||
      loaded.nodes.size() != model.nodes.size() ||
      loaded.edges != model.edges || loaded.edges != expected_edges ||
      empty_node_count != 0 || mapped_source_count != source_points.size() ||
      mapped_source_ids.size() != mapped_source_count ||
      mapped_source_ids != expected_source_ids) {
    throw std::runtime_error("saved file verification found inconsistent data");
  }
  const GraphStats graph_stats =
      computeGraphStats(loaded.nodes.size(), loaded.edges);

  std::cout << "layer=" << layer << " source_nodes=" << source_points.size()
            << " visual_nodes=" << loaded.nodes.size()
            << " visual_edges=" << loaded.edges.size()
            << " empty_nodes=" << empty_node_count
            << " connected_components=" << graph_stats.connected_components
            << " isolated_nodes=" << graph_stats.isolated_nodes
            << " elapsed_sec=" << elapsed.count()
            << " output=" << output << '\n';
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Options options = parseOptions(argc, argv);
    SourceGng source(7, 3, nullptr);
    if (!source.load(options.input.string())) {
      throw std::runtime_error("failed to load source GNG: " +
                               options.input.string());
    }

    const int layer_count = source.getCoordLayerCount();
    if (options.layer >= layer_count) {
      throw std::invalid_argument("requested layer does not exist");
    }
    if (options.layer >= 0) {
      trainLayer(source, options, options.layer);
    } else {
      for (int layer = 0; layer < layer_count; ++layer) {
        trainLayer(source, options, layer);
      }
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "visualization_gng_trainer: " << error.what() << '\n';
    printUsage(argv[0]);
    return 1;
  }
}
