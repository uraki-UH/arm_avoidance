#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace robot_sim::visualization {

struct VisualizationGngSourcePoint {
  int source_node_id = -1;
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  std::vector<int> coord_neighbor_source_node_ids;
};

struct VisualizationGngNode {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  std::vector<int> source_node_ids;
};

struct VisualizationGngTrainingParams {
  int target_nodes = 500;
  int iterations = 200000;
  int insertion_interval = 200;
  int max_edge_age = 200;
  float winner_learning_rate = 0.05f;
  float neighbor_learning_rate = 0.005f;
  float split_error_scale = 0.5f;
  float error_decay = 0.0005f;
  std::uint32_t seed = 42;
};

struct VisualizationGngModel {
  std::uint32_t coord_layer = 0;
  std::uint64_t source_signature = 0;
  std::vector<VisualizationGngNode> nodes;
  std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;

  bool save(const std::filesystem::path &path, std::string *error = nullptr) const;
  bool load(const std::filesystem::path &path, std::string *error = nullptr);
};

std::filesystem::path visualizationGngLayerPath(
    const std::filesystem::path &path_prefix, std::uint32_t coord_layer);

std::uint64_t computeVisualizationGngSourceSignature(
    const std::vector<VisualizationGngSourcePoint> &source_points);

std::vector<std::pair<std::uint32_t, std::uint32_t>>
contractVisualizationGngEdges(
    const std::vector<VisualizationGngSourcePoint> &source_points,
    const std::vector<VisualizationGngNode> &visual_nodes);

VisualizationGngModel trainVisualizationGng(
    std::vector<VisualizationGngSourcePoint> source_points,
    std::uint32_t coord_layer,
    const VisualizationGngTrainingParams &params);

template <typename GNGType>
std::vector<VisualizationGngSourcePoint> collectVisualizationGngSourcePoints(
    const GNGType &gng, int coord_layer) {
  std::vector<VisualizationGngSourcePoint> points;
  points.reserve(gng.getNodes().size());
  for (const auto &node : gng.getNodes()) {
    if (node.id < 0 || !node.status.active) {
      continue;
    }

    Eigen::Vector3f position;
    if (coord_layer >= 0 &&
        coord_layer < static_cast<int>(node.weight_coords.size())) {
      position = node.weight_coords[static_cast<std::size_t>(coord_layer)];
    } else if (coord_layer == 0) {
      position = node.weight_coord;
    } else {
      continue;
    }
    if (!position.allFinite()) {
      continue;
    }
    auto neighbors = gng.getNeighborsCoord(node.id, coord_layer);
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()),
                    neighbors.end());
    points.push_back({node.id, position, std::move(neighbors)});
  }
  return points;
}

}  // namespace robot_sim::visualization
