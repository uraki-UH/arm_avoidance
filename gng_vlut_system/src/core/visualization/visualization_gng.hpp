#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace robot_sim::visualization {

struct VisualizationGngSourcePoint {
  int source_node_id = -1;
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::VectorXf weight_angle;
  std::vector<int> angle_neighbor_source_node_ids;
  std::vector<int> coord_neighbor_source_node_ids;
};

struct VisualizationGngNode {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  std::vector<int> source_node_ids;
};

struct VisualizationGngTransitionPath {
  int source_node_id = -1;
  int target_node_id = -1;
  std::uint32_t path_offset = 0;
  std::uint16_t path_size = 0;
};

struct VisualizationGngInterpolationParams {
  float max_joint_step = 0.05f;
  int max_samples_per_edge = 256;
};

inline std::uint64_t visualizationGngSourceEdgeKey(int source_node_id,
                                                   int target_node_id) {
  const std::uint32_t source = static_cast<std::uint32_t>(
      std::min(source_node_id, target_node_id));
  const std::uint32_t target = static_cast<std::uint32_t>(
      std::max(source_node_id, target_node_id));
  return (static_cast<std::uint64_t>(source) << 32U) | target;
}

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
  std::vector<VisualizationGngTransitionPath> transition_paths;
  std::vector<std::uint16_t> transition_path_nodes;

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

using VisualizationGngFkFunction = std::function<Eigen::Vector3f(
    const Eigen::VectorXf &, std::uint32_t coord_layer)>;

void precomputeVisualizationGngTransitionPaths(
    const std::vector<VisualizationGngSourcePoint> &source_points,
    VisualizationGngModel &model, const VisualizationGngFkFunction &fk,
    const VisualizationGngInterpolationParams &params = {});

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
    auto angle_neighbors = gng.getNeighborsAngle(node.id);
    std::sort(angle_neighbors.begin(), angle_neighbors.end());
    angle_neighbors.erase(
        std::unique(angle_neighbors.begin(), angle_neighbors.end()),
        angle_neighbors.end());
    auto coord_neighbors = gng.getNeighborsCoord(node.id, coord_layer);
    std::sort(coord_neighbors.begin(), coord_neighbors.end());
    coord_neighbors.erase(
        std::unique(coord_neighbors.begin(), coord_neighbors.end()),
        coord_neighbors.end());
    points.push_back({node.id, position, node.weight_angle.template cast<float>(),
                      std::move(angle_neighbors),
                      std::move(coord_neighbors)});
  }
  return points;
}

}  // namespace robot_sim::visualization
