#include "visualization/visualization_gng.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace robot_sim::visualization {
namespace {

constexpr std::array<char, 8> kMagic = {'V', 'I', 'Z', 'G', 'N', 'G', '5', '\0'};
constexpr std::uint32_t kVersion = 5;
constexpr std::array<char, 8> kStaticMagic = {'V', 'I', 'Z', 'G', 'S', 'T', '1', '\0'};
constexpr std::uint32_t kStaticVersion = 1;
constexpr std::uint32_t kMaxSerializedItems = 10000000;
constexpr std::uint32_t kMaxJointAngleDimension = 1024;
constexpr std::uint32_t kSourceSignatureSchema = 4;

struct TrainingNode {
  Eigen::VectorXf feature;
  float error = 0.0f;
  bool active = true;
  std::unordered_map<int, int> neighbor_ages;
};

template <typename T>
bool writeValue(std::ofstream &stream, const T &value) {
  stream.write(reinterpret_cast<const char *>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

template <typename T>
bool readValue(std::ifstream &stream, T &value) {
  stream.read(reinterpret_cast<char *>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

float featureDistanceSquared(const Eigen::VectorXf &first,
                             const Eigen::VectorXf &second) {
  if (first.size() != second.size() || first.size() <= 3) {
    throw std::invalid_argument("visualization GNG feature dimension is invalid");
  }
  const int joint_num = first.size() - 3;
  const float joint_motion_time =
      (first.head(joint_num) - second.head(joint_num))
          .cwiseAbs()
          .maxCoeff();
  const float workspace_motion_time =
      (first.tail<3>() - second.tail<3>()).norm();
  return joint_motion_time * joint_motion_time +
         workspace_motion_time * workspace_motion_time;
}

void setError(std::string *error, const std::string &message) {
  if (error) {
    *error = message;
  }
}

void connectNodes(std::vector<TrainingNode> &nodes, int a, int b, int age = 0) {
  if (a == b || a < 0 || b < 0 || a >= static_cast<int>(nodes.size()) ||
      b >= static_cast<int>(nodes.size())) {
    return;
  }
  nodes[static_cast<std::size_t>(a)].neighbor_ages[b] = age;
  nodes[static_cast<std::size_t>(b)].neighbor_ages[a] = age;
}

void disconnectNodes(std::vector<TrainingNode> &nodes, int a, int b) {
  nodes[static_cast<std::size_t>(a)].neighbor_ages.erase(b);
  nodes[static_cast<std::size_t>(b)].neighbor_ages.erase(a);
}

int activeNodeCount(const std::vector<TrainingNode> &nodes) {
  return static_cast<int>(std::count_if(
      nodes.begin(), nodes.end(), [](const auto &node) { return node.active; }));
}

std::pair<int, int> findNearestTwo(const std::vector<TrainingNode> &nodes,
                                   const Eigen::VectorXf &sample) {
  int nearest = -1;
  int second = -1;
  float nearest_distance = std::numeric_limits<float>::infinity();
  float second_distance = std::numeric_limits<float>::infinity();
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    if (!nodes[static_cast<std::size_t>(i)].active) {
      continue;
    }
    const float distance = featureDistanceSquared(
        nodes[static_cast<std::size_t>(i)].feature, sample);
    if (distance < nearest_distance) {
      second = nearest;
      second_distance = nearest_distance;
      nearest = i;
      nearest_distance = distance;
    } else if (distance < second_distance) {
      second = i;
      second_distance = distance;
    }
  }
  return {nearest, second};
}

void removeOldEdgesAndIsolatedNodes(std::vector<TrainingNode> &nodes,
                                    int winner, int max_edge_age) {
  std::vector<int> expired;
  for (const auto &[neighbor, age] :
       nodes[static_cast<std::size_t>(winner)].neighbor_ages) {
    if (age > max_edge_age) {
      expired.push_back(neighbor);
    }
  }
  for (const int neighbor : expired) {
    disconnectNodes(nodes, winner, neighbor);
  }

  if (activeNodeCount(nodes) <= 2) {
    return;
  }
  for (auto &node : nodes) {
    if (node.active && node.neighbor_ages.empty()) {
      node.active = false;
      if (activeNodeCount(nodes) <= 2) {
        break;
      }
    }
  }
}

bool insertErrorNode(std::vector<TrainingNode> &nodes) {
  int q = -1;
  float max_error = -1.0f;
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    const auto &node = nodes[static_cast<std::size_t>(i)];
    if (node.active && !node.neighbor_ages.empty() && node.error > max_error) {
      q = i;
      max_error = node.error;
    }
  }
  if (q < 0) {
    return false;
  }

  int f = -1;
  float neighbor_error = -1.0f;
  for (const auto &[neighbor, _] :
       nodes[static_cast<std::size_t>(q)].neighbor_ages) {
    if (nodes[static_cast<std::size_t>(neighbor)].active &&
        nodes[static_cast<std::size_t>(neighbor)].error > neighbor_error) {
      f = neighbor;
      neighbor_error = nodes[static_cast<std::size_t>(neighbor)].error;
    }
  }
  if (f < 0) {
    return false;
  }

  TrainingNode inserted;
  inserted.feature = 0.5f * (nodes[static_cast<std::size_t>(q)].feature +
                             nodes[static_cast<std::size_t>(f)].feature);
  inserted.error = nodes[static_cast<std::size_t>(q)].error;
  const int r = static_cast<int>(nodes.size());
  nodes.push_back(std::move(inserted));
  disconnectNodes(nodes, q, f);
  connectNodes(nodes, q, r);
  connectNodes(nodes, f, r);
  return true;
}

std::vector<std::pair<int, int>> collectSourceEdges(
    const std::vector<VisualizationGngSourcePoint> &source_points) {
  std::unordered_set<int> source_ids;
  source_ids.reserve(source_points.size());
  for (const auto &point : source_points) {
    source_ids.insert(point.source_node_id);
  }

  std::vector<std::pair<int, int>> edges;
  for (const auto &point : source_points) {
    for (const int neighbor : point.coord_neighbor_source_node_ids) {
      if (neighbor == point.source_node_id ||
          source_ids.find(neighbor) == source_ids.end()) {
        continue;
      }
      edges.emplace_back(std::min(point.source_node_id, neighbor),
                         std::max(point.source_node_id, neighbor));
    }
  }
  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  return edges;
}

std::vector<std::pair<int, int>> collectSourceAngleEdges(
    const std::vector<VisualizationGngSourcePoint> &source_points) {
  std::unordered_set<int> source_ids;
  source_ids.reserve(source_points.size());
  for (const auto &point : source_points) {
    source_ids.insert(point.source_node_id);
  }

  std::vector<std::pair<int, int>> edges;
  for (const auto &point : source_points) {
    for (const int neighbor : point.angle_neighbor_source_node_ids) {
      if (neighbor == point.source_node_id ||
          source_ids.find(neighbor) == source_ids.end()) {
        continue;
      }
      edges.emplace_back(std::min(point.source_node_id, neighbor),
                         std::max(point.source_node_id, neighbor));
    }
  }
  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  return edges;
}

Eigen::VectorXf makeTrainingFeature(
    const VisualizationGngSourcePoint &source,
    const VisualizationGngTrainingParams &params) {
  const int joint_num = source.weight_angle.size();
  if (joint_num <= 0 || !source.weight_angle.allFinite()) {
    throw std::invalid_argument("visualization GNG source joint angle is invalid");
  }
  if (!params.joint_max_velocities.empty() &&
      params.joint_max_velocities.size() != static_cast<std::size_t>(joint_num)) {
    throw std::invalid_argument(
        "visualization GNG joint velocity dimension does not match source angle");
  }
  Eigen::VectorXf feature(joint_num + 3);
  for (int joint = 0; joint < joint_num; ++joint) {
    const float max_velocity = params.joint_max_velocities.empty()
                                   ? 1.0f
                                   : params.joint_max_velocities[
                                         static_cast<std::size_t>(joint)];
    if (!std::isfinite(max_velocity) || max_velocity <= 0.0f) {
      throw std::invalid_argument("visualization GNG joint velocity is invalid");
    }
    feature[joint] = params.joint_motion_weight * source.weight_angle[joint] /
                     max_velocity;
  }
  feature.tail<3>() =
      params.workspace_motion_sec_per_m * source.position;
  return feature;
}

using WorkspaceSampleCell = std::array<std::int64_t, 3>;

std::vector<std::vector<std::size_t>> makeWorkspaceSampleGroups(
    const std::vector<VisualizationGngSourcePoint> &source_points,
    float sample_resolution) {
  std::vector<std::vector<std::size_t>> groups;
  if (sample_resolution <= 0.0f) {
    std::vector<std::size_t> indices(source_points.size());
    std::iota(indices.begin(), indices.end(), 0);
    groups.push_back(std::move(indices));
    return groups;
  }

  std::map<WorkspaceSampleCell, std::vector<std::size_t>> cells;
  for (std::size_t index = 0; index < source_points.size(); ++index) {
    const auto &position = source_points[index].position;
    const WorkspaceSampleCell cell{
        static_cast<std::int64_t>(std::floor(position.x() / sample_resolution)),
        static_cast<std::int64_t>(std::floor(position.y() / sample_resolution)),
        static_cast<std::int64_t>(std::floor(position.z() / sample_resolution))};
    cells[cell].push_back(index);
  }
  groups.reserve(cells.size());
  for (auto &[_, indices] : cells) {
    groups.push_back(std::move(indices));
  }
  return groups;
}

float motionTimeSec(const Eigen::VectorXf &source_angle,
                    const Eigen::VectorXf &target_angle,
                    const std::vector<float> &joint_max_velocities) {
  if (source_angle.size() != target_angle.size() || source_angle.size() == 0 ||
      (!joint_max_velocities.empty() &&
       joint_max_velocities.size() != static_cast<std::size_t>(source_angle.size()))) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  float duration = 0.0f;
  for (int joint = 0; joint < source_angle.size(); ++joint) {
    const float max_velocity = joint_max_velocities.empty()
                                   ? 1.0f
                                   : joint_max_velocities[
                                         static_cast<std::size_t>(joint)];
    if (!std::isfinite(max_velocity) || max_velocity <= 0.0f) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    duration = std::max(duration,
                        std::abs(target_angle[joint] - source_angle[joint]) /
                            max_velocity);
  }
  return duration;
}

std::vector<float> makeAttachmentRadii(
    const std::vector<VisualizationGngNode> &visual_nodes,
    const VisualizationGngInterpolationParams &params) {
  std::vector<float> radii(visual_nodes.size(), params.min_attachment_radius);
  if (visual_nodes.size() < 2) {
    return radii;
  }

  const std::size_t neighbor_count = std::min<std::size_t>(
      static_cast<std::size_t>(params.attachment_knn), visual_nodes.size() - 1);
  for (std::size_t source = 0; source < visual_nodes.size(); ++source) {
    std::vector<float> distances;
    distances.reserve(visual_nodes.size() - 1);
    for (std::size_t target = 0; target < visual_nodes.size(); ++target) {
      if (source == target) {
        continue;
      }
      const float distance =
          (visual_nodes[source].position - visual_nodes[target].position).norm();
      if (std::isfinite(distance)) {
        distances.push_back(distance);
      }
    }
    if (distances.empty()) {
      continue;
    }
    const std::size_t used_count = std::min(neighbor_count, distances.size());
    std::nth_element(distances.begin(),
                     distances.begin() + used_count - 1,
                     distances.end());
    distances.resize(used_count);
    std::nth_element(distances.begin(),
                     distances.begin() + distances.size() / 2,
                     distances.end());
    const float local_spacing = distances[distances.size() / 2];
    radii[source] = std::max(
        params.min_attachment_radius,
        params.attachment_radius_scale * local_spacing);
  }
  return radii;
}

std::vector<std::pair<std::uint32_t, std::uint32_t>>
limitTrajectoryEdgeNeighbors(
    const std::vector<VisualizationGngNode> &visual_nodes,
    std::vector<std::pair<std::uint32_t, std::uint32_t>> candidate_edges,
    int max_edge_neighbors) {
  std::sort(candidate_edges.begin(), candidate_edges.end());
  candidate_edges.erase(
      std::unique(candidate_edges.begin(), candidate_edges.end()),
      candidate_edges.end());

  std::vector<std::vector<std::pair<float, std::uint32_t>>> neighbors(
      visual_nodes.size());
  for (const auto &[source, target] : candidate_edges) {
    if (source >= visual_nodes.size() || target >= visual_nodes.size() ||
        source == target) {
      continue;
    }
    const float distance =
        (visual_nodes[source].position - visual_nodes[target].position).norm();
    if (!std::isfinite(distance)) {
      continue;
    }
    neighbors[source].emplace_back(distance, target);
    neighbors[target].emplace_back(distance, source);
  }

  std::vector<std::pair<std::uint32_t, std::uint32_t>> limited_edges;
  for (std::uint32_t source = 0; source < neighbors.size(); ++source) {
    auto &source_neighbors = neighbors[source];
    std::sort(source_neighbors.begin(), source_neighbors.end());
    const std::size_t count = std::min<std::size_t>(
        static_cast<std::size_t>(max_edge_neighbors), source_neighbors.size());
    for (std::size_t index = 0; index < count; ++index) {
      limited_edges.emplace_back(
          std::min(source, source_neighbors[index].second),
          std::max(source, source_neighbors[index].second));
    }
  }
  std::sort(limited_edges.begin(), limited_edges.end());
  limited_edges.erase(std::unique(limited_edges.begin(), limited_edges.end()),
                      limited_edges.end());
  return limited_edges;
}

}  // namespace

std::filesystem::path visualizationGngLayerPath(
    const std::filesystem::path &path_prefix, std::uint32_t coord_layer) {
  return path_prefix.string() + "_L" + std::to_string(coord_layer) +
         ".bin";
}

std::filesystem::path visualizationGngStaticLayerPath(
    const std::filesystem::path &path_prefix, std::uint32_t coord_layer) {
  return path_prefix.string() + "_static_L" + std::to_string(coord_layer) +
         ".bin";
}

std::uint64_t computeVisualizationGngSourceSignature(
    const std::vector<VisualizationGngSourcePoint> &source_points) {
  std::vector<const VisualizationGngSourcePoint *> sorted;
  sorted.reserve(source_points.size());
  for (const auto &point : source_points) {
    sorted.push_back(&point);
  }
  std::sort(sorted.begin(), sorted.end(), [](const auto *a, const auto *b) {
    return a->source_node_id < b->source_node_id;
  });

  std::uint64_t hash = 1469598103934665603ULL;
  const auto append = [&hash](const void *data, std::size_t size) {
    const auto *bytes = static_cast<const unsigned char *>(data);
    for (std::size_t i = 0; i < size; ++i) {
      hash ^= bytes[i];
      hash *= 1099511628211ULL;
    }
  };
  append(&kSourceSignatureSchema, sizeof(kSourceSignatureSchema));
  for (const auto *point : sorted) {
    append(&point->source_node_id, sizeof(point->source_node_id));
    append(point->position.data(), 3 * sizeof(float));
    const std::uint32_t angle_size =
        static_cast<std::uint32_t>(point->weight_angle.size());
    append(&angle_size, sizeof(angle_size));
    append(point->weight_angle.data(),
           static_cast<std::size_t>(angle_size) * sizeof(float));
  }
  const auto edges = collectSourceEdges(source_points);
  const std::uint64_t edge_count = edges.size();
  append(&edge_count, sizeof(edge_count));
  for (const auto &[source, target] : edges) {
    append(&source, sizeof(source));
    append(&target, sizeof(target));
  }
  const auto angle_edges = collectSourceAngleEdges(source_points);
  const std::uint64_t angle_edge_count = angle_edges.size();
  append(&angle_edge_count, sizeof(angle_edge_count));
  for (const auto &[source, target] : angle_edges) {
    append(&source, sizeof(source));
    append(&target, sizeof(target));
  }
  return hash;
}

std::vector<std::pair<std::uint32_t, std::uint32_t>>
contractVisualizationGngEdges(
    const std::vector<VisualizationGngSourcePoint> &source_points,
    const std::vector<VisualizationGngNode> &visual_nodes) {
  std::unordered_map<int, std::uint32_t> source_to_visual;
  source_to_visual.reserve(source_points.size());
  for (std::uint32_t visual_index = 0; visual_index < visual_nodes.size();
       ++visual_index) {
    for (const int source_id :
         visual_nodes[static_cast<std::size_t>(visual_index)].source_node_ids) {
      const auto [existing, inserted] =
          source_to_visual.emplace(source_id, visual_index);
      if (!inserted && existing->second != visual_index) {
        throw std::invalid_argument(
            "source node is assigned to multiple visualization nodes");
      }
    }
  }

  std::vector<std::pair<std::uint32_t, std::uint32_t>> contracted;
  for (const auto &[source, target] : collectSourceEdges(source_points)) {
    const auto source_it = source_to_visual.find(source);
    const auto target_it = source_to_visual.find(target);
    if (source_it == source_to_visual.end() ||
        target_it == source_to_visual.end() ||
        source_it->second == target_it->second) {
      continue;
    }
    contracted.emplace_back(std::min(source_it->second, target_it->second),
                            std::max(source_it->second, target_it->second));
  }
  std::sort(contracted.begin(), contracted.end());
  contracted.erase(std::unique(contracted.begin(), contracted.end()),
                   contracted.end());
  return contracted;
}

void precomputeVisualizationGngTransitionPaths(
    const std::vector<VisualizationGngSourcePoint> &source_points,
    VisualizationGngModel &model, const VisualizationGngFkFunction &fk,
    const VisualizationGngInterpolationParams &params) {
  if (!fk || params.max_joint_step <= 0.0f ||
      params.max_samples_per_edge < 2 || params.attachment_knn < 1 ||
      params.attachment_radius_scale <= 0.0f ||
      params.min_attachment_radius < 0.0f || params.max_edge_neighbors < 1 ||
      model.nodes.empty()) {
    throw std::invalid_argument(
        "invalid visualization GNG interpolation configuration");
  }
  if (model.nodes.size() > std::numeric_limits<std::uint16_t>::max()) {
    throw std::overflow_error(
        "visualization GNG exceeds uint16 transition node capacity");
  }

  std::unordered_map<int, const VisualizationGngSourcePoint *> source_by_id;
  source_by_id.reserve(source_points.size());
  for (const auto &point : source_points) {
    source_by_id.emplace(point.source_node_id, &point);
  }

  std::unordered_map<int, std::uint32_t> source_to_visual;
  source_to_visual.reserve(source_points.size());
  for (std::uint32_t visual_id = 0; visual_id < model.nodes.size(); ++visual_id) {
    for (const int source_id : model.nodes[visual_id].source_node_ids) {
      source_to_visual.emplace(source_id, visual_id);
    }
  }

  const auto attachment_radii = makeAttachmentRadii(model.nodes, params);
  const auto nearest_visual = [&model, &attachment_radii](
                                  const Eigen::Vector3f &position) {
    int nearest = -1;
    float nearest_distance = std::numeric_limits<float>::infinity();
    for (std::uint32_t visual_id = 0; visual_id < model.nodes.size();
         ++visual_id) {
      const float distance =
          (model.nodes[visual_id].position - position).squaredNorm();
      const float radius = attachment_radii[visual_id];
      if (distance <= radius * radius && distance < nearest_distance) {
        nearest = static_cast<int>(visual_id);
        nearest_distance = distance;
      }
    }
    return nearest;
  };

  std::vector<VisualizationGngTransitionPath> transition_paths;
  std::vector<std::uint16_t> transition_path_nodes;
  std::vector<std::pair<std::uint32_t, std::uint32_t>> visual_edges;
  for (const auto &[source_id, target_id] :
       collectSourceAngleEdges(source_points)) {
    const auto source_it = source_by_id.find(source_id);
    const auto target_it = source_by_id.find(target_id);
    const auto source_visual_it = source_to_visual.find(source_id);
    const auto target_visual_it = source_to_visual.find(target_id);
    if (source_it == source_by_id.end() || target_it == source_by_id.end() ||
        source_visual_it == source_to_visual.end() ||
        target_visual_it == source_to_visual.end()) {
      continue;
    }
    const auto &source_angle = source_it->second->weight_angle;
    const auto &target_angle = target_it->second->weight_angle;
    if (source_angle.size() == 0 || source_angle.size() != target_angle.size() ||
        !source_angle.allFinite() || !target_angle.allFinite()) {
      continue;
    }
    const float motion_time_sec = motionTimeSec(
        source_angle, target_angle, params.joint_max_velocities);
    if (!std::isfinite(motion_time_sec)) {
      continue;
    }

    const float max_delta =
        (target_angle - source_angle).cwiseAbs().maxCoeff();
    const int segment_count = std::clamp(
        static_cast<int>(std::ceil(max_delta / params.max_joint_step)), 1,
        params.max_samples_per_edge - 1);
    std::vector<std::uint32_t> path;
    path.reserve(static_cast<std::size_t>(segment_count + 1));
    const auto append_node = [&path](std::uint32_t visual_id) {
      if (path.empty() || path.back() != visual_id) {
        path.push_back(visual_id);
      }
    };
    append_node(source_visual_it->second);
    for (int segment = 1; segment < segment_count; ++segment) {
      const float ratio =
          static_cast<float>(segment) / static_cast<float>(segment_count);
      const Eigen::VectorXf angle =
          source_angle + ratio * (target_angle - source_angle);
      const Eigen::Vector3f position = fk(angle, model.coord_layer);
      if (position.allFinite()) {
        const int visual_id = nearest_visual(position);
        if (visual_id >= 0) {
          append_node(static_cast<std::uint32_t>(visual_id));
        }
      }
    }
    path.push_back(target_visual_it->second);

    const bool has_visual_connection = path.size() >= 3;
    if (has_visual_connection) {
      for (std::size_t i = 1; i < path.size(); ++i) {
        if (path[i - 1] == path[i]) {
          continue;
        }
        visual_edges.emplace_back(std::min(path[i - 1], path[i]),
                                  std::max(path[i - 1], path[i]));
      }
    }
    if (path.size() > std::numeric_limits<std::uint16_t>::max() ||
        transition_path_nodes.size() + path.size() >
            std::numeric_limits<std::uint32_t>::max()) {
      throw std::overflow_error(
          "visualization GNG transition path capacity exceeded");
    }
    const std::uint32_t path_offset =
        static_cast<std::uint32_t>(transition_path_nodes.size());
    for (const std::uint32_t visual_node_id : path) {
      transition_path_nodes.push_back(
          static_cast<std::uint16_t>(visual_node_id));
    }
    transition_paths.push_back(
        {source_id, target_id, path_offset,
         static_cast<std::uint16_t>(path.size()), motion_time_sec,
         has_visual_connection});
  }
  model.edges = limitTrajectoryEdgeNeighbors(
      model.nodes, std::move(visual_edges), params.max_edge_neighbors);
  model.transition_paths = std::move(transition_paths);
  model.transition_path_nodes = std::move(transition_path_nodes);
}

VisualizationGngModel trainVisualizationGng(
    std::vector<VisualizationGngSourcePoint> source_points,
    std::uint32_t coord_layer,
    const VisualizationGngTrainingParams &params) {
  source_points.erase(
      std::remove_if(source_points.begin(), source_points.end(),
                     [](const auto &point) {
                       return point.source_node_id < 0 ||
                              !point.position.allFinite();
                     }),
      source_points.end());
  std::sort(source_points.begin(), source_points.end(),
            [](const auto &a, const auto &b) {
              return a.source_node_id < b.source_node_id;
            });
  if (source_points.size() < 2) {
    throw std::invalid_argument(
        "visualization GNG requires at least two source points");
  }
  if (!std::isfinite(params.joint_motion_weight) ||
      params.joint_motion_weight <= 0.0f ||
      !std::isfinite(params.workspace_motion_sec_per_m) ||
      params.workspace_motion_sec_per_m <= 0.0f ||
      !std::isfinite(params.workspace_sample_resolution) ||
      params.workspace_sample_resolution < 0.0f) {
    throw std::invalid_argument("visualization GNG training parameter is invalid");
  }
  std::vector<Eigen::VectorXf> features;
  features.reserve(source_points.size());
  for (const auto &source : source_points) {
    if (source.weight_angle.size() != source_points.front().weight_angle.size()) {
      throw std::invalid_argument(
          "visualization GNG source joint angle dimensions are inconsistent");
    }
    features.push_back(makeTrainingFeature(source, params));
  }
  const auto sample_groups = makeWorkspaceSampleGroups(
      source_points, params.workspace_sample_resolution);
  if (sample_groups.empty()) {
    throw std::invalid_argument("visualization GNG workspace sampling is empty");
  }

  const int target_nodes = std::clamp(
      params.target_nodes, 2, static_cast<int>(source_points.size()));
  const int insertion_interval = std::max(1, params.insertion_interval);
  const int iterations = std::max(
      params.iterations, insertion_interval * std::max(0, target_nodes - 2));

  std::mt19937 random(params.seed);
  std::uniform_int_distribution<std::size_t> group_distribution(
      0, sample_groups.size() - 1);
  const auto sample_index = [&]() {
    const auto &group = sample_groups[group_distribution(random)];
    std::uniform_int_distribution<std::size_t> sample_distribution(
        0, group.size() - 1);
    return group[sample_distribution(random)];
  };
  const std::size_t first_index = sample_index();
  std::size_t second_index = first_index;
  float farthest_distance = -1.0f;
  for (std::size_t i = 0; i < source_points.size(); ++i) {
    const float distance =
        featureDistanceSquared(features[i], features[first_index]);
    if (distance > farthest_distance) {
      farthest_distance = distance;
      second_index = i;
    }
  }
  if (second_index == first_index || farthest_distance <= 0.0f) {
    throw std::invalid_argument(
        "visualization GNG source points do not span the training feature space");
  }

  std::vector<TrainingNode> training_nodes(2);
  training_nodes[0].feature = features[first_index];
  training_nodes[1].feature = features[second_index];
  connectNodes(training_nodes, 0, 1);

  for (int iteration = 1; iteration <= iterations; ++iteration) {
    const auto &sample = features[sample_index()];
    const auto [winner, second] = findNearestTwo(training_nodes, sample);
    if (winner < 0 || second < 0) {
      continue;
    }

    auto &winner_node = training_nodes[static_cast<std::size_t>(winner)];
    winner_node.error += featureDistanceSquared(winner_node.feature, sample);
    for (auto &[_, age] : winner_node.neighbor_ages) {
      ++age;
    }

    winner_node.feature +=
        params.winner_learning_rate * (sample - winner_node.feature);
    std::vector<int> winner_neighbors;
    winner_neighbors.reserve(winner_node.neighbor_ages.size());
    for (const auto &[neighbor, _] : winner_node.neighbor_ages) {
      winner_neighbors.push_back(neighbor);
    }
    for (const int neighbor : winner_neighbors) {
      auto &neighbor_node =
          training_nodes[static_cast<std::size_t>(neighbor)];
      neighbor_node.feature += params.neighbor_learning_rate *
                               (sample - neighbor_node.feature);
    }

    connectNodes(training_nodes, winner, second);
    removeOldEdgesAndIsolatedNodes(training_nodes, winner,
                                   std::max(1, params.max_edge_age));

    if (iteration % insertion_interval == 0 &&
        activeNodeCount(training_nodes) < target_nodes) {
      if (insertErrorNode(training_nodes)) {
        auto &inserted = training_nodes.back();
        const auto connected = inserted.neighbor_ages;
        for (const auto &[neighbor, _] : connected) {
          training_nodes[static_cast<std::size_t>(neighbor)].error *=
              params.split_error_scale;
        }
        inserted.error *= params.split_error_scale;
      }
    }

    const float decay = std::clamp(1.0f - params.error_decay, 0.0f, 1.0f);
    for (auto &node : training_nodes) {
      if (node.active) {
        node.error *= decay;
      }
    }
  }

  VisualizationGngModel model;
  model.coord_layer = coord_layer;
  model.joint_angle_dimension = static_cast<std::uint32_t>(
      source_points.front().weight_angle.size());
  model.source_signature =
      computeVisualizationGngSourceSignature(source_points);

  std::vector<std::size_t> visual_to_training;
  visual_to_training.reserve(training_nodes.size());
  for (int i = 0; i < static_cast<int>(training_nodes.size()); ++i) {
    if (!training_nodes[static_cast<std::size_t>(i)].active) {
      continue;
    }
    model.nodes.push_back({Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitZ(),
                           2, -1, Eigen::VectorXf(), {}});
    visual_to_training.push_back(static_cast<std::size_t>(i));
  }

  for (std::size_t source_index = 0; source_index < source_points.size();
       ++source_index) {
    const auto &source = source_points[source_index];
    int nearest = -1;
    float nearest_distance = std::numeric_limits<float>::infinity();
    for (int i = 0; i < static_cast<int>(model.nodes.size()); ++i) {
      const float distance = featureDistanceSquared(
          training_nodes[visual_to_training[static_cast<std::size_t>(i)]].feature,
          features[source_index]);
      if (distance < nearest_distance) {
        nearest = i;
        nearest_distance = distance;
      }
    }
    if (nearest >= 0) {
      model.nodes[static_cast<std::size_t>(nearest)].source_node_ids.push_back(
          source.source_node_id);
    }
  }

  std::vector<VisualizationGngNode> assigned_nodes;
  std::vector<std::size_t> assigned_training_indices;
  assigned_nodes.reserve(model.nodes.size());
  assigned_training_indices.reserve(visual_to_training.size());
  for (std::size_t visual_index = 0; visual_index < model.nodes.size();
       ++visual_index) {
    if (model.nodes[visual_index].source_node_ids.empty()) {
      continue;
    }
    assigned_nodes.push_back(std::move(model.nodes[visual_index]));
    assigned_training_indices.push_back(visual_to_training[visual_index]);
  }
  model.nodes = std::move(assigned_nodes);
  visual_to_training = std::move(assigned_training_indices);

  std::unordered_map<int, std::size_t> source_index_by_id;
  source_index_by_id.reserve(source_points.size());
  for (std::size_t index = 0; index < source_points.size(); ++index) {
    source_index_by_id.emplace(source_points[index].source_node_id, index);
  }
  for (std::size_t visual_index = 0; visual_index < model.nodes.size();
       ++visual_index) {
    auto &visual_node = model.nodes[visual_index];
    float nearest_distance = std::numeric_limits<float>::infinity();
    Eigen::Vector3f normal_sum = Eigen::Vector3f::Zero();
    bool has_safe_member = false;
    bool has_danger_member = false;
    for (const int source_id : visual_node.source_node_ids) {
      const auto source_it = source_index_by_id.find(source_id);
      if (source_it == source_index_by_id.end()) {
        continue;
      }
      const std::size_t source_index = source_it->second;
      const auto &source = source_points[source_index];
      if (source.direction.allFinite()) {
        normal_sum += source.direction;
      }
      has_safe_member = has_safe_member || source.label == 1;
      has_danger_member = has_danger_member || source.label == 3;
      const float distance = featureDistanceSquared(
          training_nodes[visual_to_training[visual_index]].feature,
          features[source_index]);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        visual_node.representative_source_node_id = source_id;
        visual_node.position = source.position;
        visual_node.representative_joint_angle = source.weight_angle;
      }
    }
    if (visual_node.representative_source_node_id < 0) {
      throw std::runtime_error("visualization GNG node has no representative");
    }
    visual_node.normal = normal_sum.norm() > 1e-6f
                             ? normal_sum.normalized()
                             : Eigen::Vector3f::UnitZ();
    visual_node.label = has_safe_member ? 1 : (has_danger_member ? 3 : 2);
  }

  model.edges = contractVisualizationGngEdges(source_points, model.nodes);

  return model;
}

bool VisualizationGngModel::save(const std::filesystem::path &path,
                                 std::string *error) const {
  if (nodes.size() > std::numeric_limits<std::uint16_t>::max()) {
    setError(error, "visualization GNG exceeds uint16 node capacity");
    return false;
  }
  if (joint_angle_dimension == 0 ||
      joint_angle_dimension > kMaxJointAngleDimension) {
    setError(error, "visualization GNG joint angle dimension is invalid");
    return false;
  }
  std::error_code ec;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path(), ec);
    if (ec) {
      setError(error, "failed to create output directory: " + ec.message());
      return false;
    }
  }

  const auto temporary = path.string() + ".tmp";
  std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
  if (!stream) {
    setError(error, "failed to open output file: " + temporary);
    return false;
  }
  stream.write(kMagic.data(), static_cast<std::streamsize>(kMagic.size()));
  const std::uint32_t node_count = static_cast<std::uint32_t>(nodes.size());
  const std::uint32_t edge_count = static_cast<std::uint32_t>(edges.size());
  const std::uint32_t transition_count =
      static_cast<std::uint32_t>(transition_paths.size());
  if (!writeValue(stream, kVersion) || !writeValue(stream, coord_layer) ||
      !writeValue(stream, joint_angle_dimension) ||
      !writeValue(stream, source_signature) || !writeValue(stream, node_count) ||
      !writeValue(stream, edge_count) ||
      !writeValue(stream, transition_count)) {
    setError(error, "failed to write visualization GNG header");
    return false;
  }

  for (const auto &node : nodes) {
    stream.write(reinterpret_cast<const char *>(node.position.data()),
                 3 * sizeof(float));
    stream.write(reinterpret_cast<const char *>(node.normal.data()),
                 3 * sizeof(float));
    const std::int32_t representative_source_node_id =
        static_cast<std::int32_t>(node.representative_source_node_id);
    const std::uint32_t member_count =
        static_cast<std::uint32_t>(node.source_node_ids.size());
    if (!stream || !node.position.allFinite() || !node.normal.allFinite() ||
        node.normal.norm() <= 0.0f || node.label > 3 ||
        node.representative_joint_angle.size() !=
            static_cast<int>(joint_angle_dimension) ||
        !node.representative_joint_angle.allFinite() ||
        representative_source_node_id < 0 ||
        std::find(node.source_node_ids.begin(), node.source_node_ids.end(),
                  node.representative_source_node_id) ==
            node.source_node_ids.end() ||
        !writeValue(stream, node.label) ||
        !(stream.write(reinterpret_cast<const char *>(
                           node.representative_joint_angle.data()),
                       static_cast<std::streamsize>(
                           joint_angle_dimension * sizeof(float)))) ||
        !writeValue(stream, representative_source_node_id) ||
        !writeValue(stream, member_count)) {
      setError(error, "failed to write visualization GNG node");
      return false;
    }
    for (const int member : node.source_node_ids) {
      const std::int32_t serialized_member = static_cast<std::int32_t>(member);
      if (!writeValue(stream, serialized_member)) {
        setError(error, "failed to write visualization GNG membership");
        return false;
      }
    }
  }
  for (const auto &[source, target] : edges) {
    if (!writeValue(stream, source) || !writeValue(stream, target)) {
      setError(error, "failed to write visualization GNG edge");
      return false;
    }
  }
  for (const auto &transition : transition_paths) {
    const std::int32_t source = transition.source_node_id;
    const std::int32_t target = transition.target_node_id;
    const std::uint8_t has_visual_connection =
        transition.has_visual_connection ? 1U : 0U;
    const std::size_t path_begin = transition.path_offset;
    const std::size_t path_end = path_begin + transition.path_size;
    if (transition.path_size < 2 || path_end > transition_path_nodes.size() ||
        !std::isfinite(transition.motion_time_sec) ||
        transition.motion_time_sec < 0.0f) {
      setError(error, "invalid visualization GNG transition path size");
      return false;
    }
    if (!writeValue(stream, source) || !writeValue(stream, target) ||
        !writeValue(stream, transition.path_size) ||
        !writeValue(stream, transition.motion_time_sec) ||
        !writeValue(stream, has_visual_connection)) {
      setError(error, "failed to write visualization GNG transition");
      return false;
    }
    for (std::size_t path_index = path_begin; path_index < path_end;
         ++path_index) {
      const auto visual_node_id = transition_path_nodes[path_index];
      if (visual_node_id >= nodes.size()) {
        setError(error, "invalid visualization GNG transition node");
        return false;
      }
      if (!writeValue(stream, visual_node_id)) {
        setError(error, "failed to write visualization GNG transition path");
        return false;
      }
    }
  }
  stream.close();
  if (!stream) {
    setError(error, "failed to finalize visualization GNG file");
    return false;
  }

  std::filesystem::rename(temporary, path, ec);
  if (ec) {
    std::filesystem::remove(path, ec);
    ec.clear();
    std::filesystem::rename(temporary, path, ec);
  }
  if (ec) {
    setError(error, "failed to install visualization GNG file: " + ec.message());
    return false;
  }
  return true;
}

bool VisualizationGngModel::load(const std::filesystem::path &path,
                                 std::string *error) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    setError(error, "failed to open visualization GNG: " + path.string());
    return false;
  }
  std::array<char, 8> magic{};
  stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  std::uint32_t version = 0;
  std::uint32_t loaded_joint_angle_dimension = 0;
  std::uint32_t node_count = 0;
  std::uint32_t edge_count = 0;
  std::uint32_t transition_count = 0;
  if (!stream || magic != kMagic || !readValue(stream, version) ||
      version != kVersion || !readValue(stream, coord_layer) ||
      !readValue(stream, loaded_joint_angle_dimension) ||
      !readValue(stream, source_signature) || !readValue(stream, node_count) ||
      !readValue(stream, edge_count) || !readValue(stream, transition_count) ||
      node_count > std::numeric_limits<std::uint16_t>::max() ||
      loaded_joint_angle_dimension == 0 ||
      loaded_joint_angle_dimension > kMaxJointAngleDimension ||
      node_count > kMaxSerializedItems || edge_count > kMaxSerializedItems ||
      transition_count > kMaxSerializedItems) {
    setError(error, "invalid visualization GNG header: " + path.string());
    return false;
  }

  std::vector<VisualizationGngNode> loaded_nodes(node_count);
  for (auto &node : loaded_nodes) {
    stream.read(reinterpret_cast<char *>(node.position.data()),
                3 * sizeof(float));
    stream.read(reinterpret_cast<char *>(node.normal.data()),
                3 * sizeof(float));
    std::uint8_t label = 2;
    std::int32_t representative_source_node_id = -1;
    std::uint32_t member_count = 0;
    node.representative_joint_angle.resize(
        static_cast<int>(loaded_joint_angle_dimension));
    if (!stream || !readValue(stream, label) ||
        !(stream.read(reinterpret_cast<char *>(
                          node.representative_joint_angle.data()),
                      static_cast<std::streamsize>(
                          loaded_joint_angle_dimension * sizeof(float)))) ||
        !readValue(stream, representative_source_node_id) ||
        !readValue(stream, member_count) || representative_source_node_id < 0 ||
        member_count > kMaxSerializedItems || !node.position.allFinite() ||
        !node.normal.allFinite() || node.normal.norm() <= 0.0f || label > 3 ||
        !node.representative_joint_angle.allFinite()) {
      setError(error, "invalid visualization GNG node data");
      return false;
    }
    node.source_node_ids.resize(member_count);
    for (auto &member : node.source_node_ids) {
      std::int32_t serialized_member = -1;
      if (!readValue(stream, serialized_member)) {
        setError(error, "invalid visualization GNG membership data");
        return false;
      }
      member = static_cast<int>(serialized_member);
    }
    if (std::find(node.source_node_ids.begin(), node.source_node_ids.end(),
                  static_cast<int>(representative_source_node_id)) ==
        node.source_node_ids.end()) {
      setError(error, "invalid visualization GNG representative node");
      return false;
    }
    node.representative_source_node_id =
        static_cast<int>(representative_source_node_id);
    node.label = label;
  }

  std::vector<std::pair<std::uint32_t, std::uint32_t>> loaded_edges;
  loaded_edges.reserve(edge_count);
  for (std::uint32_t i = 0; i < edge_count; ++i) {
    std::uint32_t source = 0;
    std::uint32_t target = 0;
    if (!readValue(stream, source) || !readValue(stream, target) ||
        source >= node_count || target >= node_count || source == target) {
      setError(error, "invalid visualization GNG edge data");
      return false;
    }
    loaded_edges.emplace_back(source, target);
  }
  std::vector<VisualizationGngTransitionPath> loaded_transitions;
  loaded_transitions.reserve(transition_count);
  std::vector<std::uint16_t> loaded_transition_path_nodes;
  std::unordered_map<int, std::uint32_t> source_to_visual;
  for (std::uint32_t visual_id = 0; visual_id < loaded_nodes.size();
       ++visual_id) {
    for (const int source_id : loaded_nodes[visual_id].source_node_ids) {
      source_to_visual.emplace(source_id, visual_id);
    }
  }
  for (std::uint32_t i = 0; i < transition_count; ++i) {
    std::int32_t source = -1;
    std::int32_t target = -1;
    std::uint16_t path_size = 0;
    float motion_time_sec = 0.0f;
    std::uint8_t has_visual_connection = 0;
    if (!readValue(stream, source) || !readValue(stream, target) ||
        !readValue(stream, path_size) || !readValue(stream, motion_time_sec) ||
        !readValue(stream, has_visual_connection) || has_visual_connection > 1 ||
        source < 0 || target < 0 || source >= target || path_size < 2 ||
        !std::isfinite(motion_time_sec) || motion_time_sec < 0.0f ||
        source_to_visual.find(source) == source_to_visual.end() ||
        source_to_visual.find(target) == source_to_visual.end()) {
      setError(error, "invalid visualization GNG transition data");
      return false;
    }
    VisualizationGngTransitionPath transition;
    transition.source_node_id = source;
    transition.target_node_id = target;
    transition.path_offset =
        static_cast<std::uint32_t>(loaded_transition_path_nodes.size());
    transition.path_size = path_size;
    transition.motion_time_sec = motion_time_sec;
    transition.has_visual_connection = has_visual_connection != 0;
    for (std::uint32_t path_index = 0; path_index < path_size;
         ++path_index) {
      std::uint16_t visual_node_id = 0;
      if (!readValue(stream, visual_node_id) || visual_node_id >= node_count) {
        setError(error, "invalid visualization GNG transition path");
        return false;
      }
      loaded_transition_path_nodes.push_back(visual_node_id);
    }
    if (loaded_transition_path_nodes[transition.path_offset] !=
            static_cast<std::uint16_t>(source_to_visual[source]) ||
        loaded_transition_path_nodes.back() !=
            static_cast<std::uint16_t>(source_to_visual[target])) {
      setError(error, "invalid visualization GNG transition endpoints");
      return false;
    }
    loaded_transitions.push_back(std::move(transition));
  }
  if (stream.peek() != std::ifstream::traits_type::eof()) {
    setError(error, "unexpected trailing data in visualization GNG");
    return false;
  }

  nodes = std::move(loaded_nodes);
  joint_angle_dimension = loaded_joint_angle_dimension;
  edges = std::move(loaded_edges);
  transition_paths = std::move(loaded_transitions);
  transition_path_nodes = std::move(loaded_transition_path_nodes);
  return true;
}

VisualizationGngStaticModel makeVisualizationGngStaticModel(
    const VisualizationGngModel &model) {
  VisualizationGngStaticModel static_model;
  static_model.coord_layer = model.coord_layer;
  static_model.joint_angle_dimension = model.joint_angle_dimension;
  static_model.nodes.reserve(model.nodes.size());
  for (const auto &node : model.nodes) {
    static_model.nodes.push_back(
        {node.position, node.normal, node.label, node.representative_joint_angle});
  }
  static_model.edges = model.edges;
  return static_model;
}

bool VisualizationGngStaticModel::save(const std::filesystem::path &path,
                                       std::string *error) const {
  if (nodes.size() > std::numeric_limits<std::uint16_t>::max() ||
      joint_angle_dimension == 0 ||
      joint_angle_dimension > kMaxJointAngleDimension) {
    setError(error, "visualization static GNG header is invalid");
    return false;
  }
  std::error_code ec;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path(), ec);
    if (ec) {
      setError(error, "failed to create static GNG output directory: " +
                          ec.message());
      return false;
    }
  }

  const auto temporary = path.string() + ".tmp";
  std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
  const std::uint32_t node_count = static_cast<std::uint32_t>(nodes.size());
  const std::uint32_t edge_count = static_cast<std::uint32_t>(edges.size());
  if (!stream ||
      !(stream.write(kStaticMagic.data(),
                     static_cast<std::streamsize>(kStaticMagic.size()))) ||
      !writeValue(stream, kStaticVersion) || !writeValue(stream, coord_layer) ||
      !writeValue(stream, joint_angle_dimension) || !writeValue(stream, node_count) ||
      !writeValue(stream, edge_count)) {
    setError(error, "failed to write visualization static GNG header");
    return false;
  }
  for (const auto &node : nodes) {
    stream.write(reinterpret_cast<const char *>(node.position.data()),
                 3 * sizeof(float));
    stream.write(reinterpret_cast<const char *>(node.normal.data()),
                 3 * sizeof(float));
    if (!stream || !node.position.allFinite() || !node.normal.allFinite() ||
        node.normal.norm() <= 0.0f || node.label > 3 ||
        node.representative_joint_angle.size() !=
            static_cast<int>(joint_angle_dimension) ||
        !node.representative_joint_angle.allFinite() || !writeValue(stream, node.label) ||
        !(stream.write(reinterpret_cast<const char *>(
                           node.representative_joint_angle.data()),
                       static_cast<std::streamsize>(
                           joint_angle_dimension * sizeof(float))))) {
      setError(error, "failed to write visualization static GNG node");
      return false;
    }
  }
  for (const auto &[source, target] : edges) {
    if (!writeValue(stream, source) || !writeValue(stream, target) ||
        source >= nodes.size() || target >= nodes.size() || source == target) {
      setError(error, "failed to write visualization static GNG edge");
      return false;
    }
  }
  stream.close();
  if (!stream) {
    setError(error, "failed to finalize visualization static GNG file");
    return false;
  }
  std::filesystem::rename(temporary, path, ec);
  if (ec) {
    std::filesystem::remove(path, ec);
    ec.clear();
    std::filesystem::rename(temporary, path, ec);
  }
  if (ec) {
    setError(error, "failed to install visualization static GNG file: " +
                        ec.message());
    return false;
  }
  return true;
}

bool VisualizationGngStaticModel::load(const std::filesystem::path &path,
                                       std::string *error) {
  std::ifstream stream(path, std::ios::binary);
  std::array<char, 8> magic{};
  std::uint32_t version = 0;
  std::uint32_t loaded_joint_angle_dimension = 0;
  std::uint32_t node_count = 0;
  std::uint32_t edge_count = 0;
  stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  if (!stream || magic != kStaticMagic || !readValue(stream, version) ||
      version != kStaticVersion || !readValue(stream, coord_layer) ||
      !readValue(stream, loaded_joint_angle_dimension) ||
      !readValue(stream, node_count) || !readValue(stream, edge_count) ||
      node_count > std::numeric_limits<std::uint16_t>::max() ||
      loaded_joint_angle_dimension == 0 ||
      loaded_joint_angle_dimension > kMaxJointAngleDimension ||
      edge_count > kMaxSerializedItems) {
    setError(error, "invalid visualization static GNG header: " + path.string());
    return false;
  }
  std::vector<VisualizationGngStaticNode> loaded_nodes(node_count);
  for (auto &node : loaded_nodes) {
    stream.read(reinterpret_cast<char *>(node.position.data()), 3 * sizeof(float));
    stream.read(reinterpret_cast<char *>(node.normal.data()), 3 * sizeof(float));
    node.representative_joint_angle.resize(
        static_cast<int>(loaded_joint_angle_dimension));
    if (!stream || !readValue(stream, node.label) ||
        !(stream.read(reinterpret_cast<char *>(node.representative_joint_angle.data()),
                      static_cast<std::streamsize>(
                          loaded_joint_angle_dimension * sizeof(float)))) ||
        !node.position.allFinite() || !node.normal.allFinite() ||
        node.normal.norm() <= 0.0f || node.label > 3 ||
        !node.representative_joint_angle.allFinite()) {
      setError(error, "invalid visualization static GNG node data");
      return false;
    }
  }
  std::vector<std::pair<std::uint32_t, std::uint32_t>> loaded_edges;
  loaded_edges.reserve(edge_count);
  for (std::uint32_t index = 0; index < edge_count; ++index) {
    std::uint32_t source = 0;
    std::uint32_t target = 0;
    if (!readValue(stream, source) || !readValue(stream, target) ||
        source >= node_count || target >= node_count || source == target) {
      setError(error, "invalid visualization static GNG edge data");
      return false;
    }
    loaded_edges.emplace_back(source, target);
  }
  if (stream.peek() != std::ifstream::traits_type::eof()) {
    setError(error, "unexpected trailing data in visualization static GNG");
    return false;
  }
  joint_angle_dimension = loaded_joint_angle_dimension;
  nodes = std::move(loaded_nodes);
  edges = std::move(loaded_edges);
  return true;
}

}  // namespace robot_sim::visualization
