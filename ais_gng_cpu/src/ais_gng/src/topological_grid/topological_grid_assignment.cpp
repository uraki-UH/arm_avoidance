#include <ais_gng/topological_grid/topological_grid_assignment.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <sstream>
#include <unordered_map>

namespace
{
struct VoxelAccumulator
{
  std::array<std::size_t, 256> label_counts{};
  std::size_t node_count = 0;
  std::size_t input_point_count = 0;
  std::size_t edge_support_count = 0;
  std::size_t triangle_support_count = 0;
  std::vector<fuzzrobo::topological_grid::NodeObservation> node_observations;
};

struct Vec3d
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vec3d operator-(const Vec3d &lhs, const Vec3d &rhs)
{
  return Vec3d{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3d cross(const Vec3d &lhs, const Vec3d &rhs)
{
  return Vec3d{
    lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.z * rhs.x - lhs.x * rhs.z,
    lhs.x * rhs.y - lhs.y * rhs.x};
}

double dot(const Vec3d &lhs, const Vec3d &rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double squaredNorm(const Vec3d &value)
{
  return dot(value, value);
}

Vec3d nodePosition(const ais_gng_msgs::msg::TopologicalNode &node)
{
  return Vec3d{node.pos.x, node.pos.y, node.pos.z};
}

Vec3d nodeNormal(const ais_gng_msgs::msg::TopologicalNode &node)
{
  return Vec3d{node.normal.x, node.normal.y, node.normal.z};
}

std::uint8_t dominantLabel(const std::array<std::size_t, 256> &label_counts)
{
  std::uint8_t dominant = 0;
  std::size_t dominant_count = 0;
  for (std::size_t label = 0; label < label_counts.size(); ++label) {
    if (label_counts[label] > dominant_count) {
      dominant = static_cast<std::uint8_t>(label);
      dominant_count = label_counts[label];
    }
  }
  return dominant;
}

std::uint8_t dominantLabel(const VoxelAccumulator &accumulator)
{
  return dominantLabel(accumulator.label_counts);
}

bool cellLess(
  const fuzzrobo::topological_grid::LabeledGridVoxel &lhs,
  const fuzzrobo::topological_grid::LabeledGridVoxel &rhs)
{
  if (lhs.cell.x != rhs.cell.x) {
    return lhs.cell.x < rhs.cell.x;
  }
  if (lhs.cell.y != rhs.cell.y) {
    return lhs.cell.y < rhs.cell.y;
  }
  return lhs.cell.z < rhs.cell.z;
}

bool gridCellLess(
  const fuzzrobo::topological_grid::GridCell &lhs,
  const fuzzrobo::topological_grid::GridCell &rhs)
{
  if (lhs.x != rhs.x) {
    return lhs.x < rhs.x;
  }
  if (lhs.y != rhs.y) {
    return lhs.y < rhs.y;
  }
  return lhs.z < rhs.z;
}

struct VoxelEdge
{
  fuzzrobo::topological_grid::GridCell first;
  fuzzrobo::topological_grid::GridCell second;

  bool operator==(const VoxelEdge &other) const noexcept
  {
    return first == other.first && second == other.second;
  }
};

struct VoxelEdgeHash
{
  std::size_t operator()(const VoxelEdge &edge) const noexcept
  {
    const fuzzrobo::topological_grid::GridCellHash cell_hash;
    std::size_t seed = cell_hash(edge.first);
    seed ^= cell_hash(edge.second) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

VoxelEdge canonicalVoxelEdge(
  const fuzzrobo::topological_grid::GridCell &lhs,
  const fuzzrobo::topological_grid::GridCell &rhs)
{
  return gridCellLess(rhs, lhs) ? VoxelEdge{rhs, lhs} : VoxelEdge{lhs, rhs};
}

std::vector<fuzzrobo::topological_grid::GridCell> rasterizeGridLine(
  const fuzzrobo::topological_grid::GridCell &start,
  const fuzzrobo::topological_grid::GridCell &end)
{
  using fuzzrobo::topological_grid::GridCell;
  std::vector<GridCell> cells;
  const int dx = std::abs(end.x - start.x);
  const int dy = std::abs(end.y - start.y);
  const int dz = std::abs(end.z - start.z);
  cells.reserve(static_cast<std::size_t>(std::max({dx, dy, dz}) + 1));

  int x = start.x;
  int y = start.y;
  int z = start.z;
  const int step_x = end.x >= start.x ? 1 : -1;
  const int step_y = end.y >= start.y ? 1 : -1;
  const int step_z = end.z >= start.z ? 1 : -1;
  cells.push_back(GridCell{x, y, z});

  if (dx >= dy && dx >= dz) {
    int error_y = 2 * dy - dx;
    int error_z = 2 * dz - dx;
    while (x != end.x) {
      x += step_x;
      if (error_y >= 0) {
        y += step_y;
        error_y -= 2 * dx;
      }
      if (error_z >= 0) {
        z += step_z;
        error_z -= 2 * dx;
      }
      error_y += 2 * dy;
      error_z += 2 * dz;
      cells.push_back(GridCell{x, y, z});
    }
  } else if (dy >= dx && dy >= dz) {
    int error_x = 2 * dx - dy;
    int error_z = 2 * dz - dy;
    while (y != end.y) {
      y += step_y;
      if (error_x >= 0) {
        x += step_x;
        error_x -= 2 * dy;
      }
      if (error_z >= 0) {
        z += step_z;
        error_z -= 2 * dy;
      }
      error_x += 2 * dx;
      error_z += 2 * dz;
      cells.push_back(GridCell{x, y, z});
    }
  } else {
    int error_x = 2 * dx - dz;
    int error_y = 2 * dy - dz;
    while (z != end.z) {
      z += step_z;
      if (error_x >= 0) {
        x += step_x;
        error_x -= 2 * dz;
      }
      if (error_y >= 0) {
        y += step_y;
        error_y -= 2 * dz;
      }
      error_x += 2 * dx;
      error_y += 2 * dy;
      cells.push_back(GridCell{x, y, z});
    }
  }
  return cells;
}

bool overlapOnAxis(
  const Vec3d &axis,
  const std::array<Vec3d, 3> &vertices,
  const Vec3d &box_half_extent)
{
  if (squaredNorm(axis) <= 1.0e-18) {
    return true;
  }
  const std::array<double, 3> projections{
    dot(vertices[0], axis), dot(vertices[1], axis), dot(vertices[2], axis)};
  const auto [minimum, maximum] = std::minmax_element(
    projections.begin(), projections.end());
  const double radius =
    box_half_extent.x * std::abs(axis.x) +
    box_half_extent.y * std::abs(axis.y) +
    box_half_extent.z * std::abs(axis.z);
  return *minimum <= radius && *maximum >= -radius;
}

bool triangleIntersectsGridCell(
  const std::array<Vec3d, 3> &triangle,
  const fuzzrobo::topological_grid::GridCell &cell,
  const fuzzrobo::topological_grid::GridSpec &spec)
{
  const double half = spec.cell_size * 0.5;
  const Vec3d center{
    spec.origin_x + (static_cast<double>(cell.x) + 0.5) * spec.cell_size,
    spec.origin_y + (static_cast<double>(cell.y) + 0.5) * spec.cell_size,
    spec.origin_z + (static_cast<double>(cell.z) + 0.5) * spec.cell_size};
  const std::array<Vec3d, 3> local{
    triangle[0] - center, triangle[1] - center, triangle[2] - center};
  const Vec3d box_half_extent{half, half, half};
  const std::array<Vec3d, 3> box_axes{
    Vec3d{1.0, 0.0, 0.0}, Vec3d{0.0, 1.0, 0.0}, Vec3d{0.0, 0.0, 1.0}};
  const std::array<Vec3d, 3> edges{
    local[1] - local[0], local[2] - local[1], local[0] - local[2]};

  for (const auto &axis : box_axes) {
    if (!overlapOnAxis(axis, local, box_half_extent)) {
      return false;
    }
  }
  if (!overlapOnAxis(cross(edges[0], edges[1]), local, box_half_extent)) {
    return false;
  }
  for (const auto &edge : edges) {
    for (const auto &box_axis : box_axes) {
      if (!overlapOnAxis(cross(edge, box_axis), local, box_half_extent)) {
        return false;
      }
    }
  }
  return true;
}

bool triangleNormalsAreConsistent(
  const std::array<const ais_gng_msgs::msg::TopologicalNode *, 3> &nodes,
  const Vec3d &triangle_normal,
  double maximum_angle_degrees)
{
  const double triangle_normal_norm = std::sqrt(squaredNorm(triangle_normal));
  if (triangle_normal_norm <= 1.0e-9) {
    return false;
  }
  constexpr double kPi = 3.14159265358979323846;
  const double cosine_threshold = std::cos(
    std::clamp(maximum_angle_degrees, 0.0, 180.0) * kPi / 180.0);
  for (const auto *node : nodes) {
    const Vec3d normal = nodeNormal(*node);
    const double normal_norm = std::sqrt(squaredNorm(normal));
    if (normal_norm <= 1.0e-9) {
      continue;
    }
    const double alignment = std::abs(dot(normal, triangle_normal)) /
      (normal_norm * triangle_normal_norm);
    if (alignment < cosine_threshold) {
      return false;
    }
  }
  return true;
}

}  // namespace

namespace fuzzrobo::topological_grid
{

std::size_t GridCellHash::operator()(const GridCell &cell) const noexcept
{
  std::size_t seed = std::hash<int>{}(cell.x);
  seed ^= std::hash<int>{}(cell.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  seed ^= std::hash<int>{}(cell.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  return seed;
}

std::size_t NodeIdentityHash::operator()(const NodeIdentity &identity) const noexcept
{
  std::size_t seed = std::hash<std::uint16_t>{}(identity.id);
  seed ^= std::hash<std::uint32_t>{}(identity.frame) +
    0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  return seed;
}

namespace
{

bool nodeIdentityRetainsVoxel(
  const LabeledGridVoxel &voxel,
  const NodeObservationMap *current_nodes,
  double maximum_displacement)
{
  if (!current_nodes) {
    return false;
  }
  const double maximum_displacement_squared =
    maximum_displacement * maximum_displacement;
  return std::any_of(
    voxel.node_observations.begin(), voxel.node_observations.end(),
    [current_nodes, maximum_displacement_squared](const NodeObservation &anchor) {
      const auto current_it = current_nodes->find(anchor.identity);
      if (current_it == current_nodes->end()) {
        return false;
      }
      const auto &current = current_it->second;
      const double dx = current.x - anchor.x;
      const double dy = current.y - anchor.y;
      const double dz = current.z - anchor.z;
      return dx * dx + dy * dy + dz * dz <= maximum_displacement_squared;
    });
}

bool deletionDisconnectsVerticalNeighbors(
  const GridCell &candidate,
  const std::unordered_set<GridCell, GridCellHash> &active_cells)
{
  std::vector<GridCell> lower_neighbors;
  std::vector<GridCell> upper_neighbors;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dz == 0) {
          continue;
        }
        const GridCell neighbor{
          candidate.x + dx, candidate.y + dy, candidate.z + dz};
        if (active_cells.find(neighbor) == active_cells.end()) {
          continue;
        }
        (dz < 0 ? lower_neighbors : upper_neighbors).push_back(neighbor);
      }
    }
  }
  if (lower_neighbors.empty() || upper_neighbors.empty()) {
    return false;
  }

  std::unordered_set<GridCell, GridCellHash> visited;
  visited.reserve(active_cells.size());
  std::deque<GridCell> pending;
  pending.push_back(lower_neighbors.front());
  visited.insert(lower_neighbors.front());
  while (!pending.empty()) {
    const GridCell current = pending.front();
    pending.pop_front();
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridCell neighbor{current.x + dx, current.y + dy, current.z + dz};
          if (neighbor == candidate ||
            active_cells.find(neighbor) == active_cells.end() ||
            !visited.insert(neighbor).second)
          {
            continue;
          }
          pending.push_back(neighbor);
        }
      }
    }
  }

  const auto is_disconnected = [&visited](const GridCell &cell) {
      return visited.find(cell) == visited.end();
    };
  return std::any_of(lower_neighbors.begin(), lower_neighbors.end(), is_disconnected) ||
         std::any_of(upper_neighbors.begin(), upper_neighbors.end(), is_disconnected);
}

}  // namespace

TemporalVoxelFilter::TemporalVoxelFilter(TemporalVoxelFilterConfig config)
: config_(config)
{
  config_.history_window_size = std::max<std::size_t>(1, config_.history_window_size);
  config_.minimum_label_history_count = std::clamp<std::size_t>(
    config_.minimum_label_history_count, 1, config_.history_window_size);
  config_.minimum_point_input_history_count = std::clamp<std::size_t>(
    config_.minimum_point_input_history_count, 1, config_.history_window_size);
  config_.isolated_minimum_label_history_count = std::clamp<std::size_t>(
    config_.isolated_minimum_label_history_count, 1, config_.history_window_size);
  config_.isolated_minimum_point_input_history_count = std::clamp<std::size_t>(
    config_.isolated_minimum_point_input_history_count, 1, config_.history_window_size);
  config_.maximum_missing_label_updates = std::min(
    config_.maximum_missing_label_updates, config_.history_window_size);
  config_.node_identity_max_displacement = std::max(
    0.0, config_.node_identity_max_displacement);
}

void TemporalVoxelFilter::appendSample(
  History &history, const HistorySample &sample)
{
  if (history.samples.size() == config_.history_window_size) {
    const auto &expired = history.samples.front();
    if (expired.has_label) {
      --history.label_counts[expired.label];
      --history.label_observation_count;
    }
    if (expired.has_input_points) {
      --history.point_input_observation_count;
    }
    history.samples.pop_front();
  }

  history.samples.push_back(sample);
  if (sample.has_label) {
    ++history.label_counts[sample.label];
    ++history.label_observation_count;
  }
  if (sample.has_input_points) {
    ++history.point_input_observation_count;
  }
}

std::vector<LabeledGridVoxel> TemporalVoxelFilter::update(
  const std::vector<LabeledGridVoxel> &label_voxels,
  bool require_input_points,
  std::size_t minimum_input_points_per_voxel,
  const GridPointCounts *input_point_counts,
  const NodeObservationMap *current_nodes)
{
  struct ExpirationCandidate
  {
    GridCell cell;
    std::size_t input_point_count = 0;
  };

  std::vector<LabeledGridVoxel> stable_voxels;
  stable_voxels.reserve(label_voxels.size());
  std::vector<ExpirationCandidate> expiration_candidates;
  std::unordered_set<GridCell, GridCellHash> updated_cells;
  updated_cells.reserve(label_voxels.size());

  for (const auto &voxel : label_voxels) {
    auto &history = history_[voxel.cell];
    const bool has_input_points = !require_input_points ||
      voxel.input_point_count >= minimum_input_points_per_voxel;
    appendSample(history, HistorySample{voxel.label, true, has_input_points});
    updated_cells.insert(voxel.cell);
    history.consecutive_missing_label_updates = 0;
    history.last_voxel = voxel;
    const bool isolated = voxel.neighbor_count == 0;
    const std::size_t minimum_label_count = isolated
      ? config_.isolated_minimum_label_history_count
      : config_.minimum_label_history_count;
    const std::size_t minimum_point_count = isolated
      ? config_.isolated_minimum_point_input_history_count
      : config_.minimum_point_input_history_count;
    const bool label_history_sufficient =
      history.label_observation_count >= minimum_label_count;
    const bool point_history_sufficient =
      history.point_input_observation_count >= minimum_point_count;
    if (!history.active) {
      if (!label_history_sufficient || !point_history_sufficient || !has_input_points) {
        continue;
      }
      history.active = true;
    }
    history.active_label = dominantLabel(history.label_counts);

    auto stable_voxel = voxel;
    stable_voxel.label = history.active_label;
    stable_voxel.history_sample_count = history.samples.size();
    stable_voxel.label_history_count = history.label_observation_count;
    stable_voxel.point_input_history_count = history.point_input_observation_count;
    stable_voxel.retained_by_node_identity = false;
    history.last_voxel = stable_voxel;
    stable_voxels.push_back(stable_voxel);
  }

  for (auto it = history_.begin(); it != history_.end(); ++it) {
    if (updated_cells.find(it->first) != updated_cells.end()) {
      continue;
    }
    bool has_input_points = !require_input_points;
    std::size_t input_point_count = 0;
    if (input_point_counts) {
      const auto point_it = input_point_counts->find(it->first);
      if (point_it != input_point_counts->end()) {
        input_point_count = point_it->second;
        has_input_points = !require_input_points ||
          point_it->second >= minimum_input_points_per_voxel;
      }
    }
    appendSample(it->second, HistorySample{0, false, has_input_points});
    ++it->second.consecutive_missing_label_updates;

    auto &history = it->second;
    const bool isolated = history.last_voxel.neighbor_count == 0;
    const std::size_t minimum_point_count = isolated
      ? config_.isolated_minimum_point_input_history_count
      : config_.minimum_point_input_history_count;
    const bool point_history_sufficient =
      !isolated || history.point_input_observation_count >= minimum_point_count;
    const bool within_retention_window =
      history.consecutive_missing_label_updates <= config_.maximum_missing_label_updates;
    const bool retained_by_node_identity =
      config_.node_identity_retention_enabled && !isolated &&
      nodeIdentityRetainsVoxel(
      history.last_voxel, current_nodes, config_.node_identity_max_displacement);
    if (history.active && !point_history_sufficient) {
      history.active = false;
    } else if (history.active &&
      (within_retention_window || retained_by_node_identity))
    {
      if (history.label_observation_count > 0) {
        history.active_label = dominantLabel(history.label_counts);
      }
      auto stable_voxel = history.last_voxel;
      stable_voxel.label = history.active_label;
      stable_voxel.node_count = 0;
      stable_voxel.input_point_count = input_point_count;
      stable_voxel.history_sample_count = history.samples.size();
      stable_voxel.label_history_count = history.label_observation_count;
      stable_voxel.point_input_history_count = history.point_input_observation_count;
      stable_voxel.retained_by_node_identity = retained_by_node_identity;
      stable_voxels.push_back(stable_voxel);
    } else if (history.active) {
      expiration_candidates.push_back(ExpirationCandidate{it->first, input_point_count});
    }
  }

  std::unordered_set<GridCell, GridCellHash> active_cells;
  active_cells.reserve(history_.size());
  for (const auto &[cell, history] : history_) {
    if (history.active) {
      active_cells.insert(cell);
    }
  }
  std::sort(
    expiration_candidates.begin(), expiration_candidates.end(),
    [](const ExpirationCandidate &lhs, const ExpirationCandidate &rhs) {
      return gridCellLess(lhs.cell, rhs.cell);
    });
  for (const auto &candidate : expiration_candidates) {
    auto history_it = history_.find(candidate.cell);
    if (history_it == history_.end() || !history_it->second.active) {
      continue;
    }
    auto &history = history_it->second;
    if (deletionDisconnectsVerticalNeighbors(candidate.cell, active_cells)) {
      auto stable_voxel = history.last_voxel;
      stable_voxel.label = history.active_label;
      stable_voxel.node_count = 0;
      stable_voxel.input_point_count = candidate.input_point_count;
      stable_voxel.history_sample_count = history.samples.size();
      stable_voxel.label_history_count = history.label_observation_count;
      stable_voxel.point_input_history_count = history.point_input_observation_count;
      stable_voxels.push_back(stable_voxel);
      continue;
    }
    history.active = false;
    active_cells.erase(candidate.cell);
  }

  for (auto it = history_.begin(); it != history_.end();) {
    if (!it->second.active && it->second.label_observation_count == 0) {
      it = history_.erase(it);
    } else {
      ++it;
    }
  }

  std::sort(stable_voxels.begin(), stable_voxels.end(), cellLess);
  return stable_voxels;
}

void TemporalVoxelFilter::clear()
{
  history_.clear();
}

std::size_t TemporalVoxelFilter::trackedVoxelCount() const noexcept
{
  return history_.size();
}

std::string gridCellToString(const GridCell &cell)
{
  std::ostringstream oss;
  oss << "(" << cell.x << "," << cell.y << "," << cell.z << ")";
  return oss.str();
}

GridCell positionToGridCell(
  double x, double y, double z,
  const GridSpec &spec)
{
  const double inv = spec.cell_size > 0.0 ? 1.0 / spec.cell_size : 0.0;
  const double grid_x = (x - spec.origin_x) * inv;
  const double grid_y = (y - spec.origin_y) * inv;
  const double grid_z = (z - spec.origin_z) * inv;
  return GridCell{
    static_cast<int>(std::floor(grid_x)),
    static_cast<int>(std::floor(grid_y)),
    static_cast<int>(std::floor(grid_z))
  };
}

GridCell nodeToGridCell(
  const ais_gng_msgs::msg::TopologicalNode &node,
  const GridSpec &spec)
{
  return positionToGridCell(node.pos.x, node.pos.y, node.pos.z, spec);
}

std::vector<GridAssignment> assignNodesToGrid(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridSpec &spec)
{
  std::vector<GridAssignment> assignments;
  assignments.reserve(map.nodes.size());
  for (std::size_t i = 0; i < map.nodes.size(); ++i) {
    assignments.push_back(GridAssignment{i, nodeToGridCell(map.nodes[i], spec)});
  }
  return assignments;
}

std::unordered_map<std::size_t, GridCell> assignNodeGridMap(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridSpec &spec)
{
  std::unordered_map<std::size_t, GridCell> result;
  result.reserve(map.nodes.size());
  for (std::size_t i = 0; i < map.nodes.size(); ++i) {
    result.emplace(i, nodeToGridCell(map.nodes[i], spec));
  }
  return result;
}

GridVoxelizationResult voxelizeNodes(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridSpec &spec,
  const VoxelizationOptions &options,
  const GridPointCounts *input_point_counts)
{
  GridVoxelizationResult result;
  std::unordered_map<GridCell, VoxelAccumulator, GridCellHash> accumulators;
  accumulators.reserve(map.nodes.size());
  result.eligible_nodes.reserve(map.nodes.size());
  std::unordered_set<NodeIdentity, NodeIdentityHash> ambiguous_node_identities;

  for (const auto &node : map.nodes) {
    if (options.excluded_labels.find(node.label) != options.excluded_labels.end()) {
      ++result.excluded_node_count;
      continue;
    }
    const GridCell cell = nodeToGridCell(node, spec);
    const NodeObservation observation{
      NodeIdentity{node.id, node.frame}, node.pos.x, node.pos.y, node.pos.z, node.label};
    if (ambiguous_node_identities.find(observation.identity) ==
      ambiguous_node_identities.end())
    {
      const auto [identity_it, inserted] = result.eligible_nodes.emplace(
        observation.identity, observation);
      if (!inserted) {
        result.eligible_nodes.erase(identity_it);
        ambiguous_node_identities.insert(observation.identity);
      }
    }
    auto &accumulator = accumulators[cell];
    ++accumulator.node_count;
    ++accumulator.label_counts[node.label];
    accumulator.node_observations.push_back(observation);
  }

  result.label_voxels.reserve(accumulators.size());
  for (auto &[cell, accumulator] : accumulators) {
    if (input_point_counts) {
      const auto point_it = input_point_counts->find(cell);
      if (point_it != input_point_counts->end()) {
        accumulator.input_point_count = point_it->second;
      }
    }
    LabeledGridVoxel voxel;
    voxel.cell = cell;
    voxel.label = dominantLabel(accumulator);
    voxel.node_count = accumulator.node_count;
    voxel.input_point_count = accumulator.input_point_count;
    voxel.node_observations = std::move(accumulator.node_observations);
    result.label_voxels.push_back(std::move(voxel));
    if (options.require_input_points && accumulator.input_point_count <
      options.minimum_input_points_per_voxel)
    {
      result.unsupported_node_count += accumulator.node_count;
      ++result.insufficient_point_voxel_count;
      continue;
    }
    result.included_node_count += accumulator.node_count;
  }

  std::unordered_set<GridCell, GridCellHash> label_cells;
  label_cells.reserve(result.label_voxels.size());
  for (const auto &voxel : result.label_voxels) {
    label_cells.insert(voxel.cell);
  }

  const int radius = std::max(0, options.neighbor_radius_cells);
  result.voxels.reserve(result.label_voxels.size());
  for (auto &voxel : result.label_voxels) {
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridCell neighbor{
            voxel.cell.x + dx, voxel.cell.y + dy, voxel.cell.z + dz};
          if (label_cells.find(neighbor) != label_cells.end()) {
            ++voxel.neighbor_count;
          }
        }
      }
    }
    if (options.require_input_points &&
      voxel.input_point_count < options.minimum_input_points_per_voxel)
    {
      continue;
    }
    if (voxel.neighbor_count == 0) {
      ++result.isolated_voxel_count;
    }
    result.voxels.push_back(voxel);
  }

  std::sort(result.label_voxels.begin(), result.label_voxels.end(), cellLess);
  std::sort(result.voxels.begin(), result.voxels.end(), cellLess);
  return result;
}

EdgeVoxelizationResult inferVoxelsFromStableVoxelEdges(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridSpec &spec,
  const std::vector<LabeledGridVoxel> &stable_direct_voxels,
  const std::unordered_set<std::uint8_t> &excluded_labels,
  const EdgeInferenceOptions &options)
{
  EdgeVoxelizationResult result;
  if (!options.enabled) {
    return result;
  }

  result.input_edge_count = map.edges.size() / 2U;
  if (map.edges.size() % 2U != 0U) {
    ++result.invalid_edge_count;
  }

  std::unordered_map<GridCell, std::uint8_t, GridCellHash> direct_cells;
  direct_cells.reserve(stable_direct_voxels.size());
  for (const auto &voxel : stable_direct_voxels) {
    direct_cells.emplace(voxel.cell, voxel.label);
  }

  std::unordered_set<VoxelEdge, VoxelEdgeHash> seen_voxel_edges;
  seen_voxel_edges.reserve(result.input_edge_count);
  std::unordered_map<GridCell, VoxelAccumulator, GridCellHash> inferred;

  for (std::size_t edge_index = 0; edge_index + 1U < map.edges.size(); edge_index += 2U) {
    const std::size_t lhs_index = static_cast<std::size_t>(map.edges[edge_index]);
    const std::size_t rhs_index = static_cast<std::size_t>(map.edges[edge_index + 1U]);
    if (lhs_index >= map.nodes.size() || rhs_index >= map.nodes.size()) {
      ++result.invalid_edge_count;
      continue;
    }

    const auto &lhs_node = map.nodes[lhs_index];
    const auto &rhs_node = map.nodes[rhs_index];
    if (excluded_labels.find(lhs_node.label) != excluded_labels.end() ||
      excluded_labels.find(rhs_node.label) != excluded_labels.end())
    {
      ++result.excluded_edge_count;
      continue;
    }

    const GridCell lhs_cell = nodeToGridCell(lhs_node, spec);
    const GridCell rhs_cell = nodeToGridCell(rhs_node, spec);
    if (direct_cells.find(lhs_cell) == direct_cells.end() ||
      direct_cells.find(rhs_cell) == direct_cells.end())
    {
      ++result.inactive_endpoint_edge_count;
      continue;
    }
    if (lhs_cell == rhs_cell) {
      ++result.same_voxel_edge_count;
      continue;
    }

    const VoxelEdge voxel_edge = canonicalVoxelEdge(lhs_cell, rhs_cell);
    if (!seen_voxel_edges.insert(voxel_edge).second) {
      ++result.duplicate_voxel_edge_count;
      continue;
    }

    const double delta_x = static_cast<double>(rhs_cell.x - lhs_cell.x) * spec.cell_size;
    const double delta_y = static_cast<double>(rhs_cell.y - lhs_cell.y) * spec.cell_size;
    const double delta_z = static_cast<double>(rhs_cell.z - lhs_cell.z) * spec.cell_size;
    if (std::sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z) >
      options.maximum_edge_length)
    {
      ++result.overlength_edge_count;
      continue;
    }

    ++result.voxel_edge_count;
    const auto line_cells = rasterizeGridLine(lhs_cell, rhs_cell);
    for (std::size_t line_index = 1U; line_index + 1U < line_cells.size(); ++line_index) {
      const auto &cell = line_cells[line_index];
      if (direct_cells.find(cell) != direct_cells.end()) {
        continue;
      }
      auto &accumulator = inferred[cell];
      const bool use_lhs_label = line_index * 2U <= line_cells.size() - 1U;
      ++accumulator.label_counts[use_lhs_label ? lhs_node.label : rhs_node.label];
      ++accumulator.edge_support_count;
    }
  }

  result.voxels.reserve(inferred.size());
  for (const auto &[cell, accumulator] : inferred) {
    LabeledGridVoxel voxel;
    voxel.cell = cell;
    voxel.label = dominantLabel(accumulator);
    voxel.edge_support_count = accumulator.edge_support_count;
    result.voxels.push_back(voxel);
  }
  std::sort(result.voxels.begin(), result.voxels.end(), cellLess);
  return result;
}

TriangleVoxelizationResult inferVoxelsFromStableVoxelTriangles(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridSpec &spec,
  const std::vector<LabeledGridVoxel> &stable_direct_voxels,
  const std::unordered_set<std::uint8_t> &excluded_labels,
  const TriangleInferenceOptions &options,
  const GridPointCounts *input_point_counts)
{
  TriangleVoxelizationResult result;
  if (!options.enabled || map.nodes.size() < 3U) {
    return result;
  }

  std::unordered_set<GridCell, GridCellHash> direct_cells;
  direct_cells.reserve(stable_direct_voxels.size());
  for (const auto &voxel : stable_direct_voxels) {
    direct_cells.insert(voxel.cell);
  }

  std::vector<std::unordered_set<std::size_t>> adjacency(map.nodes.size());
  for (std::size_t edge_index = 0; edge_index + 1U < map.edges.size(); edge_index += 2U) {
    const std::size_t lhs = static_cast<std::size_t>(map.edges[edge_index]);
    const std::size_t rhs = static_cast<std::size_t>(map.edges[edge_index + 1U]);
    if (lhs >= map.nodes.size() || rhs >= map.nodes.size() || lhs == rhs) {
      continue;
    }
    adjacency[lhs].insert(rhs);
    adjacency[rhs].insert(lhs);
  }

  std::unordered_map<GridCell, VoxelAccumulator, GridCellHash> inferred;
  for (std::size_t first = 0; first < map.nodes.size(); ++first) {
    for (const std::size_t second : adjacency[first]) {
      if (second <= first) {
        continue;
      }
      for (const std::size_t third : adjacency[first]) {
        if (third <= second || adjacency[second].find(third) == adjacency[second].end()) {
          continue;
        }
        ++result.candidate_triangle_count;
        const std::array<const ais_gng_msgs::msg::TopologicalNode *, 3> nodes{
          &map.nodes[first], &map.nodes[second], &map.nodes[third]};
        if (std::any_of(
            nodes.begin(), nodes.end(),
            [&excluded_labels](const auto *node) {
              return excluded_labels.find(node->label) != excluded_labels.end();
            }))
        {
          ++result.excluded_triangle_count;
          continue;
        }

        const std::array<GridCell, 3> node_cells{
          nodeToGridCell(*nodes[0], spec),
          nodeToGridCell(*nodes[1], spec),
          nodeToGridCell(*nodes[2], spec)};
        if (std::any_of(
            node_cells.begin(), node_cells.end(),
            [&direct_cells](const GridCell &cell) {
              return direct_cells.find(cell) == direct_cells.end();
            }))
        {
          ++result.inactive_vertex_triangle_count;
          continue;
        }

        const std::array<Vec3d, 3> triangle{
          nodePosition(*nodes[0]), nodePosition(*nodes[1]), nodePosition(*nodes[2])};
        const std::array<Vec3d, 3> edges{
          triangle[1] - triangle[0],
          triangle[2] - triangle[1],
          triangle[0] - triangle[2]};
        const std::array<double, 3> edge_lengths_squared{
          squaredNorm(edges[0]), squaredNorm(edges[1]), squaredNorm(edges[2])};
        const double maximum_edge_length_squared = *std::max_element(
          edge_lengths_squared.begin(), edge_lengths_squared.end());
        if (maximum_edge_length_squared >
          options.maximum_edge_length * options.maximum_edge_length)
        {
          ++result.overlength_triangle_count;
          continue;
        }

        const Vec3d triangle_normal = cross(edges[0], triangle[2] - triangle[0]);
        const double twice_area = std::sqrt(squaredNorm(triangle_normal));
        const double area = 0.5 * twice_area;
        const double aspect_ratio = maximum_edge_length_squared > 0.0
          ? twice_area / maximum_edge_length_squared : 0.0;
        if (area < options.minimum_area || aspect_ratio < options.minimum_aspect_ratio) {
          ++result.degenerate_triangle_count;
          continue;
        }
        if (!triangleNormalsAreConsistent(
            nodes, triangle_normal, options.maximum_normal_angle_degrees))
        {
          ++result.normal_rejected_triangle_count;
          continue;
        }

        const Vec3d minimum{
          std::min({triangle[0].x, triangle[1].x, triangle[2].x}),
          std::min({triangle[0].y, triangle[1].y, triangle[2].y}),
          std::min({triangle[0].z, triangle[1].z, triangle[2].z})};
        const Vec3d maximum{
          std::max({triangle[0].x, triangle[1].x, triangle[2].x}),
          std::max({triangle[0].y, triangle[1].y, triangle[2].y}),
          std::max({triangle[0].z, triangle[1].z, triangle[2].z})};
        const GridCell minimum_cell = positionToGridCell(
          minimum.x, minimum.y, minimum.z, spec);
        const GridCell maximum_cell = positionToGridCell(
          maximum.x, maximum.y, maximum.z, spec);
        std::vector<GridCell> triangle_cells;
        for (int x = minimum_cell.x; x <= maximum_cell.x; ++x) {
          for (int y = minimum_cell.y; y <= maximum_cell.y; ++y) {
            for (int z = minimum_cell.z; z <= maximum_cell.z; ++z) {
              const GridCell cell{x, y, z};
              if (triangleIntersectsGridCell(triangle, cell, spec)) {
                triangle_cells.push_back(cell);
              }
            }
          }
        }

        const auto supported_cell_count = static_cast<std::size_t>(std::count_if(
          triangle_cells.begin(), triangle_cells.end(),
          [input_point_counts](const GridCell &cell) {
            return input_point_counts && input_point_counts->find(cell) !=
                   input_point_counts->end();
          }));
        const double support_ratio = triangle_cells.empty() ? 0.0 :
          static_cast<double>(supported_cell_count) /
          static_cast<double>(triangle_cells.size());
        if (support_ratio < options.minimum_point_support_ratio) {
          ++result.point_support_rejected_triangle_count;
          continue;
        }

        std::array<std::size_t, 256> triangle_label_counts{};
        for (const auto *node : nodes) {
          ++triangle_label_counts[node->label];
        }
        const std::uint8_t triangle_label = dominantLabel(triangle_label_counts);
        ++result.accepted_triangle_count;
        for (const auto &cell : triangle_cells) {
          if (direct_cells.find(cell) != direct_cells.end()) {
            continue;
          }
          auto &accumulator = inferred[cell];
          ++accumulator.label_counts[triangle_label];
          ++accumulator.triangle_support_count;
        }
      }
    }
  }

  result.voxels.reserve(inferred.size());
  for (const auto &[cell, accumulator] : inferred) {
    LabeledGridVoxel voxel;
    voxel.cell = cell;
    voxel.label = dominantLabel(accumulator);
    voxel.triangle_support_count = accumulator.triangle_support_count;
    result.voxels.push_back(voxel);
  }
  std::sort(result.voxels.begin(), result.voxels.end(), cellLess);
  return result;
}

std::vector<LabeledGridVoxel> mergeDirectAndInferredVoxels(
  const std::vector<LabeledGridVoxel> &direct_voxels,
  const std::vector<LabeledGridVoxel> &inferred_voxels)
{
  std::vector<LabeledGridVoxel> merged = direct_voxels;
  merged.reserve(direct_voxels.size() + inferred_voxels.size());
  std::unordered_map<GridCell, std::size_t, GridCellHash> merged_indices;
  merged_indices.reserve(direct_voxels.size() + inferred_voxels.size());
  for (std::size_t index = 0; index < merged.size(); ++index) {
    merged_indices.emplace(merged[index].cell, index);
  }
  for (const auto &voxel : inferred_voxels) {
    const auto existing = merged_indices.find(voxel.cell);
    if (existing == merged_indices.end()) {
      merged_indices.emplace(voxel.cell, merged.size());
      merged.push_back(voxel);
      continue;
    }
    auto &merged_voxel = merged[existing->second];
    merged_voxel.edge_support_count += voxel.edge_support_count;
    merged_voxel.triangle_support_count += voxel.triangle_support_count;
  }
  std::sort(merged.begin(), merged.end(), cellLess);
  return merged;
}

VoxelIsolationSplit splitVoxelsByIsolation(
  const std::vector<LabeledGridVoxel> &voxels)
{
  VoxelIsolationSplit split;
  split.connected_voxels.reserve(voxels.size());
  split.isolated_voxels.reserve(voxels.size());
  for (const auto &voxel : voxels) {
    if (voxel.neighbor_count == 0) {
      split.isolated_voxels.push_back(voxel);
    } else {
      split.connected_voxels.push_back(voxel);
    }
  }
  return split;
}

}  // namespace fuzzrobo::topological_grid
