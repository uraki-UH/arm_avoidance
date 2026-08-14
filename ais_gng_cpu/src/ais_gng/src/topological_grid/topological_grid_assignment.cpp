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
};

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
  const GridPointCounts *input_point_counts)
{
  std::vector<LabeledGridVoxel> stable_voxels;
  stable_voxels.reserve(label_voxels.size());
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
    if (history.active) {
      if (isolated && !point_history_sufficient) {
        history.active = false;
        continue;
      }
    } else {
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
    history.last_voxel = stable_voxel;
    stable_voxels.push_back(stable_voxel);
  }

  for (auto it = history_.begin(); it != history_.end();) {
    if (updated_cells.find(it->first) != updated_cells.end()) {
      ++it;
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
    if (history.active &&
      history.consecutive_missing_label_updates <= config_.maximum_missing_label_updates &&
      (!isolated || history.point_input_observation_count >= minimum_point_count))
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
      stable_voxels.push_back(stable_voxel);
    } else {
      history.active = false;
    }
    if (!history.active && history.label_observation_count == 0) {
      it = history_.erase(it);
      continue;
    }
    ++it;
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

  for (const auto &node : map.nodes) {
    if (options.excluded_labels.find(node.label) != options.excluded_labels.end()) {
      ++result.excluded_node_count;
      continue;
    }
    const GridCell cell = nodeToGridCell(node, spec);
    auto &accumulator = accumulators[cell];
    ++accumulator.node_count;
    ++accumulator.label_counts[node.label];
  }

  result.label_voxels.reserve(accumulators.size());
  for (auto &[cell, accumulator] : accumulators) {
    if (input_point_counts) {
      const auto point_it = input_point_counts->find(cell);
      if (point_it != input_point_counts->end()) {
        accumulator.input_point_count = point_it->second;
      }
    }
    result.label_voxels.push_back(LabeledGridVoxel{
      cell, dominantLabel(accumulator), accumulator.node_count,
      accumulator.input_point_count, 0});
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

std::vector<LabeledGridVoxel> mergeDirectAndInferredVoxels(
  const std::vector<LabeledGridVoxel> &direct_voxels,
  const std::vector<LabeledGridVoxel> &inferred_voxels)
{
  std::vector<LabeledGridVoxel> merged = direct_voxels;
  merged.reserve(direct_voxels.size() + inferred_voxels.size());
  std::unordered_set<GridCell, GridCellHash> direct_cells;
  direct_cells.reserve(direct_voxels.size());
  for (const auto &voxel : direct_voxels) {
    direct_cells.insert(voxel.cell);
  }
  for (const auto &voxel : inferred_voxels) {
    if (direct_cells.find(voxel.cell) == direct_cells.end()) {
      merged.push_back(voxel);
    }
  }
  std::sort(merged.begin(), merged.end(), cellLess);
  return merged;
}

}  // namespace fuzzrobo::topological_grid
