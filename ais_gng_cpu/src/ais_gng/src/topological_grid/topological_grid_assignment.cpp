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
};

std::uint8_t dominantLabel(const VoxelAccumulator &accumulator)
{
  std::uint8_t dominant = 0;
  std::size_t dominant_count = 0;
  for (std::size_t label = 0; label < accumulator.label_counts.size(); ++label) {
    if (accumulator.label_counts[label] > dominant_count) {
      dominant = static_cast<std::uint8_t>(label);
      dominant_count = accumulator.label_counts[label];
    }
  }
  return dominant;
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
  config_.minimum_observations = std::max<std::size_t>(1, config_.minimum_observations);
  config_.isolated_minimum_observations = std::max<std::size_t>(
    1, config_.isolated_minimum_observations);
}

std::vector<LabeledGridVoxel> TemporalVoxelFilter::update(
  const std::vector<LabeledGridVoxel> &observed_voxels)
{
  ++update_count_;
  std::vector<LabeledGridVoxel> stable_voxels;
  stable_voxels.reserve(observed_voxels.size());

  for (const auto &voxel : observed_voxels) {
    auto [history_it, inserted] = history_.try_emplace(voxel.cell);
    auto &history = history_it->second;
    const bool isolated = voxel.neighbor_count == 0;
    const std::size_t allowed_misses = history.isolated
      ? config_.isolated_maximum_missed_updates
      : config_.maximum_missed_updates;
    const std::size_t missed_updates = inserted || history.last_update == 0 ?
      0 : update_count_ - history.last_update - 1;
    const bool label_changed = !inserted && history.label != voxel.label;

    if (inserted || label_changed || missed_updates > allowed_misses) {
      history.label = voxel.label;
      history.consecutive_observations = 1;
      history.confirmed = false;
    } else if (!history.confirmed && missed_updates > 0) {
      history.consecutive_observations = 1;
    } else if (!history.confirmed) {
      ++history.consecutive_observations;
    }
    history.last_update = update_count_;
    history.isolated = isolated;

    const std::size_t required_observations = isolated
      ? config_.isolated_minimum_observations
      : config_.minimum_observations;
    if (!history.confirmed &&
      history.consecutive_observations >= required_observations)
    {
      history.confirmed = true;
    }
    if (history.confirmed) {
      stable_voxels.push_back(voxel);
    }
  }

  for (auto it = history_.begin(); it != history_.end();) {
    const std::size_t allowed_misses = it->second.isolated
      ? config_.isolated_maximum_missed_updates
      : config_.maximum_missed_updates;
    if (update_count_ - it->second.last_update > allowed_misses) {
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
  update_count_ = 0;
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
    const bool included_label = options.included_labels.empty() ||
      options.included_labels.find(node.label) != options.included_labels.end();
    if (!included_label ||
      options.excluded_labels.find(node.label) != options.excluded_labels.end())
    {
      ++result.excluded_node_count;
      continue;
    }
    const GridCell cell = nodeToGridCell(node, spec);
    auto &accumulator = accumulators[cell];
    ++accumulator.node_count;
    ++accumulator.label_counts[node.label];
  }

  result.voxels.reserve(accumulators.size());
  for (auto &[cell, accumulator] : accumulators) {
    if (input_point_counts) {
      const auto point_it = input_point_counts->find(cell);
      if (point_it != input_point_counts->end()) {
        accumulator.input_point_count = point_it->second;
      }
    }
    if (options.require_input_points &&
      accumulator.input_point_count < options.minimum_input_points_per_voxel)
    {
      result.unsupported_node_count += accumulator.node_count;
      ++result.insufficient_point_voxel_count;
      continue;
    }
    result.included_node_count += accumulator.node_count;
    result.voxels.push_back(LabeledGridVoxel{
      cell, dominantLabel(accumulator), accumulator.node_count,
      accumulator.input_point_count, 0});
  }

  std::unordered_set<GridCell, GridCellHash> occupied_cells;
  occupied_cells.reserve(result.voxels.size());
  for (const auto &voxel : result.voxels) {
    occupied_cells.insert(voxel.cell);
  }

  const int radius = std::max(0, options.neighbor_radius_cells);
  for (auto &voxel : result.voxels) {
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridCell neighbor{
            voxel.cell.x + dx, voxel.cell.y + dy, voxel.cell.z + dz};
          if (occupied_cells.find(neighbor) != occupied_cells.end()) {
            ++voxel.neighbor_count;
          }
        }
      }
    }
    if (voxel.neighbor_count == 0) {
      ++result.isolated_voxel_count;
    }
  }

  std::sort(result.voxels.begin(), result.voxels.end(), cellLess);
  return result;
}

}  // namespace fuzzrobo::topological_grid
