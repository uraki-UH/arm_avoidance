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

TemporalVoxelFilter::TemporalVoxelFilter(
  std::size_t minimum_observations,
  std::size_t maximum_missed_updates)
: minimum_observations_(std::max<std::size_t>(1, minimum_observations)),
  maximum_missed_updates_(maximum_missed_updates)
{
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
    const std::size_t missed_updates = inserted || history.last_update == 0
      ? 0
      : update_count_ - history.last_update - 1;
    if (inserted || missed_updates > maximum_missed_updates_ || history.label != voxel.label) {
      history.label = voxel.label;
      history.observations = 1;
    } else if (history.observations < minimum_observations_) {
      ++history.observations;
    }
    history.last_update = update_count_;

    if (history.observations >= minimum_observations_) {
      stable_voxels.push_back(voxel);
    }
  }

  for (auto it = history_.begin(); it != history_.end();) {
    if (update_count_ - it->second.last_update > maximum_missed_updates_) {
      it = history_.erase(it);
    } else {
      ++it;
    }
  }
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

GridCell nodeToGridCell(
  const ais_gng_msgs::msg::TopologicalNode &node,
  const GridSpec &spec)
{
  const double inv = spec.cell_size > 0.0 ? 1.0 / spec.cell_size : 0.0;
  const double x = (static_cast<double>(node.pos.x) - spec.origin_x) * inv;
  const double y = (static_cast<double>(node.pos.y) - spec.origin_y) * inv;
  const double z = (static_cast<double>(node.pos.z) - spec.origin_z) * inv;
  return GridCell{
    static_cast<int>(std::floor(x)),
    static_cast<int>(std::floor(y)),
    static_cast<int>(std::floor(z))
  };
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
  const std::unordered_set<std::uint8_t> &excluded_labels)
{
  GridVoxelizationResult result;
  std::unordered_map<GridCell, VoxelAccumulator, GridCellHash> accumulators;
  accumulators.reserve(map.nodes.size());

  for (const auto &node : map.nodes) {
    if (excluded_labels.find(node.label) != excluded_labels.end()) {
      ++result.excluded_node_count;
      continue;
    }

    ++result.included_node_count;
    const GridCell cell = nodeToGridCell(node, spec);
    auto &accumulator = accumulators[cell];
    ++accumulator.node_count;
    ++accumulator.label_counts[node.label];
  }

  result.voxels.reserve(accumulators.size());
  for (const auto &[cell, accumulator] : accumulators) {
    result.voxels.push_back(LabeledGridVoxel{
      cell, dominantLabel(accumulator), accumulator.node_count});
  }

  std::sort(
    result.voxels.begin(), result.voxels.end(),
    [](const LabeledGridVoxel &lhs, const LabeledGridVoxel &rhs) {
      if (lhs.cell.x != rhs.cell.x) {
        return lhs.cell.x < rhs.cell.x;
      }
      if (lhs.cell.y != rhs.cell.y) {
        return lhs.cell.y < rhs.cell.y;
      }
      return lhs.cell.z < rhs.cell.z;
    });
  return result;
}

}  // namespace fuzzrobo::topological_grid
