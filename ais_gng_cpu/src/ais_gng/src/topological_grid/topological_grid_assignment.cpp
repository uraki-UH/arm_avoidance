#include <ais_gng/topological_grid/topological_grid_assignment.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>

namespace
{
// This is dimensionless: the normal displacement is divided by the node's
// local incident-edge spacing.  Below half a local spacing, the movement is
// treated as ordinary GNG/depth jitter rather than deletion evidence.
constexpr double kNormalDriftEvidenceThreshold = 0.5;

struct VoxelAccumulator
{
  std::array<std::size_t, 256> label_counts{};
  std::size_t node_count = 0;
  // GNG input IDs and live geometric point support are independent evidence.
  // Keep them separate until the requested support mode combines them.
  std::size_t node_input_id_count = 0;
  std::size_t input_point_count = 0;
  std::size_t edge_support_count = 0;
  std::size_t triangle_support_count = 0;
  std::vector<fuzzrobo::topological_grid::NodeObservation> node_observations;
};

struct UnknownNodeComponent
{
  std::vector<std::size_t> node_indices;
  std::unordered_set<fuzzrobo::topological_grid::GridCell,
    fuzzrobo::topological_grid::GridCellHash> cells;
};

// The point-support radius is queried for every labeled cell.  At a 10 mm
// grid and 20 mm radius, the direct implementation performs 125 hash lookups
// per query.  This sparse tiled summed-volume cache returns the exact same
// inclusive grid cube sum with at most eight tile intersections.  A bounded
// allocation and the legacy fallback keep it safe for very sparse or very
// wide coordinate ranges.
class SparsePointSupportIndex
{
public:
  using GridCell = fuzzrobo::topological_grid::GridCell;
  using GridCellHash = fuzzrobo::topological_grid::GridCellHash;
  using GridPointCounts = fuzzrobo::topological_grid::GridPointCounts;

  void build(const GridPointCounts &point_counts, int radius_cells)
  {
    enabled_ = false;
    tiles_.clear();
    if (radius_cells < 2 || point_counts.empty()) {
      return;
    }

    const std::size_t width = static_cast<std::size_t>(2 * radius_cells + 1);
    const std::size_t query_volume = width * width * width;
    // The legacy path scans the sparse source directly when it has fewer
    // entries than the requested cube, so a prefix cache cannot win there.
    if (point_counts.size() <= query_volume) {
      return;
    }

    std::unordered_set<GridCell, GridCellHash> tile_keys;
    tile_keys.reserve(point_counts.size());
    for (const auto &[cell, count] : point_counts) {
      (void)count;
      tile_keys.insert(tileCell(cell));
    }
    constexpr std::size_t kMaximumPrefixEntries = 4U * 1024U * 1024U;
    const std::size_t entries_per_tile = kPrefixSide * kPrefixSide * kPrefixSide;
    if (tile_keys.size() > kMaximumPrefixEntries / entries_per_tile) {
      return;
    }

    tiles_.reserve(tile_keys.size());
    for (const auto &key : tile_keys) {
      tiles_.emplace(key, Tile{});
    }
    for (const auto &[cell, count] : point_counts) {
      auto &tile = tiles_.at(tileCell(cell));
      const int local_x = floorMod(cell.x, kTileWidth);
      const int local_y = floorMod(cell.y, kTileWidth);
      const int local_z = floorMod(cell.z, kTileWidth);
      tile.prefix[prefixIndex(local_x + 1, local_y + 1, local_z + 1)] += count;
    }
    for (auto &[key, tile] : tiles_) {
      (void)key;
      for (int x = 1; x <= kTileWidth; ++x) {
        for (int y = 1; y <= kTileWidth; ++y) {
          for (int z = 1; z <= kTileWidth; ++z) {
            auto &value = tile.prefix[prefixIndex(x, y, z)];
            value += tile.prefix[prefixIndex(x - 1, y, z)] +
              tile.prefix[prefixIndex(x, y - 1, z)] +
              tile.prefix[prefixIndex(x, y, z - 1)] -
              tile.prefix[prefixIndex(x - 1, y - 1, z)] -
              tile.prefix[prefixIndex(x - 1, y, z - 1)] -
              tile.prefix[prefixIndex(x, y - 1, z - 1)] +
              tile.prefix[prefixIndex(x - 1, y - 1, z - 1)];
          }
        }
      }
    }
    enabled_ = true;
  }

  bool enabled() const noexcept
  {
    return enabled_;
  }

  std::size_t countCube(const GridCell &cell, int radius_cells) const
  {
    const GridCell minimum{
      cell.x - radius_cells, cell.y - radius_cells, cell.z - radius_cells};
    const GridCell maximum{
      cell.x + radius_cells, cell.y + radius_cells, cell.z + radius_cells};
    const GridCell minimum_tile = tileCell(minimum);
    const GridCell maximum_tile = tileCell(maximum);
    std::size_t count = 0;
    for (int tile_x = minimum_tile.x; tile_x <= maximum_tile.x; ++tile_x) {
      for (int tile_y = minimum_tile.y; tile_y <= maximum_tile.y; ++tile_y) {
        for (int tile_z = minimum_tile.z; tile_z <= maximum_tile.z; ++tile_z) {
          const GridCell tile_key{tile_x, tile_y, tile_z};
          const auto found = tiles_.find(tile_key);
          if (found == tiles_.end()) {
            continue;
          }
          const int base_x = tile_x * kTileWidth;
          const int base_y = tile_y * kTileWidth;
          const int base_z = tile_z * kTileWidth;
          const int local_min_x = std::max(0, minimum.x - base_x);
          const int local_min_y = std::max(0, minimum.y - base_y);
          const int local_min_z = std::max(0, minimum.z - base_z);
          const int local_max_x = std::min(kTileWidth - 1, maximum.x - base_x);
          const int local_max_y = std::min(kTileWidth - 1, maximum.y - base_y);
          const int local_max_z = std::min(kTileWidth - 1, maximum.z - base_z);
          count += boxSum(
            found->second, local_min_x, local_min_y, local_min_z,
            local_max_x, local_max_y, local_max_z);
        }
      }
    }
    return count;
  }

private:
  static constexpr int kTileWidth = 8;
  static constexpr std::size_t kPrefixSide = static_cast<std::size_t>(kTileWidth + 1);

  struct Tile
  {
    std::array<std::size_t, kPrefixSide * kPrefixSide * kPrefixSide> prefix{};
  };

  static int floorDiv(int value, int divisor)
  {
    const int quotient = value / divisor;
    const int remainder = value % divisor;
    return remainder < 0 ? quotient - 1 : quotient;
  }

  static int floorMod(int value, int divisor)
  {
    return value - floorDiv(value, divisor) * divisor;
  }

  static GridCell tileCell(const GridCell &cell)
  {
    return GridCell{
      floorDiv(cell.x, kTileWidth), floorDiv(cell.y, kTileWidth),
      floorDiv(cell.z, kTileWidth)};
  }

  static std::size_t prefixIndex(int x, int y, int z)
  {
    return (static_cast<std::size_t>(x) * kPrefixSide + static_cast<std::size_t>(y)) *
      kPrefixSide + static_cast<std::size_t>(z);
  }

  static std::size_t boxSum(
    const Tile &tile, int min_x, int min_y, int min_z,
    int max_x, int max_y, int max_z)
  {
    const auto at = [&tile](int x, int y, int z) {
        return tile.prefix[prefixIndex(x, y, z)];
      };
    const int x = max_x + 1;
    const int y = max_y + 1;
    const int z = max_z + 1;
    return at(x, y, z) - at(min_x, y, z) - at(x, min_y, z) - at(x, y, min_z) +
      at(min_x, min_y, z) + at(min_x, y, min_z) + at(x, min_y, min_z) -
      at(min_x, min_y, min_z);
  }

  bool enabled_ = false;
  std::unordered_map<GridCell, Tile, GridCellHash> tiles_;
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

}  // namespace

TemporalVoxelFilter::TemporalVoxelFilter(TemporalVoxelFilterConfig config)
: config_(config)
{
  config_.history_window_size = std::max<std::size_t>(1, config_.history_window_size);
  config_.time_constant_sec = std::max(1.0e-6, config_.time_constant_sec);
  config_.activation_score = std::clamp(config_.activation_score, 0.0, 1.0);
  config_.retention_score = std::clamp(
    config_.retention_score, 0.0, config_.activation_score);
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

void TemporalVoxelFilter::migrateHistoriesByNodeIdentity(
  const std::vector<LabeledGridVoxel> &label_voxels)
{
  if (!config_.node_identity_history_migration_enabled || history_.empty()) {
    return;
  }

  struct HistoryAnchor
  {
    GridCell cell;
    NodeObservation observation;
    bool ambiguous = false;
  };
  std::unordered_map<NodeIdentity, HistoryAnchor, NodeIdentityHash> anchors;
  for (const auto &[cell, history] : history_) {
    for (const auto &observation : history.last_voxel.node_observations) {
      const auto [it, inserted] = anchors.emplace(
        observation.identity, HistoryAnchor{cell, observation, false});
      if (!inserted && !(it->second.cell == cell)) {
        it->second.ambiguous = true;
      }
    }
  }

  const double maximum_displacement_squared =
    config_.node_identity_max_displacement * config_.node_identity_max_displacement;
  std::unordered_set<GridCell, GridCellHash> migrated_sources;
  for (const auto &voxel : label_voxels) {
    if (history_.find(voxel.cell) != history_.end()) {
      continue;
    }

    auto source = history_.end();
    std::size_t best_observation_count = 0;
    for (const auto &current : voxel.node_observations) {
      const auto anchor_it = anchors.find(current.identity);
      if (anchor_it == anchors.end() || anchor_it->second.ambiguous ||
        anchor_it->second.cell == voxel.cell ||
        migrated_sources.find(anchor_it->second.cell) != migrated_sources.end())
      {
        continue;
      }
      const auto &anchor = anchor_it->second.observation;
      const double dx = current.x - anchor.x;
      const double dy = current.y - anchor.y;
      const double dz = current.z - anchor.z;
      if (dx * dx + dy * dy + dz * dz > maximum_displacement_squared) {
        continue;
      }
      const auto candidate = history_.find(anchor_it->second.cell);
      if (candidate != history_.end() &&
        (source == history_.end() ||
        candidate->second.label_observation_count > best_observation_count))
      {
        source = candidate;
        best_observation_count = candidate->second.label_observation_count;
      }
    }
    if (source == history_.end()) {
      continue;
    }

    const GridCell source_cell = source->first;
    auto history_node = history_.extract(source);
    history_node.key() = voxel.cell;
    history_node.mapped().last_voxel.cell = voxel.cell;
    history_.insert(std::move(history_node));
    migrated_sources.insert(source_cell);
  }
}

std::vector<LabeledGridVoxel> TemporalVoxelFilter::update(
  const std::vector<LabeledGridVoxel> &label_voxels,
  bool require_input_points,
  std::size_t minimum_input_points_per_voxel,
  const GridPointCounts *input_point_counts,
  const NodeObservationMap *current_nodes,
  double elapsed_seconds,
  const GridVisibilityStates *visibility_states,
  const NodeLocalStructureStates *local_structure_states)
{
  const double alpha = std::isfinite(elapsed_seconds) && elapsed_seconds > 0.0
    ? -std::expm1(-elapsed_seconds / config_.time_constant_sec)
    : 0.0;
  const auto updateTemporalScores = [alpha](History &history, double observation_score) {
    const double observation = std::clamp(observation_score, 0.0, 1.0);
    const double transition = history.has_previous_observation
      ? std::abs(observation - history.previous_observation_score)
      : 0.0;
    history.presence_score =
      (1.0 - alpha) * history.presence_score + alpha * observation;
    history.switching_score =
      (1.0 - alpha) * history.switching_score + alpha * transition;
    history.stability_score = history.presence_score * (1.0 - history.switching_score);
    history.previous_observation_score = observation;
    history.has_previous_observation = true;
  };
  const auto holdsHistory = [visibility_states](const GridCell &cell) {
    if (!visibility_states) {
      return false;
    }
    const auto found = visibility_states->find(cell);
    if (found == visibility_states->end()) {
      return false;
    }
    // A matched depth image may still have holes.  Only FREE is a measured
    // negative observation; every other state is intentionally non-destructive.
    return found->second != VoxelVisibilityState::Free;
  };
  const auto isConfirmedFree = [visibility_states](const GridCell &cell) {
    if (!visibility_states) {
      return false;
    }
    const auto found = visibility_states->find(cell);
    return found != visibility_states->end() &&
           found->second == VoxelVisibilityState::Free;
  };
  const auto holdsByLocalStructure = [local_structure_states](const History &history) {
    if (!local_structure_states) {
      return false;
    }
    for (const auto &observation : history.last_voxel.node_observations) {
      const auto found = local_structure_states->find(observation.identity);
      if (found == local_structure_states->end() ||
        found->second == NodeLocalStructureState::Unknown)
      {
        continue;
      }
      // Only an agreed local translation deletes the old cell. Static and
      // ambiguous neighbourhoods are deliberately non-destructive.
      if (found->second != NodeLocalStructureState::Moving) {
        return true;
      }
    }
    return false;
  };
  const auto localMotionScore = [local_structure_states](const LabeledGridVoxel &voxel) {
    if (!local_structure_states || voxel.node_observations.empty()) {
      return 0.0;
    }
    std::size_t observed_count = 0U;
    std::size_t moving_count = 0U;
    for (const auto &observation : voxel.node_observations) {
      const auto found = local_structure_states->find(observation.identity);
      if (found == local_structure_states->end() ||
        found->second == NodeLocalStructureState::Unknown)
      {
        continue;
      }
      ++observed_count;
      if (found->second == NodeLocalStructureState::Moving) {
        ++moving_count;
      }
    }
    return observed_count == 0U ? 0.0 :
      static_cast<double>(moving_count) / static_cast<double>(observed_count);
  };

  std::vector<LabeledGridVoxel> stable_voxels;
  stable_voxels.reserve(label_voxels.size());
  migrateHistoriesByNodeIdentity(label_voxels);
  std::unordered_set<GridCell, GridCellHash> updated_cells;
  updated_cells.reserve(label_voxels.size());

  // A point count is meaningful only relative to the surrounding sample
  // density.  This avoids a global "N points" rule becoming stricter when the
  // GNG/node density or the grid resolution changes.
  GridPointCounts current_input_counts;
  current_input_counts.reserve(label_voxels.size());
  for (const auto &voxel : label_voxels) {
    auto [it, inserted] = current_input_counts.emplace(
      voxel.cell, voxel.input_point_count);
    if (!inserted) {
      it->second = std::max(it->second, voxel.input_point_count);
    }
  }
  std::unordered_map<GridCell, double, GridCellHash> local_density_reference;
  local_density_reference.reserve(label_voxels.size());
  for (const auto &voxel : label_voxels) {
    // A 3x3x3 neighborhood has a fixed upper bound.  Keeping its values on
    // the stack avoids one heap allocation for every labeled voxel while
    // retaining the exact median-based density normalization.
    std::array<std::size_t, 27> neighborhood_counts{};
    std::size_t neighborhood_count = 0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const GridCell neighbor{voxel.cell.x + dx, voxel.cell.y + dy, voxel.cell.z + dz};
          const auto count_it = current_input_counts.find(neighbor);
          if (count_it != current_input_counts.end() && count_it->second > 0) {
            neighborhood_counts[neighborhood_count++] = count_it->second;
          }
        }
      }
    }
    if (neighborhood_count == 0) {
      local_density_reference.emplace(
        voxel.cell, static_cast<double>(std::max<std::size_t>(1, voxel.input_point_count)));
      continue;
    }
    std::sort(
      neighborhood_counts.begin(),
      neighborhood_counts.begin() + static_cast<std::ptrdiff_t>(neighborhood_count));
    const std::size_t middle = neighborhood_count / 2;
    const double median = neighborhood_count % 2 == 0
      ? 0.5 * static_cast<double>(
      neighborhood_counts[middle - 1] + neighborhood_counts[middle])
      : static_cast<double>(neighborhood_counts[middle]);
    local_density_reference.emplace(voxel.cell, median);
  }

  // A current component event gets one persistent identity across all of its
  // cells.  Later terrain relabeling may keep that component alive, but a
  // surviving cell may not detach and become a new one-cell object.
  std::unordered_map<std::uint64_t, std::uint64_t> history_id_by_event_id;
  history_id_by_event_id.reserve(label_voxels.size());
  for (const auto &voxel : label_voxels) {
    if (!voxel.unknown_component_event || voxel.unknown_component_event_id == 0U) {
      continue;
    }
    const auto history_it = history_.find(voxel.cell);
    if (history_it == history_.end() ||
      history_it->second.unknown_component_history_id == 0U)
    {
      continue;
    }
    history_id_by_event_id.try_emplace(
      voxel.unknown_component_event_id,
      history_it->second.unknown_component_history_id);
  }
  for (const auto &voxel : label_voxels) {
    if (!voxel.unknown_component_event || voxel.unknown_component_event_id == 0U) {
      continue;
    }
    history_id_by_event_id.try_emplace(
      voxel.unknown_component_event_id, next_unknown_component_history_id_++);
  }

  for (const auto &voxel : label_voxels) {
    const bool unknown_requires_component_event =
      voxel.label == ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT &&
      voxel.unknown_component_evaluated;
    // SAFE_TERRAIN is a geometric observation, not a grasp-label observation.
    // An extended UNKNOWN_OBJECT component can override a terrain-dominant
    // cell: the event says that at least one unknown node occupied this cell
    // simultaneously with the rest of the component.
    const bool records_candidate_label = voxel.unknown_component_event ||
      (voxel.label != ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN &&
      !unknown_requires_component_event);
    const std::uint8_t candidate_label = voxel.unknown_component_event ?
      ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT : voxel.label;
    auto history_it = history_.find(voxel.cell);
    // Do not allocate temporal history for the whole floor. Terrain-only
    // cells are still kept in the current GNG graph and local point-density
    // calculation, but matter temporally only after an object event has
    // created candidate history in the same cell.
    if (history_it == history_.end() && !records_candidate_label) {
      continue;
    }
    if (history_it == history_.end()) {
      history_it = history_.emplace(voxel.cell, History{}).first;
    }
    auto &history = history_it->second;
    if (voxel.unknown_component_event && voxel.unknown_component_event_id != 0U) {
      history.unknown_component_history_id =
        history_id_by_event_id.at(voxel.unknown_component_event_id);
    }
    const bool has_input_points = !require_input_points ||
      voxel.input_point_count >= minimum_input_points_per_voxel;
    updated_cells.insert(voxel.cell);
    const double local_reference = local_density_reference.at(voxel.cell);
    const double point_support_score = !require_input_points ? 1.0 :
      (has_input_points ? std::sqrt(std::min(
      1.0, static_cast<double>(voxel.input_point_count) / local_reference)) : 0.0);
    const double normal_drift_score = std::clamp(voxel.normal_drift_score, 0.0, 1.0);
    const double local_motion_score = std::clamp(localMotionScore(voxel), 0.0, 1.0);
    // Tiny normal-direction shifts are ubiquitous in a continuously adapting
    // GNG.  Do not turn them into a permanent point-support penalty.  Only a
    // displacement of at least half the local GNG spacing is evidence of a
    // potential moving surface.
    const double normal_drift_evidence =
      normal_drift_score >= kNormalDriftEvidenceThreshold ? normal_drift_score : 0.0;
    const double supported_observation_score =
      point_support_score * (1.0 - normal_drift_evidence) * (1.0 - local_motion_score);
    // Do not let an isolated one-cell node accumulate confidence forever.
    // `neighbor_count` covers all current label cells (including UNKNOWN),
    // while the edge flag protects a real GNG surface that happens to span a
    // quantization gap at a fine grid resolution.  A graspable object needs
    // spatial support, so this only removes the unanchored remainder.
    const bool isolated_temporal_decay =
      voxel.neighbor_count == 0U && !voxel.has_cross_cell_gng_edge;
    const double temporal_observation_score = isolated_temporal_decay
      ? 0.0 : supported_observation_score;
    // Raw point support is admission evidence for a new voxel.  It is not, by
    // itself, deletion evidence for an already active GNG surface: a current
    // node in this cell still anchors its incident GNG edges.  Otherwise a
    // point-cloud dropout removes the endpoints before edge inference gets a
    // chance to restore its intermediate cells.
    //
    // A matched depth image proving FREE space or a large normal-direction
    // GNG displacement are explicit negative evidence and deliberately do
    // not receive this hold.
    const bool retained_by_gng_structure = history.active &&
      voxel.node_count > 0 &&
      !has_input_points &&
      normal_drift_evidence == 0.0 &&
      local_motion_score == 0.0 &&
      !isConfirmedFree(voxel.cell);
    const bool preserve_missing_support = !isolated_temporal_decay &&
      temporal_observation_score == 0.0 &&
      point_support_score == 0.0 &&
      (holdsHistory(voxel.cell) || retained_by_gng_structure);
    if (!preserve_missing_support) {
      appendSample(history, HistorySample{
        candidate_label, records_candidate_label, has_input_points});
      history.last_voxel = voxel;
      updateTemporalScores(history, temporal_observation_score);
    }
    if (!history.active) {
      if (history.stability_score < config_.activation_score) {
        continue;
      }
      history.active = true;
    } else if (history.stability_score < config_.retention_score) {
      history.active = false;
      continue;
    }
    if (history.label_observation_count == 0) {
      continue;
    }
    history.active_label = dominantLabel(history.label_counts);

    auto stable_voxel = voxel;
    stable_voxel.label = history.active_label;
    stable_voxel.unknown_component_history_id = history.unknown_component_history_id;
    stable_voxel.history_sample_count = history.samples.size();
    stable_voxel.label_history_count = history.label_observation_count;
    stable_voxel.point_input_history_count = history.point_input_observation_count;
    stable_voxel.temporal_stability_score = history.stability_score;
    stable_voxel.point_support_score = point_support_score;
    stable_voxel.normal_drift_score = normal_drift_score;
    stable_voxel.local_motion_score = local_motion_score;
    stable_voxel.retained_by_gng_structure = retained_by_gng_structure;
    stable_voxel.retained_by_local_structure = false;
    stable_voxel.retained_by_node_identity = false;
    stable_voxel.isolated_temporal_decay_applied = isolated_temporal_decay;
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
    auto &history = it->second;
    const bool retained_by_local_structure = !isConfirmedFree(it->first) &&
      holdsByLocalStructure(history);
    if (holdsHistory(it->first) || retained_by_local_structure) {
      if (!history.active) {
        continue;
      }
      auto stable_voxel = history.last_voxel;
      stable_voxel.node_count = 0;
      stable_voxel.input_point_count = input_point_count;
      stable_voxel.history_sample_count = history.samples.size();
      stable_voxel.label_history_count = history.label_observation_count;
      stable_voxel.point_input_history_count = history.point_input_observation_count;
      stable_voxel.temporal_stability_score = history.stability_score;
      stable_voxel.point_support_score = 0.0;
      stable_voxel.local_motion_score = 0.0;
      stable_voxel.unknown_component_history_id = history.unknown_component_history_id;
      stable_voxel.retained_by_gng_structure = false;
      stable_voxel.retained_by_local_structure = retained_by_local_structure;
      stable_voxel.retained_by_node_identity = false;
      history.last_voxel = stable_voxel;
      stable_voxels.push_back(stable_voxel);
      continue;
    }
    appendSample(history, HistorySample{0, false, has_input_points});
    const bool retained_by_node_identity =
      config_.node_identity_retention_enabled &&
      nodeIdentityRetainsVoxel(
      history.last_voxel, current_nodes, config_.node_identity_max_displacement);
    updateTemporalScores(history, 0.0);
    if (!history.active ||
      (history.stability_score < config_.retention_score && !retained_by_node_identity))
    {
      history.active = false;
      continue;
    }
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
    stable_voxel.temporal_stability_score = history.stability_score;
    stable_voxel.point_support_score = 0.0;
    stable_voxel.local_motion_score = 0.0;
    stable_voxel.unknown_component_history_id = history.unknown_component_history_id;
    stable_voxel.retained_by_gng_structure = false;
    stable_voxel.retained_by_local_structure = false;
    stable_voxel.retained_by_node_identity = retained_by_node_identity;
    history.last_voxel = stable_voxel;
    stable_voxels.push_back(stable_voxel);
  }

  // The birth condition is a four-cell UNKNOWN component.  Enforce the same
  // condition while retaining it: each component may be occluded or relabeled
  // as terrain, but a fragment of fewer than four active cells is discarded.
  constexpr std::size_t kMinimumUnknownComponentCells = 4U;
  std::unordered_map<std::uint64_t, std::size_t> active_cells_by_history_id;
  active_cells_by_history_id.reserve(history_.size());
  for (const auto &[cell, history] : history_) {
    static_cast<void>(cell);
    if (history.active &&
      history.active_label == ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT &&
      history.unknown_component_history_id != 0U)
    {
      ++active_cells_by_history_id[history.unknown_component_history_id];
    }
  }
  for (auto &[cell, history] : history_) {
    static_cast<void>(cell);
    if (!history.active ||
      history.active_label != ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT ||
      history.unknown_component_history_id == 0U)
    {
      continue;
    }
    if (active_cells_by_history_id[history.unknown_component_history_id] <
      kMinimumUnknownComponentCells)
    {
      history.active = false;
    }
  }
  stable_voxels.erase(
    std::remove_if(
      stable_voxels.begin(), stable_voxels.end(),
      [&active_cells_by_history_id](LabeledGridVoxel &voxel) {
        if (voxel.label != ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT ||
          voxel.unknown_component_history_id == 0U)
        {
          return false;
        }
        const auto found = active_cells_by_history_id.find(voxel.unknown_component_history_id);
        voxel.unknown_component_active_cell_count = found == active_cells_by_history_id.end()
          ? 0U : found->second;
        return voxel.unknown_component_active_cell_count < kMinimumUnknownComponentCells;
      }),
    stable_voxels.end());

  for (auto it = history_.begin(); it != history_.end();) {
    if (it->second.label_observation_count == 0) {
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
  next_unknown_component_history_id_ = 1;
}

std::size_t TemporalVoxelFilter::trackedVoxelCount() const noexcept
{
  return history_.size();
}

std::vector<GridCell> TemporalVoxelFilter::trackedVoxels() const
{
  std::vector<GridCell> cells;
  cells.reserve(history_.size());
  for (const auto &[cell, history] : history_) {
    (void)history;
    cells.push_back(cell);
  }
  return cells;
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

GridPointCounts aggregatePointCounts(
  const GridPointCounts &source,
  const GridSpec &source_spec,
  const GridSpec &target_spec)
{
  if (source_spec.cell_size <= 0.0 || target_spec.cell_size <= 0.0) {
    throw std::invalid_argument("point-count grid cell sizes must be positive");
  }

  GridPointCounts aggregated;
  aggregated.reserve(source.size());
  for (const auto &[cell, count] : source) {
    const double x = source_spec.origin_x +
      (static_cast<double>(cell.x) + 0.5) * source_spec.cell_size;
    const double y = source_spec.origin_y +
      (static_cast<double>(cell.y) + 0.5) * source_spec.cell_size;
    const double z = source_spec.origin_z +
      (static_cast<double>(cell.z) + 0.5) * source_spec.cell_size;
    aggregated[positionToGridCell(x, y, z, target_spec)] += count;
  }
  return aggregated;
}

PointActivityScheduler::PointActivityScheduler(PointActivitySchedulerConfig config)
: config_(config)
{
  if (config_.ema_alpha <= 0.0 || config_.ema_alpha > 1.0 ||
    config_.top_fraction <= 0.0 || config_.top_fraction > 1.0 ||
    config_.occupancy_change_weight < 0.0 || config_.occupancy_change_weight > 1.0 ||
    config_.minimum_update_interval == 0 ||
    config_.maximum_update_interval < config_.minimum_update_interval)
  {
    throw std::invalid_argument("invalid point-activity scheduler configuration");
  }
}

PointActivityDecision PointActivityScheduler::update(const GridPointCounts &point_counts)
{
  PointActivityDecision decision;
  ++observed_update_count_;
  if (!config_.enabled) {
    decision.tracked_cell_count = point_counts.size();
    return decision;
  }

  if (!initialized_) {
    statistics_.reserve(point_counts.size());
    for (const auto &[cell, count] : point_counts) {
      statistics_.emplace(
        cell, CellStatistics{1.0, static_cast<double>(count), 0});
    }
    initialized_ = true;
    decision.tracked_cell_count = statistics_.size();
    decision.mean_hit_frequency = statistics_.empty() ? 0.0 : 1.0;
    return decision;
  }

  ++updates_since_process_;
  std::vector<double> changes;
  changes.reserve(statistics_.size() + point_counts.size());
  for (auto iterator = statistics_.begin(); iterator != statistics_.end();) {
    const auto current = point_counts.find(iterator->first);
    const bool occupied = current != point_counts.end();
    const double count = occupied ? static_cast<double>(current->second) : 0.0;
    auto &stats = iterator->second;
    const double occupancy_change = std::abs((occupied ? 1.0 : 0.0) - stats.hit_frequency);
    const double density_denominator = std::max({1.0, count, stats.point_density});
    const double density_change = std::abs(count - stats.point_density) / density_denominator;
    changes.push_back(
      config_.occupancy_change_weight * occupancy_change +
      (1.0 - config_.occupancy_change_weight) * density_change);
    stats.hit_frequency += config_.ema_alpha *
      ((occupied ? 1.0 : 0.0) - stats.hit_frequency);
    stats.point_density += config_.ema_alpha * (count - stats.point_density);
    stats.consecutive_misses = occupied ? 0 : stats.consecutive_misses + 1;
    if (!occupied && stats.consecutive_misses > config_.maximum_update_interval * 4 &&
      stats.hit_frequency < 0.01 && stats.point_density < 0.01)
    {
      iterator = statistics_.erase(iterator);
    } else {
      ++iterator;
    }
  }
  for (const auto &[cell, count] : point_counts) {
    if (statistics_.find(cell) != statistics_.end()) {
      continue;
    }
    statistics_.emplace(cell, CellStatistics{config_.ema_alpha,
      config_.ema_alpha * static_cast<double>(count), 0});
    changes.push_back(1.0);
  }

  if (!changes.empty()) {
    const std::size_t top_count = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(config_.top_fraction * changes.size())));
    const auto top_begin = changes.end() - static_cast<std::ptrdiff_t>(top_count);
    std::nth_element(changes.begin(), top_begin, changes.end());
    double sum = 0.0;
    for (auto iterator = top_begin; iterator != changes.end(); ++iterator) {
      sum += *iterator;
    }
    decision.activity_score = std::clamp(sum / static_cast<double>(top_count), 0.0, 1.0);
  } else {
    decision.activity_score = 0.0;
  }

  double hit_frequency_sum = 0.0;
  for (const auto &[cell, stats] : statistics_) {
    (void)cell;
    hit_frequency_sum += stats.hit_frequency;
  }
  decision.mean_hit_frequency = statistics_.empty() ? 0.0 :
    hit_frequency_sum / static_cast<double>(statistics_.size());
  const double interval = static_cast<double>(config_.maximum_update_interval) -
    decision.activity_score * static_cast<double>(
    config_.maximum_update_interval - config_.minimum_update_interval);
  decision.desired_update_interval = std::clamp<std::size_t>(
    static_cast<std::size_t>(std::llround(interval)),
    config_.minimum_update_interval, config_.maximum_update_interval);
  decision.should_process = updates_since_process_ >= decision.desired_update_interval;
  if (observed_update_count_ <= config_.warmup_update_count) {
    decision.should_process = true;
    decision.desired_update_interval = 1;
  }
  if (decision.should_process) {
    updates_since_process_ = 0;
  }
  decision.updates_since_process = updates_since_process_;
  decision.tracked_cell_count = statistics_.size();
  return decision;
}

void PointActivityScheduler::clear()
{
  statistics_.clear();
  updates_since_process_ = 0;
  observed_update_count_ = 0;
  initialized_ = false;
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

namespace
{

double medianValue(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t middle = values.size() / 2U;
  std::nth_element(
    values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
  const double upper = values[middle];
  if ((values.size() % 2U) != 0U) {
    return upper;
  }
  const auto lower = std::max_element(
    values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
  return 0.5 * (*lower + upper);
}

struct ShapeFilterResult
{
  std::vector<bool> retained;
  std::size_t candidate_count = 0;
  std::size_t seed_count = 0;
  std::size_t retained_count = 0;
  double score_median = 0.0;
  double score_mad = 0.0;
  double score_threshold = 0.0;
};

ShapeFilterResult computeUnknownShapeFilter(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const fuzzrobo::topological_grid::VoxelizationOptions &options)
{
  constexpr double kEpsilon = 1.0e-9;
  const std::size_t node_count = map.nodes.size();
  ShapeFilterResult result;
  result.retained.assign(node_count, true);
  if (!options.unknown_shape_filter_enabled || node_count == 0) {
    return result;
  }

  std::vector<std::vector<std::size_t>> adjacency(node_count);
  std::vector<double> all_edge_lengths;
  all_edge_lengths.reserve(map.edges.size() / 2U);
  for (std::size_t edge = 0; edge + 1U < map.edges.size(); edge += 2U) {
    const std::size_t first = map.edges[edge];
    const std::size_t second = map.edges[edge + 1U];
    if (first >= node_count || second >= node_count || first == second) {
      continue;
    }
    adjacency[first].push_back(second);
    adjacency[second].push_back(first);
    all_edge_lengths.push_back(std::sqrt(squaredNorm(
      nodePosition(map.nodes[first]) - nodePosition(map.nodes[second]))));
  }
  const double global_spacing = std::max(medianValue(all_edge_lengths), kEpsilon);
  std::vector<double> local_spacing(node_count, global_spacing);
  for (std::size_t index = 0; index < node_count; ++index) {
    std::vector<double> lengths;
    lengths.reserve(adjacency[index].size());
    for (const auto neighbor : adjacency[index]) {
      lengths.push_back(std::sqrt(squaredNorm(
        nodePosition(map.nodes[index]) - nodePosition(map.nodes[neighbor]))));
    }
    if (!lengths.empty()) {
      local_spacing[index] = std::max(medianValue(std::move(lengths)), kEpsilon);
    }
  }

  std::vector<double> scores(node_count, 0.0);
  std::vector<bool> valid_score(node_count, false);
  std::vector<double> candidate_scores;
  for (std::size_t index = 0; index < node_count; ++index) {
    if (map.nodes[index].label != ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT) {
      continue;
    }
    ++result.candidate_count;
    result.retained[index] = false;
    std::vector<std::size_t> neighborhood;
    std::vector<std::size_t> frontier{index};
    std::vector<bool> visited(node_count, false);
    visited[index] = true;
    double best_score = 0.0;
    for (int hop = 1; hop <= options.shape_neighborhood_hops; ++hop) {
      std::vector<std::size_t> next_frontier;
      for (const auto current : frontier) {
        for (const auto neighbor : adjacency[current]) {
          if (!visited[neighbor]) {
            visited[neighbor] = true;
            neighborhood.push_back(neighbor);
            next_frontier.push_back(neighbor);
          }
        }
      }
      frontier = std::move(next_frontier);
      if (neighborhood.size() < options.shape_minimum_neighbors) {
        continue;
      }

      Vec3d centroid{};
      Vec3d mean_normal{};
      const Vec3d center_normal = nodeNormal(map.nodes[index]);
      const double center_normal_norm = std::sqrt(squaredNorm(center_normal));
      std::vector<double> normal_variations;
      std::size_t valid_normal_count = 0;
      for (const auto neighbor : neighborhood) {
        const Vec3d position = nodePosition(map.nodes[neighbor]);
        centroid.x += position.x;
        centroid.y += position.y;
        centroid.z += position.z;
        Vec3d normal = nodeNormal(map.nodes[neighbor]);
        const double normal_norm = std::sqrt(squaredNorm(normal));
        if (normal_norm <= kEpsilon) {
          continue;
        }
        normal.x /= normal_norm;
        normal.y /= normal_norm;
        normal.z /= normal_norm;
        if (center_normal_norm > kEpsilon && dot(normal, center_normal) < 0.0) {
          normal.x = -normal.x;
          normal.y = -normal.y;
          normal.z = -normal.z;
        }
        mean_normal.x += normal.x;
        mean_normal.y += normal.y;
        mean_normal.z += normal.z;
        ++valid_normal_count;
        if (center_normal_norm > kEpsilon) {
          normal_variations.push_back(1.0 - std::abs(dot(normal, center_normal) /
            center_normal_norm));
        }
      }
      const double inverse_count = 1.0 / static_cast<double>(neighborhood.size());
      centroid.x *= inverse_count;
      centroid.y *= inverse_count;
      centroid.z *= inverse_count;
      const double mean_normal_norm = std::sqrt(squaredNorm(mean_normal));
      if (valid_normal_count < options.shape_minimum_neighbors || mean_normal_norm <= kEpsilon) {
        continue;
      }
      mean_normal.x /= mean_normal_norm;
      mean_normal.y /= mean_normal_norm;
      mean_normal.z /= mean_normal_norm;
      const double normalized_residual = std::abs(dot(
        mean_normal, nodePosition(map.nodes[index]) - centroid)) / local_spacing[index];
      const double normal_variation = medianValue(std::move(normal_variations));
      const double score = options.shape_residual_weight * normalized_residual +
        (1.0 - options.shape_residual_weight) * normal_variation;
      best_score = std::max(best_score, score);
      valid_score[index] = true;
    }
    if (valid_score[index]) {
      scores[index] = best_score;
      candidate_scores.push_back(best_score);
    }
  }

  if (candidate_scores.empty()) {
    for (std::size_t index = 0; index < node_count; ++index) {
      if (map.nodes[index].label == ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT) {
        result.retained[index] = true;
        ++result.retained_count;
      }
    }
    return result;
  }
  result.score_median = medianValue(candidate_scores);
  std::vector<double> deviations;
  deviations.reserve(candidate_scores.size());
  for (const double score : candidate_scores) {
    deviations.push_back(std::abs(score - result.score_median));
  }
  result.score_mad = medianValue(std::move(deviations));
  if (result.score_mad > kEpsilon) {
    result.score_threshold = result.score_median +
      options.shape_mad_multiplier * 1.4826 * result.score_mad;
  } else {
    double low_center = *std::min_element(candidate_scores.begin(), candidate_scores.end());
    double high_center = *std::max_element(candidate_scores.begin(), candidate_scores.end());
    for (int iteration = 0; iteration < 16 && high_center - low_center > kEpsilon; ++iteration) {
      const double boundary = 0.5 * (low_center + high_center);
      double low_sum = 0.0;
      double high_sum = 0.0;
      std::size_t low_count = 0;
      std::size_t high_count = 0;
      for (const double score : candidate_scores) {
        if (score <= boundary) {
          low_sum += score;
          ++low_count;
        } else {
          high_sum += score;
          ++high_count;
        }
      }
      if (low_count > 0) {
        low_center = low_sum / static_cast<double>(low_count);
      }
      if (high_count > 0) {
        high_center = high_sum / static_cast<double>(high_count);
      }
    }
    result.score_threshold = 0.5 * (low_center + high_center);
  }

  using QueueEntry = std::pair<double, std::size_t>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
  std::vector<double> graph_distance(node_count, std::numeric_limits<double>::infinity());
  for (std::size_t index = 0; index < node_count; ++index) {
    if (map.nodes[index].label == ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT &&
      valid_score[index] && scores[index] > result.score_threshold)
    {
      graph_distance[index] = 0.0;
      queue.emplace(0.0, index);
      ++result.seed_count;
    }
  }
  while (!queue.empty()) {
    const auto [distance, index] = queue.top();
    queue.pop();
    if (distance != graph_distance[index] || distance > options.shape_seed_expansion_scale) {
      continue;
    }
    result.retained[index] = true;
    for (const auto neighbor : adjacency[index]) {
      if (map.nodes[neighbor].label != ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT) {
        continue;
      }
      const double edge_length = std::sqrt(squaredNorm(
        nodePosition(map.nodes[index]) - nodePosition(map.nodes[neighbor])));
      const double scale = std::max(
        0.5 * (local_spacing[index] + local_spacing[neighbor]), kEpsilon);
      const double next_distance = distance + edge_length / scale;
      if (next_distance <= options.shape_seed_expansion_scale &&
        next_distance < graph_distance[neighbor])
      {
        graph_distance[neighbor] = next_distance;
        queue.emplace(next_distance, neighbor);
      }
    }
  }
  for (std::size_t index = 0; index < node_count; ++index) {
    if (map.nodes[index].label == ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT &&
      result.retained[index])
    {
      ++result.retained_count;
    }
  }
  return result;
}

std::size_t countPointSupportWithinRadius(
  const GridCell &cell,
  const GridPointCounts &point_counts,
  const GridSpec &spec,
  double radius_m,
  const SparsePointSupportIndex *support_index = nullptr)
{
  if (point_counts.empty()) {
    return 0;
  }
  if (radius_m <= 0.0) {
    const auto point_it = point_counts.find(cell);
    return point_it == point_counts.end() ? 0U : point_it->second;
  }

  const int radius_cells = std::max(
    1, static_cast<int>(std::ceil(radius_m / spec.cell_size)));
  if (support_index && support_index->enabled()) {
    return support_index->countCube(cell, radius_cells);
  }
  const std::size_t width = static_cast<std::size_t>(2 * radius_cells + 1);
  const std::size_t volume = width * width * width;
  std::size_t support_count = 0;
  if (volume <= point_counts.size()) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
          const auto point_it = point_counts.find(
            GridCell{cell.x + dx, cell.y + dy, cell.z + dz});
          if (point_it != point_counts.end()) {
            support_count += point_it->second;
          }
        }
      }
    }
    return support_count;
  }

  for (const auto &[point_cell, count] : point_counts) {
    if (std::abs(point_cell.x - cell.x) <= radius_cells &&
      std::abs(point_cell.y - cell.y) <= radius_cells &&
      std::abs(point_cell.z - cell.z) <= radius_cells)
    {
      support_count += count;
    }
  }
  return support_count;
}

}  // namespace

GridVoxelizationResult voxelizeNodes(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridSpec &spec,
  const VoxelizationOptions &options,
  const GridPointCounts *input_point_counts)
{
  GridVoxelizationResult result;
  const auto shape_filter = computeUnknownShapeFilter(map, options);
  result.shape_candidate_node_count = shape_filter.candidate_count;
  result.shape_seed_node_count = shape_filter.seed_count;
  result.shape_retained_node_count = shape_filter.retained_count;
  result.shape_rejected_node_count = shape_filter.candidate_count - shape_filter.retained_count;
  result.shape_score_median = shape_filter.score_median;
  result.shape_score_mad = shape_filter.score_mad;
  result.shape_score_threshold = shape_filter.score_threshold;

  // A one-node UNKNOWN_OBJECT label is often a classifier/GNG fluctuation.
  // Treat it as object evidence only when the current GNG graph contains an
  // extended component.  The four-node/four-cell condition represents a
  // minimal occupied volume for the grasp pre-filter, rather than a temporal
  // frame-count threshold.
  constexpr std::size_t kMinimumUnknownComponentNodes = 4U;
  constexpr std::size_t kMinimumUnknownComponentCells = 4U;
  std::vector<bool> retained_nodes(map.nodes.size(), false);
  std::vector<GridCell> node_cells(map.nodes.size());
  for (std::size_t node_index = 0; node_index < map.nodes.size(); ++node_index) {
    const auto &node = map.nodes[node_index];
    if (options.excluded_labels.find(node.label) != options.excluded_labels.end() ||
      !shape_filter.retained[node_index])
    {
      ++result.excluded_node_count;
      continue;
    }
    retained_nodes[node_index] = true;
    node_cells[node_index] = nodeToGridCell(node, spec);
  }

  std::vector<std::vector<std::size_t>> unknown_adjacency(map.nodes.size());
  for (std::size_t edge = 0; edge + 1U < map.edges.size(); edge += 2U) {
    const std::size_t first = static_cast<std::size_t>(map.edges[edge]);
    const std::size_t second = static_cast<std::size_t>(map.edges[edge + 1U]);
    if (first >= map.nodes.size() || second >= map.nodes.size() || first == second ||
      !retained_nodes[first] || !retained_nodes[second] ||
      map.nodes[first].label != ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT ||
      map.nodes[second].label != ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT)
    {
      continue;
    }
    unknown_adjacency[first].push_back(second);
    unknown_adjacency[second].push_back(first);
  }

  std::vector<UnknownNodeComponent> unknown_components;
  std::vector<bool> unknown_visited(map.nodes.size(), false);
  for (std::size_t seed = 0; seed < map.nodes.size(); ++seed) {
    if (!retained_nodes[seed] || unknown_visited[seed] ||
      map.nodes[seed].label != ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT)
    {
      continue;
    }
    UnknownNodeComponent component;
    std::queue<std::size_t> queue;
    queue.push(seed);
    unknown_visited[seed] = true;
    while (!queue.empty()) {
      const std::size_t current = queue.front();
      queue.pop();
      component.node_indices.push_back(current);
      component.cells.insert(node_cells[current]);
      for (const std::size_t neighbor : unknown_adjacency[current]) {
        if (!unknown_visited[neighbor]) {
          unknown_visited[neighbor] = true;
          queue.push(neighbor);
        }
      }
    }
    ++result.unknown_component_count;
    if (component.node_indices.size() >= kMinimumUnknownComponentNodes &&
      component.cells.size() >= kMinimumUnknownComponentCells)
    {
      unknown_components.push_back(std::move(component));
    }
  }

  std::unordered_map<GridCell, VoxelAccumulator, GridCellHash> accumulators;
  accumulators.reserve(map.nodes.size());
  result.eligible_nodes.reserve(map.nodes.size());
  std::unordered_set<NodeIdentity, NodeIdentityHash> ambiguous_node_identities;
  for (std::size_t node_index = 0; node_index < map.nodes.size(); ++node_index) {
    if (!retained_nodes[node_index]) {
      continue;
    }
    const auto &node = map.nodes[node_index];
    const GridCell &cell = node_cells[node_index];
    const NodeObservation observation{
      NodeIdentity{node.id, node.frame}, node.pos.x, node.pos.y, node.pos.z, node.label,
      node.normal.x, node.normal.y, node.normal.z};
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
    accumulator.node_input_id_count += node.inpcl_ids.size();
  }

  SparsePointSupportIndex point_support_index;
  const bool uses_radius_support = input_point_counts &&
    options.point_support_radius_m > 0.0 &&
    (options.point_support_mode == PointSupportMode::Radius ||
    options.point_support_mode == PointSupportMode::Auto);
  if (uses_radius_support) {
    const int radius_cells = std::max(
      1, static_cast<int>(std::ceil(options.point_support_radius_m / spec.cell_size)));
    point_support_index.build(*input_point_counts, radius_cells);
  }

  result.label_voxels.reserve(accumulators.size());
  for (auto &[cell, accumulator] : accumulators) {
    std::size_t geometric_point_count = 0;
    if (input_point_counts && options.point_support_mode != PointSupportMode::NodeInputIds) {
      const bool radius_support = options.point_support_mode == PointSupportMode::Radius ||
        options.point_support_mode == PointSupportMode::Auto;
      geometric_point_count = countPointSupportWithinRadius(
        cell, *input_point_counts, spec,
        radius_support ? options.point_support_radius_m : 0.0,
        point_support_index.enabled() ? &point_support_index : nullptr);
    }
    switch (options.point_support_mode) {
      case PointSupportMode::NodeInputIds:
        accumulator.input_point_count = accumulator.node_input_id_count;
        break;
      case PointSupportMode::Auto:
        // `inpcl_ids` are sometimes absent even while the current point cloud
        // is stable at the node position.  Treat Auto as a per-cell fallback,
        // not a global all-or-nothing choice: either evidence source can keep
        // the cell supported, but max() avoids double-counting the same cloud.
        accumulator.input_point_count = std::max(
          accumulator.node_input_id_count, geometric_point_count);
        break;
      case PointSupportMode::Radius:
      case PointSupportMode::SameCell:
        accumulator.input_point_count = geometric_point_count;
        break;
    }
    LabeledGridVoxel voxel;
    voxel.cell = cell;
    voxel.label = dominantLabel(accumulator);
    voxel.node_count = accumulator.node_count;
    voxel.input_point_count = accumulator.input_point_count;
    voxel.unknown_component_evaluated = true;
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

  // A component is an event only when all of its distinct output cells have
  // point support in this exact frame.  This rejects four graph nodes that
  // happen to be present over a point-cloud hole.
  std::unordered_map<GridCell, LabeledGridVoxel *, GridCellHash> label_by_cell;
  label_by_cell.reserve(result.label_voxels.size());
  for (auto &voxel : result.label_voxels) {
    label_by_cell.emplace(voxel.cell, &voxel);
  }

  // Record only cross-cell GNG connectivity.  Connections within one cell do
  // not provide spatial extent for a grasp candidate, whereas a valid edge
  // over a fine-grid quantization gap must prevent isolated-cell decay.
  for (std::size_t edge = 0; edge + 1U < map.edges.size(); edge += 2U) {
    const std::size_t first = static_cast<std::size_t>(map.edges[edge]);
    const std::size_t second = static_cast<std::size_t>(map.edges[edge + 1U]);
    if (first >= map.nodes.size() || second >= map.nodes.size() || first == second ||
      !retained_nodes[first] || !retained_nodes[second])
    {
      continue;
    }
    const GridCell &first_cell = node_cells[first];
    const GridCell &second_cell = node_cells[second];
    if (first_cell == second_cell) {
      continue;
    }
    const auto first_voxel = label_by_cell.find(first_cell);
    const auto second_voxel = label_by_cell.find(second_cell);
    if (first_voxel != label_by_cell.end()) {
      first_voxel->second->has_cross_cell_gng_edge = true;
    }
    if (second_voxel != label_by_cell.end()) {
      second_voxel->second->has_cross_cell_gng_edge = true;
    }
  }
  std::unordered_set<GridCell, GridCellHash> event_cells;
  std::uint64_t next_unknown_component_event_id = 1U;
  for (const auto &component : unknown_components) {
    const bool every_cell_supported = std::all_of(
      component.cells.begin(), component.cells.end(),
      [&label_by_cell, &options](const GridCell &cell) {
        const auto found = label_by_cell.find(cell);
        return found != label_by_cell.end() &&
               (!options.require_input_points ||
               found->second->input_point_count >= options.minimum_input_points_per_voxel);
      });
    if (!every_cell_supported) {
      continue;
    }
    ++result.unknown_component_event_count;
    result.unknown_component_event_node_count += component.node_indices.size();
    const std::uint64_t event_id = next_unknown_component_event_id++;
    for (const auto &cell : component.cells) {
      auto &voxel = *label_by_cell.at(cell);
      voxel.unknown_component_event = true;
      voxel.unknown_component_event_id = event_id;
      event_cells.insert(cell);
    }
  }
  result.unknown_component_event_voxel_count = event_cells.size();

  std::unordered_set<GridCell, GridCellHash> label_cells;
  label_cells.reserve(result.label_voxels.size());
  for (const auto &voxel : result.label_voxels) {
    label_cells.insert(voxel.cell);
  }

  const int radius = options.neighbor_radius_m > 0.0 ?
    std::max(1, static_cast<int>(std::ceil(options.neighbor_radius_m / spec.cell_size))) :
    std::max(0, options.neighbor_radius_cells);
  const std::size_t neighborhood_width = static_cast<std::size_t>(2 * radius + 1);
  const std::size_t neighborhood_volume =
    neighborhood_width * neighborhood_width * neighborhood_width - 1U;
  const bool use_pairwise_neighbor_search = neighborhood_volume > result.label_voxels.size();
  result.voxels.reserve(result.label_voxels.size());
  for (auto &voxel : result.label_voxels) {
    if (use_pairwise_neighbor_search) {
      for (const auto &candidate : result.label_voxels) {
        if (!(candidate.cell == voxel.cell) &&
          std::abs(candidate.cell.x - voxel.cell.x) <= radius &&
          std::abs(candidate.cell.y - voxel.cell.y) <= radius &&
          std::abs(candidate.cell.z - voxel.cell.z) <= radius)
        {
          ++voxel.neighbor_count;
        }
      }
    } else {
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
  const EdgeInferenceOptions &options,
  const GridPointCounts *input_point_counts)
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

  // The output grid is only a quantization.  Gate a GNG edge by its physical
  // length relative to neighbouring GNG edges, never by the distance between
  // quantized cell centres.  This preserves the same topology when grid_size
  // changes, while still rejecting an isolated graph shortcut.
  const std::size_t edge_slot_count = map.edges.size() / 2U;
  std::vector<double> physical_edge_lengths(
    edge_slot_count, std::numeric_limits<double>::quiet_NaN());
  std::vector<double> incident_length_sum(map.nodes.size(), 0.0);
  std::vector<std::size_t> incident_edge_count(map.nodes.size(), 0U);
  std::vector<double> all_edge_lengths;
  all_edge_lengths.reserve(edge_slot_count);
  for (std::size_t edge_index = 0; edge_index + 1U < map.edges.size(); edge_index += 2U) {
    const std::size_t lhs = static_cast<std::size_t>(map.edges[edge_index]);
    const std::size_t rhs = static_cast<std::size_t>(map.edges[edge_index + 1U]);
    if (lhs >= map.nodes.size() || rhs >= map.nodes.size() || lhs == rhs) {
      continue;
    }
    const Vec3d delta = nodePosition(map.nodes[rhs]) - nodePosition(map.nodes[lhs]);
    const double length = std::sqrt(squaredNorm(delta));
    if (!std::isfinite(length) || length <= 1.0e-9) {
      continue;
    }
    const std::size_t slot = edge_index / 2U;
    physical_edge_lengths[slot] = length;
    all_edge_lengths.push_back(length);
    incident_length_sum[lhs] += length;
    incident_length_sum[rhs] += length;
    ++incident_edge_count[lhs];
    ++incident_edge_count[rhs];
  }
  const double global_spacing = std::max(medianValue(all_edge_lengths), 1.0e-9);
  std::vector<double> relative_edge_lengths(
    edge_slot_count, std::numeric_limits<double>::quiet_NaN());
  std::vector<double> relative_samples;
  relative_samples.reserve(all_edge_lengths.size());
  if (options.maximum_edge_length <= 0.0) {
    for (std::size_t edge_index = 0; edge_index + 1U < map.edges.size(); edge_index += 2U) {
      const std::size_t slot = edge_index / 2U;
      const double length = physical_edge_lengths[slot];
      const std::size_t lhs = static_cast<std::size_t>(map.edges[edge_index]);
      const std::size_t rhs = static_cast<std::size_t>(map.edges[edge_index + 1U]);
      if (!std::isfinite(length) || lhs >= map.nodes.size() || rhs >= map.nodes.size()) {
        continue;
      }
      const auto leave_one_out_spacing = [&incident_length_sum, &incident_edge_count,
          global_spacing, length](std::size_t index) {
          return incident_edge_count[index] > 1U ?
                 (incident_length_sum[index] - length) /
                 static_cast<double>(incident_edge_count[index] - 1U) : global_spacing;
        };
      const double local_spacing = std::max(
        0.5 * (leave_one_out_spacing(lhs) + leave_one_out_spacing(rhs)), 1.0e-9);
      const double relative_length = length / local_spacing;
      relative_edge_lengths[slot] = relative_length;
      relative_samples.push_back(relative_length);
    }
  }
  double adaptive_relative_limit = std::numeric_limits<double>::infinity();
  if (!relative_samples.empty()) {
    const double median_relative_length = medianValue(relative_samples);
    std::vector<double> deviations;
    deviations.reserve(relative_samples.size());
    for (const double sample : relative_samples) {
      deviations.push_back(std::abs(sample - median_relative_length));
    }
    adaptive_relative_limit = median_relative_length + 3.0 * medianValue(std::move(deviations));
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

    const std::size_t slot = edge_index / 2U;
    const double physical_length = physical_edge_lengths[slot];
    const bool is_overlength = options.maximum_edge_length > 0.0 ?
      (!std::isfinite(physical_length) || physical_length > options.maximum_edge_length) :
      (!std::isfinite(relative_edge_lengths[slot]) ||
      relative_edge_lengths[slot] > adaptive_relative_limit);
    if (is_overlength) {
      ++result.overlength_edge_count;
      continue;
    }

    ++result.voxel_edge_count;
    result.connected_endpoint_cells.push_back(lhs_cell);
    result.connected_endpoint_cells.push_back(rhs_cell);
    const auto line_cells = rasterizeGridLine(lhs_cell, rhs_cell);
    for (std::size_t line_index = 1U; line_index + 1U < line_cells.size(); ++line_index) {
      const auto &cell = line_cells[line_index];
      if (direct_cells.find(cell) != direct_cells.end()) {
        continue;
      }
      if (options.require_point_support_for_output &&
        (!input_point_counts || input_point_counts->find(cell) == input_point_counts->end()))
      {
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
  const auto grid_cell_less = [](const GridCell &lhs, const GridCell &rhs) {
      if (lhs.x != rhs.x) {
        return lhs.x < rhs.x;
      }
      if (lhs.y != rhs.y) {
        return lhs.y < rhs.y;
      }
      return lhs.z < rhs.z;
    };
  std::sort(
    result.connected_endpoint_cells.begin(), result.connected_endpoint_cells.end(), grid_cell_less);
  result.connected_endpoint_cells.erase(
    std::unique(result.connected_endpoint_cells.begin(), result.connected_endpoint_cells.end()),
    result.connected_endpoint_cells.end());
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
              if (options.require_point_support_for_output &&
                (!input_point_counts ||
                input_point_counts->find(cell) == input_point_counts->end()))
              {
                continue;
              }
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

VoxelIsolationSplit splitVoxelsByGngConnectivity(
  const std::vector<LabeledGridVoxel> &voxels,
  const EdgeVoxelizationResult &edge_result)
{
  VoxelIsolationSplit split;
  split.connected_voxels.reserve(voxels.size());
  split.isolated_voxels.reserve(voxels.size());
  std::unordered_set<GridCell, GridCellHash> connected_cells;
  connected_cells.reserve(edge_result.connected_endpoint_cells.size());
  for (const auto &cell : edge_result.connected_endpoint_cells) {
    connected_cells.insert(cell);
  }
  for (const auto &voxel : voxels) {
    // A direct cell with no current endpoint is normally isolated.  The one
    // exception is a historical object cell retained by a locally static GNG
    // neighbourhood: its former edge endpoints disappeared only temporarily,
    // so hiding it on the isolated topic would undo the temporal hold.
    if (connected_cells.find(voxel.cell) == connected_cells.end() &&
      !voxel.retained_by_local_structure)
    {
      split.isolated_voxels.push_back(voxel);
    } else {
      split.connected_voxels.push_back(voxel);
    }
  }
  return split;
}

}  // namespace fuzzrobo::topological_grid
