#pragma once

#include <ais_gng_msgs/msg/topological_map.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fuzzrobo::topological_grid
{

struct GridSpec
{
  double cell_size = 0.01;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_z = 0.0;
};

enum class PointSupportMode
{
  SameCell,
  Radius,
  NodeInputIds,
  Auto
};

struct VoxelizationOptions
{
  std::unordered_set<std::uint8_t> excluded_labels{
    ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN,
    ais_gng_msgs::msg::TopologicalMap::HUMAN,
    ais_gng_msgs::msg::TopologicalMap::CAR};
  bool require_input_points = true;
  std::size_t minimum_input_points_per_voxel = 1;
  int neighbor_radius_cells = 1;
  double neighbor_radius_m = 0.0;
  double point_support_radius_m = 0.02;
  PointSupportMode point_support_mode = PointSupportMode::SameCell;
  bool unknown_shape_filter_enabled = false;
  int shape_neighborhood_hops = 2;
  std::size_t shape_minimum_neighbors = 3;
  double shape_residual_weight = 0.7;
  double shape_mad_multiplier = 3.0;
  double shape_seed_expansion_scale = 2.0;
};

struct EdgeInferenceOptions
{
  bool enabled = true;
  // A positive value is an explicit metric cap.  Zero selects the robust
  // local-edge-spacing gate, independent of output grid_size.
  double maximum_edge_length = 0.0;
  bool require_point_support_for_output = false;
};

struct TriangleInferenceOptions
{
  bool enabled = true;
  double maximum_edge_length = 0.05;
  double minimum_area = 1.0e-6;
  double minimum_aspect_ratio = 0.05;
  double maximum_normal_angle_degrees = 45.0;
  double minimum_point_support_ratio = 0.0;
  bool require_point_support_for_output = false;
};

struct GridCell
{
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const GridCell &other) const noexcept
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridCellHash
{
  std::size_t operator()(const GridCell &cell) const noexcept;
};

using GridPointCounts = std::unordered_map<GridCell, std::size_t, GridCellHash>;

// Visibility is deliberately a small, conservative interface between the
// depth-image front end and the temporal voxel filter.  It is not an
// occupancy map: only a confirmed clear line of sight is negative evidence.
enum class VoxelVisibilityState : std::uint8_t
{
  Unknown,
  OutOfView,
  Occluded,
  Free
};

using GridVisibilityStates =
  std::unordered_map<GridCell, VoxelVisibilityState, GridCellHash>;

struct PointActivitySchedulerConfig
{
  bool enabled = true;
  double ema_alpha = 0.2;
  double top_fraction = 0.1;
  double occupancy_change_weight = 0.7;
  std::size_t warmup_update_count = 5;
  std::size_t minimum_update_interval = 1;
  std::size_t maximum_update_interval = 10;
};

struct PointActivityDecision
{
  bool should_process = true;
  double activity_score = 1.0;
  double mean_hit_frequency = 0.0;
  std::size_t desired_update_interval = 1;
  std::size_t updates_since_process = 0;
  std::size_t tracked_cell_count = 0;
};

class PointActivityScheduler
{
public:
  explicit PointActivityScheduler(
    PointActivitySchedulerConfig config = PointActivitySchedulerConfig{});

  PointActivityDecision update(const GridPointCounts &point_counts);
  void clear();

private:
  struct CellStatistics
  {
    double hit_frequency = 0.0;
    double point_density = 0.0;
    std::size_t consecutive_misses = 0;
  };

  PointActivitySchedulerConfig config_;
  std::unordered_map<GridCell, CellStatistics, GridCellHash> statistics_;
  std::size_t updates_since_process_ = 0;
  std::size_t observed_update_count_ = 0;
  bool initialized_ = false;
};

struct NodeIdentity
{
  std::uint16_t id = 0;
  std::uint32_t frame = 0;

  bool operator==(const NodeIdentity &other) const noexcept
  {
    return id == other.id && frame == other.frame;
  }
};

struct NodeIdentityHash
{
  std::size_t operator()(const NodeIdentity &identity) const noexcept;
};

struct NodeObservation
{
  NodeIdentity identity;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  std::uint8_t label = 0;
  double normal_x = 0.0;
  double normal_y = 0.0;
  double normal_z = 0.0;
};

using NodeObservationMap =
  std::unordered_map<NodeIdentity, NodeObservation, NodeIdentityHash>;

// Local GNG topology gives conservative evidence for a voxel that was known
// previously but receives no node this update.  It is deliberately distinct
// from a single-node motion score: several former neighbours must agree.
enum class NodeLocalStructureState : std::uint8_t
{
  Unknown,
  Static,
  Moving,
  Ambiguous
};

using NodeLocalStructureStates =
  std::unordered_map<NodeIdentity, NodeLocalStructureState, NodeIdentityHash>;

struct GridAssignment
{
  std::size_t node_index = 0;
  GridCell cell;
};

struct LabeledGridVoxel
{
  LabeledGridVoxel() = default;
  LabeledGridVoxel(
    GridCell cell_value, std::uint8_t label_value,
    std::size_t node_count_value = 0,
    std::size_t input_point_count_value = 0,
    std::size_t neighbor_count_value = 0)
  : cell(cell_value),
    label(label_value),
    node_count(node_count_value),
    input_point_count(input_point_count_value),
    neighbor_count(neighbor_count_value)
  {
  }

  GridCell cell;
  std::uint8_t label = 0;
  std::size_t node_count = 0;
  std::size_t input_point_count = 0;
  std::size_t neighbor_count = 0;
  std::size_t history_sample_count = 0;
  std::size_t label_history_count = 0;
  std::size_t point_input_history_count = 0;
  std::size_t edge_support_count = 0;
  std::size_t triangle_support_count = 0;
  // Short-term confidence used by TemporalVoxelFilter.  These are kept on the
  // voxel solely for diagnostics; the published Voxel message is unchanged.
  double temporal_stability_score = 0.0;
  double point_support_score = 0.0;
  // Normal-direction displacement of its GNG nodes, normalized by local
  // incident-edge spacing.  Zero means no usable motion evidence.
  double normal_drift_score = 0.0;
  std::vector<NodeObservation> node_observations;
  // `UNKNOWN_OBJECT` is allowed to create new grasp-candidate evidence only
  // when it was part of a simultaneous, spatially extended GNG component.
  // Keeping this result on the voxel makes a later SAFE_TERRAIN observation
  // neutral rather than treating it as a point-cloud disappearance.
  bool unknown_component_evaluated = false;
  bool unknown_component_event = false;
  // Event IDs are local to one voxelization update.  TemporalVoxelFilter
  // translates them into a persistent component ID so a qualified UNKNOWN
  // component cannot later survive as an unrelated one-cell remnant.
  std::uint64_t unknown_component_event_id = 0;
  std::uint64_t unknown_component_history_id = 0;
  std::size_t unknown_component_active_cell_count = 0;
  // The cell was already temporally active and remains occupied by a current
  // GNG node, but its raw point support is absent.  This is a hold, not fresh
  // positive evidence: it prevents an edge-supported surface from being
  // broken by a transient point-cloud dropout.
  bool retained_by_gng_structure = false;
  // A node disappeared from this old cell, but its former GNG neighbours are
  // still locally static or inconclusive. This does not create new cells.
  bool retained_by_local_structure = false;
  bool retained_by_node_identity = false;
};

struct GridVoxelizationResult
{
  std::vector<LabeledGridVoxel> label_voxels;
  std::vector<LabeledGridVoxel> voxels;
  std::size_t included_node_count = 0;
  std::size_t excluded_node_count = 0;
  std::size_t unsupported_node_count = 0;
  std::size_t insufficient_point_voxel_count = 0;
  std::size_t isolated_voxel_count = 0;
  std::size_t shape_candidate_node_count = 0;
  std::size_t shape_seed_node_count = 0;
  std::size_t shape_retained_node_count = 0;
  std::size_t shape_rejected_node_count = 0;
  std::size_t unknown_component_count = 0;
  std::size_t unknown_component_event_count = 0;
  std::size_t unknown_component_event_node_count = 0;
  std::size_t unknown_component_event_voxel_count = 0;
  double shape_score_median = 0.0;
  double shape_score_mad = 0.0;
  double shape_score_threshold = 0.0;
  NodeObservationMap eligible_nodes;
};

struct EdgeVoxelizationResult
{
  std::vector<LabeledGridVoxel> voxels;
  std::size_t input_edge_count = 0;
  std::size_t voxel_edge_count = 0;
  std::size_t invalid_edge_count = 0;
  std::size_t excluded_edge_count = 0;
  std::size_t inactive_endpoint_edge_count = 0;
  std::size_t same_voxel_edge_count = 0;
  std::size_t overlength_edge_count = 0;
  std::size_t duplicate_voxel_edge_count = 0;
  // Stable direct cells touched by accepted GNG edges.  This separates graph
  // continuity from 26-neighbour voxel adjacency.
  std::vector<GridCell> connected_endpoint_cells;
};

struct TriangleVoxelizationResult
{
  std::vector<LabeledGridVoxel> voxels;
  std::size_t candidate_triangle_count = 0;
  std::size_t accepted_triangle_count = 0;
  std::size_t inactive_vertex_triangle_count = 0;
  std::size_t excluded_triangle_count = 0;
  std::size_t overlength_triangle_count = 0;
  std::size_t degenerate_triangle_count = 0;
  std::size_t normal_rejected_triangle_count = 0;
  std::size_t point_support_rejected_triangle_count = 0;
};

struct VoxelIsolationSplit
{
  std::vector<LabeledGridVoxel> connected_voxels;
  std::vector<LabeledGridVoxel> isolated_voxels;
};

struct TemporalVoxelFilterConfig
{
  // History bounds the lifetime of a semantic event label. Geometric
  // activation itself is governed by the continuous evidence score below,
  // rather than occurrence-count thresholds.
  std::size_t history_window_size = 32;
  // Per-update EMA alpha is derived from elapsed seconds. This keeps temporal
  // behavior independent of the input point-cloud rate.
  double time_constant_sec = 0.30;
  // Hysteresis prevents a one-frame appearance/disappearance from toggling a
  // grasp-candidate voxel. retention_score must not exceed activation_score.
  double activation_score = 0.65;
  double retention_score = 0.35;
  bool node_identity_retention_enabled = false;
  double node_identity_max_displacement = 0.02;
  bool node_identity_history_migration_enabled = false;
};

class TemporalVoxelFilter
{
public:
  explicit TemporalVoxelFilter(
    TemporalVoxelFilterConfig config = TemporalVoxelFilterConfig{});

  std::vector<LabeledGridVoxel> update(
    const std::vector<LabeledGridVoxel> &label_voxels,
    bool require_input_points = true,
    std::size_t minimum_input_points_per_voxel = 1,
    const GridPointCounts *input_point_counts = nullptr,
    const NodeObservationMap *current_nodes = nullptr,
    double elapsed_seconds = 0.10,
    const GridVisibilityStates *visibility_states = nullptr,
    const NodeLocalStructureStates *local_structure_states = nullptr);
  void clear();
  std::size_t trackedVoxelCount() const noexcept;
  std::vector<GridCell> trackedVoxels() const;

private:
  struct HistorySample
  {
    std::uint8_t label = 0;
    bool has_label = false;
    bool has_input_points = false;
  };

  struct History
  {
    std::deque<HistorySample> samples;
    std::array<std::size_t, 256> label_counts{};
    std::size_t label_observation_count = 0;
    std::size_t point_input_observation_count = 0;
    std::uint8_t active_label = 0;
    LabeledGridVoxel last_voxel;
    double presence_score = 0.0;
    double switching_score = 0.0;
    double previous_observation_score = 0.0;
    bool has_previous_observation = false;
    double stability_score = 0.0;
    bool active = false;
    std::uint64_t unknown_component_history_id = 0;
  };

  void appendSample(History &history, const HistorySample &sample);
  void migrateHistoriesByNodeIdentity(
    const std::vector<LabeledGridVoxel> &label_voxels);

  TemporalVoxelFilterConfig config_;
  std::unordered_map<GridCell, History, GridCellHash> history_;
  std::uint64_t next_unknown_component_history_id_ = 1;
};

std::string gridCellToString(const GridCell &cell);

GridCell positionToGridCell(
  double x, double y, double z,
  const GridSpec &spec);

GridPointCounts aggregatePointCounts(
  const GridPointCounts &source,
  const GridSpec &source_spec,
  const GridSpec &target_spec);

GridCell nodeToGridCell(
  const ais_gng_msgs::msg::TopologicalNode &node,
  const GridSpec &spec);

std::vector<GridAssignment> assignNodesToGrid(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridSpec &spec);

std::unordered_map<std::size_t, GridCell> assignNodeGridMap(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridSpec &spec);

GridVoxelizationResult voxelizeNodes(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridSpec &spec,
  const VoxelizationOptions &options = VoxelizationOptions{},
  const GridPointCounts *input_point_counts = nullptr);

EdgeVoxelizationResult inferVoxelsFromStableVoxelEdges(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridSpec &spec,
  const std::vector<LabeledGridVoxel> &stable_direct_voxels,
  const std::unordered_set<std::uint8_t> &excluded_labels,
  const EdgeInferenceOptions &options = EdgeInferenceOptions{},
  const GridPointCounts *input_point_counts = nullptr);

TriangleVoxelizationResult inferVoxelsFromStableVoxelTriangles(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const GridSpec &spec,
  const std::vector<LabeledGridVoxel> &stable_direct_voxels,
  const std::unordered_set<std::uint8_t> &excluded_labels,
  const TriangleInferenceOptions &options = TriangleInferenceOptions{},
  const GridPointCounts *input_point_counts = nullptr);

VoxelIsolationSplit splitVoxelsByIsolation(
  const std::vector<LabeledGridVoxel> &voxels);

VoxelIsolationSplit splitVoxelsByGngConnectivity(
  const std::vector<LabeledGridVoxel> &voxels,
  const EdgeVoxelizationResult &edge_result);

std::vector<LabeledGridVoxel> mergeDirectAndInferredVoxels(
  const std::vector<LabeledGridVoxel> &direct_voxels,
  const std::vector<LabeledGridVoxel> &inferred_voxels);

}  // namespace fuzzrobo::topological_grid
