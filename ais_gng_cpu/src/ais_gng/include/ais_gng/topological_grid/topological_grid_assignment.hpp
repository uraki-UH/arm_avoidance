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

struct VoxelizationOptions
{
  std::unordered_set<std::uint8_t> excluded_labels{
    ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN,
    ais_gng_msgs::msg::TopologicalMap::HUMAN,
    ais_gng_msgs::msg::TopologicalMap::CAR};
  bool require_input_points = true;
  std::size_t minimum_input_points_per_voxel = 1;
  int neighbor_radius_cells = 1;
};

struct EdgeInferenceOptions
{
  bool enabled = true;
  double maximum_edge_length = 0.10;
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

struct GridAssignment
{
  std::size_t node_index = 0;
  GridCell cell;
};

struct LabeledGridVoxel
{
  GridCell cell;
  std::uint8_t label = 0;
  std::size_t node_count = 0;
  std::size_t input_point_count = 0;
  std::size_t neighbor_count = 0;
  std::size_t history_sample_count = 0;
  std::size_t label_history_count = 0;
  std::size_t point_input_history_count = 0;
  std::size_t edge_support_count = 0;
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
};

struct TemporalVoxelFilterConfig
{
  std::size_t history_window_size = 100;
  std::size_t minimum_label_history_count = 3;
  std::size_t minimum_point_input_history_count = 3;
  std::size_t isolated_minimum_label_history_count = 5;
  std::size_t isolated_minimum_point_input_history_count = 5;
  std::size_t maximum_missing_label_updates = 10;
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
    const GridPointCounts *input_point_counts = nullptr);
  void clear();
  std::size_t trackedVoxelCount() const noexcept;

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
    std::size_t consecutive_missing_label_updates = 0;
    std::uint8_t active_label = 0;
    LabeledGridVoxel last_voxel;
    bool active = false;
  };

  void appendSample(History &history, const HistorySample &sample);

  TemporalVoxelFilterConfig config_;
  std::unordered_map<GridCell, History, GridCellHash> history_;
};

std::string gridCellToString(const GridCell &cell);

GridCell positionToGridCell(
  double x, double y, double z,
  const GridSpec &spec);

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
  const EdgeInferenceOptions &options = EdgeInferenceOptions{});

std::vector<LabeledGridVoxel> mergeDirectAndInferredVoxels(
  const std::vector<LabeledGridVoxel> &direct_voxels,
  const std::vector<LabeledGridVoxel> &inferred_voxels);

}  // namespace fuzzrobo::topological_grid
