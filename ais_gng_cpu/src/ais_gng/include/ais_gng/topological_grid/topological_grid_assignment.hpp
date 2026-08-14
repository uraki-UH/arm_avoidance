#pragma once

#include <ais_gng_msgs/msg/topological_map.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fuzzrobo::topological_grid
{

struct GridSpec
{
  double cell_size = 0.02;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_z = 0.0;
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
};

struct GridVoxelizationResult
{
  std::vector<LabeledGridVoxel> voxels;
  std::size_t included_node_count = 0;
  std::size_t excluded_node_count = 0;
};

class TemporalVoxelFilter
{
public:
  TemporalVoxelFilter(
    std::size_t minimum_observations = 1,
    std::size_t maximum_missed_updates = 0);

  std::vector<LabeledGridVoxel> update(
    const std::vector<LabeledGridVoxel> &observed_voxels);
  void clear();
  std::size_t trackedVoxelCount() const noexcept;

private:
  struct History
  {
    std::uint8_t label = 0;
    std::size_t observations = 0;
    std::size_t last_update = 0;
  };

  std::size_t minimum_observations_ = 1;
  std::size_t maximum_missed_updates_ = 0;
  std::size_t update_count_ = 0;
  std::unordered_map<GridCell, History, GridCellHash> history_;
};

std::string gridCellToString(const GridCell &cell);

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
  const std::unordered_set<std::uint8_t> &excluded_labels);

}  // namespace fuzzrobo::topological_grid
