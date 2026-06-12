#pragma once

#include <ais_gng_msgs/msg/topological_map.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace fuzzrobo::topological_grid
{

struct GridSpec
{
  double cell_size = 0.5;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_z = 0.0;
};

struct GridCell
{
  int x = 0;
  int y = 0;
  int z = 0;
};

struct GridAssignment
{
  std::size_t node_index = 0;
  GridCell cell;
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

}  // namespace fuzzrobo::topological_grid

