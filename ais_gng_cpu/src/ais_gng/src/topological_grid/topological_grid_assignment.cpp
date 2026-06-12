#include <ais_gng/topological_grid/topological_grid_assignment.hpp>

#include <cmath>
#include <sstream>

namespace fuzzrobo::topological_grid
{

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

}  // namespace fuzzrobo::topological_grid

