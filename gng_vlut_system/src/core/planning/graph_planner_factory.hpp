#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include "planning/graph_planner.hpp"
#include "planning/planners/dijkstra_graph_planner.hpp"

namespace robot_sim::planning {

// 未知の名前は起動失敗。暗黙の別実装への切替なし
template <typename angle_type, typename coord_type, typename graph_type>
std::unique_ptr<graph_planner<graph_type>> make_graph_planner(
    const std::string &name, const graph_type &graph,
    const graph_planner_options &options,
    std::shared_ptr<::planning::ICostEvaluator<angle_type, coord_type>> cost) {
  if (name != "gng_dijkstra" && name != "gng_dijkstra_reference") {
    throw std::invalid_argument("Unknown graph_planner: " + name);
  }
  return std::make_unique<dijkstra_graph_planner<angle_type, coord_type, graph_type>>(
      graph, options, std::move(cost), name == "gng_dijkstra");
}

}  // 名前空間robot_sim::planningの終端
