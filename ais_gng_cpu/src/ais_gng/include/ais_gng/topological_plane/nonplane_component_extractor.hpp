#pragma once

#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace fuzzrobo::topological_plane::nonplane
{

struct extractor_options
{
  std::size_t min_component_nodes = 2U;
};

struct plane_anchor_edge
{
  std::uint32_t component_id = 0U;
  std::uint32_t source_node_index = 0U;
  std::uint32_t plane_node_index = 0U;
  std::uint32_t plane_cluster_id = 0U;
};

struct extraction_result
{
  ais_gng_msgs::msg::TopologicalMap map;
  std::vector<plane_anchor_edge> plane_anchor_edges;
};

// 平面クラスタ未所属nodeの連結成分抽出。
extraction_result extract_components(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &plane_clusters,
  const extractor_options &options);

}  // namespace fuzzrobo::topological_plane::nonplane
