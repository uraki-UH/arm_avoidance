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

struct component
{
  std::uint32_t id = 0U;
  std::vector<std::uint32_t> node_indices;
};

struct extraction_result
{
  std::vector<component> components;
};

// 平面クラスタ未所属nodeの連結成分抽出。
extraction_result extract_components(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &plane_clusters,
  const extractor_options &options);

}  // namespace fuzzrobo::topological_plane::nonplane
