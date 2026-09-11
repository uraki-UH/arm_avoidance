#pragma once

#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace fuzzrobo::topological_plane::nonplane
{

struct component
{
  std::uint32_t id = 0U;
  std::vector<std::uint32_t> node_indices;
};

struct extraction_result
{
  std::vector<component> components;
};

// 単独nodeを含む全平面クラスタ未所属nodeの連結成分抽出。表示条件との分離。
extraction_result extract_components(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &plane_clusters);

}  // namespace fuzzrobo::topological_plane::nonplane
