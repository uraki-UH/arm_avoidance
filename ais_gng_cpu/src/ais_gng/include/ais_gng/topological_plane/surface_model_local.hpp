#pragma once

#include "ais_gng/topological_plane/surface_model.hpp"

namespace fuzzrobo::surface_model
{
// 検証済みmodel設定からの候補部分グラフ抽出。元配列添字へ復元済みの結果。
result extract_plane_local(const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes, const options &config,
  const std::vector<region> &retained);
}
