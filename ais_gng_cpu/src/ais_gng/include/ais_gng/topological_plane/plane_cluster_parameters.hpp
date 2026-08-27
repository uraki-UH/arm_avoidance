#pragma once

#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"

#include <algorithm>
#include <cstdint>
#include <string>

namespace fuzzrobo::topological_plane::incremental
{

template<typename NodeT>
ClusterOptions declareClusterOptions(NodeT &node, const std::string &prefix = "")
{
  ClusterOptions options;
  const auto name = [&prefix](const char *suffix) {return prefix + suffix;};

  options.min_cluster_nodes = static_cast<std::size_t>(std::max<std::int64_t>(
      3, node.template declare_parameter<int>(name("min_cluster_nodes"), 10)));
  options.growth_residual_ratio =
    node.template declare_parameter<double>(name("growth_residual_ratio"), 0.70);
  options.retention_residual_ratio =
    node.template declare_parameter<double>(name("retention_residual_ratio"), 1.40);
  options.max_effective_spacing =
    node.template declare_parameter<double>(name("max_effective_spacing"), 0.02);
  options.normal_filter_alpha =
    node.template declare_parameter<double>(name("normal_filter_alpha"), 0.30);
  options.normal_alignment_deg =
    node.template declare_parameter<double>(name("normal_alignment_deg"), 30.0);
  options.retention_normal_alignment_deg =
    node.template declare_parameter<double>(name("retention_normal_alignment_deg"), 70.0);
  options.min_cluster_planarity =
    node.template declare_parameter<double>(name("min_cluster_planarity"), 0.45);
  options.max_normalized_cluster_residual =
    node.template declare_parameter<double>(name("max_normalized_cluster_residual"), 0.70);
  options.min_growth_planarity =
    node.template declare_parameter<double>(name("min_growth_planarity"), 0.25);
  options.connection_requirement = static_cast<std::size_t>(std::max<std::int64_t>(
      1, node.template declare_parameter<int>(name("connection_requirement"), 2)));
  options.merge_connection_requirement = static_cast<std::size_t>(std::max<std::int64_t>(
      1, node.template declare_parameter<int>(name("merge_connection_requirement"), 2)));
  options.birth_neighbor_requirement = static_cast<std::size_t>(std::max<std::int64_t>(
      1, node.template declare_parameter<int>(name("birth_neighbor_requirement"), 1)));
  options.migration_improvement_margin =
    node.template declare_parameter<double>(name("migration_improvement_margin"), 0.10);
  options.donor_protection_buffer = static_cast<std::size_t>(std::max<std::int64_t>(
      0, node.template declare_parameter<int>(name("donor_protection_buffer"), 5)));
  options.enable_multi_edge_dist_relaxation =
    node.template declare_parameter<bool>(name("enable_multi_edge_dist_relaxation"), false);
  options.maintenance_iter = static_cast<std::size_t>(std::max<std::int64_t>(
      1, node.template declare_parameter<int>(name("maintenance_iter"), 2)));
  options.merge_min_planarity =
    node.template declare_parameter<double>(name("merge_min_planarity"), 0.25);
  options.merge_residual_growth_ratio =
    node.template declare_parameter<double>(name("merge_residual_growth_ratio"), 1.1);
  options.merge_residual_growth_min_th =
    node.template declare_parameter<double>(name("merge_residual_growth_min_th"), 0.1);
  options.birth_confirm_frames = static_cast<std::size_t>(std::max<std::int64_t>(
      0, node.template declare_parameter<int>(name("birth_confirm_frames"), 3)));
  options.split_confirm_frames = static_cast<std::size_t>(std::max<std::int64_t>(
      0, node.template declare_parameter<int>(name("split_confirm_frames"), 3)));
  options.weak_frame_allowance = static_cast<std::size_t>(std::max<std::int64_t>(
      0, node.template declare_parameter<int>(name("weak_frame_allowance"), 5)));
  return options;
}

}  // namespace fuzzrobo::topological_plane::incremental
