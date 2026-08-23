#pragma once

#include <ais_gng_msgs/msg/planar_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <cstddef>

namespace fuzzrobo::topological_plane
{

// All geometric gates are ratios of the GNG's local edge spacing.  This is
// important here: a fixed millimetre threshold would make the result depend
// strongly on the current GNG node density and on the downstream voxel size.
struct PlaneClusterOptions
{
  // Local boundary fans are often anisotropic despite lying on one plane, so
  // this seed gate is deliberately looser than the final cluster gate.
  double minimum_node_planarity = 0.25;
  double minimum_cluster_planarity = 0.45;
  double normal_alignment_cosine = 0.90;
  double maximum_normalized_edge_residual = 0.35;
  double maximum_normalized_cluster_residual = 0.35;
  std::size_t minimum_cluster_nodes = 4;
};

struct PlaneClusterStatistics
{
  std::size_t valid_node_count = 0;
  std::size_t locally_planar_node_count = 0;
  std::size_t cluster_count = 0;
  std::size_t clustered_node_count = 0;
};

struct PlaneClusterExtractionResult
{
  ais_gng_msgs::msg::PlanarClusterArray clusters;
  PlaneClusterStatistics statistics;
};

// Finds planes by region-growing only across existing GNG edges.  It does not
// run a global RANSAC over a dense point cloud, so its normal cost is O(V+E)
// plus the small convex-hull work needed solely for diagnostics.
class TopologicalPlaneClusterExtractor
{
public:
  explicit TopologicalPlaneClusterExtractor(
    PlaneClusterOptions options = PlaneClusterOptions{});

  PlaneClusterExtractionResult extract(
    const ais_gng_msgs::msg::TopologicalMap &map) const;

private:
  PlaneClusterOptions options_;
};

}  // namespace fuzzrobo::topological_plane
