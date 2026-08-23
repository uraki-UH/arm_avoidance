#pragma once

#include <ais_gng_msgs/msg/planar_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <cstddef>
#include <memory>

namespace fuzzrobo::topological_plane
{

// 非一直線の三点配置を、平面クラスタを開始できる局所種として使う。種そのものは
// 所属を固定せず、全ての局所平面ノードを直接GNGエッジの法線整合・接平面逸脱で
// 成長／併合する。ラベルは接続条件に使わず、出力時の属性としてだけ保持する。
struct PlaneClusterOptions
{
  double min_cluster_planarity = 0.45;
  double max_normalized_cluster_residual = 0.35;
  std::size_t min_cluster_nodes = 7;
  double min_normal_alignment_cos = 0.5;
  double max_growth_dist_ratio = 0.7;
  double max_seed_plane_dist_ratio = 2.8;
  double min_cluster_edge_per_node = 1.1;
};

struct PlaneClusterStatistics
{
  std::size_t valid_node_count = 0;
  std::size_t locally_planar_node_count = 0;
  std::size_t seed_component_count = 0;
  std::size_t insufficient_seed_component_count = 0;
  std::size_t insufficient_member_component_count = 0;
  std::size_t line_like_component_count = 0;
  std::size_t geometrically_rejected_component_count = 0;
  std::size_t cluster_count = 0;
  std::size_t clustered_node_count = 0;
  std::size_t clustered_default_node_count = 0;
  std::size_t clustered_terrain_node_count = 0;
  std::size_t clustered_wall_node_count = 0;
  std::size_t clustered_unknown_node_count = 0;
  std::size_t clustered_human_node_count = 0;
  std::size_t clustered_car_node_count = 0;
  std::size_t clustered_other_node_count = 0;
};

struct PlaneClusterExtractionResult
{
  ais_gng_msgs::msg::PlanarClusterArray clusters;
  PlaneClusterStatistics statistics;
};

// 既存GNGエッジを使って、三点配置を開始条件とする表面パッチを抽出する。
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

// フレームごとの構造パッチを時系列で保持する。既存メンバーは追跡平面の近くに
// ある限りGNGノードIDで保持するため、一時的なエッジやラベルの変化だけで
// パッチ全体が消えないようにする。
class PersistentPlaneClusterTracker
{
public:
  explicit PersistentPlaneClusterTracker(PlaneClusterOptions options);
  ~PersistentPlaneClusterTracker();

  PersistentPlaneClusterTracker(const PersistentPlaneClusterTracker &) = delete;
  PersistentPlaneClusterTracker &operator=(const PersistentPlaneClusterTracker &) = delete;

  ais_gng_msgs::msg::PlanarClusterArray update(
    const ais_gng_msgs::msg::PlanarClusterArray &frame_clusters,
    const ais_gng_msgs::msg::TopologicalMap &map);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // fuzzrobo::topological_plane 名前空間
