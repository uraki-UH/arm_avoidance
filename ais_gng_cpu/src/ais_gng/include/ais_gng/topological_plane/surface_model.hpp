#pragma once

#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace fuzzrobo::surface_model
{
struct options
{
  double max_link_length = 0.08;
  double max_link_normal_deg = 45.0;
  double max_patch_rms = 0.004;
  double max_point_residual = 0.012;
  double max_normal_deg = 35.0;
  double max_curvature_normal_error = 0.03;
  bool protect_dominant_flat_patches = true;
  double complexity_penalty = 0.001;
  double max_radius = 5.0;
  std::size_t min_fit_nodes = 12;
  std::size_t max_fit_samples = 256;
  std::size_t max_model_fits = 128;
};

struct patch_curvature
{
  bool valid = false;
  std::size_t sample_num = 0;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis_u = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis_v = Eigen::Vector3d::Zero();
  // r = -K q。単位は1/m、符号は代表法線の向きに依存。
  Eigen::Matrix2d tensor = Eigen::Matrix2d::Zero();
  Eigen::Vector2d kappa = Eigen::Vector2d::Zero();
  Eigen::Matrix2d directions_uv = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d support_cov = Eigen::Matrix2d::Zero();
  Eigen::Matrix3d normal_scatter = Eigen::Matrix3d::Zero();
  double plane_rms = 0.0;
  double fit_error = 0.0;
  // 条件数と法線予測残差による品質指標。確率としての解釈は不可。
  double confidence = 0.0;
};

struct local_patch
{
  int plane_cluster_idx = -1;
  std::vector<std::uint32_t> node_indices;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  patch_curvature curvature;
};

patch_curvature estimate_curvature(
  const local_patch &patch, const ais_gng_msgs::msg::TopologicalMap &map);

struct model
{
  std::string type = "unknown";
  // qの座標系: u = (p - origin) / scale。係数順: xx,yy,zz,xy,xz,yz,x,y,z,1。
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  double scale = 1.0;
  Eigen::Matrix<double, 10, 1> q = Eigen::Matrix<double, 10, 1>::Zero();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::Zero();
  Eigen::Vector3d major_direction = Eigen::Vector3d::Zero();
  Eigen::Vector2d radii = Eigen::Vector2d::Zero();
  double rms = 0.0;
  double max_patch_rms = 0.0;
  double score = std::numeric_limits<double>::infinity();
};

struct region
{
  std::uint32_t id = 0;
  std::vector<std::uint32_t> patch_indices;
  std::vector<std::uint32_t> node_indices;
  model shape;
  bool is_retained = false;
  std::size_t seed_plane_patch_num = 0;
  std::size_t rejected_node_num = 0;
};

struct result
{
  std::vector<local_patch> patches;
  // 元GNGの接続に由来するパッチ間edge。smooth_edgesは法線・距離ゲート通過分。
  std::vector<std::array<std::uint32_t, 2>> patch_edges;
  std::vector<std::array<std::uint32_t, 2>> smooth_edges;
  // 近接する平面パッチ間の法線不連続。迂回接続による同一曲面への再統合も禁止。
  std::vector<std::array<std::uint32_t, 2>> sharp_edges;
  std::vector<region> regions;
  std::size_t model_fits = 0;
  double update_ms = 0.0;
  double curvature_ms = 0.0;
  double retention_ms = 0.0;
};

struct node_dev
{
  double dist = std::numeric_limits<double>::infinity();
  double normal_cos = 0.0;
};

node_dev model_dev(const model &shape, const ais_gng_msgs::msg::TopologicalNode &node);
std::size_t plane_patch_num(const result &surfaces, const region &surface);

// retainedはtrackerによる現在ノードの距離・法線・支持率検証済みの候補。
result extract(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes,
  const options &config = {},
  const std::vector<region> &retained = {});
}  // namespace fuzzrobo::surface_model
