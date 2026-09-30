#pragma once

#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace fuzzrobo::surface_model
{
struct options
{
  std::string method = "model";
  // 平面核を持つ到達成分だけの新規探索。対象外ノードは出力対象外。
  bool enable_plane_local_search = false;
  // 新規探索候補の異なる元平面数。0は枚数による候補除外なし。保持曲面は枚数から独立。
  std::size_t min_candidate_plane_patches = 2;
  double max_link_length = 0.08;
  double max_link_normal_deg = 45.0;
  // smooth_graphのエッジと接平面の角度[deg]。
  double max_link_tangent_deg = 30.0;
  // 観測支持領域の分離・小欠損補完。falseは領域分離導入前の動作。
  bool enable_support_regions = true;
  // 曲面支持領域の小欠損補完距離[m]。0は補完なし。
  double max_support_gap = 0.02;
  // 局所ノード間隔に対する小欠損補完距離の倍率。
  double max_support_spacing_ratio = 2.5;
  // 曲面の核となる元平面の使用率。0は使用率検査なし。
  double min_plane_usage_ratio = 0.5;
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
  std::size_t max_boundary_fits = 32;
  // 曲率特徴の軽量検証付き再利用と、1更新あたりの再推定回数。
  bool enable_patch_history = true;
  std::size_t max_patch_fits = 32;
  // 再推定間隔[更新回数]。0と1は毎更新の再推定。
  std::size_t max_patch_refresh_frames = 5;
};

struct patch_curvature
{
  bool valid = false;
  std::size_t sample_num = 0;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis_u = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis_v = Eigen::Vector3d::Zero();
  // r = -K q。実接平面上の曲率、単位は1/m、符号は代表法線の向きに依存。
  Eigen::Matrix2d tensor = Eigen::Matrix2d::Zero();
  Eigen::Vector2d kappa = Eigen::Vector2d::Zero();
  Eigen::Matrix2d directions_uv = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d support_cov = Eigen::Matrix2d::Zero();
  Eigen::Matrix3d normal_scatter = Eigen::Matrix3d::Zero();
  double plane_rms = 0.0;
  // 正規化PCA座標の高さ式。係数順はuu,uv,vv,u,v,1。
  Eigen::Vector3d height_origin = Eigen::Vector3d::Zero();
  Eigen::Matrix3d height_basis = Eigen::Matrix3d::Identity();
  double height_scale = 1.0;
  Eigen::Matrix<double,6,1> height_coeff = Eigen::Matrix<double,6,1>::Zero();
  double position_rms = 0.0;
  // 重み付き高さ残差から面内広がりで換算した法線変化誤差。無次元。
  double fit_error = 0.0;
  // 位置支持・条件数・高さ残差による品質指標。確率としての解釈は不可。
  double confidence = 0.0;
  std::size_t fit_iter = 0;
  bool has_svd_fallback = false;
};

struct local_patch
{
  int plane_cluster_idx = -1;
  std::vector<std::uint32_t> node_indices;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d position_cov = Eigen::Matrix3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double local_spacing = 0.0;
  double plane_residual_ratio = 0.0;
  bool has_plane_features = false;
  bool has_retained_support = false;
  // 予算待ちの未評価状態。支持不足による推定失敗とは別の状態。
  bool is_curvature_deferred = false;
  double normal_change_hint = 0.0;
  // 小さい値を先行とする評価順。採用条件とは独立。
  std::size_t eval_priority = std::numeric_limits<std::size_t>::max();
  patch_curvature curvature;
};

struct patch_history
{
  using node_key = std::pair<std::uint16_t, std::uint32_t>;
  using patch_key = std::tuple<std::uint32_t, std::uint16_t, std::uint32_t>;
  struct cache_entry
  {
    std::vector<node_key> members;
    patch_curvature curvature;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Matrix3d position_cov = Eigen::Matrix3d::Zero();
    Eigen::Vector2d min_support = Eigen::Vector2d::Zero();
    Eigen::Vector2d max_support = Eigen::Vector2d::Zero();
    std::uint64_t last_eval = 0;
    std::uint64_t first_pending = 0;
    std::uint64_t last_seen = 0;
  };
  std::map<patch_key, cache_entry> entries;
  std::uint64_t clock = 0;
  void clear() { entries.clear(); clock = 0; }
};

patch_curvature estimate_curvature(
  const local_patch &patch, const ais_gng_msgs::msg::TopologicalMap &map);

Eigen::Vector3d patch_normal_at(const patch_curvature &curvature, const Eigen::Vector3d &point);

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
  // 支持領域分割の直前のID。未分割時は未設定値。
  std::uint32_t support_parent_id = std::numeric_limits<std::uint32_t>::max();
};

struct result
{
  // 平面核に基づく対象限定の適用状態と入力・候補ノード数。
  bool has_candidate_filter = false;
  std::size_t num_input_nodes = 0;
  std::size_t num_candidate_nodes = 0;
  std::string method = "model";
  std::vector<local_patch> patches;
  // 元GNGの接続に由来するパッチ間edge。smooth_edgesは法線・距離ゲート通過分。
  std::vector<std::array<std::uint32_t, 2>> patch_edges;
  std::vector<std::array<std::uint32_t, 2>> smooth_edges;
  // 近接する平面パッチ間の法線不連続。迂回接続による同一曲面への再統合も禁止。
  std::vector<std::array<std::uint32_t, 2>> sharp_edges;
  // 接平面の支持不足・品質不足。統合の確定ではなくモデル適合検査の候補。
  std::vector<std::array<std::uint32_t, 2>> uncertain_edges;
  std::vector<region> regions;
  std::size_t model_fits = 0;
  std::size_t num_curvature_fits = 0;
  std::size_t num_curvature_reused = 0;
  std::size_t num_curvature_deferred = 0;
  double update_ms = 0.0;
  double curvature_ms = 0.0;
  double boundary_ms = 0.0;
  std::size_t boundary_fit_num = 0;
  double retention_ms = 0.0;
  double support_ms = 0.0;
  std::size_t support_split_num = 0;
  std::size_t support_gap_links = 0;
  // smooth_graphの採用エッジ（入力配列添字）と差分更新の処理数。
  std::vector<std::uint16_t> connected_edges;
  std::size_t link_check_num = 0;
  std::size_t connectivity_node_num = 0;
};

struct node_dev
{
  double dist = std::numeric_limits<double>::infinity();
  double normal_cos = 0.0;
};

node_dev model_dev(const model &shape, const ais_gng_msgs::msg::TopologicalNode &node);
std::size_t plane_patch_num(const result &surfaces, const region &surface);

// 曲面上の観測支持領域の分離。新規・保持曲面の双方に共通の連続性検査。
void split_support_regions(result &surfaces,
  const ais_gng_msgs::msg::TopologicalMap &map, const options &config,
  bool has_patch_budget = false);

// 平面特徴の受渡しと、支持範囲を検査した曲率再利用・再推定順の管理。
void update_patch_curvatures(result &surfaces,
  const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes,
  const options &config, patch_history *history);

// modelのretainedはtrackerでの検証済み候補。smooth_graphは履歴なしの接続判定。
result extract(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes,
  const options &config = {},
  const std::vector<region> &retained = {},
  patch_history *history = nullptr);
}  // namespace fuzzrobo::surface_model
