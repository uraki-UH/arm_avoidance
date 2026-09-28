#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{

constexpr double kEpsilon = 1.0e-9;
constexpr double kHalfPi = 1.5707963267948966192313216916398;

// 度で与えた設定値を、内積比較用のラジアンへ一度だけ変換するための係数。
constexpr double kRadiansPerDeg = 0.017453292519943295769236907684886;

// 未所属を表すクラスタ添字。
constexpr int kUnassigned = -1;

// GNGノードIDの取りうる範囲(uint16_t)。法線EMA状態をハッシュ表ではなく
// この幅のフラット配列で直接インデックスするために使う。
constexpr std::size_t kNodeIdRange = 65536U;

template<typename point_type>
Eigen::Vector3d pointOf(const point_type &point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

geometry_msgs::msg::Point32 pointMessage(const Eigen::Vector3d &point)
{
  geometry_msgs::msg::Point32 message;
  message.x = static_cast<float>(point.x());
  message.y = static_cast<float>(point.y());
  message.z = static_cast<float>(point.z());
  return message;
}

geometry_msgs::msg::Vector3 vectorMessage(const Eigen::Vector3d &vector)
{
  geometry_msgs::msg::Vector3 message;
  message.x = vector.x();
  message.y = vector.y();
  message.z = vector.z();
  return message;
}

// 固有ベクトルの符号は一意でないため、参照方向へそろえる。参照が無い場合は
// 絶対値最大の軸が正になる向きへ固定し、フレーム間で向きがちらつかないようにする。
void orientNormal(Eigen::Vector3d &normal, const Eigen::Vector3d &reference)
{
  if (reference.squaredNorm() > kEpsilon) {
    if (normal.dot(reference) < 0.0) {
      normal = -normal;
    }
    return;
  }
  Eigen::Index dominant_axis = 0;
  normal.cwiseAbs().maxCoeff(&dominant_axis);
  if (normal[dominant_axis] < 0.0) {
    normal = -normal;
  }
}

struct PlaneFit
{
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double planarity = 0.0;
  double plane_width_std = 0.0;
  double residual = 0.0;
  bool is_valid = false;
};

// 点列を保持せず、累積和だけから平面を解く。メンバー配列を作り直さずに済むため、
// フレームごとの一時確保が増えない。
//
// 数値条件を保つため、最初に投入した点を原点とした相対座標で累積する。
struct PlaneAccumulator
{
  Eigen::Vector3d anchor = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d moment = Eigen::Matrix3d::Zero();
  Eigen::Vector3d normal_sum = Eigen::Vector3d::Zero();
  double spacing_sum = 0.0;
  std::size_t count = 0U;
  bool has_anchor = false;

  void clear()
  {
    anchor.setZero();
    sum.setZero();
    moment.setZero();
    normal_sum.setZero();
    spacing_sum = 0.0;
    count = 0U;
    has_anchor = false;
  }

  void add(const Eigen::Vector3d &position, const Eigen::Vector3d &normal, const double spacing)
  {
    if (!has_anchor) {
      anchor = position;
      has_anchor = true;
    }
    const Eigen::Vector3d delta = position - anchor;
    sum += delta;
    moment.noalias() += delta * delta.transpose();

    // 法線は符号が不定なので、累積方向へそろえてから足す。
    Eigen::Vector3d oriented = normal;
    if (normal_sum.squaredNorm() > kEpsilon && oriented.dot(normal_sum) < 0.0) {
      oriented = -oriented;
    }
    normal_sum += oriented;
    spacing_sum += spacing;
    ++count;
  }

  // 旧寄与の除去。法線和は平面法線の符号参照のみのため、差分経路では前回法線を使用。
  void remove(const Eigen::Vector3d &position, const double spacing)
  {
    if (count <= 1U) {
      clear();
      return;
    }
    const Eigen::Vector3d delta = position - anchor;
    sum -= delta;
    moment.noalias() -= delta * delta.transpose();
    spacing_sum -= spacing;
    --count;
  }

  // 別の累積量をこの累積量へ合成する。両者の原点が異なっても、二次モーメントを
  // 現在の原点へ座標変換することで、全ノードを再走査せずに結合平面を求められる。
  void mergeFrom(const PlaneAccumulator &other)
  {
    if (!other.has_anchor || other.count == 0U) {
      return;
    }
    if (!has_anchor || count == 0U) {
      *this = other;
      return;
    }

    const Eigen::Vector3d anchor_delta = other.anchor - anchor;
    const double other_count = static_cast<double>(other.count);
    sum += other.sum + other_count * anchor_delta;
    moment += other.moment;
    moment.noalias() += anchor_delta * other.sum.transpose();
    moment.noalias() += other.sum * anchor_delta.transpose();
    moment.noalias() += other_count * anchor_delta * anchor_delta.transpose();

    Eigen::Vector3d other_normal_sum = other.normal_sum;
    if (normal_sum.squaredNorm() > kEpsilon &&
      other_normal_sum.dot(normal_sum) < 0.0)
    {
      other_normal_sum = -other_normal_sum;
    }
    normal_sum += other_normal_sum;
    spacing_sum += other.spacing_sum;
    count += other.count;
  }

  double meanSpacing() const
  {
    return count > 0U ? spacing_sum / static_cast<double>(count) : 0.0;
  }

  // 累積点群から指定平面へのRMS。全点の再走査・固有値分解なしの距離評価。
  double rms_to_plane(const PlaneFit &fit) const
  {
    if (count == 0U) {
      return std::numeric_limits<double>::infinity();
    }
    const double offset = fit.normal.dot(anchor - fit.centroid);
    const double squared_dist_sum = fit.normal.dot(moment * fit.normal) +
      2.0 * offset * fit.normal.dot(sum);
    return std::sqrt(std::max(0.0,
      squared_dist_sum / static_cast<double>(count) + offset * offset));
  }

  PlaneFit solve() const
  {
    PlaneFit fit;
    if (count < 3U) {
      return fit;
    }
    const double inverse_count = 1.0 / static_cast<double>(count);
    const Eigen::Vector3d mean = sum * inverse_count;
    Eigen::Matrix3d covariance = moment * inverse_count;
    covariance.noalias() -= mean * mean.transpose();

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      return fit;
    }
    const Eigen::Vector3d eigenvalues = solver.eigenvalues();
    const double largest = eigenvalues.z();
    if (!std::isfinite(largest) || largest <= kEpsilon) {
      return fit;
    }

    fit.centroid = anchor + mean;
    fit.covariance = covariance;
    fit.normal = solver.eigenvectors().col(0).normalized();
    // 平面性は「線分状でないこと」を見る指標で、厚みは residual で別に評価する。
    fit.planarity = std::sqrt(std::clamp(eigenvalues.y() / largest, 0.0, 1.0));
    fit.plane_width_std = std::sqrt(std::max(0.0, eigenvalues.y()));
    fit.residual = std::sqrt(std::max(0.0, eigenvalues.x()));
    fit.is_valid = fit.covariance.allFinite() && fit.normal.allFinite() &&
      std::isfinite(fit.planarity) && std::isfinite(fit.residual);
    if (fit.is_valid) {
      orientNormal(fit.normal, normal_sum);
    }
    return fit;
  }
};

struct DisjointSet
{
  std::vector<std::size_t> parent;

  void reset(const std::size_t size)
  {
    parent.resize(size);
    for (std::size_t index = 0U; index < size; ++index) {
      parent[index] = index;
    }
  }

  std::size_t find(std::size_t index)
  {
    while (parent[index] != index) {
      parent[index] = parent[parent[index]];
      index = parent[index];
    }
    return index;
  }

  // どちらの根を残すかは呼び出し側が決める。添字は毎フレームの詰め直しで変わるため、
  // 添字の大小をクラスタの素性の代わりに使ってはならない。
  template<typename Prefer>
  void unite(const std::size_t first, const std::size_t second, Prefer prefer)
  {
    const std::size_t first_root = find(first);
    const std::size_t second_root = find(second);
    if (first_root == second_root) {
      return;
    }
    if (prefer(first_root, second_root)) {
      parent[second_root] = first_root;
    } else {
      parent[first_root] = second_root;
    }
  }
};

std::uint64_t clusterPairKey(const std::uint32_t first, const std::uint32_t second)
{
  const std::uint32_t low = static_cast<std::uint32_t>(std::min(first, second));
  const std::uint32_t high = static_cast<std::uint32_t>(std::max(first, second));
  return (static_cast<std::uint64_t>(high) << 32) | static_cast<std::uint64_t>(low);
}

}  // 無名名前空間

namespace fuzzrobo::topological_plane::incremental
{

struct Clusterizer::Impl
{
  explicit Impl(ClusterOptions input_options)
  : options(std::move(input_options))
  {
    options.num_acquisition_phases = std::max<std::size_t>(1U, options.num_acquisition_phases);
    options.min_cluster_nodes = std::max<std::size_t>(3U, options.min_cluster_nodes);
    options.growth_residual_ratio = std::max(0.0, options.growth_residual_ratio);
    options.retention_residual_ratio = std::max(
      options.growth_residual_ratio, options.retention_residual_ratio);
    options.max_effective_spacing = std::max(0.0, options.max_effective_spacing);
    options.merge_smaller_side_residual_ratio =
      std::max(0.0, options.merge_smaller_side_residual_ratio);
    options.max_fragment_edge_ratio_th = std::max(0.0, options.max_fragment_edge_ratio_th);
    options.max_fragment_residual_ratio_th = std::max(0.0, options.max_fragment_residual_ratio_th);
    options.min_fragment_merge_frames = std::max<std::size_t>(1U, options.min_fragment_merge_frames);
    options.min_split_edge_angle_deg_th = std::clamp(options.min_split_edge_angle_deg_th, 0.0, 90.0);
    const double split_sin = std::sin(options.min_split_edge_angle_deg_th * kRadiansPerDeg);
    split_edge_sin_squared = split_sin * split_sin;
    options.max_absorption_edge_angle_deg_th = std::clamp(
      options.max_absorption_edge_angle_deg_th, 0.0, 90.0);
    const double absorption_sin = std::sin(options.max_absorption_edge_angle_deg_th * kRadiansPerDeg);
    absorption_edge_sin_squared = absorption_sin * absorption_sin;
    options.max_absorption_edge_ratio_th = std::max(0.0, options.max_absorption_edge_ratio_th);
    options.min_split_conflict_nodes = std::max<std::size_t>(1U, options.min_split_conflict_nodes);
    options.min_split_conflict_ratio_th = std::clamp(options.min_split_conflict_ratio_th, 0.0, 1.0);
    options.normal_filter_alpha = std::clamp(options.normal_filter_alpha, 0.01, 1.0);
    options.normal_alignment_deg = std::clamp(options.normal_alignment_deg, 0.0, 90.0);
    normal_alignment_cos = std::cos(options.normal_alignment_deg * kRadiansPerDeg);
    // 保持用は種形成用より緩くする。厳しくして種形成側の意図を壊さないよう、
    // 下限を種形成用の角度に揃える(保持用の角度がそれより小さくならないようにする)。
    options.retention_normal_alignment_deg = std::clamp(
      options.retention_normal_alignment_deg, 0.0, 90.0);
    options.retention_normal_alignment_deg = std::max(
      options.retention_normal_alignment_deg, options.normal_alignment_deg);
    retention_normal_alignment_cos =
      std::cos(options.retention_normal_alignment_deg * kRadiansPerDeg);
    options.min_cluster_planarity = std::clamp(options.min_cluster_planarity, 0.0, 1.0);
    options.min_plane_width_ratio = std::max(kEpsilon, options.min_plane_width_ratio);
    options.max_normalized_cluster_residual =
      std::max(0.0, options.max_normalized_cluster_residual);
    // 取り込みが確定判定より緩いと、育てた領域が最後の残差判定で丸ごと捨てられる。
    // 実測では被覆が 67% から 31% まで落ちたため、ここで上限を揃える。
    options.growth_residual_ratio = std::min(
      options.growth_residual_ratio, options.max_normalized_cluster_residual);
    // 成長中のしきい値が確定時より厳しいと、育つ前に必ず止まってしまう。
    options.min_growth_planarity = std::clamp(
      options.min_growth_planarity, 0.0, options.min_cluster_planarity);
    options.merge_min_planarity = std::clamp(
      options.merge_min_planarity, 0.0, options.min_cluster_planarity);
    options.maintenance_iter = std::max<std::size_t>(1U, options.maintenance_iter);
    options.connection_requirement =
      std::max<std::size_t>(1U, options.connection_requirement);
    options.merge_connection_requirement =
      std::max<std::size_t>(1U, options.merge_connection_requirement);
    options.birth_neighbor_requirement =
      std::max<std::size_t>(1U, options.birth_neighbor_requirement);
  }

  // クラスタの永続状態。同一性はid、添字は削除時の詰め直しで更新。
  struct ClusterState
  {
    std::uint32_t id = 0U;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    // 前フレームのOBB接平面基底。法線が僅かに動いただけで軸が不連続に選び直される
    // unitOrthogonal() の切り替わりを避けるため、この方向へ射影して引き継ぐ。
    Eigen::Vector3d tangent_u = Eigen::Vector3d::Zero();
    bool has_tangent_u = false;
    double spacing = 0.0;
    double planarity = 0.0;
    double residual = 0.0;
    std::size_t member_count = 0U;
    std::size_t weak_frames = 0U;
    std::size_t disconnected_frames = 0U;
    std::size_t confirmed_frames = 0U;
    bool is_healthy = false;
    // フレーム内の所属変更による再集計要求。新規クラスタも再集計対象。
    bool has_fit_changes = true;
    // 前回確認した全域木の有効性と、その時点の所属数。
    bool has_connectivity_tree = false;
    std::size_t connectivity_member_count = 0U;
  };

  ClusterOptions options;
  double normal_alignment_cos = 0.50;
  double retention_normal_alignment_cos = 0.087;
  double split_edge_sin_squared = 0.25;
  double absorption_edge_sin_squared = 0.0;

  // --- フレームをまたいで保持する状態 ---
  std::vector<ClusterState> clusters;
  // 前回出力時のクラスタ添字。フレーム末尾の詰め直しまで有効な16bitノードIDの直接表。
  std::vector<int> owner_idx_by_node_id = std::vector<int>(kNodeIdRange, kUnassigned);
  std::uint32_t next_cluster_id = 1U;
  // ノードIDごとの法線EMA状態。IDをそのまま添字にしたフラット配列で持つ
  // (unordered_mapのハッシュ計算・バケット走査・ヒープ確保を避けるため)。
  // normal_filter_frame[id] が「その値を書いた時のフレーム番号」で、0は
  // 「一度も書かれていない」を表す番兵として予約する(実フレーム番号は1以上
  // しか書き込まない)。直前フレーム番号と一致する時だけ前回値とみなして混合する。
  std::vector<Eigen::Vector3d> normal_filter_values =
    std::vector<Eigen::Vector3d>(kNodeIdRange);
  std::vector<std::uint32_t> normal_filter_frame =
    std::vector<std::uint32_t>(kNodeIdRange, 0U);
  std::uint32_t normal_filter_current_frame = 0U;
  // 全エッジ消失時だけ参照する前回局所間隔と、連続孤立フレーム数。
  std::vector<double> previous_spacings = std::vector<double>(kNodeIdRange, 0.0);
  std::vector<std::size_t> isolated_frames = std::vector<std::size_t>(kNodeIdRange, 0U);

  // --- フレームごとに再利用するバッファ ---
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> normals;
  std::vector<double> spacings;
  std::vector<double> local_squared_lengths;
  std::vector<double> seed_scores;
  std::vector<std::uint8_t> usable;
  std::vector<std::uint16_t> node_ids;
  std::vector<std::uint8_t> node_labels;
  std::vector<std::uint32_t> adjacency_offsets;
  std::vector<std::uint32_t> adjacency_values;
  std::vector<std::uint32_t> adjacency_cursor;
  std::vector<int> label;
  std::vector<int> next_label;
  // 所属保守の候補集合。フレーム内では変更点の近傍のみ追加。
  std::vector<std::uint8_t> is_maintenance_candidate;
  bool has_maintenance_candidates = false;
  std::vector<PlaneAccumulator> accumulators;
  std::vector<PlaneFit> plane_fits;
  std::vector<std::size_t> neighbour_counts;
  std::vector<int> touched_clusters;
  std::vector<std::uint32_t> visit_marks;
  std::uint32_t visit_generation = 0U;
  std::vector<std::size_t> frontier;
  std::vector<std::size_t> birth_order;
  std::vector<std::uint8_t> birth_rejected;
  std::vector<int> component_of;
  std::vector<int> component_new_label;
  std::vector<int> cluster_best_component;
  std::vector<std::size_t> cluster_best_size;
  DisjointSet merge_sets;
  // 隣接クラスタ対の接続本数と、各側の接続端点統計。キーのクラスタ順での保持。
  struct AdjacentPair
  {
    std::size_t edges = 0U;
    double single_edge_ratio = 0.0;
    PlaneAccumulator first_contact;
    PlaneAccumulator second_contact;
  };
  std::unordered_map<std::uint64_t, AdjacentPair> adjacent_pair_counts;
  // 配列の詰め直しに依存しない永続クラスタID対による、1本接続の連続確認。
  struct consecutive_frame_state
  {
    std::uint32_t last_frame = 0U;
    std::size_t num_frames = 0U;
  };
  std::unordered_map<std::uint64_t, consecutive_frame_state> fragment_merge_evidence;
  // 永続クラスタIDと成分最小ノードIDによる分断根拠の連続確認。
  std::unordered_map<std::uint64_t, consecutive_frame_state> split_evidence;
  std::vector<int> remap;
  std::vector<ClusterState> kept_clusters;
  std::vector<std::vector<std::uint32_t>> member_lists;
  std::vector<PlaneAccumulator> merge_accumulators;
  std::vector<PlaneFit> merge_fits;
  // 配列再配置と独立したノードID単位の連結証明。欠落後のID再出現はフレーム番号で失効。
  std::vector<std::uint16_t> connectivity_parent = std::vector<std::uint16_t>(kNodeIdRange);
  std::vector<std::uint32_t> connectivity_owner_ids = std::vector<std::uint32_t>(kNodeIdRange, 0U);
  std::vector<std::uint32_t> connectivity_frames = std::vector<std::uint32_t>(kNodeIdRange, 0U);
  std::vector<std::size_t> connectivity_parent_slots = std::vector<std::size_t>(kNodeIdRange, 0U);
  std::vector<std::uint32_t> current_idx_by_id = std::vector<std::uint32_t>(kNodeIdRange);
  std::vector<std::uint8_t> can_reuse_connectivity;
  std::vector<std::size_t> connectivity_member_counts;

  // 差分集計の比較元。所属添字は前回末尾のクラスタ詰め直し後の値。
  std::vector<Eigen::Vector3d> stats_positions;
  std::vector<double> stats_spacings;
  std::vector<int> stats_labels;
  std::vector<std::uint16_t> stats_node_ids;
  std::vector<int> stats_idx_by_id;
  std::vector<std::uint8_t> stats_seen;
  bool has_stats_cache = false;

  // IDを分散した巡回候補。独立な乱数抽選による長期未選択の回避、ノード順序への非依存。
  bool is_acquisition_due(const std::size_t node_idx) const
  {
    if (options.num_acquisition_phases == 1U) {return true;}
    std::uint32_t key = node_ids[node_idx];
    key = (key ^ (key >> 16U)) * 0x7feb352dU;
    key = (key ^ (key >> 15U)) * 0x846ca68bU;
    key ^= key >> 16U;
    return key % options.num_acquisition_phases ==
           (normal_filter_current_frame - 1U) % options.num_acquisition_phases;
  }

  // 連続ノードの内部ブロック。外部のクラスタID・所属・形状への影響なし。
  static constexpr std::size_t num_retention_block_nodes = 64U;
  struct retention_block
  {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double radius = 0.0;
    double max_residual = 0.0;
    double min_spacing = 0.0;
    double min_alignment = 0.0;
    std::uint32_t cluster_id = 0U;
    bool is_valid = false;
  };
  struct retention_input
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double spacing = 0.0;
  };
  std::vector<retention_block> retention_blocks;
  std::vector<retention_input> retention_inputs;
  std::vector<std::uint8_t> can_retain_node;

  // 前回の全入力との厳密比較と、参照平面からの変化上限による保持証明。
  // 証明不能の場合は従来どおりの個別判定。前フレームごとの誤差の累積なし。
  void prepare_retention_blocks(ClusterStatistics &statistics)
  {
    can_retain_node.assign(label.size(), 0U);
    retention_inputs.resize(label.size());
    retention_blocks.resize((label.size() + num_retention_block_nodes - 1U) /
      num_retention_block_nodes);
    for (std::size_t begin = 0U; begin < label.size(); begin += num_retention_block_nodes) {
      const std::size_t end = std::min(label.size(), begin + num_retention_block_nodes);
      auto &block = retention_blocks[begin / num_retention_block_nodes];
      const int owner = label[begin];
      bool has_same_owner = owner != kUnassigned;
      bool has_same_input = block.is_valid;
      for (std::size_t idx = begin; idx < end; ++idx) {
        has_same_owner = has_same_owner && label[idx] == owner;
        auto &input = retention_inputs[idx];
        has_same_input = has_same_input && input.position == positions[idx] &&
          input.normal == normals[idx] && input.spacing == spacings[idx];
        input = {positions[idx], normals[idx], spacings[idx]};
      }
      if (!has_same_owner || clusters[owner].member_count < 3U) {
        block.is_valid = false;
        continue;
      }
      const auto &cluster = clusters[owner];
      if (has_same_input && block.cluster_id == cluster.id) {
        const double normal_change = (cluster.normal - block.normal).norm();
        const double dist_change = block.radius * normal_change +
          std::abs(cluster.normal.dot(block.centroid - cluster.centroid));
        if (block.max_residual + dist_change + kEpsilon <=
          options.retention_residual_ratio * block.min_spacing &&
          block.min_alignment - normal_change - kEpsilon >= retention_normal_alignment_cos)
        {
          std::fill(can_retain_node.begin() + begin, can_retain_node.begin() + end, 1U);
          statistics.num_retention_reused_nodes += end - begin;
          continue;
        }
      }
      block = retention_block{};
      block.centroid = cluster.centroid;
      block.normal = cluster.normal;
      block.cluster_id = cluster.id;
      block.min_spacing = std::numeric_limits<double>::infinity();
      block.min_alignment = 1.0;
      for (std::size_t idx = begin; idx < end; ++idx) {
        const Eigen::Vector3d delta = positions[idx] - cluster.centroid;
        block.radius = std::max(block.radius, delta.norm());
        block.max_residual = std::max(block.max_residual, std::abs(cluster.normal.dot(delta)));
        block.min_alignment = std::min(block.min_alignment, std::abs(cluster.normal.dot(normals[idx])));
        block.min_spacing = std::min(block.min_spacing, effective_spacing(spacings[idx]));
      }
      block.is_valid = true;
    }
  }

  // 所属変更の加減算。判定中のクラスタモデル・メンバー数への変更なし。
  void update_membership_statistics(const std::size_t node_idx, const int old_label,
    const int new_label)
  {
    if (!options.enable_delta_statistics) {return;}
    if (old_label != kUnassigned) {
      accumulators[old_label].remove(positions[node_idx], spacings[node_idx]);
    }
    if (new_label != kUnassigned) {
      accumulators[new_label].add(positions[node_idx], normals[node_idx], spacings[node_idx]);
    }
  }

  // ノードIDによる位置・局所間隔・所属の差分。並び替え・追加・削除への対応。
  void update_frame_statistics()
  {
    stats_idx_by_id.assign(kNodeIdRange, kUnassigned);
    stats_seen.assign(stats_node_ids.size(), 0U);
    for (std::size_t idx = 0U; idx < stats_node_ids.size(); ++idx) {
      stats_idx_by_id[stats_node_ids[idx]] = static_cast<int>(idx);
    }
    for (std::size_t idx = 0U; idx < node_ids.size(); ++idx) {
      const int old_idx = stats_idx_by_id[node_ids[idx]];
      const int old_label = old_idx == kUnassigned ? kUnassigned : stats_labels[old_idx];
      if (old_idx != kUnassigned) {
        stats_seen[old_idx] = 1U;
        if (old_label == label[idx] && stats_positions[old_idx] == positions[idx] &&
          stats_spacings[old_idx] == spacings[idx]) {continue;}
      }
      if (old_label != kUnassigned) {
        accumulators[old_label].remove(stats_positions[old_idx], stats_spacings[old_idx]);
        clusters[old_label].has_fit_changes = true;
      }
      if (label[idx] != kUnassigned) {
        accumulators[label[idx]].add(positions[idx], normals[idx], spacings[idx]);
        clusters[label[idx]].has_fit_changes = true;
      }
    }
    for (std::size_t idx = 0U; idx < stats_seen.size(); ++idx) {
      if (stats_seen[idx] == 0U && stats_labels[idx] != kUnassigned) {
        const int old_label = stats_labels[idx];
        accumulators[old_label].remove(stats_positions[idx], stats_spacings[idx]);
        clusters[old_label].has_fit_changes = true;
      }
    }
  }

  // 隣接リスト(CSR)と局所量の構築。ノード・エッジ数に比例する前処理。
  void prepareFrame(const graph_view &map, ClusterStatistics &statistics)
  {
    has_maintenance_candidates = false;
    const std::size_t node_count = map.num_nodes;
    positions.assign(node_count, Eigen::Vector3d::Zero());
    normals.assign(node_count, Eigen::Vector3d::UnitZ());
    spacings.assign(node_count, 0.0);
    seed_scores.assign(node_count, 0.0);
    usable.assign(node_count, 0U);
    node_ids.resize(node_count);
    node_labels.resize(node_count);
    std::fill(current_idx_by_id.begin(), current_idx_by_id.end(), std::numeric_limits<std::uint32_t>::max());

    for (std::size_t index = 0U; index < node_count; ++index) {
      const auto node = map.read_node(map.nodes, index);
      node_ids[index] = node.id;
      node_labels[index] = node.label;
      current_idx_by_id[node_ids[index]] = static_cast<std::uint32_t>(index);
      const Eigen::Vector3d position = pointOf(node.pos);
      if (!position.allFinite()) {
        continue;
      }
      positions[index] = position;
      // 入力ノード列の一回読出し。以降の局所量計算は密な作業配列のみ参照。
      normals[index] = pointOf(node.normal);
      seed_scores[index] = static_cast<double>(node.rho);
      usable[index] = 1U;
      ++statistics.valid_node_count;
    }

    // 次数を数えてから CSR を確定する。vector<vector> を毎フレーム作らない。
    adjacency_offsets.assign(node_count + 1U, 0U);
    const std::size_t edge_count = map.num_edge_values / 2U;
    for (std::size_t edge_index = 0U; edge_index < edge_count; ++edge_index) {
      const std::size_t first = map.edges[edge_index * 2U];
      const std::size_t second = map.edges[edge_index * 2U + 1U];
      if (first >= node_count || second >= node_count || first == second ||
        usable[first] == 0U || usable[second] == 0U)
      {
        continue;
      }
      ++adjacency_offsets[first + 1U];
      ++adjacency_offsets[second + 1U];
    }
    for (std::size_t index = 0U; index < node_count; ++index) {
      adjacency_offsets[index + 1U] += adjacency_offsets[index];
    }
    adjacency_values.resize(adjacency_offsets[node_count]);
    adjacency_cursor.assign(adjacency_offsets.begin(), adjacency_offsets.end() - 1);
    for (std::size_t edge_index = 0U; edge_index < edge_count; ++edge_index) {
      const std::size_t first = map.edges[edge_index * 2U];
      const std::size_t second = map.edges[edge_index * 2U + 1U];
      if (first >= node_count || second >= node_count || first == second ||
        usable[first] == 0U || usable[second] == 0U)
      {
        continue;
      }
      adjacency_values[adjacency_cursor[first]++] = static_cast<std::uint32_t>(second);
      adjacency_values[adjacency_cursor[second]++] = static_cast<std::uint32_t>(first);
    }

    // 局所間隔は隣接エッジ長の中央値（偶数本では小さい側）。長い橋エッジの影響抑制。
    // GNG法線の再利用と、欠損時のみ1ホップ近傍の共分散による補完。
    PlaneAccumulator local;
    for (std::size_t index = 0U; index < node_count; ++index) {
      if (usable[index] == 0U) {
        continue;
      }
      const Eigen::Vector3d supplied = normals[index];
      // 孤立・補完失敗時にも従来と同じ初期法線の維持。
      normals[index] = Eigen::Vector3d::UnitZ();
      local_squared_lengths.clear();
      for (std::size_t cursor = adjacency_offsets[index];
        cursor < adjacency_offsets[index + 1U]; ++cursor)
      {
        const double squared_dist = (positions[index] - positions[adjacency_values[cursor]]).squaredNorm();
        if (std::isfinite(squared_dist) && squared_dist > kEpsilon * kEpsilon) {
          local_squared_lengths.push_back(squared_dist);
        }
      }
      const auto id = node_ids[index];
      const bool is_isolated = local_squared_lengths.empty();
      if (is_isolated) {
        if (!options.enable_directional_split || normal_filter_current_frame == 0U ||
          normal_filter_frame[id] != normal_filter_current_frame ||
          owner_idx_by_node_id[id] == kUnassigned || isolated_frames[id] >= options.max_isolated_frames)
        {
          usable[index] = 0U;
          continue;
        }
        ++isolated_frames[id];
        spacings[index] = previous_spacings[id];
        ++statistics.num_isolated_retained_nodes;
      } else {
        isolated_frames[id] = 0U;
        // GNGの短い隣接列は固定比較で下側中央値を選択。高次数のみ汎用選択。
        auto &lengths = local_squared_lengths;
        double median;
        switch (lengths.size()) {
          case 1U: median = lengths[0]; break;
          case 2U: median = std::min(lengths[0], lengths[1]); break;
          case 3U:
            median = std::max(std::min(lengths[0], lengths[1]),
                std::min(std::max(lengths[0], lengths[1]), lengths[2]));
            break;
          case 4U: {
            const auto first = std::minmax(lengths[0], lengths[1]);
            const auto second = std::minmax(lengths[2], lengths[3]);
            median = first.first < second.first ? std::min(first.second, second.first) :
              std::min(second.second, first.first);
            break;
          }
          default: {
            if (lengths.size() <= 8U) {
              // 5〜8近傍の固定比較網。番兵は実距離の後方への配置。
              double values[8];
              std::fill(std::begin(values), std::end(values), std::numeric_limits<double>::infinity());
              std::copy(lengths.begin(), lengths.end(), values);
              const auto order = [&values](const std::size_t first, const std::size_t second) {
                  const double low = std::min(values[first], values[second]);
                  const double high = std::max(values[first], values[second]);
                  values[first] = low;
                  values[second] = high;
                };
              order(0, 1); order(2, 3); order(4, 5); order(6, 7);
              order(0, 2); order(1, 3); order(4, 6); order(5, 7);
              order(1, 2); order(5, 6); order(0, 4); order(3, 7);
              order(1, 5); order(2, 6);
              order(1, 4); order(3, 6);
              order(2, 4); order(3, 5);
              order(3, 4);
              median = values[(lengths.size() - 1U) / 2U];
            } else {
              const auto middle = lengths.begin() + (lengths.size() - 1U) / 2U;
              std::nth_element(lengths.begin(), middle, lengths.end());
              median = *middle;
            }
          }
        }
        spacings[index] = std::sqrt(median);
      }

      if (supplied.allFinite() && supplied.squaredNorm() > kEpsilon) {
        normals[index] = supplied.normalized();
        continue;
      }
      if (is_isolated) {
        normals[index] = normal_filter_values[id];
        continue;
      }
      local.clear();
      local.add(positions[index], Eigen::Vector3d::UnitZ(), spacings[index]);
      for (std::size_t cursor = adjacency_offsets[index];
        cursor < adjacency_offsets[index + 1U]; ++cursor)
      {
        const std::size_t neighbour = adjacency_values[cursor];
        local.add(positions[neighbour], Eigen::Vector3d::UnitZ(), spacings[neighbour]);
      }
      const PlaneFit fit = local.solve();
      if (!fit.is_valid) {
        usable[index] = 0U;
        continue;
      }
      normals[index] = fit.normal;
    }

    // 法線へEMA(指数移動平均)を掛け、瞬間的な推定誤差を均す。
    //
    // GNG法線のフレーム間角度変化は分布の裾が非常に重く、p99.9で約86度に達する
    // (実測)。この裾のほとんどは境界・稜線ノードの瞬間的な推定誤差であり、構造的な
    // 不安定性ではない。EMAを掛けるとp99.9が大きく縮む(alpha=0.3で約22度)ため、
    // is_normal_aligned のしきい値を緩めるより先に、ここでノイズそのものを削る。
    // ノードIDを直接添字にして前フレームの値を読み書きする。直前フレームに
    // 存在しなかったIDはフレーム番号が一致せず、自然に「初回」扱いになる。
    // 符号は法線ごとに不定なので、混合前にそろえる。
    ++normal_filter_current_frame;
    const std::uint32_t previous_frame = normal_filter_current_frame - 1U;
    for (std::size_t index = 0U; index < node_count; ++index) {
      if (usable[index] == 0U) {
        continue;
      }
      const std::uint16_t id = node_ids[index];
      if (normal_filter_frame[id] != 0U && normal_filter_frame[id] == previous_frame) {
        Eigen::Vector3d oriented = normals[index];
        const Eigen::Vector3d &previous = normal_filter_values[id];
        if (oriented.dot(previous) < 0.0) {
          oriented = -oriented;
        }
        const Eigen::Vector3d blended =
          options.normal_filter_alpha * oriented + (1.0 - options.normal_filter_alpha) * previous;
        if (blended.squaredNorm() > kEpsilon) {
          normals[index] = blended.normalized();
        }
      }
      normal_filter_values[id] = normals[index];
      normal_filter_frame[id] = normal_filter_current_frame;
      previous_spacings[id] = spacings[index];
    }

    // 新規クラスタの種は、面の内側に近いノードから選ぶ。
    // CPU GNGは rho=acos(mean(abs(n_i・n_j))) をすでに計算しているので、
    // 直結経路では-rhoをそのまま順序スコアに使い、全edge内積の重複計算を避ける。
    // rhoが無効なノードと、rhoを使わない独立経路だけ従来計算する。
    for (std::size_t index = 0U; index < node_count; ++index) {
      if (usable[index] == 0U) {
        continue;
      }
      ++statistics.usable_node_count;
      if (options.use_node_rho_for_seed_order) {
        const double rho = seed_scores[index];
        if (std::isfinite(rho) && rho >= 0.0 && rho <= kHalfPi + kEpsilon) {
          seed_scores[index] = -rho;
          continue;
        }
      }
      double coherence_sum = 0.0;
      std::size_t coherence_count = 0U;
      for (std::size_t cursor = adjacency_offsets[index];
        cursor < adjacency_offsets[index + 1U]; ++cursor)
      {
        const std::size_t neighbour = adjacency_values[cursor];
        if (usable[neighbour] == 0U) {
          continue;
        }
        coherence_sum += std::abs(normals[index].dot(normals[neighbour]));
        ++coherence_count;
      }
      const double coherence = coherence_count > 0U ?
        coherence_sum / static_cast<double>(coherence_count) : 0.0;
      seed_scores[index] = options.use_node_rho_for_seed_order ?
        -std::acos(std::clamp(coherence, 0.0, 1.0)) : coherence;
    }
  }

  // 前フレームの所属をGNGノードIDで引き継ぐ。
  void carryOverLabels(const graph_view &map)
  {
    const std::size_t node_count = map.num_nodes;
    label.assign(node_count, kUnassigned);
    if (clusters.empty()) {
      return;
    }
    for (std::size_t index = 0U; index < node_count; ++index) {
      if (usable[index] == 0U) {
        continue;
      }
      label[index] = owner_idx_by_node_id[node_ids[index]];
    }
  }

  // 従来経路は初回全平面・以降変更平面の再累積。試作経路は差分済み統計からの求解。
  void refitClusters(const bool enable_full_refit = false)
  {
    accumulators.resize(clusters.size());
    plane_fits.resize(clusters.size());
    const bool enable_rebuild = !options.enable_delta_statistics ||
      (enable_full_refit && !has_stats_cache);
    if (enable_full_refit && !enable_rebuild) {update_frame_statistics();}
    for (std::size_t idx = 0U; idx < clusters.size(); ++idx) {
      auto &cluster = clusters[idx];
      cluster.has_fit_changes = cluster.has_fit_changes || (enable_full_refit && enable_rebuild);
      if (cluster.has_fit_changes) {
        if (enable_rebuild) {accumulators[idx].clear();}
        plane_fits[idx] = PlaneFit{};
      }
    }
    for (std::size_t index = 0U; enable_rebuild && index < label.size(); ++index) {
      const int cluster_index = label[index];
      if (cluster_index == kUnassigned || !clusters[cluster_index].has_fit_changes) {
        continue;
      }
      accumulators[static_cast<std::size_t>(cluster_index)].add(
        positions[index], normals[index], spacings[index]);
    }
    for (std::size_t index = 0U; index < clusters.size(); ++index) {
      ClusterState &cluster = clusters[index];
      if (!cluster.has_fit_changes) {
        continue;
      }
      cluster.has_fit_changes = false;
      PlaneAccumulator &accumulator = accumulators[index];
      if (options.enable_delta_statistics) {accumulator.normal_sum = cluster.normal;}
      cluster.member_count = accumulator.count;
      if (accumulator.count < 3U) {
        continue;
      }
      PlaneFit fit = accumulator.solve();
      plane_fits[index] = fit;
      if (!fit.is_valid) {
        continue;
      }
      if (options.enable_delta_statistics) {
        // 解いた重心への原点移動。長時間の並進に伴う二次モーメントの肥大化防止。
        accumulator.anchor = fit.centroid;
        accumulator.sum.setZero();
        accumulator.moment = fit.covariance * static_cast<double>(accumulator.count);
      }
      // 前フレームの法線へそろえ、向きの反転で追従が切れないようにする。
      orientNormal(fit.normal, cluster.normal);
      cluster.centroid = fit.centroid;
      cluster.covariance = fit.covariance;
      cluster.normal = fit.normal;
      cluster.planarity = fit.planarity;
      cluster.residual = fit.residual;
      cluster.spacing = std::max(accumulator.meanSpacing(), kEpsilon);
    }
  }

  // 縦横比または局所間隔に対する面幅による線状領域の除外。長さへの非依存。
  bool has_plane_extent(
    const PlaneFit &fit, const double spacing, const double min_planarity) const
  {
    return fit.planarity >= min_planarity ||
           fit.plane_width_std >= options.min_plane_width_ratio * std::max(spacing, kEpsilon);
  }

  // 局所スケールの任意の絶対上限。0は上限なし。
  double effective_spacing(const double spacing) const
  {
    return options.max_effective_spacing > 0.0 ?
           std::min(std::max(spacing, kEpsilon), options.max_effective_spacing) :
           std::max(spacing, kEpsilon);
  }

  // 対象ノード自身の局所間隔による平面距離比。粗いクラスタ平均による許容幅の膨張防止。
  double fitScore(const std::size_t cluster_index, const std::size_t node_index) const
  {
    const ClusterState &cluster = clusters[cluster_index];
    const double spacing = effective_spacing(spacings[node_index]);
    return std::abs(cluster.normal.dot(positions[node_index] - cluster.centroid)) / spacing;
  }

  // 保持・取り込み・移動で使う。種の判定(normal_alignment_cos)より緩い。
  bool is_normal_aligned(const std::size_t cluster_index, const std::size_t node_index) const
  {
    return std::abs(clusters[cluster_index].normal.dot(normals[node_index])) >=
           retention_normal_alignment_cos;
  }

  // 平面から離れすぎたメンバーを解放する。取り込みより緩い閾値を使う。
  std::size_t releaseOutliers(ClusterStatistics &statistics)
  {
    std::size_t released = 0U;
    if (options.enable_block_retention) {prepare_retention_blocks(statistics);}
    for (std::size_t index = 0U; index < label.size(); ++index) {
      if (options.enable_block_retention && can_retain_node[index] != 0U) {continue;}
      const int cluster_index = label[index];
      if (cluster_index == kUnassigned) {
        continue;
      }
      const std::size_t cluster = static_cast<std::size_t>(cluster_index);
      if (clusters[cluster].member_count < 3U) {
        continue;
      }
      if (!is_normal_aligned(cluster, index)) {
        update_membership_statistics(index, cluster_index, kUnassigned);
        clusters[cluster].has_fit_changes = true;
        label[index] = kUnassigned;
        ++statistics.released_node_count;
        ++released;
        continue;
      }
      // 既所属の保持ヒステリシス。取り込み緩和フラグや接続本数とは独立。
      if (fitScore(cluster, index) <= options.retention_residual_ratio) {
        continue;
      }
      update_membership_statistics(index, cluster_index, kUnassigned);
      label[index] = kUnassigned;
      clusters[cluster].has_fit_changes = true;
      ++statistics.released_node_count;
      ++released;
    }
    return released;
  }

  // 通常条件で不採用となった未所属点の面内接続確認。候補1点の隣接走査のみ。
  bool can_absorb_coplanar_node(const std::size_t cluster_idx, const std::size_t node_idx) const
  {
    const auto &cluster = clusters[cluster_idx];
    if (cluster.member_count < options.min_cluster_nodes ||
      cluster.confirmed_frames < options.birth_confirm_frames)
    {
      return false;
    }
    bool has_short_contact = false;
    for (std::size_t cursor = adjacency_offsets[node_idx];
      cursor < adjacency_offsets[node_idx + 1U]; ++cursor)
    {
      const std::size_t neighbour_idx = adjacency_values[cursor];
      const Eigen::Vector3d edge = positions[neighbour_idx] - positions[node_idx];
      const double edge_squared = edge.squaredNorm();
      if (edge_squared <= kEpsilon * kEpsilon) {
        continue;
      }
      const double height = cluster.normal.dot(edge);
      // 未所属・別形状へ伸びるエッジも含む、面外方向の即時棄却。
      if (height * height > absorption_edge_sin_squared * edge_squared) {
        return false;
      }
      if (label[neighbour_idx] == static_cast<int>(cluster_idx)) {
        const double max_contact = options.max_absorption_edge_ratio_th *
          std::min(effective_spacing(spacings[node_idx]), effective_spacing(spacings[neighbour_idx]));
        has_short_contact = has_short_contact || edge_squared <= max_contact * max_contact;
      }
    }
    return has_short_contact;
  }

  // 保守点検の1回分。取り込みと移動の同時判定。
  //
  // 候補クラスタ C の資格は、そのノードの隣接のうち「すでに C に所属している
  // ノード」の数で決める。1本のエッジだけで所属が漏れ出すのを防ぐ条件である。
  //
  // 判定は前フレームの label を読み、結果は next_label へ書く。同じパス内で
  // 走査順に影響されないため、結果が決定的になる。
  std::size_t maintenancePass(ClusterStatistics &statistics, const graph_view &map)
  {
    if (!has_maintenance_candidates) {
      is_maintenance_candidate.assign(label.size(), 0U);
      // 無向エッジの一回走査による境界候補。自己平面内・両端未所属は対象外。
      for (std::size_t idx = 0U; idx + 1U < map.num_edge_values; idx += 2U) {
        const auto first = map.edges[idx];
        const auto second = map.edges[idx + 1U];
        if (first >= label.size() || second >= label.size() || label[first] == label[second]) {
          continue;
        }
        if (label[second] != kUnassigned) {is_maintenance_candidate[first] = 1U;}
        if (label[first] != kUnassigned) {is_maintenance_candidate[second] = 1U;}
      }
      has_maintenance_candidates = true;
    }
    next_label = label;
    neighbour_counts.assign(clusters.size(), 0U);
    std::size_t changed = 0U;

    for (std::size_t index = 0U; index < label.size(); ++index) {
      if (usable[index] == 0U || is_maintenance_candidate[index] == 0U ||
        (label[index] != kUnassigned && !is_acquisition_due(index)))
      {
        continue;
      }
      touched_clusters.clear();
      const int current_label = label[index];
      for (std::size_t cursor = adjacency_offsets[index];
        cursor < adjacency_offsets[index + 1U]; ++cursor)
      {
        const int neighbour_label = label[adjacency_values[cursor]];
        // 自平面への移籍候補は不要。未所属点の救済条件は従来どおり。
        if (neighbour_label == kUnassigned || neighbour_label == current_label) {
          continue;
        }
        const std::size_t cluster = static_cast<std::size_t>(neighbour_label);
        if (neighbour_counts[cluster] == 0U) {
          touched_clusters.push_back(neighbour_label);
        }
        ++neighbour_counts[cluster];
      }
      if (touched_clusters.empty()) {
        continue;
      }

      int best_label = kUnassigned;
      double best_score = std::numeric_limits<double>::infinity();
      bool is_best_coplanar_absorption = false;
      // 競合する平面への漏出防止。救済候補は未所属点につき最大1平面。
      const bool can_rescue_absorption = options.enable_coplanar_absorption &&
        current_label == kUnassigned && touched_clusters.size() == 1U;
      for (const int candidate_label : touched_clusters) {
        const std::size_t cluster = static_cast<std::size_t>(candidate_label);
        const bool has_req_connections = neighbour_counts[cluster] >= options.connection_requirement;
        if (clusters[cluster].member_count < 3U ||
          (!has_req_connections && !(can_rescue_absorption && options.connection_requirement == 2U)))
        {
          continue;
        }
        if (!is_normal_aligned(cluster, index)) {
          continue;
        }
        const double score = fitScore(cluster, index);
        // 既存オプションによる取り込み・移動の距離緩和。保持用上限の維持。
        const bool is_multi_edge_dist_relaxed =
          options.enable_multi_edge_dist_relaxation &&
          neighbour_counts[cluster] >= 2U &&
          score <= options.retention_residual_ratio;
        const bool is_regular_candidate = has_req_connections &&
          (is_multi_edge_dist_relaxed || score <= options.growth_residual_ratio);
        if (!is_regular_candidate) {
          const double max_residual_ratio = has_req_connections && neighbour_counts[cluster] >= 2U ?
            options.retention_residual_ratio : options.growth_residual_ratio;
          if (!can_rescue_absorption || score > max_residual_ratio ||
            !can_absorb_coplanar_node(cluster, index))
          {
            continue;
          }
        }
        // 同点時は添字の小さいクラスタを選び、フレーム間で結果を安定させる。
        if (score < best_score - kEpsilon ||
          (best_label != kUnassigned && std::abs(score - best_score) <= kEpsilon &&
          candidate_label < best_label))
        {
          best_score = score;
          best_label = candidate_label;
          is_best_coplanar_absorption = !is_regular_candidate;
        }
      }

      if (best_label != kUnassigned) {
        if (current_label == kUnassigned) {
          update_membership_statistics(index, current_label, best_label);
          next_label[index] = best_label;
          clusters[best_label].has_fit_changes = true;
          ++statistics.absorbed_node_count;
          statistics.num_coplanar_absorbed_nodes += is_best_coplanar_absorption ? 1U : 0U;
          ++changed;
        } else {
          const std::size_t current_cluster = static_cast<std::size_t>(current_label);
          // 小さいクラスタは供給しない。境界から少しずつ吸われて消えるのを防ぐ。
          // まとまるべきならクラスタ併合として一括で行われるべきである。
          const bool is_donor_protected =
            clusters[current_cluster].member_count <=
            options.min_cluster_nodes + options.donor_protection_buffer;
          if (!is_donor_protected) {
            const double current_score = fitScore(current_cluster, index);
            // 明確に当てはめが良くなる場合は移す。
            const bool is_migration_accepted =
              best_score < current_score - options.migration_improvement_margin;
            if (is_migration_accepted) {
              update_membership_statistics(index, current_label, best_label);
              next_label[index] = best_label;
              clusters[best_label].has_fit_changes = true;
              clusters[current_cluster].has_fit_changes = true;
              ++statistics.migrated_node_count;
              ++changed;
            }
          }
        }
      }

      for (const int candidate_label : touched_clusters) {
        neighbour_counts[static_cast<std::size_t>(candidate_label)] = 0U;
      }
      if (next_label[index] != current_label) {
        // 次の同期パスで新しく候補になり得る接続端点。旧候補の保持による保守的な集合。
        for (auto cursor = adjacency_offsets[index]; cursor < adjacency_offsets[index + 1U]; ++cursor) {
          is_maintenance_candidate[adjacency_values[cursor]] = 1U;
        }
      }
    }

    label.swap(next_label);
    return changed;
  }

  // 未所属ノードから新しいクラスタを起こす。
  void birthClusters(ClusterStatistics &statistics)
  {
    birth_order.clear();
    for (std::size_t index = 0U; index < label.size(); ++index) {
      if (usable[index] != 0U && label[index] == kUnassigned && is_acquisition_due(index)) {
        // 種自身と隣接2点が揃わない候補の除外。成長先としての使用は従来どおり。
        std::size_t num_support = 0U;
        for (auto cursor = adjacency_offsets[index]; cursor < adjacency_offsets[index + 1U]; ++cursor) {
          if (label[adjacency_values[cursor]] == kUnassigned && ++num_support == 2U) {break;}
        }
        if (num_support < 2U) {continue;}
        birth_order.push_back(index);
      }
    }
    if (birth_order.empty()) {
      return;
    }
    // 面の内側から始めるほど、稜線をまたがずに素直に広がる。
    std::sort(
      birth_order.begin(), birth_order.end(),
      [this](const std::size_t first, const std::size_t second) {
        if (seed_scores[first] != seed_scores[second]) {
          return seed_scores[first] > seed_scores[second];
        }
        return first < second;
      });

    visit_marks.assign(label.size(), 0U);
    // 一度失敗した領域のノードは、このフレームでは種にも成長先にもしない。
    // 同じ鎖を何度も探索し直す無駄を防ぐ。次のフレームでは再挑戦する。
    birth_rejected.assign(label.size(), 0U);
    visit_generation = 0U;
    PlaneAccumulator accumulator;

    // 失敗した領域のメンバーをまとめて却下扱いにする。
    const auto rejectFrontier = [this]() {
        for (const std::size_t member : frontier) {
          birth_rejected[member] = 1U;
        }
      };

    for (const std::size_t seed : birth_order) {
      if (label[seed] != kUnassigned || birth_rejected[seed] != 0U) {
        continue;
      }
      ++visit_generation;

      // 種とその未所属隣接で最初の平面を作る。3点そろわなければ起こさない。
      accumulator.clear();
      frontier.clear();
      accumulator.add(positions[seed], normals[seed], spacings[seed]);
      visit_marks[seed] = visit_generation;
      frontier.push_back(seed);
      for (std::size_t cursor = adjacency_offsets[seed];
        cursor < adjacency_offsets[seed + 1U]; ++cursor)
      {
        const std::size_t neighbour = adjacency_values[cursor];
        if (label[neighbour] != kUnassigned || birth_rejected[neighbour] != 0U ||
          visit_marks[neighbour] == visit_generation ||
          std::abs(normals[seed].dot(normals[neighbour])) < normal_alignment_cos)
        {
          continue;
        }
        // 初期支持点にも成長時と同じ距離制約。別の高さの面を含む種の生成防止。
        const double seed_spacing = effective_spacing(
          std::min(spacings[seed], spacings[neighbour]));
        if (std::abs(normals[seed].dot(positions[neighbour] - positions[seed])) >
          options.growth_residual_ratio * seed_spacing)
        {
          continue;
        }
        accumulator.add(positions[neighbour], normals[neighbour], spacings[neighbour]);
        visit_marks[neighbour] = visit_generation;
        frontier.push_back(neighbour);
      }
      PlaneFit fit = accumulator.solve();
      if (!fit.is_valid) {
        // 3点そろわず平面が解けなかっただけなので、領域として棄却はしない。
        // ここで却下マークを付けると、隣の種から育てば使えるノードまで潰れる。
        continue;
      }
      // 種の時点で線分状なら、そこから育てても鎖にしかならない。
      if (!has_plane_extent(fit, accumulator.meanSpacing(), options.min_growth_planarity)) {
        ++statistics.chain_rejected_count;
        rejectFrontier();
        continue;
      }

      // 育てながら平面を更新する。サイズが倍になるたびに解き直すことで、
      // 再フィット回数を log に抑えつつドリフトを止める。
      std::size_t refit_th = frontier.size() * 2U;
      bool is_chain_like = false;
      for (std::size_t frontier_index = 0U;
        frontier_index < frontier.size() && !is_chain_like; ++frontier_index)
      {
        const std::size_t current = frontier[frontier_index];
        for (std::size_t cursor = adjacency_offsets[current];
          cursor < adjacency_offsets[current + 1U]; ++cursor)
        {
          const std::size_t neighbour = adjacency_values[cursor];
          if (label[neighbour] != kUnassigned || birth_rejected[neighbour] != 0U ||
            visit_marks[neighbour] == visit_generation)
          {
            continue;
          }
          if (std::abs(fit.normal.dot(normals[neighbour])) < normal_alignment_cos) {
            continue;
          }
          const double spacing = effective_spacing(spacings[neighbour]);
          if (std::abs(fit.normal.dot(positions[neighbour] - fit.centroid)) / spacing >
            options.growth_residual_ratio)
          {
            continue;
          }
          // 生成中のクラスタに所属済みの隣接がいくつあるかを数える。
          // 要求1本の場合、探索元currentへの接続が成立済み。
          if (options.birth_neighbor_requirement > 1U) {
            std::size_t attached = 0U;
            for (std::size_t back_cursor = adjacency_offsets[neighbour];
              back_cursor < adjacency_offsets[neighbour + 1U]; ++back_cursor)
            {
              if (visit_marks[adjacency_values[back_cursor]] == visit_generation &&
                ++attached >= options.birth_neighbor_requirement)
              {
                break;
              }
            }
            if (attached < options.birth_neighbor_requirement) {
              continue;
            }
          }
          accumulator.add(positions[neighbour], normals[neighbour], spacings[neighbour]);
          visit_marks[neighbour] = visit_generation;
          frontier.push_back(neighbour);
          if (frontier.size() >= refit_th) {
            const PlaneFit updated = accumulator.solve();
            if (updated.is_valid) {
              fit = updated;
              // サイズ倍化時の線状化判定。十分な幅を保った長い平面の許容。
              if (!has_plane_extent(
                  updated, accumulator.meanSpacing(), options.min_growth_planarity))
              {
                is_chain_like = true;
                break;
              }
            }
            refit_th = frontier.size() * 2U;
          }
        }
      }

      if (is_chain_like) {
        ++statistics.chain_rejected_count;
        rejectFrontier();
        continue;
      }
      if (frontier.size() < options.min_cluster_nodes) {
        rejectFrontier();
        continue;
      }
      const PlaneFit final_fit = accumulator.solve();
      const double spacing = std::max(accumulator.meanSpacing(), kEpsilon);
      if (!final_fit.is_valid ||
        final_fit.residual / effective_spacing(spacing) >
        options.max_normalized_cluster_residual)
      {
        rejectFrontier();
        continue;
      }
      if (!has_plane_extent(final_fit, spacing, options.min_cluster_planarity)) {
        ++statistics.chain_rejected_count;
        rejectFrontier();
        continue;
      }

      ClusterState cluster;
      cluster.id = next_cluster_id++;
      cluster.centroid = final_fit.centroid;
      cluster.covariance = final_fit.covariance;
      cluster.normal = final_fit.normal;
      cluster.planarity = final_fit.planarity;
      cluster.residual = final_fit.residual;
      cluster.spacing = spacing;
      cluster.member_count = frontier.size();
      const int new_label = static_cast<int>(clusters.size());
      clusters.push_back(cluster);
      accumulators.push_back(accumulator);
      plane_fits.push_back(final_fit);
      for (const std::size_t member : frontier) {
        label[member] = new_label;
      }
      ++statistics.born_cluster_count;
    }
  }

  // 非連結成分の分割。面外エッジの根拠がない成分の元平面所属を維持。
  bool splitClusters(ClusterStatistics &statistics)
  {
    if (clusters.empty()) {
      split_evidence.clear();
      return false;
    }
    bool changed = false;
    connectivity_member_counts.assign(clusters.size(), 0U);
    can_reuse_connectivity.resize(clusters.size());
    for (std::size_t idx = 0U; idx < clusters.size(); ++idx) {
      can_reuse_connectivity[idx] = clusters[idx].has_connectivity_tree;
    }
    for (std::size_t idx = 0U; idx < label.size(); ++idx) {
      const auto id = node_ids[idx];
      if (label[idx] == kUnassigned) {
        connectivity_owner_ids[id] = 0U;
        continue;
      }
      const auto cluster_idx = static_cast<std::size_t>(label[idx]);
      ++connectivity_member_counts[cluster_idx];
      if (connectivity_owner_ids[id] != clusters[cluster_idx].id ||
        connectivity_frames[id] + 1U != normal_filter_current_frame)
      {
        can_reuse_connectivity[cluster_idx] = 0U;
      }
      connectivity_owner_ids[id] = clusters[cluster_idx].id;
      connectivity_frames[id] = normal_filter_current_frame;
    }
    bool can_reuse_all_connectivity = true;
    for (std::size_t idx = 0U; idx < clusters.size(); ++idx) {
      // 所属集合の同一性確認を先行。所属変更面の木エッジ確認の省略。
      if (connectivity_member_counts[idx] != clusters[idx].connectivity_member_count) {
        can_reuse_connectivity[idx] = 0U;
      }
      if (connectivity_member_counts[idx] != 0U && can_reuse_connectivity[idx] == 0U) {
        can_reuse_all_connectivity = false;
      }
    }
    for (std::size_t idx = 0U; idx < label.size(); ++idx) {
      if (label[idx] == kUnassigned) {continue;}
      const auto id = node_ids[idx];
      const auto cluster_idx = static_cast<std::size_t>(label[idx]);
      if (can_reuse_connectivity[cluster_idx] == 0U || connectivity_parent[id] == id) {
        continue;
      }
      // 前回のCSR位置を優先し、位置変更時だけ親エッジを行内探索。
      auto &parent_slot = connectivity_parent_slots[id];
      const auto parent_idx = current_idx_by_id[connectivity_parent[id]];
      const auto begin = adjacency_offsets[idx];
      const auto end = adjacency_offsets[idx + 1U];
      if (parent_slot >= begin && parent_slot < end &&
        adjacency_values[parent_slot] == parent_idx)
      {
        continue;
      }
      parent_slot = begin;
      while (parent_slot < end && adjacency_values[parent_slot] != parent_idx) {
        ++parent_slot;
      }
      if (parent_slot == end) {
        can_reuse_connectivity[cluster_idx] = 0U;
        can_reuse_all_connectivity = false;
      }
    }
    // 全クラスタの連結確認済みフレームでは、成分配列の生成自体を省略。
    if (can_reuse_all_connectivity) {
      split_evidence.clear();
      for (std::size_t idx = 0U; idx < clusters.size(); ++idx) {
        auto &cluster = clusters[idx];
        cluster.disconnected_frames = 0U;
        cluster.connectivity_member_count = connectivity_member_counts[idx];
        cluster.has_connectivity_tree = connectivity_member_counts[idx] != 0U;
        statistics.num_connectivity_reused_clusters += cluster.has_connectivity_tree ? 1U : 0U;
      }
      return false;
    }

    component_of.assign(label.size(), kUnassigned);
    cluster_best_component.assign(clusters.size(), kUnassigned);
    cluster_best_size.assign(clusters.size(), 0U);
    // 既存BFSの訪問列を成分ごとに保持。最大成分の方向検査と全ノード再走査の省略。
    struct split_component
    {
      int cluster_idx;
      std::size_t num_nodes;
      std::size_t begin_node;
      std::uint16_t min_node_id;
    };
    std::vector<split_component> component_info;
    frontier.clear();

    for (std::size_t index = 0U; index < label.size(); ++index) {
      const int cluster_index = label[index];
      if (cluster_index == kUnassigned || component_of[index] != kUnassigned) {
        continue;
      }
      const auto cluster_idx = static_cast<std::size_t>(cluster_index);
      if (can_reuse_connectivity[cluster_idx] != 0U) {
        // 所属が同じで全域木が残る場合は、追加エッジや非木エッジの削除があっても連結。
        if (cluster_best_component[cluster_idx] == kUnassigned) {
          cluster_best_component[cluster_idx] = static_cast<int>(component_info.size());
          cluster_best_size[cluster_idx] = connectivity_member_counts[cluster_idx];
          component_info.push_back({cluster_index, connectivity_member_counts[cluster_idx], 0U, 0U});
          ++statistics.num_connectivity_reused_clusters;
        }
        component_of[index] = cluster_best_component[cluster_idx];
        continue;
      }
      const int component_index = static_cast<int>(component_info.size());
      const auto begin_node = frontier.size();
      frontier.push_back(index);
      component_of[index] = component_index;
      connectivity_parent[node_ids[index]] = node_ids[index];
      auto min_node_id = node_ids[index];
      for (std::size_t frontier_index = begin_node; frontier_index < frontier.size(); ++frontier_index) {
        const std::size_t current = frontier[frontier_index];
        min_node_id = std::min(min_node_id, node_ids[current]);
        for (std::size_t cursor = adjacency_offsets[current];
          cursor < adjacency_offsets[current + 1U]; ++cursor)
        {
          const std::size_t neighbour = adjacency_values[cursor];
          if (label[neighbour] != cluster_index || component_of[neighbour] != kUnassigned) {
            continue;
          }
          component_of[neighbour] = component_index;
          connectivity_parent[node_ids[neighbour]] = node_ids[current];
          frontier.push_back(neighbour);
        }
      }
      const std::size_t cluster = static_cast<std::size_t>(cluster_index);
      const auto num_nodes = frontier.size() - begin_node;
      if (num_nodes > cluster_best_size[cluster]) {
        cluster_best_size[cluster] = num_nodes;
        cluster_best_component[cluster] = component_index;
      }
      component_info.push_back({cluster_index, num_nodes, begin_node, min_node_id});
      statistics.num_connectivity_scanned_nodes += num_nodes;
    }

    // クラスタごとの成分数を数え、接続が切れた状態が続いた場合だけ実際に分割する。
    std::vector<std::size_t> component_count(clusters.size(), 0U);
    for (const auto &component : component_info) {
      ++component_count[static_cast<std::size_t>(component.cluster_idx)];
    }
    std::vector<std::uint8_t> allow_split(component_info.size(), 0U);
    bool has_confirmed_split = false;
    for (std::size_t index = 0U; index < clusters.size(); ++index) {
      clusters[index].has_connectivity_tree = component_count[index] == 1U;
      clusters[index].connectivity_member_count = connectivity_member_counts[index];
      if (component_count[index] <= 1U) {
        clusters[index].disconnected_frames = 0U;
        continue;
      }
      if (!options.enable_directional_split) {
        ++clusters[index].disconnected_frames;
      }
    }

    for (std::size_t idx = 0U; idx < component_info.size(); ++idx) {
      const auto &component = component_info[idx];
      const auto cluster_idx = static_cast<std::size_t>(component.cluster_idx);
      if (cluster_best_component[cluster_idx] == static_cast<int>(idx)) {
        continue;
      }
      if (options.enable_directional_split) {
        std::size_t num_conflict_nodes = 0U;
        const double min_conflict_nodes_th = std::max(
          static_cast<double>(options.min_split_conflict_nodes),
          options.min_split_conflict_ratio_th * static_cast<double>(component.num_nodes));
        for (std::size_t member = component.begin_node;
          member < component.begin_node + component.num_nodes &&
          static_cast<double>(num_conflict_nodes) < min_conflict_nodes_th; ++member)
        {
          const auto current = frontier[member];
          for (auto cursor = adjacency_offsets[current]; cursor < adjacency_offsets[current + 1U]; ++cursor) {
            // 未所属・別クラスタへの接続も対象。平方比較によるsqrt・除算・acosの省略。
            const Eigen::Vector3d edge = positions[adjacency_values[cursor]] - positions[current];
            const double height = clusters[cluster_idx].normal.dot(edge);
            if (height * height > split_edge_sin_squared * edge.squaredNorm()) {
              ++num_conflict_nodes;
              break;
            }
          }
        }
        if (static_cast<double>(num_conflict_nodes) < min_conflict_nodes_th) {
          ++statistics.num_split_retained_components;
          continue;
        }
        const auto key = (static_cast<std::uint64_t>(clusters[cluster_idx].id) << 32) |
          component.min_node_id;
        auto &evidence = split_evidence[key];
        evidence.num_frames = evidence.last_frame + 1U == normal_filter_current_frame ?
          evidence.num_frames + 1U : 1U;
        evidence.last_frame = normal_filter_current_frame;
        allow_split[idx] = evidence.num_frames > options.split_confirm_frames;
        if (allow_split[idx] == 0U) {++statistics.num_split_pending_components;}
      } else {
        allow_split[idx] = clusters[cluster_idx].disconnected_frames > options.split_confirm_frames;
      }
      has_confirmed_split = has_confirmed_split || allow_split[idx] != 0U;
    }
    for (auto it = split_evidence.begin(); it != split_evidence.end();) {
      it = it->second.last_frame == normal_filter_current_frame ? std::next(it) : split_evidence.erase(it);
    }
    if (has_confirmed_split) {
      for (auto &cluster : clusters) {
        if (cluster.disconnected_frames > options.split_confirm_frames) {cluster.disconnected_frames = 0U;}
      }
    }

    // 確認待ちカウンタ更新後、分割不要フレームのID投票・所属再割当を省略。
    if (!has_confirmed_split) {
      return false;
    }

    // 分かれた成分が前フレームに持っていたIDを多数決で調べる。分裂と再結合を
    // 繰り返す領域へ毎回新しいIDを振ると、そのたび別クラスタが生まれたように見える。
    std::unordered_map<std::size_t, std::unordered_map<std::uint32_t, std::size_t>> votes;
    for (std::size_t index = 0U; index < label.size(); ++index) {
      const int component_index = component_of[index];
      if (component_index == kUnassigned) {
        continue;
      }
      const auto owner_idx = owner_idx_by_node_id[node_ids[index]];
      if (owner_idx != kUnassigned) {
        ++votes[static_cast<std::size_t>(component_index)][clusters[owner_idx].id];
      }
    }
    std::unordered_set<std::uint32_t> ids_in_use;
    ids_in_use.reserve(clusters.size() * 2U + 1U);
    for (const ClusterState &cluster : clusters) {
      ids_in_use.insert(cluster.id);
    }

    // 最大成分だけが元のIDを引き継ぎ、他は前フレームのIDを取り戻すか、新規に振る。
    component_new_label.assign(component_info.size(), kUnassigned);
    for (std::size_t component_index = 0U; component_index < component_info.size();
      ++component_index)
    {
      const auto &component = component_info[component_index];
      const int cluster_index = component.cluster_idx;
      const std::size_t cluster = static_cast<std::size_t>(cluster_index);
      if (cluster_best_component[cluster] == static_cast<int>(component_index) ||
        allow_split[component_index] == 0U)
      {
        // まだ分割を確定させない成分は、元のクラスタに属したままにする。
        component_new_label[component_index] = cluster_index;
        continue;
      }
      ++statistics.split_cluster_count;
      changed = true;
      if (component.num_nodes < options.min_cluster_nodes) {
        continue;
      }
      ClusterState cluster_state = clusters[cluster];
      cluster_state.has_fit_changes = true;
      // 前フレームの所属で最も多かったIDが空いていれば、それを引き継ぐ。
      std::uint32_t reclaimed = 0U;
      std::size_t best_votes = 0U;
      const auto vote_it = votes.find(component_index);
      if (vote_it != votes.end()) {
        for (const auto &[candidate_id, count] : vote_it->second) {
          if (ids_in_use.count(candidate_id) != 0U) {
            continue;
          }
          if (count > best_votes || (count == best_votes && candidate_id < reclaimed)) {
            best_votes = count;
            reclaimed = candidate_id;
          }
        }
      }
      cluster_state.id = best_votes > 0U ? reclaimed : next_cluster_id++;
      ids_in_use.insert(cluster_state.id);
      cluster_state.weak_frames = 0U;
      component_new_label[component_index] = static_cast<int>(clusters.size());
      clusters.push_back(cluster_state);
    }
    if (options.enable_delta_statistics) {accumulators.resize(clusters.size());}
    for (std::size_t index = 0U; index < label.size(); ++index) {
      if (component_of[index] == kUnassigned) {
        continue;
      }
      const int new_label = component_new_label[static_cast<std::size_t>(component_of[index])];
      if (new_label != label[index]) {
        update_membership_statistics(index, label[index], new_label);
        clusters[label[index]].has_fit_changes = true;
        if (new_label != kUnassigned) {clusters[new_label].has_fit_changes = true;}
        label[index] = new_label;
      }
    }
    return changed;
  }

  // 同じ平面に乗っていて、実際にエッジでつながっているクラスタ同士を併合する。
  bool mergeClusters(ClusterStatistics &statistics)
  {
    if (clusters.size() < 2U) {
      fragment_merge_evidence.clear();
      return false;
    }
    adjacent_pair_counts.clear();
    for (std::size_t index = 0U; index < label.size(); ++index) {
      const int first_label = label[index];
      if (first_label == kUnassigned) {
        continue;
      }
      for (std::size_t cursor = adjacency_offsets[index];
        cursor < adjacency_offsets[index + 1U]; ++cursor)
      {
        const std::size_t neighbour = adjacency_values[cursor];
        if (neighbour <= index) {
          continue;
        }
        const int second_label = label[neighbour];
        if (second_label == kUnassigned || second_label == first_label) {
          continue;
        }
        auto &pair = adjacent_pair_counts[clusterPairKey(first_label, second_label)];
        ++pair.edges;
        if (pair.edges == 1U) {
          pair.single_edge_ratio = (positions[index] - positions[neighbour]).norm() /
            std::max(kEpsilon, std::min(spacings[index], spacings[neighbour]));
        }
        const auto first_idx = first_label < second_label ? index : neighbour;
        const auto second_idx = first_label < second_label ? neighbour : index;
        pair.first_contact.add(positions[first_idx], Eigen::Vector3d::Zero(), spacings[first_idx]);
        pair.second_contact.add(positions[second_idx], Eigen::Vector3d::Zero(), spacings[second_idx]);
      }
    }
    if (adjacent_pair_counts.empty()) {
      fragment_merge_evidence.clear();
      return false;
    }

    // 同一フレームの再フィット・生成で得た統計の再利用。全点の再累積・再求解の省略。
    merge_accumulators = accumulators;
    merge_fits = plane_fits;

    merge_sets.reset(clusters.size());
    const bool can_merge_single_edge =
      options.enable_fragment_merge && options.merge_connection_requirement == 2U;
    const auto can_fit_plane = [this](
      const PlaneAccumulator &side, const PlaneFit &fit, const double max_residual_ratio_th) {
        return side.rms_to_plane(fit) / effective_spacing(side.meanSpacing()) <= max_residual_ratio_th;
      };
    for (const auto &[key, pair] : adjacent_pair_counts) {
      ++statistics.merge_adjacent_pair_count;
      const std::size_t first = merge_sets.find(static_cast<std::size_t>(key & 0xFFFFFFFFULL));
      const std::size_t second = merge_sets.find(static_cast<std::size_t>(key >> 32));
      if (first == second) {
        continue;
      }
      const PlaneAccumulator &first_accumulator = merge_accumulators[first];
      const PlaneAccumulator &second_accumulator = merge_accumulators[second];
      const bool is_fragment_merge = pair.edges < options.merge_connection_requirement;
      // 接続が弱い場合だけ、短い1本接続としての救済可否を確認。
      if (is_fragment_merge && !(can_merge_single_edge && pair.edges == 1U &&
        pair.single_edge_ratio <= options.max_fragment_edge_ratio_th))
      {
        ++statistics.merge_insufficient_edge_pair_count;
        continue;
      }
      const PlaneFit &first_fit = merge_fits[first];
      const PlaneFit &second_fit = merge_fits[second];
      if (!first_fit.is_valid || !second_fit.is_valid) {
        ++statistics.merge_invalid_fit_pair_count;
        continue;
      }
      // 元の隣接対ではなく、統合済み成分全体に対する平面判定。
      PlaneAccumulator merged_fit = merge_accumulators[first];
      merged_fit.mergeFrom(merge_accumulators[second]);
      const PlaneFit union_fit = merged_fit.solve();
      const double union_spacing = std::max(merged_fit.meanSpacing(), kEpsilon);
      const double union_residual_ratio = union_fit.residual / effective_spacing(union_spacing);
      if (!union_fit.is_valid) {
        ++statistics.merge_invalid_fit_pair_count;
        continue;
      }
      // 統合後の面幅判定。各側の残差と接触部の整合性による誤吸収防止との併用。
      if (!has_plane_extent(union_fit, union_spacing, options.merge_min_planarity)) {
        ++statistics.merge_planarity_rejected_pair_count;
        continue;
      }
      // 通常統合と1本救済で共通の残差判定。接続の強さによる上限値だけの切替。
      const double max_pair_ratio_th = is_fragment_merge ?
        options.max_fragment_residual_ratio_th : std::numeric_limits<double>::infinity();
      const double max_cluster_ratio_th = std::min(options.max_normalized_cluster_residual, max_pair_ratio_th);
      if (union_residual_ratio > max_cluster_ratio_th) {
        ++statistics.merge_absolute_residual_rejected_pair_count;
        continue;
      }
      // つないだ結果、元より当てはめが悪くなっていないことも確かめる。
      // 残差の絶対値だけだと、小さなクラスタ同士は何をつないでも通ってしまう。
      const double first_residual_ratio =
        first_fit.residual / effective_spacing(first_accumulator.meanSpacing());
      const double second_residual_ratio =
        second_fit.residual / effective_spacing(second_accumulator.meanSpacing());
      const double allowed_residual = std::max(
        options.merge_residual_growth_ratio *
        std::max(first_residual_ratio, second_residual_ratio),
        options.merge_residual_growth_min_th);
      if (union_residual_ratio > allowed_residual) {
        ++statistics.merge_residual_growth_rejected_pair_count;
        continue;
      }
      // 各側全体から統合後平面への適合判定。大面の点数・間隔による小面の誤吸収防止。
      // 相手の元平面との相互照合は接続端点のみ。小面の法線誤差の遠方外挿の回避。
      // 連鎖統合時も現在の成分平面で接触部を評価。平行段差の重心移動による隠蔽防止。
      const double max_side_ratio_th = std::min(options.merge_smaller_side_residual_ratio, max_pair_ratio_th);
      if (!can_fit_plane(first_accumulator, union_fit, max_side_ratio_th) ||
        !can_fit_plane(second_accumulator, union_fit, max_side_ratio_th) ||
        !can_fit_plane(pair.first_contact, second_fit, max_side_ratio_th) ||
        !can_fit_plane(pair.second_contact, first_fit, max_side_ratio_th))
      {
        ++statistics.merge_smaller_side_rejected_pair_count;
        continue;
      }
      if (is_fragment_merge) {
        auto &evidence = fragment_merge_evidence[clusterPairKey(clusters[first].id, clusters[second].id)];
        if (evidence.last_frame != normal_filter_current_frame) {
          evidence.num_frames = evidence.last_frame + 1U == normal_filter_current_frame ?
            std::min(evidence.num_frames + 1U, options.min_fragment_merge_frames) : 1U;
          evidence.last_frame = normal_filter_current_frame;
        }
        if (evidence.num_frames < options.min_fragment_merge_frames) {
          ++statistics.num_fragment_pending_pairs;
          continue;
        }
        ++statistics.num_fragment_merged_clusters;
      }
      // 統合済み成分のノード数によるID選択。同数時は古いIDを優先。
      merge_sets.unite(first, second, [this](const std::size_t a, const std::size_t b) {
          if (merge_accumulators[a].count != merge_accumulators[b].count) {
            return merge_accumulators[a].count > merge_accumulators[b].count;
          }
          return clusters[a].id < clusters[b].id;
        });
      const std::size_t root_idx = merge_sets.find(first);
      merge_accumulators[root_idx] = merged_fit;
      merge_fits[root_idx] = union_fit;
    }

    // 接続消失・幾何不適合・ID変更で途切れた履歴の破棄。再出現時の確認回数の初期化。
    for (auto it = fragment_merge_evidence.begin(); it != fragment_merge_evidence.end();) {
      it = it->second.last_frame == normal_filter_current_frame ? std::next(it) :
        fragment_merge_evidence.erase(it);
    }

    std::size_t absorbed = 0U;
    for (std::size_t index = 0U; index < clusters.size(); ++index) {
      if (merge_sets.find(index) != index) {
        clusters[index].has_fit_changes = true;
        clusters[merge_sets.find(index)].has_fit_changes = true;
        ++absorbed;
      }
    }
    if (absorbed == 0U) {
      return false;
    }
    statistics.merged_cluster_count += absorbed;
    if (options.enable_delta_statistics) {
      accumulators = merge_accumulators;
      for (std::size_t idx = 0U; idx < clusters.size(); ++idx) {
        if (merge_sets.find(idx) != idx) {accumulators[idx].clear();}
      }
    }
    for (std::size_t index = 0U; index < label.size(); ++index) {
      if (label[index] == kUnassigned) {
        continue;
      }
      label[index] = static_cast<int>(merge_sets.find(static_cast<std::size_t>(label[index])));
    }
    return true;
  }

  // 条件を満たさない状態が続いたクラスタを捨て、添字を詰める。
  void cullClusters(ClusterStatistics &statistics)
  {
    // 存続の可否はメンバー数だけで決める。
    //
    // クラスタ全体の平面性や残差を削除条件に使ってはならない。面を平面に保つのは
    // ノード単位の逸脱判定の仕事であり、逸脱ノードを外せば集約値は自然に収まる。
    // 集約値で切ると、実在する面が集約残差の一時的な悪化だけで丸ごと消える。
    // 平面性と残差は、生成時と併合時の条件としてのみ使う。
    bool has_removed_cluster = false;
    for (ClusterState &cluster : clusters) {
      cluster.is_healthy = cluster.member_count >= options.min_cluster_nodes;
      if (cluster.is_healthy) {
        cluster.weak_frames = 0U;
        ++cluster.confirmed_frames;
      } else {
        ++cluster.weak_frames;
      }
      has_removed_cluster = has_removed_cluster ||
        cluster.member_count < 3U || cluster.weak_frames > options.weak_frame_allowance;
    }

    // 生存・確認カウンタ更新後、削除なしの場合のクラスタコピーと全所属の再採番を省略。
    if (!has_removed_cluster) {
      return;
    }

    remap.assign(clusters.size(), kUnassigned);
    kept_clusters.clear();
    kept_clusters.reserve(clusters.size());
    for (std::size_t index = 0U; index < clusters.size(); ++index) {
      const ClusterState &cluster = clusters[index];
      if (cluster.member_count < 3U || cluster.weak_frames > options.weak_frame_allowance) {
        ++statistics.removed_cluster_count;
        continue;
      }
      remap[index] = static_cast<int>(kept_clusters.size());
      if (options.enable_delta_statistics) {
        accumulators[kept_clusters.size()] = accumulators[index];
        plane_fits[kept_clusters.size()] = plane_fits[index];
      }
      kept_clusters.push_back(cluster);
    }
    for (std::size_t index = 0U; index < label.size(); ++index) {
      if (label[index] == kUnassigned) {
        continue;
      }
      label[index] = remap[static_cast<std::size_t>(label[index])];
    }
    clusters.swap(kept_clusters);
    if (options.enable_delta_statistics) {
      accumulators.resize(clusters.size());
      plane_fits.resize(clusters.size());
    }
  }

  // 出力メッセージを作り、同時に所属表を書き戻す。
  void buildOutput(const graph_view &, ClusterResult &result)
  {
    std::fill(owner_idx_by_node_id.begin(), owner_idx_by_node_id.end(), kUnassigned);

    member_lists.resize(clusters.size());
    for (std::size_t idx = 0U; idx < clusters.size(); ++idx) {
      member_lists[idx].clear();
      member_lists[idx].reserve(clusters[idx].member_count);
    }
    for (std::size_t index = 0U; index < label.size(); ++index) {
      const int cluster_index = label[index];
      if (cluster_index == kUnassigned) {
        continue;
      }
      const std::size_t cluster = static_cast<std::size_t>(cluster_index);
      member_lists[cluster].push_back(static_cast<std::uint32_t>(index));
      owner_idx_by_node_id[node_ids[index]] = cluster_index;
    }

    for (std::size_t cluster_index = 0U; cluster_index < clusters.size(); ++cluster_index) {
      ClusterState &state = clusters[cluster_index];
      // 出力の条件はノード数だけにする。平面性や残差が一時的に閾値をまたいだだけで
      // クラスタ全体を出力から外すと、内部では生きているのに表示だけが明滅する。
      // 実測では120フレームで同一IDの復活が178件あった。
      //
      // 平面性・残差は is_healthy として weak_frames にだけ効かせ、条件を満たさない状態が
      // weak_frame_allowance を超えて続いた場合に cullClusters が消す。こうすると
      // 消えるのは一度きりで、途中はメンバーが減って縮小するだけになる。
      if (state.member_count < options.min_cluster_nodes) {
        continue;
      }
      // 生まれてすぐ消えるクラスタは出力しない。確認できるまで待つ。
      if (state.confirmed_frames < options.birth_confirm_frames) {
        continue;
      }
      const std::vector<std::uint32_t> &members = member_lists[cluster_index];

      // OBBの接平面基底。unitOrthogonal() を毎フレーム独立に呼ぶと、法線がわずかに
      // 動いただけで軸の選び方が離散的に切り替わり、OBBの向きが不連続にジャンプする
      // (実測で15度超の飛びが1.1%)。前フレームの tangent_u を新しい法線平面へ射影して
      // 引き継ぐことで、法線自体の回転ぶんだけ滑らかに追従させる。射影後の長さが
      // 短すぎる場合(前フレームのtangent_uがほぼ法線と平行だった場合)だけ作り直す。
      Eigen::Vector3d tangent_u;
      if (state.has_tangent_u) {
        tangent_u = state.tangent_u - state.normal * state.normal.dot(state.tangent_u);
      }
      if (!state.has_tangent_u || tangent_u.squaredNorm() < kEpsilon) {
        tangent_u = state.normal.unitOrthogonal();
      } else {
        tangent_u.normalize();
      }
      state.tangent_u = tangent_u;
      state.has_tangent_u = true;
      const Eigen::Vector3d tangent_v = state.normal.cross(tangent_u).normalized();
      double min_u = std::numeric_limits<double>::infinity();
      double max_u = -std::numeric_limits<double>::infinity();
      double min_v = std::numeric_limits<double>::infinity();
      double max_v = -std::numeric_limits<double>::infinity();
      std::size_t terrain_nodes = 0U;
      std::size_t wall_nodes = 0U;

      ais_gng_msgs::msg::PlaneCluster cluster;
      cluster.id = state.id;
      cluster.node_indices.reserve(members.size());
      std::size_t num_support_points = 0U;
      for (const std::uint32_t member : members) {
        if (options.enable_support_edges) {
          num_support_points += adjacency_offsets[member + 1U] - adjacency_offsets[member];
        }
        const Eigen::Vector3d delta = positions[member] - state.centroid;
        const double u = delta.dot(tangent_u);
        const double v = delta.dot(tangent_v);
        min_u = std::min(min_u, u);
        max_u = std::max(max_u, u);
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);

        switch (node_labels[member]) {
          case ais_gng_msgs::msg::TopologicalMap::DEFAULT:
            ++result.statistics.clustered_default_node_count;
            break;
          case ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN:
            ++result.statistics.clustered_terrain_node_count;
            ++terrain_nodes;
            break;
          case ais_gng_msgs::msg::TopologicalMap::WALL:
            ++result.statistics.clustered_wall_node_count;
            ++wall_nodes;
            break;
          case ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT:
            ++result.statistics.clustered_unknown_node_count;
            break;
          case ais_gng_msgs::msg::TopologicalMap::HUMAN:
            ++result.statistics.clustered_human_node_count;
            break;
          case ais_gng_msgs::msg::TopologicalMap::CAR:
            ++result.statistics.clustered_car_node_count;
            break;
          default:
            ++result.statistics.clustered_other_node_count;
            break;
        }
        cluster.node_indices.push_back(member);
      }

      // 任意の互換出力。無効時は隣接走査・座標複製・出力配列の確保を省略。
      if (options.enable_support_edges) {
        cluster.support_edges.reserve(num_support_points);
        for (const std::uint32_t member : members) {
          for (std::size_t cursor = adjacency_offsets[member];
            cursor < adjacency_offsets[member + 1U]; ++cursor)
          {
            const std::uint32_t neighbour = adjacency_values[cursor];
            if (neighbour <= member || label[neighbour] != static_cast<int>(cluster_index)) {
              continue;
            }
            cluster.support_edges.push_back(pointMessage(positions[member]));
            cluster.support_edges.push_back(pointMessage(positions[neighbour]));
          }
        }
      }

      cluster.source_label = terrain_nodes + wall_nodes == 0U ?
        ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT :
        (terrain_nodes >= wall_nodes ?
        ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN :
        ais_gng_msgs::msg::TopologicalMap::WALL);
      cluster.centroid = pointMessage(state.centroid);
      cluster.normal = vectorMessage(state.normal);
      cluster.tangent_u = vectorMessage(tangent_u);
      cluster.tangent_v = vectorMessage(tangent_v);
      for (std::size_t row = 0U; row < 3U; ++row) {
        for (std::size_t column = 0U; column < 3U; ++column) {
          cluster.position_covariance[row * 3U + column] =
            static_cast<float>(state.covariance(row, column));
        }
      }
      cluster.area = 0.0F;
      cluster.extent_u = static_cast<float>(max_u - min_u);
      cluster.extent_v = static_cast<float>(max_v - min_v);
      cluster.local_spacing = static_cast<float>(state.spacing);
      cluster.planarity = static_cast<float>(state.planarity);
      cluster.residual_ratio =
        static_cast<float>(state.residual / std::max(state.spacing, kEpsilon));

      result.statistics.clustered_node_count += members.size();
      result.clusters.clusters.push_back(std::move(cluster));
    }
    result.statistics.cluster_count = result.clusters.clusters.size();
  }
};

Clusterizer::Clusterizer(ClusterOptions options)
: impl_(std::make_unique<Impl>(std::move(options)))
{}

Clusterizer::~Clusterizer() = default;

void Clusterizer::reset()
{
  impl_->has_stats_cache = false;
  impl_->retention_blocks.clear();
  // 採番は巻き戻さない。同じIDが別のクラスタを指すと、過去のIDを覚えている
  // 利用側が取り違えるため。
  impl_->clusters.clear();
  std::fill(impl_->owner_idx_by_node_id.begin(), impl_->owner_idx_by_node_id.end(), kUnassigned);
  impl_->fragment_merge_evidence.clear();
  impl_->split_evidence.clear();
  // フレーム番号の一致で「直前フレームの値か」を判定しているため、frame配列を
  // 番兵の0へ戻すだけで全エントリが無効化される(current_frameは巻き戻さない)。
  std::fill(
    impl_->normal_filter_frame.begin(), impl_->normal_filter_frame.end(), 0U);
}

void Clusterizer::setUseNodeRhoForSeedOrder(const bool enabled)
{
  impl_->options.use_node_rho_for_seed_order = enabled;
}

ClusterResult Clusterizer::update(const ais_gng_msgs::msg::TopologicalMap &map)
{
  return update(make_graph_view(map.nodes.data(), map.nodes.size(), map.edges.data(), map.edges.size()),
    map.header, map.frame_number);
}

ClusterResult Clusterizer::update(const graph_view &map, const std_msgs::msg::Header &header,
  const std::uint32_t frame_number)
{
  if ((map.num_nodes != 0U && (!map.nodes || !map.read_node)) ||
    (map.num_edge_values != 0U && !map.edges))
  {
    throw std::invalid_argument("graph_view has a null data pointer");
  }
  ClusterResult result;
  result.clusters.header = header;
  result.clusters.frame_number = frame_number;
  if (map.num_nodes == 0U) {
    impl_->has_stats_cache = false;
    impl_->retention_blocks.clear();
    impl_->clusters.clear();
    std::fill(impl_->owner_idx_by_node_id.begin(), impl_->owner_idx_by_node_id.end(), kUnassigned);
    impl_->fragment_merge_evidence.clear();
    impl_->split_evidence.clear();
    return result;
  }

  impl_->prepareFrame(map, result.statistics);
  impl_->carryOverLabels(map);
  impl_->refitClusters(true);
  bool labels_changed = impl_->releaseOutliers(result.statistics) != 0U;

  // 時系列方式はフレーム先頭の判定平面を固定した1パス。従来方式は反復間の再推定。
  const auto num_passes = impl_->options.enable_temporal_update ? 1U : impl_->options.maintenance_iter;
  for (std::size_t iter = 0U; iter < num_passes; ++iter) {
    if (labels_changed && !impl_->options.enable_temporal_update) {
      impl_->refitClusters();
      labels_changed = false;
    }
    ++result.statistics.maintenance_iter_num;
    if (impl_->maintenancePass(result.statistics, map) == 0U) {
      break;
    }
    labels_changed = true;
  }

  if (labels_changed) {
    impl_->refitClusters();
    // 固定モデルでの一括取り込み後の逸脱確認。再獲得反復なしの安全側解放。
    if (impl_->options.enable_temporal_update && impl_->releaseOutliers(result.statistics) != 0U) {
      impl_->refitClusters();
    }
  }
  impl_->birthClusters(result.statistics);
  const bool split_changed = impl_->splitClusters(result.statistics);
  if (split_changed) {
    impl_->refitClusters();
  }
  const bool merge_changed = impl_->mergeClusters(result.statistics);
  if (merge_changed) {
    impl_->refitClusters();
  }
  impl_->cullClusters(result.statistics);
  impl_->buildOutput(map, result);
  if (impl_->options.enable_delta_statistics) {
    // 次入力で再初期化する作業配列と履歴の交換。全ノードの履歴コピーの省略。
    impl_->stats_positions.swap(impl_->positions);
    impl_->stats_spacings.swap(impl_->spacings);
    impl_->stats_labels.swap(impl_->label);
    impl_->stats_node_ids.swap(impl_->node_ids);
    impl_->has_stats_cache = true;
  }
  return result;
}

}  // fuzzrobo::topological_plane::incremental 名前空間
