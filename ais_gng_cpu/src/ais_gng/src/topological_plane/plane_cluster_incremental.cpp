#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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

Eigen::Vector3d pointOf(const geometry_msgs::msg::Point32 &point)
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

std::uint64_t clusterPairKey(const int first, const int second)
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
    options.min_cluster_nodes = std::max<std::size_t>(3U, options.min_cluster_nodes);
    options.growth_residual_ratio = std::max(0.0, options.growth_residual_ratio);
    options.retention_residual_ratio = std::max(
      options.growth_residual_ratio, options.retention_residual_ratio);
    options.max_effective_spacing = std::max(kEpsilon, options.max_effective_spacing);
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

  // クラスタの永続状態。添字は1フレーム内でのみ有効で、同一性は id が担う。
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
  };

  ClusterOptions options;
  double normal_alignment_cos = 0.50;
  double retention_normal_alignment_cos = 0.087;

  // --- フレームをまたいで保持する状態 ---
  std::vector<ClusterState> clusters;
  std::unordered_map<std::uint16_t, std::uint32_t> owner_by_node_id;
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

  // --- フレームごとに再利用するバッファ ---
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> normals;
  std::vector<double> spacings;
  std::vector<double> seed_scores;
  std::vector<std::uint8_t> usable;
  std::vector<std::uint16_t> node_ids;
  std::vector<std::uint32_t> adjacency_offsets;
  std::vector<std::uint32_t> adjacency_values;
  std::vector<std::uint32_t> adjacency_cursor;
  std::vector<int> label;
  std::vector<int> next_label;
  std::vector<PlaneAccumulator> accumulators;
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
  // 隣接クラスタ対の、つながっているエッジ本数と境界ノードの平面ずれ。
  struct AdjacentPair
  {
    std::size_t edges = 0U;
  };
  std::unordered_map<std::uint64_t, AdjacentPair> adjacent_pair_counts;
  std::vector<int> remap;
  std::vector<ClusterState> kept_clusters;
  std::vector<std::vector<std::uint32_t>> member_lists;
  std::vector<PlaneAccumulator> merge_accumulators;

  // 隣接リスト(CSR)と局所量を作る。ここが唯一の O(N + E) 主走査になる。
  void prepareFrame(const ais_gng_msgs::msg::TopologicalMap &map, ClusterStatistics &statistics)
  {
    const std::size_t node_count = map.nodes.size();
    positions.assign(node_count, Eigen::Vector3d::Zero());
    normals.assign(node_count, Eigen::Vector3d::UnitZ());
    spacings.assign(node_count, 0.0);
    seed_scores.assign(node_count, 0.0);
    usable.assign(node_count, 0U);
    node_ids.assign(node_count, 0U);

    for (std::size_t index = 0U; index < node_count; ++index) {
      node_ids[index] = map.nodes[index].id;
      const Eigen::Vector3d position = pointOf(map.nodes[index].pos);
      if (!position.allFinite()) {
        continue;
      }
      positions[index] = position;
      usable[index] = 1U;
      ++statistics.valid_node_count;
    }

    // 次数を数えてから CSR を確定する。vector<vector> を毎フレーム作らない。
    adjacency_offsets.assign(node_count + 1U, 0U);
    const std::size_t edge_count = map.edges.size() / 2U;
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
    adjacency_values.assign(adjacency_offsets[node_count], 0U);
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

    // 局所ノード間隔と法線を決める。GNGが法線を持つ場合はそのまま使い、
    // 持たないノードだけ1ホップ近傍の共分散から補う。
    PlaneAccumulator local;
    for (std::size_t index = 0U; index < node_count; ++index) {
      if (usable[index] == 0U) {
        continue;
      }
      double spacing_sum = 0.0;
      std::size_t spacing_count = 0U;
      for (std::size_t cursor = adjacency_offsets[index];
        cursor < adjacency_offsets[index + 1U]; ++cursor)
      {
        const double dist = (positions[index] - positions[adjacency_values[cursor]]).norm();
        if (std::isfinite(dist) && dist > kEpsilon) {
          spacing_sum += dist;
          ++spacing_count;
        }
      }
      if (spacing_count == 0U) {
        usable[index] = 0U;
        continue;
      }
      spacings[index] = spacing_sum / static_cast<double>(spacing_count);

      const Eigen::Vector3d supplied = pointOf(map.nodes[index].normal);
      if (supplied.allFinite() && supplied.squaredNorm() > kEpsilon) {
        normals[index] = supplied.normalized();
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
        const double rho = static_cast<double>(map.nodes[index].rho);
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
  void carryOverLabels(const ais_gng_msgs::msg::TopologicalMap &map)
  {
    const std::size_t node_count = map.nodes.size();
    label.assign(node_count, kUnassigned);
    if (owner_by_node_id.empty() || clusters.empty()) {
      return;
    }
    std::unordered_map<std::uint32_t, int> index_by_cluster_id;
    index_by_cluster_id.reserve(clusters.size() * 2U + 1U);
    for (std::size_t index = 0U; index < clusters.size(); ++index) {
      index_by_cluster_id.emplace(clusters[index].id, static_cast<int>(index));
    }
    for (std::size_t index = 0U; index < node_count; ++index) {
      if (usable[index] == 0U) {
        continue;
      }
      const auto owner_it = owner_by_node_id.find(map.nodes[index].id);
      if (owner_it == owner_by_node_id.end()) {
        continue;
      }
      const auto cluster_it = index_by_cluster_id.find(owner_it->second);
      if (cluster_it != index_by_cluster_id.end()) {
        label[index] = cluster_it->second;
      }
    }
  }

  // 現在の所属から各クラスタの平面を解き直す。
  void refitClusters()
  {
    accumulators.assign(clusters.size(), PlaneAccumulator{});
    for (std::size_t index = 0U; index < label.size(); ++index) {
      const int cluster_index = label[index];
      if (cluster_index == kUnassigned) {
        continue;
      }
      accumulators[static_cast<std::size_t>(cluster_index)].add(
        positions[index], normals[index], spacings[index]);
    }
    for (std::size_t index = 0U; index < clusters.size(); ++index) {
      ClusterState &cluster = clusters[index];
      const PlaneAccumulator &accumulator = accumulators[index];
      cluster.member_count = accumulator.count;
      if (accumulator.count < 3U) {
        continue;
      }
      PlaneFit fit = accumulator.solve();
      if (!fit.is_valid) {
        continue;
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

  // ノードから見た、あるクラスタ平面への正規化距離。
  double fitScore(const std::size_t cluster_index, const std::size_t node_index) const
  {
    const ClusterState &cluster = clusters[cluster_index];
    const double spacing = std::min(
      std::max({cluster.spacing, spacings[node_index], kEpsilon}),
      options.max_effective_spacing);
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
    for (std::size_t index = 0U; index < label.size(); ++index) {
      const int cluster_index = label[index];
      if (cluster_index == kUnassigned) {
        continue;
      }
      const std::size_t cluster = static_cast<std::size_t>(cluster_index);
      if (clusters[cluster].member_count < 3U) {
        continue;
      }
      if (!is_normal_aligned(cluster, index)) {
        label[index] = kUnassigned;
        ++statistics.released_node_count;
        ++released;
        continue;
      }
      // 逸脱判定は常に効かせる。接続本数はしきい値を緩めるだけで、判定そのものを
      // 無効にはしない。無効にすると平面から離れても誰も外れず、集約値だけが
      // 悪化していく。
      std::size_t same_cluster_neighbours = 0U;
      if (options.enable_multi_edge_dist_relaxation) {
        for (std::size_t cursor = adjacency_offsets[index];
          cursor < adjacency_offsets[index + 1U]; ++cursor)
        {
          if (label[adjacency_values[cursor]] == cluster_index) {
            ++same_cluster_neighbours;
          }
        }
      }
      const double retention_limit = same_cluster_neighbours >= 2U ?
        options.retention_residual_ratio : options.growth_residual_ratio;
      if (fitScore(cluster, index) <= retention_limit) {
        continue;
      }
      label[index] = kUnassigned;
      ++statistics.released_node_count;
      ++released;
    }
    return released;
  }

  // 保守点検の1回分。取り込みと移動を同時に判定する。
  //
  // 候補クラスタ C の資格は、そのノードの隣接のうち「すでに C に所属している
  // ノード」の数で決める。1本のエッジだけで所属が漏れ出すのを防ぐ条件である。
  //
  // 判定は前フレームの label を読み、結果は next_label へ書く。同じパス内で
  // 走査順に影響されないため、結果が決定的になる。
  std::size_t maintenancePass(ClusterStatistics &statistics)
  {
    next_label = label;
    neighbour_counts.assign(clusters.size(), 0U);
    std::size_t changed = 0U;

    for (std::size_t index = 0U; index < label.size(); ++index) {
      if (usable[index] == 0U) {
        continue;
      }
      touched_clusters.clear();
      for (std::size_t cursor = adjacency_offsets[index];
        cursor < adjacency_offsets[index + 1U]; ++cursor)
      {
        const int neighbour_label = label[adjacency_values[cursor]];
        if (neighbour_label == kUnassigned) {
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

      const int current_label = label[index];

      int best_label = kUnassigned;
      double best_score = std::numeric_limits<double>::infinity();
      for (const int candidate_label : touched_clusters) {
        if (candidate_label == current_label) {
          continue;
        }
        const std::size_t cluster = static_cast<std::size_t>(candidate_label);
        if (clusters[cluster].member_count < 3U ||
          neighbour_counts[cluster] < options.connection_requirement)
        {
          continue;
        }
        if (!is_normal_aligned(cluster, index)) {
          continue;
        }
        const double score = fitScore(cluster, index);
        // 同一クラスタのノードから複数のエッジが伸びているなら、平面までの距離は
        // 問わない。距離は候補が複数あるときの優先順位にだけ使う。
        //
        // 接続本数が足りていれば、しきい値を保持と同じ上限まで緩める。
        // 逸脱ノードまで取り込まないよう、上限そのものは残す。
        const bool is_multi_edge_dist_relaxed =
          options.enable_multi_edge_dist_relaxation &&
          neighbour_counts[cluster] >= 2U &&
          score <= options.retention_residual_ratio;
        if (!is_multi_edge_dist_relaxed && score > options.growth_residual_ratio) {
          continue;
        }
        // 同点時は添字の小さいクラスタを選び、フレーム間で結果を安定させる。
        if (score < best_score - kEpsilon ||
          (best_label != kUnassigned && std::abs(score - best_score) <= kEpsilon &&
          candidate_label < best_label))
        {
          best_score = score;
          best_label = candidate_label;
        }
      }

      if (best_label != kUnassigned) {
        if (current_label == kUnassigned) {
          next_label[index] = best_label;
          ++statistics.absorbed_node_count;
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
              next_label[index] = best_label;
              ++statistics.migrated_node_count;
              ++changed;
            }
          }
        }
      }

      for (const int candidate_label : touched_clusters) {
        neighbour_counts[static_cast<std::size_t>(candidate_label)] = 0U;
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
      if (usable[index] != 0U && label[index] == kUnassigned) {
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
      if (fit.planarity < options.min_growth_planarity) {
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
          const double spacing = std::max(
            {accumulator.meanSpacing(), spacings[neighbour], kEpsilon});
          if (std::abs(fit.normal.dot(positions[neighbour] - fit.centroid)) / spacing >
            options.growth_residual_ratio)
          {
            continue;
          }
          // 生成中のクラスタに所属済みの隣接がいくつあるかを数える。
          std::size_t attached = 0U;
          for (std::size_t back_cursor = adjacency_offsets[neighbour];
            back_cursor < adjacency_offsets[neighbour + 1U]; ++back_cursor)
          {
            if (visit_marks[adjacency_values[back_cursor]] == visit_generation) {
              ++attached;
            }
          }
          if (attached < options.birth_neighbor_requirement) {
            continue;
          }
          accumulator.add(positions[neighbour], normals[neighbour], spacings[neighbour]);
          visit_marks[neighbour] = visit_generation;
          frontier.push_back(neighbour);
          if (frontier.size() >= refit_th) {
            const PlaneFit updated = accumulator.solve();
          if (updated.is_valid) {
              fit = updated;
              // サイズが倍になるたびに形を確認する。平面なら倍化しても
              // 第2固有値の比は保たれるが、鎖状に伸び始めるとここで落ちる。
              if (updated.planarity < options.min_growth_planarity) {
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
        final_fit.residual / spacing > options.max_normalized_cluster_residual)
      {
        rejectFrontier();
        continue;
      }
      if (final_fit.planarity < options.min_cluster_planarity) {
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
      for (const std::size_t member : frontier) {
        label[member] = new_label;
      }
      ++statistics.born_cluster_count;
    }
  }

  // 障害物などで接続が切れたクラスタを、連結成分ごとに分ける。
  bool splitClusters(ClusterStatistics &statistics)
  {
    if (clusters.empty()) {
      return false;
    }
    bool changed = false;
    component_of.assign(label.size(), kUnassigned);
    cluster_best_component.assign(clusters.size(), kUnassigned);
    cluster_best_size.assign(clusters.size(), 0U);

    // 成分ごとの所属クラスタとサイズ。ノード列は component_of から引き直す。
    std::vector<std::pair<int, std::size_t>> component_info;

    for (std::size_t index = 0U; index < label.size(); ++index) {
      const int cluster_index = label[index];
      if (cluster_index == kUnassigned || component_of[index] != kUnassigned) {
        continue;
      }
      const int component_index = static_cast<int>(component_info.size());
      frontier.clear();
      frontier.push_back(index);
      component_of[index] = component_index;
      for (std::size_t frontier_index = 0U; frontier_index < frontier.size(); ++frontier_index) {
        const std::size_t current = frontier[frontier_index];
        for (std::size_t cursor = adjacency_offsets[current];
          cursor < adjacency_offsets[current + 1U]; ++cursor)
        {
          const std::size_t neighbour = adjacency_values[cursor];
          if (label[neighbour] != cluster_index || component_of[neighbour] != kUnassigned) {
            continue;
          }
          component_of[neighbour] = component_index;
          frontier.push_back(neighbour);
        }
      }
      const std::size_t cluster = static_cast<std::size_t>(cluster_index);
      if (frontier.size() > cluster_best_size[cluster]) {
        cluster_best_size[cluster] = frontier.size();
        cluster_best_component[cluster] = component_index;
      }
      component_info.emplace_back(cluster_index, frontier.size());
    }

    // クラスタごとの成分数を数え、接続が切れた状態が続いた場合だけ実際に分割する。
    std::vector<std::size_t> component_count(clusters.size(), 0U);
    for (const auto &[cluster_index, component_size] : component_info) {
      static_cast<void>(component_size);
      ++component_count[static_cast<std::size_t>(cluster_index)];
    }
    std::vector<std::uint8_t> allow_split(clusters.size(), 0U);
    for (std::size_t index = 0U; index < clusters.size(); ++index) {
      if (component_count[index] <= 1U) {
        clusters[index].disconnected_frames = 0U;
        continue;
      }
      ++clusters[index].disconnected_frames;
      if (clusters[index].disconnected_frames > options.split_confirm_frames) {
        allow_split[index] = 1U;
        clusters[index].disconnected_frames = 0U;
      }
    }

    // 分かれた成分が前フレームに持っていたIDを多数決で調べる。分裂と再結合を
    // 繰り返す領域へ毎回新しいIDを振ると、そのたび別クラスタが生まれたように見える。
    std::unordered_map<std::size_t, std::unordered_map<std::uint32_t, std::size_t>> votes;
    if (!owner_by_node_id.empty()) {
      for (std::size_t index = 0U; index < label.size(); ++index) {
        const int component_index = component_of[index];
        if (component_index == kUnassigned) {
          continue;
        }
        const auto owner_it = owner_by_node_id.find(node_ids[index]);
        if (owner_it != owner_by_node_id.end()) {
          ++votes[static_cast<std::size_t>(component_index)][owner_it->second];
        }
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
      const auto [cluster_index, component_size] = component_info[component_index];
      const std::size_t cluster = static_cast<std::size_t>(cluster_index);
      if (cluster_best_component[cluster] == static_cast<int>(component_index) ||
        allow_split[cluster] == 0U)
      {
        // まだ分割を確定させない成分は、元のクラスタに属したままにする。
        component_new_label[component_index] = cluster_index;
        continue;
      }
      ++statistics.split_cluster_count;
      changed = true;
      if (component_size < options.min_cluster_nodes) {
        continue;
      }
      ClusterState cluster_state = clusters[cluster];
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
    for (std::size_t index = 0U; index < label.size(); ++index) {
      if (component_of[index] == kUnassigned) {
        continue;
      }
      label[index] = component_new_label[static_cast<std::size_t>(component_of[index])];
    }
    return changed;
  }

  // 同じ平面に乗っていて、実際にエッジでつながっているクラスタ同士を併合する。
  bool mergeClusters(ClusterStatistics &statistics)
  {
    if (clusters.size() < 2U) {
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
        ++adjacent_pair_counts[clusterPairKey(first_label, second_label)].edges;
      }
    }
    if (adjacent_pair_counts.empty()) {
      return false;
    }

    // 各クラスタの累積共分散を1回だけ作る。候補対ごとのノード走査を避け、
    // 結合平面を一定量の累積値合成と3x3固有値分解で判定する。
    // メンバーの索引一覧も同じ1パスで作り、少数側単体をunion平面へ当てはめる
    // チェックに使う(buildOutputのmember_listsとは別に、ここで作り直す)。
    merge_accumulators.assign(clusters.size(), PlaneAccumulator{});
    member_lists.assign(clusters.size(), std::vector<std::uint32_t>{});
    for (std::size_t index = 0U; index < label.size(); ++index) {
      if (label[index] != kUnassigned) {
        const auto cluster_index = static_cast<std::size_t>(label[index]);
        merge_accumulators[cluster_index].add(
          positions[index], normals[index], spacings[index]);
        member_lists[cluster_index].push_back(static_cast<std::uint32_t>(index));
      }
    }

    merge_sets.reset(clusters.size());
    for (const auto &[key, pair] : adjacent_pair_counts) {
      ++statistics.merge_adjacent_pair_count;
      if (pair.edges < options.merge_connection_requirement) {
        ++statistics.merge_insufficient_edge_pair_count;
        continue;
      }
      const std::size_t first = static_cast<std::size_t>(key & 0xFFFFFFFFULL);
      const std::size_t second = static_cast<std::size_t>(key >> 32);
      const ClusterState &first_cluster = clusters[first];
      const ClusterState &second_cluster = clusters[second];
      if (first_cluster.member_count < 3U || second_cluster.member_count < 3U) {
        continue;
      }
      // 結合したら1枚の平面として成立するかを、そのまま確かめる。
      //
      // 局所クラスタ法線は小さいパッチでは揺れるため、事前の法線一致では棄却しない。
      // GNG edgeで接続された対を、結合後の共分散平面そのもので判定する。
      PlaneAccumulator merged_fit = merge_accumulators[first];
      merged_fit.mergeFrom(merge_accumulators[second]);
      const PlaneFit union_fit = merged_fit.solve();
      const double union_spacing = std::max(merged_fit.meanSpacing(), kEpsilon);
      if (!union_fit.is_valid) {
        ++statistics.merge_invalid_fit_pair_count;
        continue;
      }
      // 確定条件(min_cluster_planarity)より緩い専用しきい値を使う。誤併合を防ぐのは
      // 次の絶対残差条件の役目で、平面性は鎖状のまま結合されるのだけを防げばよい。
      if (union_fit.planarity < options.merge_min_planarity) {
        ++statistics.merge_planarity_rejected_pair_count;
        continue;
      }
      if (union_fit.residual / union_spacing > options.max_normalized_cluster_residual) {
        ++statistics.merge_absolute_residual_rejected_pair_count;
        continue;
      }
      // つないだ結果、元より当てはめが悪くなっていないことも確かめる。
      // 残差の絶対値だけだと、小さなクラスタ同士は何をつないでも通ってしまう。
      const double first_residual_ratio =
        first_cluster.residual / std::max(first_cluster.spacing, kEpsilon);
      const double second_residual_ratio =
        second_cluster.residual / std::max(second_cluster.spacing, kEpsilon);
      const double allowed_residual = std::max(
        options.merge_residual_growth_ratio *
        std::max(first_residual_ratio, second_residual_ratio),
        options.merge_residual_growth_min_th);
      if (union_fit.residual / union_spacing > allowed_residual) {
        ++statistics.merge_residual_growth_rejected_pair_count;
        continue;
      }
      // union_fitは全メンバーの平均統計なので、大きい側に小さい側を混ぜても
      // 全体の当てはめはほとんど動かず、小さい側だけが実際にはunion平面から
      // 離れているケースを見逃す。少数側の点群をunion平面へ個別に当てはめ、
      // RMS残差比で確かめる。
      {
        const std::size_t smaller = first_cluster.member_count <= second_cluster.member_count ?
          first : second;
        const ClusterState &smaller_cluster = clusters[smaller];
        const auto &smaller_members = member_lists[smaller];
        double sum_sq = 0.0;
        for (const std::uint32_t member : smaller_members) {
          const double d = union_fit.normal.dot(positions[member] - union_fit.centroid);
          sum_sq += d * d;
        }
        const double smaller_rms = smaller_members.empty() ?
          0.0 : std::sqrt(sum_sq / static_cast<double>(smaller_members.size()));
        const double smaller_spacing = std::min(
          std::max({smaller_cluster.spacing, union_spacing, kEpsilon}),
          options.max_effective_spacing);
        if (smaller_rms / smaller_spacing > options.merge_smaller_side_residual_ratio) {
          ++statistics.merge_smaller_side_rejected_pair_count;
          continue;
        }
      }
      // ノード数の多い側のIDを残す。少数側へ吸収されると、画面上は大きなクラスタが
      // 消えて別IDへ置き換わったように見える。同数なら若いID(古い方)を残す。
      merge_sets.unite(first, second, [this](const std::size_t a, const std::size_t b) {
          if (clusters[a].member_count != clusters[b].member_count) {
            return clusters[a].member_count > clusters[b].member_count;
          }
          return clusters[a].id < clusters[b].id;
        });
    }

    std::size_t absorbed = 0U;
    for (std::size_t index = 0U; index < clusters.size(); ++index) {
      if (merge_sets.find(index) != index) {
        ++absorbed;
      }
    }
    if (absorbed == 0U) {
      return false;
    }
    statistics.merged_cluster_count += absorbed;
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
    for (ClusterState &cluster : clusters) {
      cluster.is_healthy = cluster.member_count >= options.min_cluster_nodes;
      if (cluster.is_healthy) {
        cluster.weak_frames = 0U;
        ++cluster.confirmed_frames;
      } else {
        ++cluster.weak_frames;
      }
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
      kept_clusters.push_back(cluster);
    }
    for (std::size_t index = 0U; index < label.size(); ++index) {
      if (label[index] == kUnassigned) {
        continue;
      }
      label[index] = remap[static_cast<std::size_t>(label[index])];
    }
    clusters.swap(kept_clusters);
  }

  // 出力メッセージを作り、同時に所属表を書き戻す。
  void buildOutput(const ais_gng_msgs::msg::TopologicalMap &map, ClusterResult &result)
  {
    owner_by_node_id.clear();
    owner_by_node_id.reserve(label.size());

    member_lists.assign(clusters.size(), std::vector<std::uint32_t>{});
    for (std::size_t index = 0U; index < label.size(); ++index) {
      const int cluster_index = label[index];
      if (cluster_index == kUnassigned) {
        continue;
      }
      const std::size_t cluster = static_cast<std::size_t>(cluster_index);
      member_lists[cluster].push_back(static_cast<std::uint32_t>(index));
      owner_by_node_id[map.nodes[index].id] = clusters[cluster].id;
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

      ais_gng_msgs::msg::PlanarCluster cluster;
      cluster.id = state.id;
      cluster.node_indices.reserve(members.size());
      for (const std::uint32_t member : members) {
        const Eigen::Vector3d delta = positions[member] - state.centroid;
        const double u = delta.dot(tangent_u);
        const double v = delta.dot(tangent_v);
        min_u = std::min(min_u, u);
        max_u = std::max(max_u, u);
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);

        switch (map.nodes[member].label) {
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

      // 実際に観測されたGNGエッジだけを残す。凸包は推定しない。
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
  // 採番は巻き戻さない。同じIDが別のクラスタを指すと、過去のIDを覚えている
  // 利用側が取り違えるため。
  impl_->clusters.clear();
  impl_->owner_by_node_id.clear();
  // フレーム番号の一致で「直前フレームの値か」を判定しているため、frame配列を
  // 番兵の0へ戻すだけで全エントリが無効化される(current_frameは巻き戻さない)。
  std::fill(
    impl_->normal_filter_frame.begin(), impl_->normal_filter_frame.end(), 0U);
}

ClusterResult Clusterizer::update(const ais_gng_msgs::msg::TopologicalMap &map)
{
  ClusterResult result;
  result.clusters.header = map.header;
  result.clusters.frame_number = map.frame_number;
  if (map.nodes.empty()) {
    impl_->clusters.clear();
    impl_->owner_by_node_id.clear();
    return result;
  }

  impl_->prepareFrame(map, result.statistics);
  impl_->carryOverLabels(map);
  impl_->refitClusters();
  bool labels_changed = impl_->releaseOutliers(result.statistics) != 0U;

  // 解放と取り込みで平面が動くため、再フィットを挟みながら数回だけ回す。
  // 変化が止まった時点で抜けるので、定常状態では1回で終わる。
  for (std::size_t iter = 0U; iter < impl_->options.maintenance_iter; ++iter) {
    if (labels_changed) {
      impl_->refitClusters();
      labels_changed = false;
    }
    ++result.statistics.maintenance_iter_num;
    if (impl_->maintenancePass(result.statistics) == 0U) {
      break;
    }
    labels_changed = true;
  }

  if (labels_changed) {
    impl_->refitClusters();
  }
  impl_->birthClusters(result.statistics);
  const bool split_changed = impl_->splitClusters(result.statistics);
  const bool merge_changed = impl_->mergeClusters(result.statistics);
  if (split_changed || merge_changed) {
    impl_->refitClusters();
  }
  impl_->cullClusters(result.statistics);
  impl_->buildOutput(map, result);
  return result;
}

}  // fuzzrobo::topological_plane::incremental 名前空間
