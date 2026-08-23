#include <ais_gng/topological_plane/topological_plane_cluster.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{

constexpr double kEpsilon = 1.0e-9;

struct PlaneFit
{
  bool valid = false;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double planarity = 0.0;
  double residual = 0.0;
};

struct NodeFeature
{
  bool valid = false;
  bool planar = false;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double local_spacing = 0.0;
  double planarity = 0.0;
};

struct GraphEdge
{
  std::size_t first = 0U;
  std::size_t second = 0U;
  double length = 0.0;
};

struct SurfaceSupport
{
  std::vector<std::pair<std::size_t, std::size_t>> edges;

  bool isSurfaceLike(const std::size_t node_count, const double min_edge_per_node) const
  {
    // 一本鎖・閉路だけでは、内部エッジ数はノード数を超えない。面状のGNG接続だけを
    // 残すため、ノード数に対する内部エッジ数で判定する。
    return static_cast<double>(edges.size()) >=
      min_edge_per_node * static_cast<double>(node_count);
  }
};

struct SurfaceSupportWorkspace
{
  explicit SurfaceSupportWorkspace(const std::size_t node_count)
  : marks(node_count, 0U)
  {
  }

  std::vector<std::uint32_t> marks;
  std::uint32_t generation = 0U;
};

class DisjointSet
{
public:
  explicit DisjointSet(std::size_t size)
  : parent_(size), rank_(size, 0U)
  {
    std::iota(parent_.begin(), parent_.end(), 0U);
  }

  std::size_t find(std::size_t index)
  {
    while (parent_[index] != index) {
      parent_[index] = parent_[parent_[index]];
      index = parent_[index];
    }
    return index;
  }

  void unite(std::size_t first, std::size_t second)
  {
    first = find(first);
    second = find(second);
    if (first == second) {
      return;
    }
    if (rank_[first] < rank_[second]) {
      std::swap(first, second);
    }
    parent_[second] = first;
    if (rank_[first] == rank_[second]) {
      ++rank_[first];
    }
  }

private:
  std::vector<std::size_t> parent_;
  std::vector<std::uint8_t> rank_;
};

Eigen::Vector3d pointOf(const geometry_msgs::msg::Point32 &point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

Eigen::Vector3d normalOf(const geometry_msgs::msg::Point32 &normal)
{
  return Eigen::Vector3d(normal.x, normal.y, normal.z);
}

bool finiteVector(const Eigen::Vector3d &value)
{
  return value.allFinite();
}

PlaneFit fitPlane(const std::vector<Eigen::Vector3d> &points)
{
  PlaneFit fit;
  if (points.size() < 3U) {
    return fit;
  }

  for (const auto &point : points) {
    fit.centroid += point;
  }
  fit.centroid /= static_cast<double>(points.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto &point : points) {
    const Eigen::Vector3d delta = point - fit.centroid;
    covariance.noalias() += delta * delta.transpose();
  }
  covariance /= static_cast<double>(points.size());

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return fit;
  }
  const Eigen::Vector3d eigenvalues = solver.eigenvalues();
  const double largest = eigenvalues.z();
  if (!std::isfinite(largest) || largest <= kEpsilon) {
    return fit;
  }

  fit.normal = solver.eigenvectors().col(0).normalized();
  // 厚みは下の `residual` で別に評価する。ここでは線分状の成分だけを除外する。
  // 平方根を取ることで、細長くても完全に平坦な壁・床パッチに対しても、既存の
  // 0.45 という閾値が意味を持つようにする。面内の二方向は同じ長さでなくてよい。
  fit.planarity = std::sqrt(std::clamp(eigenvalues.y() / largest, 0.0, 1.0));
  fit.residual = std::sqrt(std::max(0.0, eigenvalues.x()));
  fit.valid = finiteVector(fit.normal) && std::isfinite(fit.planarity) &&
    std::isfinite(fit.residual);
  return fit;
}

void orientNormal(Eigen::Vector3d &normal, const Eigen::Vector3d &reference)
{
  if (reference.squaredNorm() > kEpsilon) {
    if (normal.dot(reference) < 0.0) {
      normal = -normal;
    }
    return;
  }

  // 固有ベクトルの符号は一意でない。入力地図が法線を与えない場合でも、
  // 決定的な符号にしてマーカー矢印のちらつきを防ぐ。
  Eigen::Index dominant_axis = 0;
  normal.cwiseAbs().maxCoeff(&dominant_axis);
  if (normal[dominant_axis] < 0.0) {
    normal = -normal;
  }
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

SurfaceSupport collectSurfaceSupport(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const std::vector<std::size_t> &members,
  SurfaceSupportWorkspace &workspace)
{
  if (++workspace.generation == 0U) {
    std::fill(workspace.marks.begin(), workspace.marks.end(), 0U);
    workspace.generation = 1U;
  }

  for (const std::size_t node_index : members) {
    if (node_index >= map.nodes.size() || workspace.marks[node_index] == workspace.generation) {
      continue;
    }
    workspace.marks[node_index] = workspace.generation;
  }

  SurfaceSupport support;
  for (std::size_t edge_index = 0U; edge_index + 1U < map.edges.size(); edge_index += 2U) {
    const std::size_t first = map.edges[edge_index];
    const std::size_t second = map.edges[edge_index + 1U];
    if (first >= map.nodes.size() || second >= map.nodes.size() ||
      workspace.marks[first] != workspace.generation ||
      workspace.marks[second] != workspace.generation)
    {
      continue;
    }
    support.edges.emplace_back(first, second);
  }
  return support;
}

using NodeKey = std::uint64_t;

NodeKey nodeKey(const ais_gng_msgs::msg::TopologicalNode &node)
{
  // `frame` はGNG実装でノード更新時に更新されるため、生成時の識別子ではない。
  // `id` は永続的なGNGノードスロットなので、短時間のパッチ追跡に使える唯一のIDである。
  return static_cast<NodeKey>(node.id);
}

Eigen::Vector3d vectorOf(const geometry_msgs::msg::Vector3 &vector)
{
  return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

std::size_t overlapCount(const std::vector<NodeKey> &first, const std::vector<NodeKey> &second)
{
  std::size_t overlap = 0U;
  std::size_t first_index = 0U;
  std::size_t second_index = 0U;
  while (first_index < first.size() && second_index < second.size()) {
    if (first[first_index] == second[second_index]) {
      ++overlap;
      ++first_index;
      ++second_index;
    } else if (first[first_index] < second[second_index]) {
      ++first_index;
    } else {
      ++second_index;
    }
  }
  return overlap;
}

}  // 無名名前空間

namespace fuzzrobo::topological_plane
{

TopologicalPlaneClusterExtractor::TopologicalPlaneClusterExtractor(PlaneClusterOptions options)
: options_(std::move(options))
{
  options_.min_cluster_planarity = std::clamp(options_.min_cluster_planarity, 0.0, 1.0);
  options_.max_normalized_cluster_residual = std::max(
    0.0, options_.max_normalized_cluster_residual);
  options_.min_cluster_nodes = std::max<std::size_t>(3U, options_.min_cluster_nodes);
  options_.min_normal_alignment_cos = std::clamp(options_.min_normal_alignment_cos, 0.0, 1.0);
  options_.max_growth_dist_ratio = std::max(0.0, options_.max_growth_dist_ratio);
  options_.max_seed_plane_dist_ratio = std::max(0.0, options_.max_seed_plane_dist_ratio);
  options_.min_cluster_edge_per_node = std::max(0.0, options_.min_cluster_edge_per_node);
}

PlaneClusterExtractionResult TopologicalPlaneClusterExtractor::extract(
  const ais_gng_msgs::msg::TopologicalMap &map) const
{
  PlaneClusterExtractionResult result;
  result.clusters.header = map.header;
  result.clusters.frame_number = map.frame_number;

  const std::size_t node_count = map.nodes.size();
  std::vector<std::vector<std::size_t>> adjacency(node_count);
  for (std::size_t edge_index = 0; edge_index + 1U < map.edges.size(); edge_index += 2U) {
    const std::size_t first = map.edges[edge_index];
    const std::size_t second = map.edges[edge_index + 1U];
    if (first >= node_count || second >= node_count || first == second) {
      continue;
    }
    adjacency[first].push_back(second);
    adjacency[second].push_back(first);
  }
  for (auto &neighbours : adjacency) {
    std::sort(neighbours.begin(), neighbours.end());
    neighbours.erase(std::unique(neighbours.begin(), neighbours.end()), neighbours.end());
  }

  std::vector<NodeFeature> features(node_count);
  for (std::size_t node_index = 0; node_index < node_count; ++node_index) {
    NodeFeature &feature = features[node_index];
    feature.position = pointOf(map.nodes[node_index].pos);
    feature.valid = finiteVector(feature.position);
    if (!feature.valid) {
      continue;
    }
    ++result.statistics.valid_node_count;

    // 通常経路ではGNGが既に持つ法線をそのまま利用する。これまで全ノードで
    // 局所PCAを解いていたため、ここがフレーム時間の大半を占めていた。
    double spacing_sum = 0.0;
    std::size_t spacing_count = 0U;
    for (const std::size_t neighbour : adjacency[node_index]) {
      const Eigen::Vector3d neighbour_position = pointOf(map.nodes[neighbour].pos);
      if (!finiteVector(neighbour_position)) {
        continue;
      }
      const double distance = (feature.position - neighbour_position).norm();
      if (std::isfinite(distance) && distance > kEpsilon) {
        spacing_sum += distance;
        ++spacing_count;
      }
    }
    feature.local_spacing = spacing_count > 0U ?
      spacing_sum / static_cast<double>(spacing_count) : 0.0;
    Eigen::Vector3d supplied_normal = normalOf(map.nodes[node_index].normal);
    if (feature.local_spacing > kEpsilon && finiteVector(supplied_normal) &&
      supplied_normal.squaredNorm() > kEpsilon)
    {
      feature.normal = supplied_normal.normalized();
      feature.planarity = 1.0;
      feature.planar = true;
      ++result.statistics.locally_planar_node_count;
      continue;
    }

    // 法線が未設定・不正なノードだけに限定して局所PCAを使う。
    std::vector<std::size_t> patch_indices;
    patch_indices.reserve(adjacency[node_index].size() + 1U);
    patch_indices.push_back(node_index);
    for (const std::size_t neighbour : adjacency[node_index]) {
      const Eigen::Vector3d neighbour_position = pointOf(map.nodes[neighbour].pos);
      if (!finiteVector(neighbour_position)) {
        continue;
      }
      patch_indices.push_back(neighbour);
    }
    if (feature.local_spacing <= kEpsilon || patch_indices.size() < 3U) {
      continue;
    }
    std::vector<Eigen::Vector3d> patch_points;
    patch_points.reserve(patch_indices.size());
    for (const std::size_t patch_index : patch_indices) {
      const Eigen::Vector3d point = pointOf(map.nodes[patch_index].pos);
      if (finiteVector(point)) {
        patch_points.push_back(point);
      }
    }
    PlaneFit fit = fitPlane(patch_points);
    if (!fit.valid) {
      continue;
    }
    orientNormal(fit.normal, normalOf(map.nodes[node_index].normal));
    feature.normal = fit.normal;
    feature.planarity = fit.planarity;
    feature.planar = fit.planarity >= options_.min_cluster_planarity;
    if (feature.planar) {
      ++result.statistics.locally_planar_node_count;
    }
  }

  // 三点配置は、一中心から伸びる二本の直接GNGエッジで作る。端点同士のエッジを
  // 必須にしないため、格子状のGNGでも面内の種を得られる。直線列や閉路の輪郭は、
  // 後段の内部エッジ密度で除外する。
  struct TriangleSeed
  {
    std::size_t center = 0U;
    std::size_t first = 0U;
    std::size_t second = 0U;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double spacing = 0.0;
    double quality = 0.0;
  };

  std::vector<TriangleSeed> triangle_seeds;
  triangle_seeds.reserve(node_count);
  for (std::size_t center = 0U; center < node_count; ++center) {
    if (!features[center].planar || adjacency[center].size() < 2U)
    {
      continue;
    }
    bool has_best_seed = false;
    double best_seed_quality = -std::numeric_limits<double>::infinity();
    TriangleSeed best_seed;
    for (std::size_t first_offset = 0U;
      first_offset + 1U < adjacency[center].size(); ++first_offset)
    {
      const std::size_t first = adjacency[center][first_offset];
      if (!features[first].planar) {
        continue;
      }
      for (std::size_t second_offset = first_offset + 1U;
        second_offset < adjacency[center].size(); ++second_offset)
      {
        const std::size_t second = adjacency[center][second_offset];
        if (!features[second].planar) {
          continue;
        }
        const double spacing =
          (features[center].local_spacing + features[first].local_spacing +
          features[second].local_spacing) / 3.0;
        if (!std::isfinite(spacing) || spacing <= kEpsilon) {
          continue;
        }
        const Eigen::Vector3d first_edge =
          features[first].position - features[center].position;
        const Eigen::Vector3d second_edge =
          features[second].position - features[center].position;
        Eigen::Vector3d normal = first_edge.cross(second_edge);
        const double triangle_area_ratio = normal.norm() / (spacing * spacing);
        if (!std::isfinite(triangle_area_ratio) || normal.squaredNorm() <= kEpsilon) {
          continue;
        }
        normal.normalize();

        Eigen::Vector3d reference_normal = features[center].normal;
        Eigen::Vector3d first_normal = features[first].normal;
        Eigen::Vector3d second_normal = features[second].normal;
        if (first_normal.dot(reference_normal) < 0.0) {
          first_normal = -first_normal;
        }
        if (second_normal.dot(reference_normal) < 0.0) {
          second_normal = -second_normal;
        }
        reference_normal += first_normal + second_normal;
        if (reference_normal.squaredNorm() <= kEpsilon) {
          continue;
        }
        reference_normal.normalize();
        if (std::abs(normal.dot(reference_normal)) < options_.min_normal_alignment_cos ||
          std::abs(features[center].normal.dot(reference_normal)) <
          options_.min_normal_alignment_cos ||
          std::abs(features[first].normal.dot(reference_normal)) <
          options_.min_normal_alignment_cos ||
          std::abs(features[second].normal.dot(reference_normal)) <
          options_.min_normal_alignment_cos)
        {
          continue;
        }
        orientNormal(normal, reference_normal);
        const double normal_agreement = std::abs(normal.dot(reference_normal));
        const double quality = triangle_area_ratio * normal_agreement;
        if (quality <= best_seed_quality) {
          continue;
        }
        best_seed_quality = quality;
        best_seed = TriangleSeed{
          center, first, second,
          (features[center].position + features[first].position + features[second].position) / 3.0,
          normal, spacing, quality};
        has_best_seed = true;
      }
    }
    if (has_best_seed) {
      triangle_seeds.push_back(std::move(best_seed));
    }
  }
  std::sort(
    triangle_seeds.begin(), triangle_seeds.end(),
    [](const TriangleSeed &first, const TriangleSeed &second) {
      return first.quality > second.quality;
    });
  // 種は局所的な開始点に過ぎない。法線と平面オフセットが同じ種を多数置くと、
  // 同じ面を何度も探索してしまう。ここで間引くのは種候補だけであり、全ノードは
  // 後段の成長・所属更新の対象として残す。
  const double growth_distance_ratio = options_.max_growth_dist_ratio;
  const double seed_duplicate_distance_ratio = options_.max_seed_plane_dist_ratio;
  std::vector<TriangleSeed> selected_triangle_seeds;
  selected_triangle_seeds.reserve(triangle_seeds.size());
  for (const TriangleSeed &seed : triangle_seeds) {
    bool represents_known_plane = false;
    for (std::size_t selected_index = 0U;
      selected_index < selected_triangle_seeds.size(); ++selected_index)
    {
      const TriangleSeed &known = selected_triangle_seeds[selected_index];
      if (std::abs(seed.normal.dot(known.normal)) < options_.min_normal_alignment_cos) {
        continue;
      }
      const double spacing = std::max({seed.spacing, known.spacing, kEpsilon});
      const Eigen::Vector3d delta = seed.centroid - known.centroid;
      if (std::abs(seed.normal.dot(delta)) <= seed_duplicate_distance_ratio * spacing &&
        std::abs(known.normal.dot(delta)) <= seed_duplicate_distance_ratio * spacing)
      {
        represents_known_plane = true;
        break;
      }
    }
    if (represents_known_plane) {
      continue;
    }
    selected_triangle_seeds.push_back(seed);
  }
  triangle_seeds = std::move(selected_triangle_seeds);
  result.statistics.seed_component_count = triangle_seeds.size();

  // 追加判定は、現在までに所属したノード群の共分散平面に対して行う。三点種は
  // 初期平面を作るだけで、成長の波ごとに平面を更新する。
  const auto belongsToPlane = [&](const PlaneFit &plane, const double plane_spacing,
    std::size_t node_index) {
      const NodeFeature &feature = features[node_index];
      if (!feature.planar ||
        std::abs(plane.normal.dot(feature.normal)) < options_.min_normal_alignment_cos)
      {
        return false;
      }
      const double spacing = std::max(plane_spacing, feature.local_spacing);
      return std::abs(plane.normal.dot(feature.position - plane.centroid)) <=
             growth_distance_ratio * spacing;
    };

  struct SeedPatch
  {
    TriangleSeed seed;
    PlaneFit fit;
    double spacing = 0.0;
    std::vector<std::size_t> nodes;
  };
  std::vector<SeedPatch> seed_patches;
  seed_patches.reserve(triangle_seeds.size());
  struct PropagationEntry
  {
    double score = 0.0;
    std::size_t seed_index = 0U;
    std::size_t node_index = 0U;
  };
  const auto propagation_order = [](const PropagationEntry &first,
    const PropagationEntry &second) {
      if (first.score != second.score) {
        return first.score > second.score;
      }
      if (first.seed_index != second.seed_index) {
        return first.seed_index > second.seed_index;
      }
      return first.node_index > second.node_index;
    };
  struct PropagationResult
  {
    std::vector<int> owners;
    std::vector<double> scores;
  };
  const auto propagate = [&](const std::vector<PlaneFit> &planes,
    const std::vector<double> &spacings, const bool use_seed_zero_score) {
      PropagationResult propagated;
      propagated.owners.assign(node_count, -1);
      propagated.scores.assign(node_count, std::numeric_limits<double>::infinity());
      std::priority_queue<PropagationEntry, std::vector<PropagationEntry>,
        decltype(propagation_order)> queue(propagation_order);
      const auto propose = [&](const std::size_t seed_index, const std::size_t node_index,
        const bool is_seed_node) {
          const PlaneFit &plane = planes[seed_index];
          if (!plane.valid || !features[node_index].planar) {
            return;
          }
          const double spacing = std::max(spacings[seed_index], kEpsilon);
          const double score = is_seed_node && use_seed_zero_score ? 0.0 :
            std::abs(plane.normal.dot(features[node_index].position - plane.centroid)) / spacing;
          if ((!is_seed_node && !belongsToPlane(plane, spacing, node_index)) ||
            score > growth_distance_ratio ||
            (score > propagated.scores[node_index] - kEpsilon &&
            (std::abs(score - propagated.scores[node_index]) > kEpsilon ||
            propagated.owners[node_index] <= static_cast<int>(seed_index))))
          {
            return;
          }
          propagated.scores[node_index] = score;
          propagated.owners[node_index] = static_cast<int>(seed_index);
          queue.push(PropagationEntry{score, seed_index, node_index});
        };
      for (std::size_t seed_index = 0U; seed_index < triangle_seeds.size(); ++seed_index) {
        propose(seed_index, triangle_seeds[seed_index].center, true);
        propose(seed_index, triangle_seeds[seed_index].first, true);
        propose(seed_index, triangle_seeds[seed_index].second, true);
      }
      while (!queue.empty()) {
        const PropagationEntry current = queue.top();
        queue.pop();
        if (propagated.owners[current.node_index] != static_cast<int>(current.seed_index) ||
          current.score > propagated.scores[current.node_index] + kEpsilon)
        {
          continue;
        }
        for (const std::size_t neighbour : adjacency[current.node_index]) {
          propose(current.seed_index, neighbour, false);
        }
      }
      return propagated;
    };
  const auto makePlanes = [&](const PropagationResult &propagated,
    std::vector<PlaneFit> &planes, std::vector<double> &spacings) {
      std::vector<std::vector<std::size_t>> members(triangle_seeds.size());
      for (std::size_t node_index = 0U; node_index < node_count; ++node_index) {
        const int owner = propagated.owners[node_index];
        if (owner >= 0) {
          members[static_cast<std::size_t>(owner)].push_back(node_index);
        }
      }
      for (std::size_t seed_index = 0U; seed_index < triangle_seeds.size(); ++seed_index) {
        if (members[seed_index].size() < 3U) {
          continue;
        }
        std::vector<Eigen::Vector3d> points;
        points.reserve(members[seed_index].size());
        double spacing_sum = 0.0;
        for (const std::size_t node_index : members[seed_index]) {
          points.push_back(features[node_index].position);
          spacing_sum += features[node_index].local_spacing;
        }
        PlaneFit fit = fitPlane(points);
        if (!fit.valid) {
          continue;
        }
        orientNormal(fit.normal, triangle_seeds[seed_index].normal);
        planes[seed_index] = fit;
        spacings[seed_index] = spacing_sum / static_cast<double>(members[seed_index].size());
      }
    };

  std::vector<PlaneFit> growth_planes(triangle_seeds.size());
  std::vector<double> growth_spacings(triangle_seeds.size(), 0.0);
  for (std::size_t seed_index = 0U; seed_index < triangle_seeds.size(); ++seed_index) {
    growth_planes[seed_index].valid = true;
    growth_planes[seed_index].centroid = triangle_seeds[seed_index].centroid;
    growth_planes[seed_index].normal = triangle_seeds[seed_index].normal;
    growth_spacings[seed_index] = triangle_seeds[seed_index].spacing;
  }
  const PropagationResult initial_propagation = propagate(
    growth_planes, growth_spacings, true);
  makePlanes(initial_propagation, growth_planes, growth_spacings);
  const PropagationResult refined_propagation = propagate(
    growth_planes, growth_spacings, false);

  std::vector<std::vector<std::size_t>> final_members(triangle_seeds.size());
  for (std::size_t node_index = 0U; node_index < node_count; ++node_index) {
    const int owner = refined_propagation.owners[node_index];
    if (owner >= 0) {
      final_members[static_cast<std::size_t>(owner)].push_back(node_index);
    }
  }
  for (std::size_t seed_index = 0U; seed_index < triangle_seeds.size(); ++seed_index) {
    std::vector<std::size_t> &members = final_members[seed_index];
    if (members.size() < options_.min_cluster_nodes) {
      ++result.statistics.insufficient_seed_component_count;
      continue;
    }
    std::vector<Eigen::Vector3d> points;
    points.reserve(members.size());
    double spacing_sum = 0.0;
    for (const std::size_t node_index : members) {
      points.push_back(features[node_index].position);
      spacing_sum += features[node_index].local_spacing;
    }
    const double spacing = spacing_sum / static_cast<double>(members.size());
    PlaneFit fit = fitPlane(points);
    if (!fit.valid || spacing <= kEpsilon ||
      fit.planarity < options_.min_cluster_planarity ||
      fit.residual / spacing > options_.max_normalized_cluster_residual)
    {
      ++result.statistics.geometrically_rejected_component_count;
      continue;
    }
    orientNormal(fit.normal, growth_planes[seed_index].normal);
    seed_patches.push_back(SeedPatch{
      triangle_seeds[seed_index], fit, spacing, std::move(members)});
  }

  if (seed_patches.empty()) {
    return result;
  }

  const auto overlapCount = [](const std::vector<std::size_t> &first,
    const std::vector<std::size_t> &second) {
      std::size_t first_index = 0U;
      std::size_t second_index = 0U;
      std::size_t overlap = 0U;
      while (first_index < first.size() && second_index < second.size()) {
        if (first[first_index] == second[second_index]) {
          ++overlap;
          ++first_index;
          ++second_index;
        } else if (first[first_index] < second[second_index]) {
          ++first_index;
        } else {
          ++second_index;
        }
      }
      return overlap;
    };
  const auto arePatchesCompatible = [&](const SeedPatch &first, const SeedPatch &second) {
      if (std::abs(first.fit.normal.dot(second.fit.normal)) <
        options_.min_normal_alignment_cos)
      {
        return false;
      }
      const double spacing = std::max({first.spacing, second.spacing, kEpsilon});
      const Eigen::Vector3d delta = second.fit.centroid - first.fit.centroid;
      return std::abs(first.fit.normal.dot(delta)) <= growth_distance_ratio * spacing &&
             std::abs(second.fit.normal.dot(delta)) <= growth_distance_ratio * spacing;
    };

  // 成長後の重なりを使ってだけ種パッチを併合する。先着順の種IDや種ノードの
  // 固定所属は使わないため、同一平面上のノードはより適合する側へ移れる。
  DisjointSet patch_sets(seed_patches.size());
  for (std::size_t first = 0U; first < seed_patches.size(); ++first) {
    for (std::size_t second = first + 1U; second < seed_patches.size(); ++second) {
      if (overlapCount(seed_patches[first].nodes, seed_patches[second].nodes) > 0U &&
        arePatchesCompatible(seed_patches[first], seed_patches[second]))
      {
        patch_sets.unite(first, second);
      }
    }
  }

  struct CandidateGroup
  {
    std::size_t root = 0U;
    std::size_t patch_count = 0U;
    Eigen::Vector3d reference_normal = Eigen::Vector3d::Zero();
    PlaneFit fit;
    double spacing = 0.0;
    std::vector<std::size_t> nodes;
  };
  std::unordered_map<std::size_t, std::size_t> group_indices;
  group_indices.reserve(seed_patches.size());
  std::vector<CandidateGroup> candidate_groups;
  for (std::size_t patch_index = 0U; patch_index < seed_patches.size(); ++patch_index) {
    const std::size_t root = patch_sets.find(patch_index);
    const auto [entry, inserted] = group_indices.emplace(root, candidate_groups.size());
    if (inserted) {
      candidate_groups.emplace_back();
      candidate_groups.back().root = root;
    }
    CandidateGroup &group = candidate_groups[entry->second];
    Eigen::Vector3d normal = seed_patches[patch_index].fit.normal;
    if (group.reference_normal.squaredNorm() > kEpsilon &&
      normal.dot(group.reference_normal) < 0.0)
    {
      normal = -normal;
    }
    group.reference_normal += normal;
    group.spacing += seed_patches[patch_index].spacing;
    ++group.patch_count;
    group.nodes.insert(
      group.nodes.end(), seed_patches[patch_index].nodes.begin(), seed_patches[patch_index].nodes.end());
  }

  std::vector<CandidateGroup> output_candidates;
  output_candidates.reserve(candidate_groups.size());
  for (CandidateGroup &group : candidate_groups) {
    std::sort(group.nodes.begin(), group.nodes.end());
    group.nodes.erase(std::unique(group.nodes.begin(), group.nodes.end()), group.nodes.end());
    group.spacing /= static_cast<double>(std::max<std::size_t>(1U, group.patch_count));
    std::vector<Eigen::Vector3d> points;
    points.reserve(group.nodes.size());
    for (const std::size_t node_index : group.nodes) {
      points.push_back(features[node_index].position);
    }
    group.fit = fitPlane(points);
    if (group.fit.valid && group.spacing > kEpsilon &&
      group.fit.planarity >= options_.min_cluster_planarity &&
      group.fit.residual / group.spacing <= options_.max_normalized_cluster_residual)
    {
      orientNormal(group.fit.normal, group.reference_normal);
      output_candidates.push_back(std::move(group));
      continue;
    }

    // 併合後の全体fitが悪くても、すでに平面として確認済みの種パッチまで一括で
    // 捨てない。各パッチを独立候補として戻し、所属更新で重複だけを解消する。
    for (std::size_t patch_index = 0U; patch_index < seed_patches.size(); ++patch_index) {
      if (patch_sets.find(patch_index) != group.root) {
        continue;
      }
      const SeedPatch &patch = seed_patches[patch_index];
      CandidateGroup fallback;
      fallback.reference_normal = patch.fit.normal;
      fallback.fit = patch.fit;
      fallback.spacing = patch.spacing;
      fallback.nodes = patch.nodes;
      output_candidates.push_back(std::move(fallback));
    }
  }

  std::vector<int> node_owner(node_count, -1);
  std::vector<double> node_score(node_count, std::numeric_limits<double>::infinity());
  for (std::size_t candidate_index = 0U; candidate_index < output_candidates.size(); ++candidate_index) {
    const CandidateGroup &candidate = output_candidates[candidate_index];
    for (const std::size_t node_index : candidate.nodes) {
      const double spacing = std::max(candidate.spacing, features[node_index].local_spacing);
      const double score = std::abs(candidate.fit.normal.dot(
        features[node_index].position - candidate.fit.centroid)) / std::max(spacing, kEpsilon);
      if (score < node_score[node_index]) {
        node_score[node_index] = score;
        node_owner[node_index] = static_cast<int>(candidate_index);
      }
    }
  }

  SurfaceSupportWorkspace support_workspace(node_count);
  std::uint32_t cluster_id = 1U;
  for (std::size_t candidate_index = 0U; candidate_index < output_candidates.size(); ++candidate_index) {
    const CandidateGroup &candidate = output_candidates[candidate_index];
    std::vector<std::size_t> members;
    members.reserve(candidate.nodes.size());
    for (const std::size_t node_index : candidate.nodes) {
      if (node_owner[node_index] != static_cast<int>(candidate_index)) {
        continue;
      }
      const double normalized_deviation = std::abs(candidate.fit.normal.dot(
        features[node_index].position - candidate.fit.centroid)) /
        std::max(candidate.spacing, kEpsilon);
      if (normalized_deviation <= options_.max_normalized_cluster_residual) {
        members.push_back(node_index);
      }
    }
    if (members.size() < options_.min_cluster_nodes) {
      ++result.statistics.insufficient_member_component_count;
      continue;
    }
    std::vector<Eigen::Vector3d> points;
    points.reserve(members.size());
    for (const std::size_t node_index : members) {
      points.push_back(features[node_index].position);
    }
    PlaneFit fit = fitPlane(points);
    if (!fit.valid || fit.planarity < options_.min_cluster_planarity ||
      fit.residual / std::max(candidate.spacing, kEpsilon) >
      options_.max_normalized_cluster_residual)
    {
      ++result.statistics.geometrically_rejected_component_count;
      continue;
    }
    const SurfaceSupport support = collectSurfaceSupport(map, members, support_workspace);
    if (!support.isSurfaceLike(
        members.size(), options_.min_cluster_edge_per_node))
    {
      ++result.statistics.line_like_component_count;
      continue;
    }
    orientNormal(fit.normal, candidate.reference_normal);
    const Eigen::Vector3d tangent_u = fit.normal.unitOrthogonal();
    const Eigen::Vector3d tangent_v = fit.normal.cross(tangent_u).normalized();
    double min_u = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    double min_v = std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    std::size_t terrain_nodes = 0U;
    std::size_t wall_nodes = 0U;
    for (const std::size_t node_index : members) {
      const Eigen::Vector3d delta = features[node_index].position - fit.centroid;
      const double u = delta.dot(tangent_u);
      const double v = delta.dot(tangent_v);
      min_u = std::min(min_u, u);
      max_u = std::max(max_u, u);
      min_v = std::min(min_v, v);
      max_v = std::max(max_v, v);
      terrain_nodes += map.nodes[node_index].label ==
        ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN;
      wall_nodes += map.nodes[node_index].label == ais_gng_msgs::msg::TopologicalMap::WALL;
    }
    ais_gng_msgs::msg::PlanarCluster cluster;
    cluster.id = cluster_id++;
    cluster.source_label = terrain_nodes + wall_nodes == 0U ?
      ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT :
      (terrain_nodes >= wall_nodes ?
      ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN :
      ais_gng_msgs::msg::TopologicalMap::WALL);
    cluster.node_indices.reserve(members.size());
    for (const std::size_t node_index : members) {
      cluster.node_indices.push_back(static_cast<std::uint32_t>(node_index));
    }
    cluster.support_edges.reserve(2U * support.edges.size());
    for (const auto &[first, second] : support.edges) {
      cluster.support_edges.push_back(pointMessage(features[first].position));
      cluster.support_edges.push_back(pointMessage(features[second].position));
    }
    cluster.centroid = pointMessage(fit.centroid);
    cluster.normal = vectorMessage(fit.normal);
    cluster.tangent_u = vectorMessage(tangent_u);
    cluster.tangent_v = vectorMessage(tangent_v);
    cluster.area = 0.0F;
    cluster.extent_u = static_cast<float>(max_u - min_u);
    cluster.extent_v = static_cast<float>(max_v - min_v);
    cluster.local_spacing = static_cast<float>(candidate.spacing);
    cluster.planarity = static_cast<float>(fit.planarity);
    cluster.residual_ratio = static_cast<float>(fit.residual / candidate.spacing);
    result.statistics.clustered_node_count += members.size();
    for (const std::size_t node_index : members) {
      switch (map.nodes[node_index].label) {
        case ais_gng_msgs::msg::TopologicalMap::DEFAULT:
          ++result.statistics.clustered_default_node_count;
          break;
        case ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN:
          ++result.statistics.clustered_terrain_node_count;
          break;
        case ais_gng_msgs::msg::TopologicalMap::WALL:
          ++result.statistics.clustered_wall_node_count;
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
    }
    result.clusters.clusters.push_back(std::move(cluster));
  }
  result.statistics.cluster_count = result.clusters.clusters.size();
  return result;
}

struct PersistentPlaneClusterTracker::Impl
{
  struct Track
  {
    std::uint32_t id = 0U;
    std::uint8_t source_label = ais_gng_msgs::msg::TopologicalMap::DEFAULT;
    ais_gng_msgs::msg::PlanarCluster cluster;
    std::vector<NodeKey> node_keys;
    std::vector<std::size_t> current_indices;
  };

  struct Candidate
  {
    const ais_gng_msgs::msg::PlanarCluster *cluster = nullptr;
    std::vector<NodeKey> node_keys;
    std::vector<std::size_t> indices;
  };

  explicit Impl(PlaneClusterOptions input_options)
  : options(std::move(input_options))
  {}

  PlaneClusterOptions options;
  std::uint32_t next_id = 1U;
  std::vector<Track> tracks;
  std::unordered_map<NodeKey, std::uint32_t> node_owner_ids;
};

PersistentPlaneClusterTracker::PersistentPlaneClusterTracker(PlaneClusterOptions options)
: impl_(std::make_unique<Impl>(std::move(options)))
{}

PersistentPlaneClusterTracker::~PersistentPlaneClusterTracker() = default;

ais_gng_msgs::msg::PlanarClusterArray PersistentPlaneClusterTracker::update(
  const ais_gng_msgs::msg::PlanarClusterArray &frame_clusters,
  const ais_gng_msgs::msg::TopologicalMap &map)
{
  const double normalized_deviation_limit = impl_->options.max_growth_dist_ratio;

  ais_gng_msgs::msg::PlanarClusterArray output;
  output.header = frame_clusters.header;
  output.frame_number = frame_clusters.frame_number;

  std::unordered_map<NodeKey, std::size_t> current_node_indices;
  current_node_indices.reserve(map.nodes.size());
  for (std::size_t index = 0; index < map.nodes.size(); ++index) {
    current_node_indices.emplace(nodeKey(map.nodes[index]), index);
  }

  std::vector<Impl::Candidate> candidates;
  candidates.reserve(frame_clusters.clusters.size());
  for (const auto &cluster : frame_clusters.clusters) {
    Impl::Candidate candidate;
    candidate.cluster = &cluster;
    candidate.indices.reserve(cluster.node_indices.size());
    for (const std::uint32_t index : cluster.node_indices) {
      if (index >= map.nodes.size()) {
        continue;
      }
      candidate.indices.push_back(index);
      candidate.node_keys.push_back(nodeKey(map.nodes[index]));
    }
    std::sort(candidate.node_keys.begin(), candidate.node_keys.end());
    candidate.node_keys.erase(
      std::unique(candidate.node_keys.begin(), candidate.node_keys.end()), candidate.node_keys.end());
    if (candidate.node_keys.size() >= impl_->options.min_cluster_nodes) {
      candidates.push_back(std::move(candidate));
    }
  }

  // 抽出器が同一平面のフレーム内パッチを併合した結果、過去には別だった複数の
  // 永続クラスタの所属ノードを共有することがある。この場合は最小IDを統合先として残す。
  // 先に node_owner_ids の消滅側IDを統合先IDへ移してから逆引きリストを作り直すため、
  // 併合の途中でもノードが未所属・二重所属の状態にならない。
  if (impl_->tracks.size() > 1U) {
    DisjointSet merge_sets(impl_->tracks.size());
    bool has_merge = false;
    for (const auto &candidate : candidates) {
      std::size_t first_track = impl_->tracks.size();
      for (std::size_t track_index = 0; track_index < impl_->tracks.size(); ++track_index) {
        if (overlapCount(impl_->tracks[track_index].node_keys, candidate.node_keys) == 0U) {
          continue;
        }
        if (first_track == impl_->tracks.size()) {
          first_track = track_index;
          continue;
        }
        merge_sets.unite(first_track, track_index);
        has_merge = true;
      }
    }
    if (has_merge) {
      std::vector<std::size_t> survivor_indices(impl_->tracks.size(), impl_->tracks.size());
      for (std::size_t track_index = 0; track_index < impl_->tracks.size(); ++track_index) {
        const std::size_t root = merge_sets.find(track_index);
        if (survivor_indices[root] == impl_->tracks.size() ||
          impl_->tracks[track_index].id < impl_->tracks[survivor_indices[root]].id)
        {
          survivor_indices[root] = track_index;
        }
      }

      std::unordered_map<std::uint32_t, std::uint32_t> merged_into;
      merged_into.reserve(impl_->tracks.size());
      for (std::size_t track_index = 0; track_index < impl_->tracks.size(); ++track_index) {
        const std::size_t survivor = survivor_indices[merge_sets.find(track_index)];
        merged_into.emplace(impl_->tracks[track_index].id, impl_->tracks[survivor].id);
      }
      for (auto &owner : impl_->node_owner_ids) {
        const auto target = merged_into.find(owner.second);
        if (target != merged_into.end()) {
          owner.second = target->second;
        }
      }

      std::vector<Impl::Track> merged_tracks;
      merged_tracks.reserve(impl_->tracks.size());
      for (std::size_t root = 0; root < impl_->tracks.size(); ++root) {
        if (merge_sets.find(root) != root) {
          continue;
        }
        const std::size_t survivor = survivor_indices[root];
        Impl::Track merged = std::move(impl_->tracks[survivor]);
        for (std::size_t track_index = 0; track_index < impl_->tracks.size(); ++track_index) {
          if (track_index == survivor || merge_sets.find(track_index) != root) {
            continue;
          }
          const auto &merged_away = impl_->tracks[track_index];
          merged.node_keys.insert(
            merged.node_keys.end(), merged_away.node_keys.begin(), merged_away.node_keys.end());
          merged.current_indices.insert(
            merged.current_indices.end(), merged_away.current_indices.begin(),
            merged_away.current_indices.end());
        }
        std::sort(merged.node_keys.begin(), merged.node_keys.end());
        merged.node_keys.erase(
          std::unique(merged.node_keys.begin(), merged.node_keys.end()), merged.node_keys.end());
        std::sort(merged.current_indices.begin(), merged.current_indices.end());
        merged.current_indices.erase(
          std::unique(merged.current_indices.begin(), merged.current_indices.end()),
          merged.current_indices.end());
        merged.cluster.id = merged.id;
        merged_tracks.push_back(std::move(merged));
      }
      impl_->tracks = std::move(merged_tracks);
    }
  }

  std::vector<bool> candidate_claimed(candidates.size(), false);
  // 前フレームの所属ノードを共有する候補のうち、最も多く共有する候補だけを
  // 対応先にする。ラベルは対応条件に使わない。
  for (auto &track : impl_->tracks) {
    const Eigen::Vector3d previous_centroid = pointOf(track.cluster.centroid);
    const Eigen::Vector3d previous_normal = vectorOf(track.cluster.normal).normalized();
    const double spacing = std::max(static_cast<double>(track.cluster.local_spacing), kEpsilon);

    std::size_t best_candidate = candidates.size();
    std::size_t best_overlap = 0U;
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (candidate_claimed[index]) {
        continue;
      }
      const std::size_t overlap = overlapCount(track.node_keys, candidates[index].node_keys);
      // 追跡の対応付けは、近くにある二つの同一平面パッチではなく、永続的な
      // GNGノードIDの共有で行う。WALL/UNKNOWN/TERRAIN ラベルの変動は無視する。
      if (overlap == 0U) {
        continue;
      }
      if (best_candidate == candidates.size() || overlap > best_overlap) {
        best_overlap = overlap;
        best_candidate = index;
      }
    }
    if (best_candidate < candidates.size()) {
      candidate_claimed[best_candidate] = true;
    } else {
      best_candidate = candidates.size();
    }

    std::vector<std::pair<NodeKey, std::size_t>> retained;
    retained.reserve(track.node_keys.size());
    const auto retainIfClose = [&](NodeKey key, std::size_t index) {
        const Eigen::Vector3d position = pointOf(map.nodes[index].pos);
        const double normalized_deviation = std::abs(previous_normal.dot(position - previous_centroid)) /
          spacing;
        if (std::isfinite(normalized_deviation) &&
          normalized_deviation <= normalized_deviation_limit)
        {
          retained.emplace_back(key, index);
        }
      };

    for (const NodeKey key : track.node_keys) {
      const auto node_it = current_node_indices.find(key);
      if (node_it != current_node_indices.end()) {
        retainIfClose(key, node_it->second);
      }
    }
    if (best_candidate < candidates.size()) {
      for (const std::size_t index : candidates[best_candidate].indices) {
        retainIfClose(nodeKey(map.nodes[index]), index);
      }
    }
    std::sort(retained.begin(), retained.end(), [](const auto &first, const auto &second) {
      return first.first < second.first;
    });
    retained.erase(std::unique(retained.begin(), retained.end(), [](const auto &first, const auto &second) {
      return first.first == second.first;
    }), retained.end());

    if (retained.empty()) {
      track.node_keys.clear();
      track.current_indices.clear();
      track.cluster.node_indices.clear();
      continue;
    }

    std::vector<Eigen::Vector3d> points;
    points.reserve(retained.size());
    for (const auto &[key, index] : retained) {
      static_cast<void>(key);
      points.push_back(pointOf(map.nodes[index].pos));
    }
    // 時系列保持中のパッチは、誕生時にすでに平面判定へ通っている。ここからは
    // 追跡平面までの個別距離だけでメンバーを除外する。全体再フィットでパッチ全体を
    // 消してはならない。
    PlaneFit fit = fitPlane(points);
    if (!fit.valid) {
      fit.centroid = previous_centroid;
      fit.normal = previous_normal;
      fit.planarity = track.cluster.planarity;
      fit.residual = static_cast<double>(track.cluster.residual_ratio) * spacing;
    }
    orientNormal(fit.normal, previous_normal);
    const Eigen::Vector3d tangent_u = fit.normal.unitOrthogonal();
    const Eigen::Vector3d tangent_v = fit.normal.cross(tangent_u).normalized();
    double min_u = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    double min_v = std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    for (const auto &[key, index] : retained) {
      static_cast<void>(key);
      const Eigen::Vector3d delta = pointOf(map.nodes[index].pos) - fit.centroid;
      const double u = delta.dot(tangent_u);
      const double v = delta.dot(tangent_v);
      min_u = std::min(min_u, u);
      max_u = std::max(max_u, u);
      min_v = std::min(min_v, v);
      max_v = std::max(max_v, v);
    }
    track.cluster.id = track.id;
    track.cluster.source_label = track.source_label;
    track.cluster.node_indices.clear();
    track.cluster.node_indices.reserve(retained.size());
    track.node_keys.clear();
    track.node_keys.reserve(retained.size());
    track.current_indices.clear();
    track.current_indices.reserve(retained.size());
    for (const auto &[key, index] : retained) {
      track.node_keys.push_back(key);
      track.current_indices.push_back(index);
      track.cluster.node_indices.push_back(static_cast<std::uint32_t>(index));
    }
    track.cluster.centroid = pointMessage(fit.centroid);
    track.cluster.normal = vectorMessage(fit.normal);
    track.cluster.tangent_u = vectorMessage(tangent_u);
    track.cluster.tangent_v = vectorMessage(tangent_v);
    track.cluster.extent_u = static_cast<float>(max_u - min_u);
    track.cluster.extent_v = static_cast<float>(max_v - min_v);
    track.cluster.planarity = static_cast<float>(fit.planarity);
    track.cluster.residual_ratio = static_cast<float>(fit.residual / spacing);
  }

  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (candidate_claimed[index]) {
      continue;
    }
    const Impl::Candidate &candidate = candidates[index];
    Impl::Track track;
    track.id = impl_->next_id++;
    track.source_label = candidate.cluster->source_label;
    track.cluster = *candidate.cluster;
    track.cluster.id = track.id;
    track.node_keys = candidate.node_keys;
    track.current_indices = candidate.indices;
    impl_->tracks.push_back(std::move(track));
  }

  // ノードIDから所属クラスタIDへの対応を唯一の所有権として保持する。各クラスタの
  // node_keys はこの対応表から再構築する逆引きキャッシュであり、平面までの距離で
  // 毎フレーム所属を奪い合わない。既存所有者がこのフレームで当該ノードを報告しない
  // 場合だけ所有権を解放するため、重なった平面パッチの所属がちらつかない。
  std::unordered_map<std::uint32_t, std::size_t> track_indices_by_id;
  track_indices_by_id.reserve(impl_->tracks.size());
  for (std::size_t track_index = 0; track_index < impl_->tracks.size(); ++track_index) {
    track_indices_by_id.emplace(impl_->tracks[track_index].id, track_index);
  }

  std::unordered_map<NodeKey, std::uint32_t> owner_reported_nodes;
  owner_reported_nodes.reserve(impl_->node_owner_ids.size());
  for (const auto &track : impl_->tracks) {
    for (const std::size_t node_index : track.current_indices) {
      if (node_index >= map.nodes.size()) {
        continue;
      }
      const NodeKey key = nodeKey(map.nodes[node_index]);
      const auto owner = impl_->node_owner_ids.find(key);
      if (owner != impl_->node_owner_ids.end() && owner->second == track.id) {
        owner_reported_nodes.emplace(key, track.id);
      }
    }
  }
  for (auto owner = impl_->node_owner_ids.begin(); owner != impl_->node_owner_ids.end();) {
    const auto track = track_indices_by_id.find(owner->second);
    const auto reporter = owner_reported_nodes.find(owner->first);
    const bool owner_reports_node = reporter != owner_reported_nodes.end() &&
      reporter->second == owner->second;
    if (track == track_indices_by_id.end() || !owner_reports_node) {
      owner = impl_->node_owner_ids.erase(owner);
    } else {
      ++owner;
    }
  }

  // 新規ノードだけは最も古いクラスタIDへ決定的に渡す。既存の所有権は上書きしない。
  // 将来クラスタを併合する場合は、消滅側のIDをこの対応表で統合先IDへ一括置換して
  // から消滅側を削除する。この順序を守れば、併合後に同じノードが二重所属しない。
  std::vector<std::size_t> ownership_order(impl_->tracks.size());
  std::iota(ownership_order.begin(), ownership_order.end(), 0U);
  std::sort(ownership_order.begin(), ownership_order.end(), [this](
      const std::size_t first, const std::size_t second) {
      return impl_->tracks[first].id < impl_->tracks[second].id;
    });
  std::vector<std::vector<std::pair<NodeKey, std::size_t>>> owned_members(impl_->tracks.size());
  for (const std::size_t track_index : ownership_order) {
    const auto &track = impl_->tracks[track_index];
    auto &members = owned_members[track_index];
    members.reserve(track.current_indices.size());
    for (const std::size_t node_index : track.current_indices) {
      if (node_index >= map.nodes.size()) {
        continue;
      }
      const NodeKey key = nodeKey(map.nodes[node_index]);
      const auto [owner, inserted] = impl_->node_owner_ids.emplace(key, track.id);
      static_cast<void>(inserted);
      if (owner->second == track.id) {
        members.emplace_back(key, node_index);
      }
    }
  }

  std::vector<int> node_track_owner(map.nodes.size(), -1);
  for (std::size_t track_index = 0; track_index < impl_->tracks.size(); ++track_index) {
    auto &track = impl_->tracks[track_index];
    auto &members = owned_members[track_index];
    std::sort(members.begin(), members.end(), [](const auto &first, const auto &second) {
      return first.first < second.first;
    });
    members.erase(
      std::unique(members.begin(), members.end(), [](const auto &first, const auto &second) {
        return first.first == second.first;
      }), members.end());

    track.node_keys.clear();
    track.current_indices.clear();
    track.cluster.node_indices.clear();
    track.node_keys.reserve(members.size());
    track.current_indices.reserve(members.size());
    track.cluster.node_indices.reserve(members.size());
    for (const auto &[key, node_index] : members) {
      track.node_keys.push_back(key);
      track.current_indices.push_back(node_index);
      track.cluster.node_indices.push_back(static_cast<std::uint32_t>(node_index));
      node_track_owner[node_index] = static_cast<int>(track_index);
    }
    track.cluster.support_edges.clear();
    if (track.current_indices.size() < impl_->options.min_cluster_nodes) {
      continue;
    }

    std::vector<Eigen::Vector3d> points;
    points.reserve(track.current_indices.size());
    for (const std::size_t node_index : track.current_indices) {
      points.push_back(pointOf(map.nodes[node_index].pos));
    }
    PlaneFit fit = fitPlane(points);
    const double spacing = std::max(static_cast<double>(track.cluster.local_spacing), kEpsilon);
    // 追跡中も共分散から面内の二方向への広がりを確認する。平面上にある直鎖は
    // 残差だけでは除外できないため、sqrt(λ₂ / λ₃) が下限を割った時点で
    // クラスタを種へ戻す。GNGエッジの一時的な変化はここでは使わない。
    if (!fit.valid || fit.planarity < impl_->options.min_cluster_planarity ||
      fit.residual / spacing > impl_->options.max_normalized_cluster_residual)
    {
      track.node_keys.clear();
      track.current_indices.clear();
      track.cluster.node_indices.clear();
      continue;
    }
    Eigen::Vector3d previous_normal = vectorOf(track.cluster.normal);
    orientNormal(fit.normal, previous_normal);
    const Eigen::Vector3d tangent_u = fit.normal.unitOrthogonal();
    const Eigen::Vector3d tangent_v = fit.normal.cross(tangent_u).normalized();
    double min_u = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    double min_v = std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d &point : points) {
      const Eigen::Vector3d delta = point - fit.centroid;
      const double u = delta.dot(tangent_u);
      const double v = delta.dot(tangent_v);
      min_u = std::min(min_u, u);
      max_u = std::max(max_u, u);
      min_v = std::min(min_v, v);
      max_v = std::max(max_v, v);
    }
    track.cluster.centroid = pointMessage(fit.centroid);
    track.cluster.normal = vectorMessage(fit.normal);
    track.cluster.tangent_u = vectorMessage(tangent_u);
    track.cluster.tangent_v = vectorMessage(tangent_v);
    track.cluster.extent_u = static_cast<float>(max_u - min_u);
    track.cluster.extent_v = static_cast<float>(max_v - min_v);
    track.cluster.planarity = static_cast<float>(fit.planarity);
    track.cluster.residual_ratio = static_cast<float>(fit.residual / spacing);
  }

  std::vector<double> edge_length_sums(impl_->tracks.size(), 0.0);
  std::vector<std::size_t> edge_counts(impl_->tracks.size(), 0U);
  for (std::size_t edge_index = 0; edge_index + 1U < map.edges.size(); edge_index += 2U) {
    const std::size_t first = map.edges[edge_index];
    const std::size_t second = map.edges[edge_index + 1U];
    if (first >= node_track_owner.size() || second >= node_track_owner.size()) {
      continue;
    }
    const int owner = node_track_owner[first];
    if (owner < 0 || owner != node_track_owner[second]) {
      continue;
    }
    const Eigen::Vector3d edge = pointOf(map.nodes[second].pos) - pointOf(map.nodes[first].pos);
    const double length = edge.norm();
    if (!std::isfinite(length) || length <= kEpsilon) {
      continue;
    }
    auto &cluster = impl_->tracks[static_cast<std::size_t>(owner)].cluster;
    cluster.support_edges.push_back(pointMessage(pointOf(map.nodes[first].pos)));
    cluster.support_edges.push_back(pointMessage(pointOf(map.nodes[second].pos)));
    edge_length_sums[static_cast<std::size_t>(owner)] += length;
    ++edge_counts[static_cast<std::size_t>(owner)];
  }

  for (std::size_t track_index = 0; track_index < impl_->tracks.size(); ++track_index) {
    auto &track = impl_->tracks[track_index];
    if (track.current_indices.size() < impl_->options.min_cluster_nodes)
    {
      continue;
    }
    // ラベルは平面抽出の条件に使わない。保守更新中にも現在の所属ノードから更新すれば、
    // ラベル変化だけで全地図を再抽出する必要がない。
    std::size_t terrain_node_count = 0U;
    std::size_t wall_node_count = 0U;
    for (const std::size_t node_index : track.current_indices) {
      if (node_index >= map.nodes.size()) {
        continue;
      }
      terrain_node_count += map.nodes[node_index].label ==
        ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN;
      wall_node_count += map.nodes[node_index].label == ais_gng_msgs::msg::TopologicalMap::WALL;
    }
    track.source_label = terrain_node_count + wall_node_count == 0U ?
      ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT :
      (terrain_node_count >= wall_node_count ?
      ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN : ais_gng_msgs::msg::TopologicalMap::WALL);
    track.cluster.source_label = track.source_label;
    if (edge_counts[track_index] > 0U) {
      track.cluster.local_spacing = static_cast<float>(
        edge_length_sums[track_index] / static_cast<double>(edge_counts[track_index]));
    }
    output.clusters.push_back(track.cluster);
  }
  impl_->tracks.erase(
    std::remove_if(
      impl_->tracks.begin(), impl_->tracks.end(), [this](const Impl::Track &track) {
        return track.current_indices.size() < impl_->options.min_cluster_nodes;
      }),
    impl_->tracks.end());
  std::unordered_map<std::uint32_t, bool> active_cluster_ids;
  active_cluster_ids.reserve(impl_->tracks.size());
  for (const auto &track : impl_->tracks) {
    active_cluster_ids.emplace(track.id, true);
  }
  for (auto owner = impl_->node_owner_ids.begin(); owner != impl_->node_owner_ids.end();) {
    if (active_cluster_ids.find(owner->second) == active_cluster_ids.end()) {
      owner = impl_->node_owner_ids.erase(owner);
    } else {
      ++owner;
    }
  }
  return output;
}

}  // fuzzrobo::topological_plane 名前空間
