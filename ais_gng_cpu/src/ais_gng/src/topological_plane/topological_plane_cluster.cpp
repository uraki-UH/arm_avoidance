#include <ais_gng/topological_plane/topological_plane_cluster.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>
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

struct Point2d
{
  double u = 0.0;
  double v = 0.0;
  std::size_t node_index = 0;
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

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
  const double upper = values[middle];
  if (values.size() % 2U != 0U) {
    return upper;
  }
  const auto lower_it = std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
  return 0.5 * (*lower_it + upper);
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
  fit.planarity = std::max(0.0, (eigenvalues.y() - eigenvalues.x()) / largest);
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

  // Eigenvectors are sign-ambiguous.  A deterministic sign keeps marker
  // arrows from flickering if the upstream map does not supply a normal.
  Eigen::Index dominant_axis = 0;
  normal.cwiseAbs().maxCoeff(&dominant_axis);
  if (normal[dominant_axis] < 0.0) {
    normal = -normal;
  }
}

double cross2d(const Point2d &origin, const Point2d &first, const Point2d &second)
{
  return (first.u - origin.u) * (second.v - origin.v) -
         (first.v - origin.v) * (second.u - origin.u);
}

std::vector<Point2d> convexHull(std::vector<Point2d> points)
{
  if (points.size() < 3U) {
    return {};
  }
  std::sort(points.begin(), points.end(), [](const Point2d &left, const Point2d &right) {
      if (left.u != right.u) {
        return left.u < right.u;
      }
      if (left.v != right.v) {
        return left.v < right.v;
      }
      return left.node_index < right.node_index;
    });

  std::vector<Point2d> unique;
  unique.reserve(points.size());
  for (const auto &point : points) {
    if (unique.empty() || std::abs(point.u - unique.back().u) > kEpsilon ||
      std::abs(point.v - unique.back().v) > kEpsilon)
    {
      unique.push_back(point);
    }
  }
  if (unique.size() < 3U) {
    return {};
  }

  std::vector<Point2d> hull;
  hull.reserve(unique.size() * 2U);
  for (const auto &point : unique) {
    while (hull.size() >= 2U &&
      cross2d(hull[hull.size() - 2U], hull.back(), point) <= kEpsilon)
    {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const std::size_t lower_size = hull.size();
  for (auto it = unique.rbegin() + 1; it != unique.rend(); ++it) {
    while (hull.size() > lower_size &&
      cross2d(hull[hull.size() - 2U], hull.back(), *it) <= kEpsilon)
    {
      hull.pop_back();
    }
    hull.push_back(*it);
  }
  if (!hull.empty()) {
    hull.pop_back();
  }
  return hull.size() >= 3U ? hull : std::vector<Point2d>{};
}

double polygonArea(const std::vector<Point2d> &polygon)
{
  if (polygon.size() < 3U) {
    return 0.0;
  }
  double twice_area = 0.0;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto &first = polygon[index];
    const auto &second = polygon[(index + 1U) % polygon.size()];
    twice_area += first.u * second.v - first.v * second.u;
  }
  return 0.5 * std::abs(twice_area);
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

}  // namespace

namespace fuzzrobo::topological_plane
{

TopologicalPlaneClusterExtractor::TopologicalPlaneClusterExtractor(PlaneClusterOptions options)
: options_(std::move(options))
{
  options_.minimum_node_planarity = std::clamp(options_.minimum_node_planarity, 0.0, 1.0);
  options_.minimum_cluster_planarity = std::clamp(options_.minimum_cluster_planarity, 0.0, 1.0);
  options_.normal_alignment_cosine = std::clamp(options_.normal_alignment_cosine, 0.0, 1.0);
  options_.maximum_normalized_edge_residual = std::max(0.0, options_.maximum_normalized_edge_residual);
  options_.maximum_normalized_cluster_residual = std::max(
    0.0, options_.maximum_normalized_cluster_residual);
  options_.minimum_cluster_nodes = std::max<std::size_t>(3U, options_.minimum_cluster_nodes);
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
  for (auto &neighbors : adjacency) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
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

    std::vector<double> edge_lengths;
    edge_lengths.reserve(adjacency[node_index].size());
    std::vector<std::size_t> patch_indices;
    patch_indices.reserve(adjacency[node_index].size() + 1U);
    patch_indices.push_back(node_index);
    for (const std::size_t neighbor : adjacency[node_index]) {
      if (!features[neighbor].valid && !finiteVector(pointOf(map.nodes[neighbor].pos))) {
        continue;
      }
      patch_indices.push_back(neighbor);
      const double distance = (feature.position - pointOf(map.nodes[neighbor].pos)).norm();
      if (std::isfinite(distance) && distance > kEpsilon) {
        edge_lengths.push_back(distance);
      }
    }

    // Boundary nodes have an anisotropic one-hop fan even on a perfectly flat
    // surface.  Expand only low-degree fans to two hops for the local PCA;
    // the eventual region grow still follows direct edges and therefore
    // cannot bridge a gap or merge two disconnected objects.
    if (adjacency[node_index].size() <= 3U) {
      for (const std::size_t neighbor : adjacency[node_index]) {
        for (const std::size_t two_hop : adjacency[neighbor]) {
          patch_indices.push_back(two_hop);
        }
      }
      std::sort(patch_indices.begin(), patch_indices.end());
      patch_indices.erase(std::unique(patch_indices.begin(), patch_indices.end()), patch_indices.end());
    }
    feature.local_spacing = median(std::move(edge_lengths));
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
    feature.planar = fit.planarity >= options_.minimum_node_planarity;
    if (feature.planar) {
      ++result.statistics.locally_planar_node_count;
    }
  }

  std::vector<bool> visited(node_count, false);
  std::uint32_t cluster_id = 1U;
  for (std::size_t seed = 0; seed < node_count; ++seed) {
    if (visited[seed] || !features[seed].planar) {
      continue;
    }
    std::vector<std::size_t> component;
    std::queue<std::size_t> frontier;
    frontier.push(seed);
    visited[seed] = true;
    while (!frontier.empty()) {
      const std::size_t current = frontier.front();
      frontier.pop();
      component.push_back(current);
      const NodeFeature &current_feature = features[current];
      for (const std::size_t neighbor : adjacency[current]) {
        if (visited[neighbor] || !features[neighbor].planar) {
          continue;
        }
        const NodeFeature &neighbor_feature = features[neighbor];
        if (std::abs(current_feature.normal.dot(neighbor_feature.normal)) <
          options_.normal_alignment_cosine)
        {
          continue;
        }
        const Eigen::Vector3d edge = neighbor_feature.position - current_feature.position;
        const double current_residual = std::abs(current_feature.normal.dot(edge)) /
          std::max(current_feature.local_spacing, kEpsilon);
        const double neighbor_residual = std::abs(neighbor_feature.normal.dot(edge)) /
          std::max(neighbor_feature.local_spacing, kEpsilon);
        if (std::max(current_residual, neighbor_residual) >
          options_.maximum_normalized_edge_residual)
        {
          continue;
        }
        visited[neighbor] = true;
        frontier.push(neighbor);
      }
    }

    if (component.size() < options_.minimum_cluster_nodes) {
      continue;
    }
    std::sort(component.begin(), component.end());
    std::vector<Eigen::Vector3d> component_points;
    std::vector<double> component_spacings;
    component_points.reserve(component.size());
    component_spacings.reserve(component.size());
    Eigen::Vector3d reference_normal = Eigen::Vector3d::Zero();
    for (const std::size_t index : component) {
      component_points.push_back(features[index].position);
      component_spacings.push_back(features[index].local_spacing);
      reference_normal += features[index].normal;
    }
    PlaneFit fit = fitPlane(component_points);
    const double local_spacing = median(std::move(component_spacings));
    if (!fit.valid || local_spacing <= kEpsilon ||
      fit.planarity < options_.minimum_cluster_planarity ||
      fit.residual / local_spacing > options_.maximum_normalized_cluster_residual)
    {
      continue;
    }
    orientNormal(fit.normal, reference_normal);

    Eigen::Vector3d tangent_u = fit.normal.unitOrthogonal();
    Eigen::Vector3d tangent_v = fit.normal.cross(tangent_u).normalized();
    std::vector<Point2d> projected;
    projected.reserve(component.size());
    double minimum_u = std::numeric_limits<double>::infinity();
    double maximum_u = -std::numeric_limits<double>::infinity();
    double minimum_v = std::numeric_limits<double>::infinity();
    double maximum_v = -std::numeric_limits<double>::infinity();
    for (const std::size_t index : component) {
      const Eigen::Vector3d delta = features[index].position - fit.centroid;
      const double u = delta.dot(tangent_u);
      const double v = delta.dot(tangent_v);
      projected.push_back(Point2d{u, v, index});
      minimum_u = std::min(minimum_u, u);
      maximum_u = std::max(maximum_u, u);
      minimum_v = std::min(minimum_v, v);
      maximum_v = std::max(maximum_v, v);
    }
    const std::vector<Point2d> hull = convexHull(std::move(projected));
    const double area = polygonArea(hull);
    if (hull.empty() || area <= kEpsilon) {
      continue;
    }

    ais_gng_msgs::msg::PlanarCluster cluster;
    cluster.id = cluster_id++;
    cluster.node_indices.reserve(component.size());
    for (const std::size_t index : component) {
      cluster.node_indices.push_back(static_cast<std::uint32_t>(index));
    }
    cluster.centroid = pointMessage(fit.centroid);
    cluster.normal = vectorMessage(fit.normal);
    cluster.tangent_u = vectorMessage(tangent_u);
    cluster.tangent_v = vectorMessage(tangent_v);
    cluster.boundary.reserve(hull.size());
    for (const auto &vertex : hull) {
      cluster.boundary.push_back(pointMessage(
        fit.centroid + tangent_u * vertex.u + tangent_v * vertex.v));
    }
    cluster.area = static_cast<float>(area);
    cluster.extent_u = static_cast<float>(maximum_u - minimum_u);
    cluster.extent_v = static_cast<float>(maximum_v - minimum_v);
    cluster.local_spacing = static_cast<float>(local_spacing);
    cluster.planarity = static_cast<float>(fit.planarity);
    cluster.residual_ratio = static_cast<float>(fit.residual / local_spacing);
    result.statistics.clustered_node_count += component.size();
    result.clusters.clusters.push_back(std::move(cluster));
  }
  result.statistics.cluster_count = result.clusters.clusters.size();
  return result;
}

}  // namespace fuzzrobo::topological_plane
