#pragma once

#include <ais_gng_msgs/msg/planar_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace grasping_system::candidate
{

struct TopGraspSurfaceConfig
{
  Eigen::Vector3d up_axis = Eigen::Vector3d::UnitZ();
  double higher_neighbor_z_tolerance = 0.01;
  std::size_t minimum_region_nodes = 4U;
  double grasp_size_x = 0.061;
  double grasp_size_y = 0.074;
  double footprint_margin = 0.005;
  double footprint_padding = 0.005;
  double tcp_standoff = 0.0;
  std::size_t maximum_candidates = 20U;
};

struct TopGraspSurfaceCandidate
{
  std::uint32_t cluster_id = 0U;
  std::vector<std::uint32_t> cluster_ids;
  std::vector<std::uint32_t> node_indices;
  Eigen::Vector3d tcp_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond tcp_orientation = Eigen::Quaterniond::Identity();
  double extent_x = 0.0;
  double extent_y = 0.0;
  double surface_height = 0.0;
  double footprint_fill_ratio = 0.0;
};

struct TopGraspSurfaceResult
{
  std::size_t region_count = 0U;
  std::size_t adjacent_region_pair_count = 0U;
  std::size_t candidate_group_count = 0U;
  std::size_t rejected_small_region = 0U;
  std::size_t rejected_invalid_region = 0U;
  std::size_t rejected_seed_oversize_region = 0U;
  std::size_t rejected_group_oversize_region = 0U;
  std::vector<TopGraspSurfaceCandidate> candidates;
};

class TopGraspSurfaceEstimator
{
public:
  explicit TopGraspSurfaceEstimator(TopGraspSurfaceConfig config = {})
  : config_(std::move(config))
  {
    validateConfig();
  }

  TopGraspSurfaceResult estimate(
    const ais_gng_msgs::msg::TopologicalMap &map,
    const ais_gng_msgs::msg::PlanarClusterArray &clusters) const
  {
    TopGraspSurfaceResult result;
    result.region_count = clusters.clusters.size();
    if (map.nodes.empty() || clusters.clusters.empty()) {
      return result;
    }

    const auto [basis_u, basis_v] = horizontalBasis();
    const std::size_t region_count = clusters.clusters.size();
    std::vector<double> centroid_heights(region_count, 0.0);
    std::vector<std::uint8_t> seed_eligible(region_count, 0U);
    std::vector<int> owner_by_node(map.nodes.size(), -1);

    for (std::size_t region_index = 0U; region_index < region_count; ++region_index) {
      const auto &cluster = clusters.clusters[region_index];
      centroid_heights[region_index] = config_.up_axis.dot(Eigen::Vector3d(
          cluster.centroid.x, cluster.centroid.y, cluster.centroid.z));
      if (!std::isfinite(centroid_heights[region_index]) || std::any_of(
          cluster.node_indices.begin(), cluster.node_indices.end(),
          [&map](std::uint32_t node_index) {return node_index >= map.nodes.size();}))
      {
        ++result.rejected_invalid_region;
        continue;
      }
      for (const std::uint32_t node_index : cluster.node_indices) {
        if (owner_by_node[node_index] < 0) {
          owner_by_node[node_index] = static_cast<int>(region_index);
        }
      }
      if (cluster.node_indices.size() < config_.minimum_region_nodes) {
        ++result.rejected_small_region;
        continue;
      }
      const Footprint footprint = fitFootprint(cluster.node_indices, map, basis_u, basis_v);
      if (!footprint.valid) {
        ++result.rejected_invalid_region;
      } else if (!footprint.fits) {
        ++result.rejected_seed_oversize_region;
      } else {
        seed_eligible[region_index] = 1U;
      }
    }

    std::vector<std::unordered_set<std::size_t>> adjacency(region_count);
    for (std::size_t edge = 0U; edge + 1U < map.edges.size(); edge += 2U) {
      const std::size_t first_node = map.edges[edge];
      const std::size_t second_node = map.edges[edge + 1U];
      if (first_node >= owner_by_node.size() || second_node >= owner_by_node.size()) {
        continue;
      }
      const int first_owner = owner_by_node[first_node];
      const int second_owner = owner_by_node[second_node];
      if (first_owner < 0 || second_owner < 0 || first_owner == second_owner) {
        continue;
      }
      adjacency[static_cast<std::size_t>(first_owner)].insert(
        static_cast<std::size_t>(second_owner));
      adjacency[static_cast<std::size_t>(second_owner)].insert(
        static_cast<std::size_t>(first_owner));
    }
    for (const auto &neighbours : adjacency) {
      result.adjacent_region_pair_count += neighbours.size();
    }
    result.adjacent_region_pair_count /= 2U;

    std::unordered_map<std::size_t, std::unordered_set<std::size_t>> group_by_apex;
    for (std::size_t seed = 0U; seed < region_count; ++seed) {
      if (seed_eligible[seed] == 0U) {
        continue;
      }
      const std::vector<std::size_t> closure = higherClosure(
        seed, adjacency, centroid_heights);
      std::size_t apex = seed;
      for (const std::size_t region : closure) {
        if (centroid_heights[region] > centroid_heights[apex] ||
          (centroid_heights[region] == centroid_heights[apex] &&
          clusters.clusters[region].id < clusters.clusters[apex].id))
        {
          apex = region;
        }
      }
      auto &group = group_by_apex[apex];
      group.insert(closure.begin(), closure.end());
    }

    result.candidate_group_count = group_by_apex.size();
    for (auto &[apex, region_indices] : group_by_apex) {
      std::unordered_set<std::uint32_t> unique_nodes;
      std::vector<std::uint32_t> cluster_ids;
      for (const std::size_t region_index : region_indices) {
        const auto &cluster = clusters.clusters[region_index];
        cluster_ids.push_back(cluster.id);
        unique_nodes.insert(cluster.node_indices.begin(), cluster.node_indices.end());
      }
      std::vector<std::uint32_t> node_indices(unique_nodes.begin(), unique_nodes.end());
      std::sort(node_indices.begin(), node_indices.end());
      std::sort(cluster_ids.begin(), cluster_ids.end());
      const Footprint footprint = fitFootprint(node_indices, map, basis_u, basis_v);
      if (!footprint.valid || !footprint.fits) {
        ++result.rejected_group_oversize_region;
        continue;
      }

      const Eigen::Vector3d world_y =
        (basis_u * footprint.local_y_axis.x() +
        basis_v * footprint.local_y_axis.y()).normalized();
      const Eigen::Vector3d world_z = -config_.up_axis;
      const Eigen::Vector3d world_x = world_y.cross(world_z).normalized();
      Eigen::Matrix3d rotation;
      rotation.col(0) = world_x;
      rotation.col(1) = world_y;
      rotation.col(2) = world_z;

      TopGraspSurfaceCandidate candidate;
      candidate.cluster_id = clusters.clusters[apex].id;
      candidate.cluster_ids = std::move(cluster_ids);
      candidate.node_indices = std::move(node_indices);
      candidate.tcp_position = basis_u * footprint.center_uv.x() +
        basis_v * footprint.center_uv.y() +
        config_.up_axis * (footprint.maximum_height + config_.tcp_standoff);
      candidate.tcp_orientation = Eigen::Quaterniond(rotation).normalized();
      candidate.extent_x = footprint.extent_x;
      candidate.extent_y = footprint.extent_y;
      candidate.surface_height = footprint.maximum_height;
      candidate.footprint_fill_ratio = footprint.fill_ratio;
      result.candidates.push_back(std::move(candidate));
    }

    std::sort(
      result.candidates.begin(), result.candidates.end(),
      [](const TopGraspSurfaceCandidate &first, const TopGraspSurfaceCandidate &second) {
        if (first.surface_height != second.surface_height) {
          return first.surface_height > second.surface_height;
        }
        return first.footprint_fill_ratio > second.footprint_fill_ratio;
      });
    if (result.candidates.size() > config_.maximum_candidates) {
      result.candidates.resize(config_.maximum_candidates);
    }
    return result;
  }

private:
  struct Footprint
  {
    bool valid = false;
    bool fits = false;
    Eigen::Vector2d local_y_axis = Eigen::Vector2d::UnitY();
    Eigen::Vector2d center_uv = Eigen::Vector2d::Zero();
    double extent_x = 0.0;
    double extent_y = 0.0;
    double maximum_height = 0.0;
    double fill_ratio = 0.0;
  };

  void validateConfig()
  {
    if (!config_.up_axis.allFinite() || config_.up_axis.norm() < 1.0e-12) {
      throw std::invalid_argument("up_axis must be finite and non-zero");
    }
    config_.up_axis.normalize();
    if (!std::isfinite(config_.higher_neighbor_z_tolerance) ||
      config_.higher_neighbor_z_tolerance < 0.0)
    {
      throw std::invalid_argument("higher_neighbor_z_tolerance must be finite and non-negative");
    }
    if (!std::isfinite(config_.grasp_size_x) || !std::isfinite(config_.grasp_size_y) ||
      config_.grasp_size_x <= 0.0 || config_.grasp_size_y <= 0.0 ||
      config_.footprint_margin < 0.0 || config_.footprint_padding < 0.0 ||
      2.0 * config_.footprint_margin >= std::min(config_.grasp_size_x, config_.grasp_size_y))
    {
      throw std::invalid_argument("grasp footprint parameters are invalid");
    }
    if (config_.minimum_region_nodes == 0U || config_.maximum_candidates == 0U) {
      throw std::invalid_argument("region and candidate limits must be positive");
    }
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> horizontalBasis() const
  {
    const Eigen::Vector3d reference =
      std::abs(config_.up_axis.dot(Eigen::Vector3d::UnitX())) < 0.9 ?
      Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d basis_u =
      (reference - config_.up_axis * config_.up_axis.dot(reference)).normalized();
    return {basis_u, config_.up_axis.cross(basis_u).normalized()};
  }

  std::vector<std::size_t> higherClosure(
    std::size_t seed,
    const std::vector<std::unordered_set<std::size_t>> &adjacency,
    const std::vector<double> &centroid_heights) const
  {
    std::vector<std::size_t> closure;
    std::vector<std::uint8_t> visited(adjacency.size(), 0U);
    std::queue<std::size_t> frontier;
    visited[seed] = 1U;
    frontier.push(seed);
    while (!frontier.empty()) {
      const std::size_t current = frontier.front();
      frontier.pop();
      closure.push_back(current);
      for (const std::size_t neighbour : adjacency[current]) {
        if (visited[neighbour] != 0U ||
          centroid_heights[neighbour] <=
          centroid_heights[current] + config_.higher_neighbor_z_tolerance)
        {
          continue;
        }
        visited[neighbour] = 1U;
        frontier.push(neighbour);
      }
    }
    return closure;
  }

  Footprint fitFootprint(
    const std::vector<std::uint32_t> &node_indices,
    const ais_gng_msgs::msg::TopologicalMap &map,
    const Eigen::Vector3d &basis_u,
    const Eigen::Vector3d &basis_v) const
  {
    Footprint result;
    if (node_indices.size() < config_.minimum_region_nodes) {
      return result;
    }
    std::vector<Eigen::Vector2d> projected;
    projected.reserve(node_indices.size());
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    result.maximum_height = -std::numeric_limits<double>::infinity();
    for (const std::uint32_t node_index : node_indices) {
      if (node_index >= map.nodes.size()) {
        return result;
      }
      const auto &point = map.nodes[node_index].pos;
      const Eigen::Vector3d position(point.x, point.y, point.z);
      if (!position.allFinite()) {
        return result;
      }
      projected.emplace_back(position.dot(basis_u), position.dot(basis_v));
      mean += projected.back();
      result.maximum_height = std::max(
        result.maximum_height, position.dot(config_.up_axis));
    }
    mean /= static_cast<double>(projected.size());

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const Eigen::Vector2d &point : projected) {
      const Eigen::Vector2d delta = point - mean;
      covariance.noalias() += delta * delta.transpose();
    }
    covariance /= static_cast<double>(projected.size());
    const double principal_angle = 0.5 * std::atan2(
      2.0 * covariance(0, 1), covariance(0, 0) - covariance(1, 1));
    const Eigen::Vector2d major_axis(std::cos(principal_angle), std::sin(principal_angle));
    const Eigen::Vector2d minor_axis(-major_axis.y(), major_axis.x());

    const auto bounds = [&projected](const Eigen::Vector2d &axis) {
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = -std::numeric_limits<double>::infinity();
        for (const Eigen::Vector2d &point : projected) {
          const double projection = point.dot(axis);
          minimum = std::min(minimum, projection);
          maximum = std::max(maximum, projection);
        }
        return std::pair<double, double>{minimum, maximum};
      };
    const auto major_bounds = bounds(major_axis);
    const auto minor_bounds = bounds(minor_axis);
    const double major_extent = major_bounds.second - major_bounds.first +
      2.0 * config_.footprint_padding;
    const double minor_extent = minor_bounds.second - minor_bounds.first +
      2.0 * config_.footprint_padding;
    const double usable_x = config_.grasp_size_x - 2.0 * config_.footprint_margin;
    const double usable_y = config_.grasp_size_y - 2.0 * config_.footprint_margin;

    Eigen::Vector2d local_x_axis = Eigen::Vector2d::UnitX();
    double center_x = 0.0;
    double center_y = 0.0;
    if (major_extent <= usable_x && minor_extent <= usable_y) {
      local_x_axis = major_axis;
      result.local_y_axis = minor_axis;
      result.extent_x = major_extent;
      result.extent_y = minor_extent;
      center_x = 0.5 * (major_bounds.first + major_bounds.second);
      center_y = 0.5 * (minor_bounds.first + minor_bounds.second);
      result.fits = true;
    } else if (minor_extent <= usable_x && major_extent <= usable_y) {
      local_x_axis = minor_axis;
      result.local_y_axis = major_axis;
      result.extent_x = minor_extent;
      result.extent_y = major_extent;
      center_x = 0.5 * (minor_bounds.first + minor_bounds.second);
      center_y = 0.5 * (major_bounds.first + major_bounds.second);
      result.fits = true;
    } else {
      result.extent_x = major_extent;
      result.extent_y = minor_extent;
    }
    if (result.fits) {
      result.center_uv = local_x_axis * center_x + result.local_y_axis * center_y;
    }
    result.fill_ratio = (result.extent_x * result.extent_y) /
      std::max(usable_x * usable_y, 1.0e-12);
    result.valid = true;
    return result;
  }

  TopGraspSurfaceConfig config_;
};

}  // namespace grasping_system::candidate
