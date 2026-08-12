#pragma once

#include <graph/grasp_graph_model.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace grasping_system::graph
{

enum class GripperVolumeShape
{
  kBox,
  kEllipsoid,
  kCylinder
};

struct GripperVolumeGraphSpec
{
  GripperVolumeShape shape{GripperVolumeShape::kBox};
  std::array<double, 3> dimensions{0.08, 0.04, 0.10};
  double resolution{0.01};
  geometry_msgs::msg::Pose pose_in_frame{};
};

struct GripperVolumeGraph
{
  GraspGraphModel graph;
  GripperVolumeGraphSpec spec;
};

inline GripperVolumeShape parseGripperVolumeShape(const std::string &value)
{
  if (value == "box") {
    return GripperVolumeShape::kBox;
  }
  if (value == "ellipsoid" || value == "sphere") {
    return GripperVolumeShape::kEllipsoid;
  }
  if (value == "cylinder") {
    return GripperVolumeShape::kCylinder;
  }
  throw std::invalid_argument(
          "unsupported gripper volume shape '" + value +
          "' (expected box, ellipsoid, or cylinder)");
}

class GripperVolumeGraphBuilder
{
public:
  static GripperVolumeGraph build(GripperVolumeGraphSpec spec)
  {
    validate(spec);
    normalizeOrientation(spec.pose_in_frame);

    GripperVolumeGraph result;
    result.spec = spec;

    const auto counts = cellCounts(spec);
    const Eigen::Quaterniond rotation = toEigen(spec.pose_in_frame.orientation);
    const Eigen::Vector3d translation(
      spec.pose_in_frame.position.x,
      spec.pose_in_frame.position.y,
      spec.pose_in_frame.position.z);

    std::unordered_map<GridKey, int, GridKeyHash> node_ids;
    for (int x = 0; x < counts[0]; ++x) {
      for (int y = 0; y < counts[1]; ++y) {
        for (int z = 0; z < counts[2]; ++z) {
          const GridKey key{x, y, z};
          const Eigen::Vector3d local = localCenter(key, counts, spec.resolution);
          if (!contains(local, spec)) {
            continue;
          }
          if (result.graph.nodes().size() >=
            static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max()))
          {
            throw std::length_error(
                    "gripper volume exceeds TopologicalMap uint16 node capacity; increase resolution");
          }

          GraspGraphNode node;
          node.id = static_cast<int>(result.graph.nodes().size());
          const Eigen::Vector3d position = translation + rotation * local;
          node.pose_in_object.position.x = position.x();
          node.pose_in_object.position.y = position.y();
          node.pose_in_object.position.z = position.z();
          node.pose_in_object.orientation = spec.pose_in_frame.orientation;
          node.weight = 1.0;
          result.graph.addNode(node);
          node_ids.emplace(key, node.id);
        }
      }
    }

    static constexpr std::array<GridKey, 3> kPositiveNeighbors{
      GridKey{1, 0, 0}, GridKey{0, 1, 0}, GridKey{0, 0, 1}};
    for (const auto &[key, node_id] : node_ids) {
      for (const auto &offset : kPositiveNeighbors) {
        const GridKey neighbor{
          key.x + offset.x, key.y + offset.y, key.z + offset.z};
        const auto neighbor_it = node_ids.find(neighbor);
        if (neighbor_it == node_ids.end()) {
          continue;
        }
        result.graph.addEdge({node_id, neighbor_it->second, spec.resolution, true});
      }
    }
    return result;
  }

private:
  struct GridKey
  {
    int x;
    int y;
    int z;

    bool operator==(const GridKey &other) const noexcept
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct GridKeyHash
  {
    std::size_t operator()(const GridKey &key) const noexcept
    {
      std::size_t seed = std::hash<int>{}(key.x);
      seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
      seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
      return seed;
    }
  };

  static void validate(const GripperVolumeGraphSpec &spec)
  {
    if (!std::isfinite(spec.resolution) || spec.resolution <= 0.0) {
      throw std::invalid_argument("gripper volume resolution must be positive");
    }
    for (const double dimension : spec.dimensions) {
      if (!std::isfinite(dimension) || dimension <= 0.0) {
        throw std::invalid_argument("all gripper volume dimensions must be positive");
      }
    }
  }

  static void normalizeOrientation(geometry_msgs::msg::Pose &pose)
  {
    Eigen::Quaterniond q = toEigen(pose.orientation);
    if (!q.coeffs().allFinite() || q.norm() < 1e-12) {
      q = Eigen::Quaterniond::Identity();
    } else {
      q.normalize();
    }
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
  }

  static Eigen::Quaterniond toEigen(const geometry_msgs::msg::Quaternion &q)
  {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  static std::array<int, 3> cellCounts(const GripperVolumeGraphSpec &spec)
  {
    std::array<int, 3> counts{};
    for (std::size_t axis = 0; axis < counts.size(); ++axis) {
      counts[axis] = std::max(
        1, static_cast<int>(std::ceil(spec.dimensions[axis] / spec.resolution)));
    }
    return counts;
  }

  static Eigen::Vector3d localCenter(
    const GridKey &key, const std::array<int, 3> &counts, double resolution)
  {
    return {
      (static_cast<double>(key.x) - 0.5 * static_cast<double>(counts[0] - 1)) * resolution,
      (static_cast<double>(key.y) - 0.5 * static_cast<double>(counts[1] - 1)) * resolution,
      (static_cast<double>(key.z) - 0.5 * static_cast<double>(counts[2] - 1)) * resolution};
  }

  static bool contains(
    const Eigen::Vector3d &point, const GripperVolumeGraphSpec &spec)
  {
    const Eigen::Array3d half = Eigen::Map<const Eigen::Vector3d>(
      spec.dimensions.data()).array() * 0.5;
    constexpr double kTolerance = 1e-9;
    switch (spec.shape) {
      case GripperVolumeShape::kBox:
        return (point.array().abs() <= half + kTolerance).all();
      case GripperVolumeShape::kEllipsoid:
        return (point.array() / half).square().sum() <= 1.0 + kTolerance;
      case GripperVolumeShape::kCylinder: {
          const double radial =
            std::pow(point.x() / half.x(), 2.0) +
            std::pow(point.y() / half.y(), 2.0);
          return radial <= 1.0 + kTolerance &&
                 std::abs(point.z()) <= half.z() + kTolerance;
        }
    }
    return false;
  }
};

}  // namespace grasping_system::graph
