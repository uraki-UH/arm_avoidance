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
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace grasping_system::graph
{

enum class GripperVolumeShape
{
  kBox,
  kEllipsoid,
  kCylinder
};

struct GripperVolumeMeshExclusion
{
  std::string path;
  std::array<double, 3> scale{1.0, 1.0, 1.0};
  geometry_msgs::msg::Pose pose_in_frame{};
};

struct GripperVolumeGraphSpec
{
  GripperVolumeShape shape{GripperVolumeShape::kBox};
  std::array<double, 3> dimensions{0.08, 0.04, 0.10};
  double resolution{0.01};
  geometry_msgs::msg::Pose pose_in_frame{};
  std::vector<GripperVolumeMeshExclusion> mesh_exclusions;
  double mesh_exclusion_clearance{0.0};
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
    const auto exclusions = prepareMeshExclusions(spec.mesh_exclusions);

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
          const Eigen::Vector3d position = translation + rotation * local;
          if (isMeshOccupied(position, exclusions, spec.mesh_exclusion_clearance)) {
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
  struct Triangle
  {
    Eigen::Vector3d a;
    Eigen::Vector3d b;
    Eigen::Vector3d c;
  };

  struct PreparedMeshExclusion
  {
    std::vector<Triangle> triangles;
    Eigen::Vector3d min = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
    Eigen::Vector3d max = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
  };

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
    if (!std::isfinite(spec.mesh_exclusion_clearance) ||
      spec.mesh_exclusion_clearance < 0.0)
    {
      throw std::invalid_argument("mesh exclusion clearance must not be negative");
    }
    for (const auto &exclusion : spec.mesh_exclusions) {
      if (exclusion.path.empty()) {
        throw std::invalid_argument("mesh exclusion path must not be empty");
      }
      for (const double scale : exclusion.scale) {
        if (!std::isfinite(scale) || scale == 0.0) {
          throw std::invalid_argument("mesh exclusion scales must be finite and non-zero");
        }
      }
    }
  }

  static std::vector<PreparedMeshExclusion> prepareMeshExclusions(
    std::vector<GripperVolumeMeshExclusion> exclusions)
  {
    std::vector<PreparedMeshExclusion> prepared;
    prepared.reserve(exclusions.size());
    for (auto &exclusion : exclusions) {
      normalizeOrientation(exclusion.pose_in_frame);
      prepared.push_back(loadBinaryStl(exclusion));
    }
    return prepared;
  }

  static PreparedMeshExclusion loadBinaryStl(
    const GripperVolumeMeshExclusion &exclusion)
  {
    std::ifstream input(exclusion.path, std::ios::binary);
    if (!input) {
      throw std::runtime_error("failed to open mesh exclusion: " + exclusion.path);
    }

    std::array<char, 80> header{};
    std::uint32_t triangle_count = 0;
    input.read(header.data(), static_cast<std::streamsize>(header.size()));
    input.read(reinterpret_cast<char *>(&triangle_count), sizeof(triangle_count));
    if (!input) {
      throw std::runtime_error("invalid binary STL header: " + exclusion.path);
    }

    PreparedMeshExclusion mesh;
    mesh.triangles.reserve(triangle_count);
    const Eigen::Quaterniond rotation = toEigen(exclusion.pose_in_frame.orientation);
    const Eigen::Vector3d translation(
      exclusion.pose_in_frame.position.x,
      exclusion.pose_in_frame.position.y,
      exclusion.pose_in_frame.position.z);
    const Eigen::Vector3d scale = Eigen::Map<const Eigen::Vector3d>(
      exclusion.scale.data());

    for (std::uint32_t index = 0; index < triangle_count; ++index) {
      std::array<float, 3> normal{};
      std::array<std::array<float, 3>, 3> raw_vertices{};
      std::uint16_t attributes = 0;
      input.read(reinterpret_cast<char *>(normal.data()), sizeof(normal));
      input.read(
        reinterpret_cast<char *>(raw_vertices.data()),
        static_cast<std::streamsize>(sizeof(raw_vertices)));
      input.read(reinterpret_cast<char *>(&attributes), sizeof(attributes));
      if (!input) {
        throw std::runtime_error("truncated binary STL: " + exclusion.path);
      }

      Triangle triangle;
      std::array<Eigen::Vector3d *, 3> vertices{&triangle.a, &triangle.b, &triangle.c};
      for (std::size_t vertex_index = 0; vertex_index < vertices.size(); ++vertex_index) {
        const auto &raw = raw_vertices[vertex_index];
        const Eigen::Vector3d local(raw[0], raw[1], raw[2]);
        *vertices[vertex_index] = translation + rotation * local.cwiseProduct(scale);
        mesh.min = mesh.min.cwiseMin(*vertices[vertex_index]);
        mesh.max = mesh.max.cwiseMax(*vertices[vertex_index]);
      }
      mesh.triangles.push_back(std::move(triangle));
    }
    if (mesh.triangles.empty()) {
      throw std::runtime_error("mesh exclusion contains no triangles: " + exclusion.path);
    }
    return mesh;
  }

  static bool isMeshOccupied(
    const Eigen::Vector3d &point,
    const std::vector<PreparedMeshExclusion> &meshes,
    double clearance)
  {
    constexpr double kSurfaceTolerance = 1e-12;
    constexpr double kTwoPi = 6.28318530717958647692;
    const double clearance_squared = clearance * clearance;
    for (const auto &mesh : meshes) {
      const Eigen::Array3d padding = Eigen::Array3d::Constant(clearance);
      if ((point.array() < mesh.min.array() - padding).any() ||
        (point.array() > mesh.max.array() + padding).any())
      {
        continue;
      }

      double winding = 0.0;
      double min_distance_squared = std::numeric_limits<double>::infinity();
      for (const auto &triangle : mesh.triangles) {
        min_distance_squared = std::min(
          min_distance_squared, pointTriangleDistanceSquared(point, triangle));
        winding += solidAngle(point, triangle);
      }
      if (min_distance_squared <= clearance_squared + kSurfaceTolerance ||
        std::abs(winding) > kTwoPi)
      {
        return true;
      }
    }
    return false;
  }

  static double solidAngle(const Eigen::Vector3d &point, const Triangle &triangle)
  {
    const Eigen::Vector3d a = triangle.a - point;
    const Eigen::Vector3d b = triangle.b - point;
    const Eigen::Vector3d c = triangle.c - point;
    const double denominator =
      a.norm() * b.norm() * c.norm() + a.dot(b) * c.norm() +
      b.dot(c) * a.norm() + c.dot(a) * b.norm();
    return 2.0 * std::atan2(a.dot(b.cross(c)), denominator);
  }

  static double pointTriangleDistanceSquared(
    const Eigen::Vector3d &point, const Triangle &triangle)
  {
    const Eigen::Vector3d ab = triangle.b - triangle.a;
    const Eigen::Vector3d ac = triangle.c - triangle.a;
    const Eigen::Vector3d ap = point - triangle.a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
      return ap.squaredNorm();
    }

    const Eigen::Vector3d bp = point - triangle.b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) {
      return bp.squaredNorm();
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
      const Eigen::Vector3d projection = triangle.a + (d1 / (d1 - d3)) * ab;
      return (point - projection).squaredNorm();
    }

    const Eigen::Vector3d cp = point - triangle.c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) {
      return cp.squaredNorm();
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
      const Eigen::Vector3d projection = triangle.a + (d2 / (d2 - d6)) * ac;
      return (point - projection).squaredNorm();
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0) {
      const Eigen::Vector3d bc = triangle.c - triangle.b;
      const Eigen::Vector3d projection =
        triangle.b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * bc;
      return (point - projection).squaredNorm();
    }

    const double inverse_sum = 1.0 / (va + vb + vc);
    const Eigen::Vector3d projection =
      triangle.a + ab * (vb * inverse_sum) + ac * (vc * inverse_sum);
    return (point - projection).squaredNorm();
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
