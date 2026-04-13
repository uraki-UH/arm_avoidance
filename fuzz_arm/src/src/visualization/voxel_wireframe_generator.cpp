#include "spatial/voxel_wireframe_generator.hpp"
#include <algorithm>
#include <set>

namespace GNG {
namespace Analysis {

struct Edge {
  Eigen::Vector3i v1, v2;
  bool operator<(const Edge &other) const {
    if (v1.x() != other.v1.x())
      return v1.x() < other.v1.x();
    if (v1.y() != other.v1.y())
      return v1.y() < other.v1.y();
    if (v1.z() != other.v1.z())
      return v1.z() < other.v1.z();
    if (v2.x() != other.v2.x())
      return v2.x() < other.v2.x();
    if (v2.y() != other.v2.y())
      return v2.y() < other.v2.y();
    return v2.z() < other.v2.z();
  }
};

static Edge make_edge(Eigen::Vector3i a, Eigen::Vector3i b) {
  if (a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()) ||
      (a.x() == b.x() && a.y() == b.y() && a.z() < b.z())) {
    return {a, b};
  }
  return {b, a};
}

std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>
VoxelWireframeGenerator::generate(const std::vector<Eigen::Vector3i> &voxels,
                                  double voxel_size) {
  std::set<Edge> edges;
  float s = (float)voxel_size;

  for (const auto &v : voxels) {
    // 8 corners of the voxel
    Eigen::Vector3i c[8] = {
        v + Eigen::Vector3i(0, 0, 0), v + Eigen::Vector3i(1, 0, 0),
        v + Eigen::Vector3i(0, 1, 0), v + Eigen::Vector3i(1, 1, 0),
        v + Eigen::Vector3i(0, 0, 1), v + Eigen::Vector3i(1, 0, 1),
        v + Eigen::Vector3i(0, 1, 1), v + Eigen::Vector3i(1, 1, 1)};

    // 12 edges
    edges.insert(make_edge(c[0], c[1]));
    edges.insert(make_edge(c[2], c[3]));
    edges.insert(make_edge(c[4], c[5]));
    edges.insert(make_edge(c[6], c[7]));
    edges.insert(make_edge(c[0], c[2]));
    edges.insert(make_edge(c[1], c[3]));
    edges.insert(make_edge(c[4], c[6]));
    edges.insert(make_edge(c[5], c[7]));
    edges.insert(make_edge(c[0], c[4]));
    edges.insert(make_edge(c[1], c[5]));
    edges.insert(make_edge(c[2], c[6]));
    edges.insert(make_edge(c[3], c[7]));
  }

  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> result;
  result.reserve(edges.size());
  for (const auto &e : edges) {
    result.push_back({e.v1.cast<float>() * s, e.v2.cast<float>() * s});
  }
  return result;
}

std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>
VoxelWireframeGenerator::generateSurface(
    const std::vector<Eigen::Vector3i> &voxels, double voxel_size) {
  // For now, simple implementation returns all edges.
  // In a future optimization, we can check if an edge is shared by multiple
  // voxels and if those voxels cover all surrounding space.
  return generate(voxels, voxel_size);
}

} // namespace Analysis
} // namespace GNG
