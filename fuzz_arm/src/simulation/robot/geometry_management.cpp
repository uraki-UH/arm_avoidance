/**
 * @file geometry_management.cpp
 * @brief Implementation of mesh caching and geometry simplification.
 */

#include "simulation/robot/geometry_management.hpp"
#include "common/resource_utils.hpp"
#include "simulation/robot/stl_loader.hpp"
#include <algorithm>
#include <cctype>
#include <iostream>
#include <limits>
#include <sstream>

namespace robot_sim {
namespace simulation {

// =========================================================
// MeshCache Implementation
// =========================================================

MeshCache::MeshCache() {}

MeshCache::~MeshCache() { clear(); }

void MeshCache::clear() {
  cache_.clear();
}

std::shared_ptr<MeshEntry> MeshCache::getMesh(const std::string &filename,
                                              const Eigen::Vector3d &scale) {
  std::string resolved_path = robot_sim::common::resolvePath(filename);
  std::string ext = "";
  if (resolved_path.length() > 4) {
    ext = resolved_path.substr(resolved_path.length() - 4);
    for (auto &c : ext) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
  }

  if (ext != ".stl") {
    std::cerr << "MeshCache: Only .stl files are supported. Skipping: "
              << resolved_path << std::endl;
    return nullptr;
  }

  // Generate cache key
  std::stringstream ss;
  ss << filename << "_" << scale.x() << "_" << scale.y() << "_" << scale.z();
  std::string key = ss.str();

  auto it = cache_.find(key);
  if (it != cache_.end()) {
    return it->second;
  }

  // Load and create new entry using StlLoader
  using ::simulation::MeshData;
  using ::simulation::StlLoader;
  MeshData mesh_data = StlLoader::loadBinaryStl(resolved_path, scale);
  if (mesh_data.vertices.empty()) {
    return nullptr;
  }

  auto entry = std::make_shared<MeshEntry>();
  entry->original_vertices = mesh_data.vertices;
  entry->indices = mesh_data.indices;

  // Generate flattened vertices for rendering
  entry->flattened_vertices.reserve(entry->indices.size() * 3);
  for (uint32_t idx : entry->indices) {
    if (idx * 3 + 2 >= entry->original_vertices.size()) {
      continue;
    }
    entry->flattened_vertices.push_back(entry->original_vertices[idx * 3]);
    entry->flattened_vertices.push_back(entry->original_vertices[idx * 3 + 1]);
    entry->flattened_vertices.push_back(entry->original_vertices[idx * 3 + 2]);
  }

  cache_[key] = entry;

  std::cout << "MeshCache: Cached new mesh: " << key
            << " (triangles: " << entry->indices.size() / 3 << ")" << std::endl;

  return entry;
}

// =========================================================
// GeometrySimplifier Implementation
// =========================================================

::simulation::Geometry
GeometrySimplifier::simplifyMeshToBox(const ::simulation::MeshData &mesh_data) {
  ::simulation::Geometry simplified;
  simplified.type = ::simulation::GeometryType::BOX;

  // Calculate AABB
  Eigen::Vector3d min_corner, max_corner;
  calculateAABB(mesh_data, min_corner, max_corner);

  // Set box size
  simplified.size = getSize(min_corner, max_corner);

  return simplified;
}

void GeometrySimplifier::calculateAABB(const ::simulation::MeshData &mesh_data,
                                       Eigen::Vector3d &min_corner,
                                       Eigen::Vector3d &max_corner) {
  if (mesh_data.vertices.empty()) {
    min_corner = Eigen::Vector3d::Zero();
    max_corner = Eigen::Vector3d::Zero();
    return;
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double max_z = std::numeric_limits<double>::lowest();

  for (size_t i = 0; i < mesh_data.vertices.size(); i += 3) {
    double x = mesh_data.vertices[i];
    double y = mesh_data.vertices[i + 1];
    double z = mesh_data.vertices[i + 2];

    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    min_z = std::min(min_z, z);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
    max_z = std::max(max_z, z);
  }

  min_corner = Eigen::Vector3d(min_x, min_y, min_z);
  max_corner = Eigen::Vector3d(max_x, max_y, max_z);
}

Eigen::Vector3d
GeometrySimplifier::getCenter(const Eigen::Vector3d &min_corner,
                              const Eigen::Vector3d &max_corner) {
  return (min_corner + max_corner) * 0.5;
}

Eigen::Vector3d GeometrySimplifier::getSize(const Eigen::Vector3d &min_corner,
                                            const Eigen::Vector3d &max_corner) {
  return max_corner - min_corner;
}

} // namespace simulation
} // namespace robot_sim
