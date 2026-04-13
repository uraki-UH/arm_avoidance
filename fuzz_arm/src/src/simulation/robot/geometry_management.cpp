/**
 * @file geometry_management.cpp
 * @brief Implementation of mesh caching and geometry simplification.
 */

#include "simulation/robot/geometry_management.hpp"
#include "common/resource_utils.hpp"
#include "simulation/robot/stl_loader.hpp"
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
  for (auto &pair : cache_) {
    dGeomTriMeshDataDestroy(pair.second->data_id);
  }
  cache_.clear();
  reverse_cache_.clear();
}

dTriMeshDataID MeshCache::getMesh(const std::string &filename,
                                  const Eigen::Vector3d &scale) {
  std::string path_to_load = filename;

  // Handle package:// prefix
  if (filename.substr(0, 10) == "package://") {
    std::string path_after_package = filename.substr(10);
    if (path_after_package.find("piper_description") != std::string::npos) {
      path_to_load = "urdf/未作成/" + path_after_package;
    } else if (path_after_package.find("mycobot_description") !=
               std::string::npos) {
      path_to_load = "urdf/mycobot/" + path_after_package;
    } else if (path_after_package.find("topoarm_description") !=
               std::string::npos) {
      path_to_load =
          "urdf/real_model/topoarm_description/" +
          path_after_package.substr(path_after_package.find("/") + 1);
    }
  }

  std::string resolved_path = robot_sim::common::resolvePath(path_to_load);
  std::string ext = "";
  if (resolved_path.length() > 4) {
    ext = resolved_path.substr(resolved_path.length() - 4);
    for (auto &c : ext)
      c = std::tolower(c);
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
    return it->second->data_id;
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

  // Convert float to dReal
  entry->vertices.reserve(mesh_data.vertices.size());
  for (float v : mesh_data.vertices) {
    entry->vertices.push_back(static_cast<dReal>(v));
  }

  // Copy indices
  entry->indices = mesh_data.indices;

  // Create ODE TriMeshData
  entry->data_id = dGeomTriMeshDataCreate();

#ifdef dDOUBLE
  dGeomTriMeshDataBuildDouble(
      entry->data_id, entry->vertices.data(), sizeof(dReal) * 3,
      (int)entry->vertices.size() / 3, entry->indices.data(),
      (int)entry->indices.size(), sizeof(dTriIndex) * 3);
#else
  dGeomTriMeshDataBuildSingle(
      entry->data_id, entry->vertices.data(), sizeof(dReal) * 3,
      (int)entry->vertices.size() / 3, entry->indices.data(),
      (int)entry->indices.size(), sizeof(dTriIndex) * 3);
#endif

  // Generate flattened vertices for rendering
  entry->flattened_vertices.reserve(entry->indices.size() * 3);
  for (size_t i = 0; i < entry->indices.size(); ++i) {
    dTriIndex idx = entry->indices[i];
    entry->flattened_vertices.push_back(entry->vertices[idx * 3]);
    entry->flattened_vertices.push_back(entry->vertices[idx * 3 + 1]);
    entry->flattened_vertices.push_back(entry->vertices[idx * 3 + 2]);
  }

  cache_[key] = entry;
  reverse_cache_[entry->data_id] = entry;

  std::cout << "MeshCache: Cached new mesh: " << key
            << " (DataID: " << entry->data_id << ")" << std::endl;

  return entry->data_id;
}

std::shared_ptr<MeshEntry>
MeshCache::getMeshEntry(dTriMeshDataID data_id) const {
  auto it = reverse_cache_.find(data_id);
  if (it != reverse_cache_.end()) {
    return it->second;
  }
  return nullptr;
}

const std::vector<float> &
MeshCache::getOriginalVertices(dTriMeshDataID data_id) const {
  static const std::vector<float> empty;
  auto it = reverse_cache_.find(data_id);
  if (it != reverse_cache_.end()) {
    return it->second->original_vertices;
  }
  return empty;
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
