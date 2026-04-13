/**
 * @file geometry_management.hpp
 * @brief Mesh caching and geometry simplification for simulation.
 */

#pragma once

#include "simulation/robot/robot_model.hpp"
#include "simulation/robot/stl_loader.hpp"
#include <Eigen/Dense>
#include <map>
#include <memory>
#include <ode/ode.h>
#include <string>
#include <vector>

namespace robot_sim {
namespace simulation {

/**
 * @brief Cache entry for mesh data used in ODE.
 */
struct MeshEntry {
  dTriMeshDataID data_id;
  std::vector<dReal> vertices;    // ODE expects dReal (float or double)
  std::vector<dTriIndex> indices; // ODE index type
  std::vector<dReal>
      flattened_vertices; // For batch rendering (x,y,z * 3 * num_tri)
  std::vector<float>
      original_vertices; // Original vertex data for simplification
};

/**
 * @brief Manages loading and caching of 3D meshes for simulation.
 */
class MeshCache {
public:
  MeshCache();
  ~MeshCache();

  /**
   * @brief Get or create mesh data for a given STL file and scale.
   */
  dTriMeshDataID getMesh(const std::string &filename,
                         const Eigen::Vector3d &scale);

  /**
   * @brief Clear all cached meshes.
   */
  void clear();

  /**
   * @brief Get mesh entry by DataID (used for rendering).
   */
  std::shared_ptr<MeshEntry> getMeshEntry(dTriMeshDataID data_id) const;

  /**
   * @brief Get original vertices for a mesh DataID.
   */
  const std::vector<float> &getOriginalVertices(dTriMeshDataID data_id) const;

private:
  // Key: filename + "_" + scale_x + "_" + scale_y + "_" + scale_z
  std::map<std::string, std::shared_ptr<MeshEntry>> cache_;
  // Key: DataID
  std::map<dTriMeshDataID, std::shared_ptr<MeshEntry>> reverse_cache_;
};

/**
 * @brief Utility class to generate simplified geometry from mesh data.
 */
class GeometrySimplifier {
public:
  /**
   * @brief Generate a simplified bounding box geometry from mesh data.
   */
  static ::simulation::Geometry
  simplifyMeshToBox(const ::simulation::MeshData &mesh_data);

  /**
   * @brief Calculate axis-aligned bounding box from mesh vertices.
   */
  static void calculateAABB(const ::simulation::MeshData &mesh_data,
                            Eigen::Vector3d &min_corner,
                            Eigen::Vector3d &max_corner);

  /**
   * @brief Get the center offset for simplified geometry.
   */
  static Eigen::Vector3d getCenter(const Eigen::Vector3d &min_corner,
                                   const Eigen::Vector3d &max_corner);

  /**
   * @brief Get the size (dimensions) of the bounding box.
   */
  static Eigen::Vector3d getSize(const Eigen::Vector3d &min_corner,
                                 const Eigen::Vector3d &max_corner);
};

} // namespace simulation
} // namespace robot_sim
