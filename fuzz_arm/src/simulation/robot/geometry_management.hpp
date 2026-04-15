/**
 * @file geometry_management.hpp
 * @brief Mesh caching and geometry simplification for simulation.
 */

#pragma once

#include "simulation/robot/robot_model.hpp"
#include "simulation/robot/stl_loader.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace robot_sim {
namespace simulation {

/**
 * @brief Cache entry for mesh data used by simulation and visualization.
 */
struct MeshEntry {
  std::vector<float> original_vertices;
  std::vector<uint32_t> indices;
  std::vector<float> flattened_vertices;
  std::shared_ptr<void> backend_data;
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
  std::shared_ptr<MeshEntry> getMesh(const std::string &filename,
                                    const Eigen::Vector3d &scale);

  /**
   * @brief Clear all cached meshes.
   */
  void clear();

private:
  // Key: filename + "_" + scale_x + "_" + scale_y + "_" + scale_z
  std::map<std::string, std::shared_ptr<MeshEntry>> cache_;
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
