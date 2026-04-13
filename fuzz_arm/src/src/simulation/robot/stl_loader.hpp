#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace simulation {

struct MeshData {
  std::vector<float> vertices;   // [x1, y1, z1, x2, y2, z2, ...]
  std::vector<uint32_t> indices; // [i1, i2, i3, ...]
};

class StlLoader {
public:
  /**
   * @brief Load a binary STL file
   * @param filename Path to the STL file
   * @param scale Scaling factor (applied to vertices)
   * @return MeshData containing vertices and indices
   */
  static MeshData
  loadBinaryStl(const std::string &filename,
                const Eigen::Vector3d &scale = Eigen::Vector3d::Ones());
};

} // namespace simulation
