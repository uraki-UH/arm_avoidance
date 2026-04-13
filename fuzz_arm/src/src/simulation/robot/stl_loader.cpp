#include "simulation/robot/stl_loader.hpp"
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>

namespace simulation {

MeshData StlLoader::loadBinaryStl(const std::string &filename,
                                  const Eigen::Vector3d &scale) {
  MeshData mesh;
  std::ifstream file(filename, std::ios::binary);
  if (!file) {
    std::cerr << "StlLoader: Failed to open file: " << filename << std::endl;
    return mesh;
  }

  // Binary STL format:
  // Header (80 bytes)
  // Number of triangles (4 bytes)
  // For each triangle:
  //   Normal (3 * 4 bytes)
  //   Vertex 1 (3 * 4 bytes)
  //   Vertex 2 (3 * 4 bytes)
  //   Vertex 3 (3 * 4 bytes)
  //   Attribute byte count (2 bytes)

  char header[80];
  file.read(header, 80);

  uint32_t num_triangles;
  file.read(reinterpret_cast<char *>(&num_triangles), 4);

  struct Vertex {
    float x, y, z;
    bool operator<(const Vertex &other) const {
      if (x != other.x)
        return x < other.x;
      if (y != other.y)
        return y < other.y;
      return z < other.z;
    }
  };

  std::map<Vertex, uint32_t> vertex_to_index;

  for (uint32_t i = 0; i < num_triangles; ++i) {
    float normal[3];
    file.read(reinterpret_cast<char *>(normal), 12);

    for (int j = 0; j < 3; ++j) {
      float v_raw[3];
      file.read(reinterpret_cast<char *>(v_raw), 12);

      Vertex v;
      v.x = v_raw[0] * (float)scale.x();
      v.y = v_raw[1] * (float)scale.y();
      v.z = v_raw[2] * (float)scale.z();

      auto it = vertex_to_index.find(v);
      if (it == vertex_to_index.end()) {
        uint32_t index = static_cast<uint32_t>(mesh.vertices.size() / 3);
        mesh.vertices.push_back(v.x);
        mesh.vertices.push_back(v.y);
        mesh.vertices.push_back(v.z);
        vertex_to_index[v] = index;
        mesh.indices.push_back(index);
      } else {
        mesh.indices.push_back(it->second);
      }
    }

    uint16_t attr_count;
    file.read(reinterpret_cast<char *>(&attr_count), 2);
  }

  float min_x = 1e9, min_y = 1e9, min_z = 1e9;
  float max_x = -1e9, max_y = -1e9, max_z = -1e9;

  for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
    float x = mesh.vertices[i];
    float y = mesh.vertices[i + 1];
    float z = mesh.vertices[i + 2];
    if (x < min_x)
      min_x = x;
    if (y < min_y)
      min_y = y;
    if (z < min_z)
      min_z = z;
    if (x > max_x)
      max_x = x;
    if (y > max_y)
      max_y = y;
    if (z > max_z)
      max_z = z;
  }

  return mesh;
}

} // namespace simulation
