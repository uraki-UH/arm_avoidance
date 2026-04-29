#pragma once

#include "index_voxel_grid.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace GNG {
namespace Analysis {

struct SpatialLink {
  int target_node_id;
  float min_distance;
  uint8_t type; // 1: Collision (Conflict), 2: Danger
};

using SpatialMap = std::unordered_map<int, std::vector<SpatialLink>>;

class SpatialNetworkLoader {
public:
  static bool load(const std::string &filename, SpatialMap &out_map,
                   double &out_voxel_size, double &out_eef_radius) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) {
      std::cerr << "[SpatialNetworkLoader] Failed to open " << filename
                << std::endl;
      return false;
    }

    // Header v11: [VoxelSize(double)][EEFRadius(double)][NumNodes(uint32)]
    ifs.read(reinterpret_cast<char *>(&out_voxel_size), sizeof(double));
    ifs.read(reinterpret_cast<char *>(&out_eef_radius), sizeof(double));

    uint32_t map_size;
    ifs.read(reinterpret_cast<char *>(&map_size), sizeof(uint32_t));

    out_map.clear();
    for (uint32_t i = 0; i < map_size; ++i) {
      uint32_t src_id;
      uint32_t vec_size;
      ifs.read(reinterpret_cast<char *>(&src_id), sizeof(uint32_t));
      ifs.read(reinterpret_cast<char *>(&vec_size), sizeof(uint32_t));

      std::vector<SpatialLink> links;
      links.reserve(vec_size);
      for (uint32_t j = 0; j < vec_size; ++j) {
        uint32_t target_id;
        uint8_t type;
        ifs.read(reinterpret_cast<char *>(&target_id), sizeof(uint32_t));
        ifs.read(reinterpret_cast<char *>(&type), sizeof(uint8_t));

        SpatialLink link;
        link.target_node_id = (int)target_id;
        link.min_distance = 0.0f; // Exact distance not stored in conflict map
        link.type = type;
        links.push_back(link);
      }
      out_map[(int)src_id] = std::move(links);
    }

    std::cout << "[SpatialNetworkLoader] Loaded " << out_map.size()
              << " conflict entries (VoxelSize=" << out_voxel_size
              << ", R=" << out_eef_radius << ")." << std::endl;
    return true;
  }
};

/**
 * @brief Loader for the Voxel Occupancy Map (Inverse mapping)
 */
class VoxelOccupancyLoader {
public:
  static bool
  load(const std::string &filename,
       std::unordered_map<Eigen::Vector3i, std::vector<int>,
                          GNG::Analysis::Vector3iHash> &out_voxel_to_ids,
       double &out_voxel_size) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs)
      return false;

    uint32_t num_voxels;
    ifs.read(reinterpret_cast<char *>(&out_voxel_size), sizeof(double));
    ifs.read(reinterpret_cast<char *>(&num_voxels), sizeof(uint32_t));

    out_voxel_to_ids.clear();
    for (uint32_t i = 0; i < num_voxels; ++i) {
      int x, y, z;
      ifs.read(reinterpret_cast<char *>(&x), sizeof(int));
      ifs.read(reinterpret_cast<char *>(&y), sizeof(int));
      ifs.read(reinterpret_cast<char *>(&z), sizeof(int));

      uint32_t count;
      ifs.read(reinterpret_cast<char *>(&count), sizeof(uint32_t));
      std::vector<int> ids(count);
      for (uint32_t j = 0; j < count; ++j) {
        uint32_t uid;
        ifs.read(reinterpret_cast<char *>(&uid), sizeof(uint32_t));
        ids[j] = (int)uid;
      }
      out_voxel_to_ids[Eigen::Vector3i(x, y, z)] = std::move(ids);
    }
    std::cout << "[VoxelOccupancyLoader] Loaded " << out_voxel_to_ids.size()
              << " occupied voxels." << std::endl;
    return true;
  }
};

} // namespace Analysis
} // namespace GNG
