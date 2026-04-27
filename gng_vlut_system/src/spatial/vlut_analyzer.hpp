#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <cstddef>

namespace robot_sim {
namespace spatial {

class VlutAnalyzer {
public:
  // Binary compatible with Version 3 VLUT file format (4 bytes per record)
  struct Relation {
    int node_id;
  };

  // CSR Voxel Index Header
  struct VoxelHeader {
    uint32_t offset;
    uint32_t count;
  };

  /**
   * @brief Decodes a flat 64-bit voxel ID into 3D grid coordinates.
   */
  static inline void decodeVoxelId(long id, int& x, int& y, int& z) {
    x = static_cast<int>((id >> 42) - 1000000L);
    y = static_cast<int>(((id >> 21) & 0x1FFFFFL) - 1000000L);
    z = static_cast<int>((id & 0x1FFFFFL) - 1000000L);
  }

  /**
   * @brief Fast CSR-based node occupancy update.
   * Simple hit-counting logic with no distance or link checks.
   *
   * @param occupied_vids Input list of flat voxel IDs.
   * @param headers Pointer to the local grid headers.
   * @param data Pointer to the flattened list of node IDs (CSR data).
   * @param dim_x, dim_y, dim_z Dimensions of the dense local grid.
   * @param off_x, off_y, off_z Origin offset of the local grid in voxel units.
   * @param counts Pointer to the node counter array.
   * @param counts_size Size of the counts array.
   * @param delta Value to add to counters (+1 or -1).
   */
  /**
   * @brief Fastest update path using direct array indices.
   * Bypasses Global ID decoding.
   */
  static void updateNodeCountsDirect(
      const std::vector<int>& indices,
      const VoxelHeader* headers,
      const Relation* data,
      size_t /*headers_size*/, // For bounds check if needed
      int* counts,
      size_t counts_size,
      int delta) {
    
    for (int idx : indices) {
      if (idx < 0) continue; // Boundary/invalid check

      const VoxelHeader &h = headers[idx];
      for (uint32_t i = 0; i < h.count; ++i) {
        int nid = data[h.offset + i].node_id;
        if (nid >= 0 && static_cast<size_t>(nid) < counts_size) {
          int& c = counts[nid];
          c += delta;
          if (c < 0) c = 0;
        }
      }
    }
  }

  static void updateNodeCounts(
      const std::vector<long>& occupied_vids,
      const VoxelHeader* headers,
      const Relation* data,
      int dim_x, int dim_y, int dim_z,
      int off_x, int off_y, int off_z,
      int* counts,
      size_t counts_size,
      int delta) {
    
    for (long vid : occupied_vids) {
      int vx, vy, vz;
      decodeVoxelId(vid, vx, vy, vz);

      int gx = vx - off_x;
      int gy = vy - off_y;
      int gz = vz - off_z;

      // Local grid bounds check
      if (gx < 0 || gx >= dim_x || gy < 0 || gy >= dim_y || gz < 0 || gz >= dim_z) {
        continue;
      }

      int flat_idx = gx + gy * dim_x + gz * dim_x * dim_y;
      const VoxelHeader &h = headers[flat_idx];

      // Inner mapping loop - just binary hits
      for (uint32_t i = 0; i < h.count; ++i) {
        int nid = data[h.offset + i].node_id;
        if (nid >= 0 && static_cast<size_t>(nid) < counts_size) {
          int& c = counts[nid];
          c += delta;
          if (c < 0) c = 0;
        }
      }
    }
  }

  /**
   * @brief Map-based update version (for SparseSpatialIndex or external mapping).
   */
  static void updateNodeCounts(
      const std::vector<long>& occupied_vids,
      const std::unordered_map<long, std::vector<int>>& voxel_to_nodes_map,
      int* counts,
      size_t counts_size,
      int delta) {
    
    for (long vid : occupied_vids) {
      auto it = voxel_to_nodes_map.find(vid);
      if (it != voxel_to_nodes_map.end()) {
        for (int nid : it->second) {
          if (nid >= 0 && static_cast<size_t>(nid) < counts_size) {
            int& c = counts[nid];
            c += delta;
            if (c < 0) c = 0;
          }
        }
      }
    }
  }
};

} // namespace spatial
} // namespace robot_sim
