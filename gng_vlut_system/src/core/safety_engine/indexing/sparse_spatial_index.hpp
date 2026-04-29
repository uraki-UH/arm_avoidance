#pragma once

#include "common/config_manager.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"
#include "safety_engine/indexing/ispatial_index.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace robot_sim {
namespace analysis {

/**
 * @brief Sparse implementation of SpatialIndex using std::unordered_map.
 * Good for memory efficiency when nodes are sparse, but slower than dense
 * grids.
 */
class SparseSpatialIndex : public ISpatialIndex {
public:
  SparseSpatialIndex(double voxel_size) : voxel_size_(voxel_size), world_min_(Eigen::Vector3d::Zero()) {}

  double getVoxelSize() const override { return voxel_size_; }

  Eigen::Vector3d getWorldMin() const override { return world_min_; }

  void clear() override { voxel_to_nodes_map_.clear(); }

  void insert(int id, const Eigen::Vector3d &pos) override {
    Eigen::Vector3i idx =
        (pos.cast<float>() / (float)voxel_size_).array().floor().cast<int>();
    long vid = GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);
    voxel_to_nodes_map_[vid].push_back(id);
  }

  long getVoxelId(const Eigen::Vector3d &pos) const override {
    Eigen::Vector3i idx =
        (pos.cast<float>() / (float)voxel_size_).array().floor().cast<int>();
    return GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);
  }

  bool load(const std::string &filename) override {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.is_open())
      return false;

    // --- Read Header (Version 2 compatible) ---
    uint32_t magic = 0;
    ifs.read(reinterpret_cast<char *>(&magic), sizeof(uint32_t));

    size_t total_relations = 0;
    const uint32_t VLUT_MAGIC = 0x564C5554; // "VLUT" in big-endian bit order, but we check both
    const uint32_t VLUT_MAGIC_LE = 0x54554C56; // "VLUT" in little-endian

    if (magic == VLUT_MAGIC || magic == VLUT_MAGIC_LE) {
      uint32_t version = 0;
      ifs.read(reinterpret_cast<char *>(&version), sizeof(uint32_t));
      
      float file_res = 0;
      ifs.read(reinterpret_cast<char *>(&file_res), sizeof(float));
      
      // Update voxel size if it's different (optional, but good for consistency)
      if (std::abs(file_res - (float)voxel_size_) > 1e-6f && file_res > 0) {
          std::cout << "[SparseSpatialIndex] Auto-updating voxel size: " << voxel_size_ << " -> " << file_res << std::endl;
          voxel_size_ = (double)file_res;
      }

      // Read bounds (12 bytes for min, 12 bytes for max)
      float min_b[3], max_b[3];
      ifs.read(reinterpret_cast<char *>(min_b), sizeof(float) * 3);
      ifs.read(reinterpret_cast<char *>(max_b), sizeof(float) * 3);
      world_min_ = Eigen::Vector3d((double)min_b[0], (double)min_b[1], (double)min_b[2]);
      
      // Read actual total relations
      ifs.read(reinterpret_cast<char *>(&total_relations), sizeof(size_t));
    } else {
      // Legacy format: no magic header, first 8 bytes is total_relations
      ifs.seekg(0, std::ios::beg);
      ifs.read(reinterpret_cast<char *>(&total_relations), sizeof(size_t));
    }

    voxel_to_nodes_map_.clear();
    if (total_relations == 0) return true;

    // Use a buffer to read all data at once for better performance
    const size_t record_size = sizeof(long) + sizeof(int) + sizeof(float) + sizeof(int);
    std::vector<char> buffer(total_relations * record_size);
    ifs.read(buffer.data(), buffer.size());

    const char* ptr = buffer.data();
    for (size_t i = 0; i < total_relations; ++i) {
      long vid;
      int nid;
      std::memcpy(&vid, ptr, sizeof(long));
      ptr += sizeof(long);
      std::memcpy(&nid, ptr, sizeof(int));
      ptr += sizeof(int);
      ptr += sizeof(float); // skip dist
      ptr += sizeof(int);   // skip lid

      voxel_to_nodes_map_[vid].push_back(nid);
    }
    return true;
  }

  void queryAABB(const Eigen::Vector3d &min_pt, const Eigen::Vector3d &max_pt,
                 std::vector<int> &collision_counts,
                 std::vector<int> &danger_counts, float threshold,
                 int delta) const override {
    // Basic implementation: treat all as collision for sparse if dist is
    // unknown or 0
    queryAABB(min_pt, max_pt, collision_counts, delta);
    (void)danger_counts;
    (void)threshold;
  }

  void queryAABB(const Eigen::Vector3d &min_pt, const Eigen::Vector3d &max_pt,
                 std::vector<int> &counts, int delta) const override {

    // Convert AABB to Voxel Index Range
    Eigen::Vector3i min_idx =
        (min_pt.cast<float>() / (float)voxel_size_).array().floor().cast<int>();
    Eigen::Vector3i max_idx =
        (max_pt.cast<float>() / (float)voxel_size_).array().ceil().cast<int>();

    for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
      for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
        for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
          long vid = GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(
              Eigen::Vector3i(x, y, z));
          auto it = voxel_to_nodes_map_.find(vid);
          if (it != voxel_to_nodes_map_.end()) {
            for (int nid : it->second) {
              if (nid >= 0 && nid < static_cast<int>(counts.size())) {
                counts[nid] += delta;
                if (counts[nid] < 0)
                  counts[nid] = 0;
              }
            }
          }
        }
      }
    }
  }

  void updateCounts(const std::vector<long> &voxel_ids,
                    std::vector<int> &counts, int delta,
                    float /*threshold*/ = -1.0f) const override {
    for (long vid : voxel_ids) {
      auto it = voxel_to_nodes_map_.find(vid);
      if (it != voxel_to_nodes_map_.end()) {
        for (int nid : it->second) {
          if (nid >= 0 && nid < static_cast<int>(counts.size())) {
            counts[nid] += delta;
            if (counts[nid] < 0)
              counts[nid] = 0;
          }
        }
      }
    }
  }

  std::vector<int>
  getNodesInVoxel(const Eigen::Vector3d &point) const override {
    Eigen::Vector3i idx =
        (point.cast<float>() / (float)voxel_size_).array().floor().cast<int>();
    long vid = GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);

    auto it = voxel_to_nodes_map_.find(vid);
    if (it != voxel_to_nodes_map_.end()) {
      return it->second;
    }
    return {};
  }

  std::vector<int> getNodesInVoxel(long voxel_id) const override {
    auto it = voxel_to_nodes_map_.find(voxel_id);
    if (it != voxel_to_nodes_map_.end()) {
      return it->second;
    }
    return {};
  }

private:
  std::unordered_map<long, std::vector<int>> voxel_to_nodes_map_;
  double voxel_size_;
  Eigen::Vector3d world_min_;
};

} // namespace analysis
} // namespace robot_sim
