#pragma once

#include "common/config_manager.hpp"
#include "spatial/index_voxel_grid.hpp"
#include "spatial/ispatial_index.hpp"
#include <algorithm>
#include <cmath>
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
  SparseSpatialIndex(double voxel_size) : voxel_size_(voxel_size) {}

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

    size_t total_relations = 0;
    ifs.read(reinterpret_cast<char *>(&total_relations), sizeof(size_t));

    voxel_to_nodes_map_.clear();
    for (size_t i = 0; i < total_relations; ++i) {
      long vid;
      int nid, lid;
      float dist;
      ifs.read(reinterpret_cast<char *>(&vid), sizeof(long));
      ifs.read(reinterpret_cast<char *>(&nid), sizeof(int));
      ifs.read(reinterpret_cast<char *>(&dist), sizeof(float));
      ifs.read(reinterpret_cast<char *>(&lid), sizeof(int));
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
};

} // namespace analysis
} // namespace robot_sim
