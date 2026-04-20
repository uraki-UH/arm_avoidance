#pragma once

#include "index_voxel_grid.hpp"
#include "ispatial_index.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>

namespace robot_sim {
namespace analysis {

/**
 * @brief High-performance Dense Spatial Index using CSR (Compressed Sparse Row)
 * format. Optimized for cache-locality and O(1) grid access.
 */
class DenseSpatialIndex : public ISpatialIndex {
public:
  struct Relation {
    int node_id;
    float distance;
    int link_id;
  };

  struct VoxelHeader {
    uint32_t offset;
    uint32_t count;
  };

  /**
   * @brief Construct Dense Index.
   * @param res Resolution of the lookup table.
   * @param min_bounds Minimum bounds of the workspace.
   * @param max_bounds Maximum bounds of the workspace.
   */
  DenseSpatialIndex(double res, const Eigen::Vector3d &min_bounds,
                    const Eigen::Vector3d &max_bounds)
      : resolution_(res), world_min_(min_bounds) {

    dims_ = ((max_bounds - min_bounds) / res)
                .array()
                .ceil()
                .cast<int>()
                .cwiseMax(1);
    headers_.assign(dims_.x() * dims_.y() * dims_.z(), {0, 0});
    index_offset_ = (min_bounds.array() / res).floor().cast<int>();

    std::cout << "[DenseSpatialIndex] Created grid: " << dims_.transpose()
              << " (" << headers_.size() << " voxels)" << std::endl;
  }

  double getVoxelSize() const override { return resolution_; }

  Eigen::Vector3d getWorldMin() const override { return world_min_; }

  bool load(const std::string &filename) override {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.is_open())
      return false;

    // --- Header Loading & Auto-Configuration ---
    uint32_t file_id = 0;
    uint32_t version = 0;
    float file_res = 0.0f;

    ifs.read(reinterpret_cast<char *>(&file_id), sizeof(uint32_t));
    
    // Check if the file's binary identifier matches a 4-character string
    auto hasMatchingIdentifier = [](uint32_t val, const char* s) {
        uint32_t target = (uint32_t)(s[0] << 24 | s[1] << 16 | s[2] << 8 | s[3]);
        return val == target;
    };

    if (hasMatchingIdentifier(file_id, "VLUT")) {
        ifs.read(reinterpret_cast<char *>(&version), sizeof(uint32_t));        ifs.read(reinterpret_cast<char *>(&file_res), sizeof(float));
        
        // Auto-adopt resolution from file
        resolution_ = (double)file_res;

        if (version >= 2) {
            float min_b[3], max_b[3];
            ifs.read(reinterpret_cast<char *>(min_b), sizeof(float) * 3);
            ifs.read(reinterpret_cast<char *>(max_b), sizeof(float) * 3);

            // FORCE adopt workspace bounds and resolution from file to ensure efficiency
            world_min_ = Eigen::Vector3d((double)min_b[0], (double)min_b[1], (double)min_b[2]);
            Eigen::Vector3d world_max((double)max_b[0], (double)max_b[1], (double)max_b[2]);
            resolution_ = (double)file_res;

            // Re-calculate grid dimensions and offset based on file header
            dims_ = ((world_max - world_min_) / resolution_)
                        .array().ceil().cast<int>().cwiseMax(1);
            index_offset_ = (world_min_.array() / resolution_).floor().cast<int>();

            // Re-allocate headers to match the optimal bounds from the file
            headers_.assign(dims_.x() * dims_.y() * dims_.z(), {0, 0});
            std::cout << "[DenseSpatialIndex] Adopted File Config: Res=" << resolution_ << "m, Dims=" << dims_.transpose() << std::endl;
        } else {
            std::cout << "[DenseSpatialIndex] Version 1 VLUT: Adopting resolution " << file_res << "m (using default bounds)" << std::endl;
        }
    } else {
        // Old format: First bytes were part of size_t total_relations
        ifs.seekg(0);
        std::cout << "[DenseSpatialIndex] Warning: Loading legacy format. Make sure config.txt matches the file data!" << std::endl;
    }

    size_t total_relations = 0;
    ifs.read(reinterpret_cast<char *>(&total_relations), sizeof(size_t));
    
    if (total_relations > 100000000) {
        std::cerr << "[DenseSpatialIndex] ERROR: Suspiciously large relation count: " << total_relations << std::endl;
        return false;
    }
    std::cout << "[DenseSpatialIndex] Loading " << total_relations << " relations..." << std::endl;

    // Step 1: Read all relations into a flat temporary buffer first
    struct RawRel { long vid; int nid; float dist; int lid; };
    std::vector<RawRel> raw_buffer(total_relations);
    
    // Counting occurrences for each voxel
    std::vector<uint32_t> counts(headers_.size(), 0);

    for (size_t i = 0; i < total_relations; ++i) {
      ifs.read(reinterpret_cast<char *>(&raw_buffer[i].vid), sizeof(long));
      ifs.read(reinterpret_cast<char *>(&raw_buffer[i].nid), sizeof(int));
      ifs.read(reinterpret_cast<char *>(&raw_buffer[i].dist), sizeof(float));
      ifs.read(reinterpret_cast<char *>(&raw_buffer[i].lid), sizeof(int));

      Eigen::Vector3i v_idx = GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(raw_buffer[i].vid);
      Eigen::Vector3i g_idx = v_idx - index_offset_;

      if ((g_idx.array() >= 0).all() && (g_idx.array() < dims_.array()).all()) {
        int flat_g_idx = g_idx.x() + g_idx.y() * dims_.x() + g_idx.z() * dims_.x() * dims_.y();
        counts[flat_g_idx]++;
      }
      if (i % 5000000 == 0 && i > 0) {
          std::cout << "  Reading file: " << (i*100/total_relations) << "%\r" << std::flush;
      }
    }
    std::cout << "  File read complete. Building index structure..." << std::endl;

    // Step 2: Compute offsets in the final node_data_ vector
    uint32_t current_offset = 0;
    for (size_t i = 0; i < headers_.size(); ++i) {
      headers_[i].offset = current_offset;
      headers_[i].count = 0; // Will be used as a current-fill counter in next step
      current_offset += counts[i];
    }

    node_data_.assign(current_offset, {0, 0.0f, 0});
    
    // Step 3: Populate node_data_ using the offsets
    for (size_t i = 0; i < total_relations; ++i) {
      // Use the raw voxel index from the file
      Eigen::Vector3i v_idx = GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(raw_buffer[i].vid);
      
      // Convert world-voxel-index to local-grid-index using the offset
      Eigen::Vector3i g_idx = v_idx - index_offset_;

      if ((g_idx.array() >= 0).all() && (g_idx.array() < dims_.array()).all()) {
        int flat_g_idx = g_idx.x() + g_idx.y() * dims_.x() + g_idx.z() * dims_.x() * dims_.y();
        uint32_t write_pos = headers_[flat_g_idx].offset + headers_[flat_g_idx].count;
        node_data_[write_pos] = {raw_buffer[i].nid, raw_buffer[i].dist, raw_buffer[i].lid};
        headers_[flat_g_idx].count++;
      }
    }

    // Step 4: Sort only non-empty voxels by distance
    std::cout << "  Sorting active voxels..." << std::endl;
    for (size_t i = 0; i < headers_.size(); ++i) {
      if (headers_[i].count > 1) {
        std::sort(node_data_.begin() + headers_[i].offset,
                  node_data_.begin() + headers_[i].offset + headers_[i].count,
                  [](const Relation &a, const Relation &b) {
                    return a.distance < b.distance;
                  });
      }
    }

    std::cout << "[DenseSpatialIndex] Successfully loaded " << node_data_.size() << " relations." << std::endl;
    is_static_ = true;
    return true;
  }

  long getVoxelId(const Eigen::Vector3d &pos) const override {
    Eigen::Vector3i idx = (pos.array() / resolution_).floor().cast<int>();
    return GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);
  }

  void queryAABB(const Eigen::Vector3d &min_pt, const Eigen::Vector3d &max_pt,
                 std::vector<int> &collision_counts,
                 std::vector<int> &danger_counts, float threshold,
                 int delta) const override {

    Eigen::Vector3i start_idx = ((min_pt - world_min_).array() / resolution_)
                                    .floor()
                                    .cast<int>()
                                    .cwiseMax(0);
    Eigen::Vector3i end_idx =
        ((max_pt - world_min_).array() / resolution_).ceil().cast<int>();

    end_idx = end_idx.cwiseMin(dims_ - Eigen::Vector3i::Ones()).cwiseMax(0);
    start_idx = start_idx.cwiseMin(dims_ - Eigen::Vector3i::Ones());

    for (int z = start_idx.z(); z <= end_idx.z(); ++z) {
      int z_offset = z * dims_.x() * dims_.y();
      for (int y = start_idx.y(); y <= end_idx.y(); ++y) {
        int y_offset = z_offset + y * dims_.x();
        for (int x = start_idx.x(); x <= end_idx.x(); ++x) {
          int flat_idx = y_offset + x;
          const auto &h = headers_[flat_idx];

          const Relation *rels = &node_data_[h.offset];
          for (uint32_t i = 0; i < h.count; ++i) {
            const auto &rel = rels[i];
            if (rel.distance <= 1e-6) {
              collision_counts[rel.node_id] += delta;
            } else if (rel.distance <= threshold) {
              danger_counts[rel.node_id] += delta;
            } else {
              break; // Early exit since relations are sorted by distance
            }
          }
        }
      }
    }
  }

  void queryAABB(const Eigen::Vector3d &min_pt, const Eigen::Vector3d &max_pt,
                 std::vector<int> &counts, int delta) const override {
    std::vector<int> dummy_danger(counts.size(), 0);
    queryAABB(min_pt, max_pt, counts, dummy_danger, 0.0f, delta);
  }

  void updateCounts(const std::vector<long> &voxel_ids,
                    std::vector<int> &counts, int delta,
                    float threshold = -1.0f) const override {
    for (long vid : voxel_ids) {
      Eigen::Vector3i v_idx =
          GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);

      Eigen::Vector3i g_idx = v_idx - index_offset_;

      if ((g_idx.array() < 0).any() || (g_idx.array() >= dims_.array()).any()) {
        continue;
      }

      int flat_idx =
          g_idx.x() + g_idx.y() * dims_.x() + g_idx.z() * dims_.x() * dims_.y();
      const VoxelHeader &h = headers_[flat_idx];
      const Relation *rels = &node_data_[h.offset];
      for (uint32_t i = 0; i < h.count; ++i) {
        if (threshold >= 0.0f && rels[i].distance > threshold) {
          break; // Relations are sorted by distance, early exit
        }
        int nid = rels[i].node_id;
        if (nid >= 0 && nid < static_cast<int>(counts.size())) {
          counts[nid] += delta;
          if (counts[nid] < 0)
            counts[nid] = 0;
        }
      }
    }
  }

  /**
   * @brief Get all node IDs located in the voxel containing the given point.
   */
  std::vector<int>
  getNodesInVoxel(const Eigen::Vector3d &point) const override {
    Eigen::Vector3i g_idx =
        ((point - world_min_).array() / resolution_).floor().cast<int>();

    if ((g_idx.array() < 0).any() || (g_idx.array() >= dims_.array()).any()) {
      return {};
    }

    int flat_idx =
        g_idx.x() + g_idx.y() * dims_.x() + g_idx.z() * dims_.x() * dims_.y();

    // Bounds check for flat index
    if (flat_idx < 0 || flat_idx >= (int)headers_.size())
      return {};

    const auto &h = headers_[flat_idx];
    if (h.count == 0)
      return {};

    const Relation *rels = &node_data_[h.offset];
    std::vector<int> result;
    result.reserve(h.count);
    for (uint32_t i = 0; i < h.count; ++i) {
      result.push_back(rels[i].node_id);
    }
    return result;
  }

  std::vector<int> getNodesInVoxel(long voxel_id) const override {
    Eigen::Vector3i v_idx =
        GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(voxel_id);
    Eigen::Vector3i g_idx = v_idx - index_offset_;

    if ((g_idx.array() < 0).any() || (g_idx.array() >= dims_.array()).any()) {
      return {};
    }

    int flat_idx =
        g_idx.x() + g_idx.y() * dims_.x() + g_idx.z() * dims_.x() * dims_.y();
    if (flat_idx < 0 || flat_idx >= (int)headers_.size())
      return {};

    const auto &h = headers_[flat_idx];
    if (h.count == 0)
      return {};

    const Relation *rels = &node_data_[h.offset];
    std::vector<int> result;
    result.reserve(h.count);
    for (uint32_t i = 0; i < h.count; ++i) {
      result.push_back(rels[i].node_id);
    }
    return result;
  }

  /**
   * @brief Returns a direct pointer and count of nodes in a voxel (Zero-Copy).
   * @param voxel_idx 3D grid index
   * @return pair<const int*, int> (ptr, count)
   */
  std::pair<const int *, int>
  getNodesInVoxelRaw(const Eigen::Vector3d &p_voxel) const {
    Eigen::Vector3i g_idx =
        ((p_voxel - world_min_).array() / resolution_).floor().cast<int>();
    return getNodesInVoxelRawByIndex(g_idx);
  }

  std::pair<const Relation *, int> getRelationsInVoxelRaw(long voxel_id) const {
    Eigen::Vector3i v_idx =
        GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(voxel_id);
    Eigen::Vector3i g_idx = v_idx - index_offset_;
    return getRelationsInVoxelRawByIndex(g_idx);
  }

  std::pair<const Relation *, int>
  getRelationsInVoxelRawByIndex(const Eigen::Vector3i &g_idx) const {
    if (g_idx.minCoeff() < 0 || (g_idx.array() >= dims_.array()).any())
      return {nullptr, 0};

    int flat_idx = g_idx.x() + dims_.x() * (g_idx.y() + dims_.y() * g_idx.z());
    if (flat_idx < 0 || flat_idx >= (int)headers_.size())
      return {nullptr, 0};

    const auto &h = headers_[flat_idx];
    if (h.count == 0)
      return {nullptr, 0};

    return {&node_data_[h.offset], (int)h.count};
  }

  // Deprecated or for internal use only (returns nullptr as it can't return
  // int*)
  std::pair<const int *, int>
  getNodesInVoxelRawByIndex(const Eigen::Vector3i &) const {
    return {nullptr, 0};
  }

  double getResolution() const { return resolution_; }
  const Eigen::Vector3i &getDims() const { return dims_; }

  // --- Dynamic Update Implementation ---
  void clear() override {
    if (is_static_) {
      std::cout << "[DenseSpatialIndex] Warning: Attempting to clear a static (loaded) index. Operation ignored." << std::endl;
      return;
    }
    // Reset headers and data
    std::fill(headers_.begin(), headers_.end(), VoxelHeader{0, 0});
    node_data_.clear();
    pending_updates_.clear();
  }

  void insert(int id, const Eigen::Vector3d &pos) override {
    if (is_static_) return;
    // Calculate Voxel ID
    Eigen::Vector3i g_idx =
        ((pos - world_min_).array() / resolution_).floor().cast<int>();

    if ((g_idx.array() >= 0).all() && (g_idx.array() < dims_.array()).all()) {
      int flat_g_idx =
          g_idx.x() + g_idx.y() * dims_.x() + g_idx.z() * dims_.x() * dims_.y();

      // Calculate distance from voxel center (approximation)
      Eigen::Vector3d voxel_center =
          world_min_ +
          (g_idx.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * resolution_;
      float dist = (float)(pos - voxel_center).norm();

      pending_updates_[flat_g_idx].push_back({id, dist, -1});
    }
  }

  void build() override {
    if (is_static_) return;
    node_data_.clear();
    // Rebuild from pending_updates_
    for (int i = 0; i < (int)headers_.size(); ++i) {
      auto it = pending_updates_.find(i);
      if (it != pending_updates_.end()) {
        headers_[i].offset = static_cast<uint32_t>(node_data_.size());
        headers_[i].count = static_cast<uint32_t>(it->second.size());

        // Sort
        std::sort(it->second.begin(), it->second.end(),
                  [](const Relation &a, const Relation &b) {
                    return a.distance < b.distance;
                  });

        node_data_.insert(node_data_.end(), it->second.begin(),
                          it->second.end());
      } else {
        headers_[i] = {0, 0};
      }
    }
  }

private:
  double resolution_;
  Eigen::Vector3d world_min_;
  Eigen::Vector3i dims_;
  Eigen::Vector3i index_offset_;
  bool is_static_ = false;

  std::vector<VoxelHeader> headers_;
  std::vector<Relation> node_data_;
  std::unordered_map<int, std::vector<Relation>>
      pending_updates_; // For dynamic updates
};

} // namespace analysis
} // namespace robot_sim
