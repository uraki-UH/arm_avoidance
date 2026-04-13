#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace GNG {
namespace Analysis {

/**
 * @brief Custom hash for Eigen::Vector3i to use with unordered_map.
 */
struct Vector3iHash {
  std::size_t operator()(const Eigen::Vector3i &v) const {
    std::size_t h = 0;
    h ^= std::hash<int>{}(v.x()) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(v.y()) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(v.z()) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

/**
 * @brief Bidirectional Voxel Grid that stores indices (e.g., Node IDs).
 *
 * Supports fast lookup from Voxel -> IDs and ID -> Voxels.
 */
class IndexVoxelGrid {
public:
  IndexVoxelGrid(double voxel_size) : voxel_size_(voxel_size) {}

  void clear() {
    voxel_to_ids_.clear();
    id_to_voxels_.clear();
  }

  /**
   * @brief Helper to get the voxel index for a given point.
   */
  Eigen::Vector3i getIndex(const Eigen::Vector3f &p) const {
    return (p / (float)voxel_size_).array().floor().cast<int>();
  }

  // Stores raw node ID directly
  void insert(const Eigen::Vector3f &p, int index) {
    Eigen::Vector3i idx = getIndex(p);
    id_to_voxels_[index].push_back(idx);
    voxel_to_ids_[idx].push_back(index);
  }

  /**
   * @brief Insert an index into ALL voxels covered by an AABB.
   */
  void insertAABB(const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt,
                  int index) {
    Eigen::Vector3i min_idx = getIndex(min_pt);
    Eigen::Vector3i max_idx = getIndex(max_pt);

    for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
      for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
        for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
          Eigen::Vector3i idx(x, y, z);
          voxel_to_ids_[idx].push_back(index);
          id_to_voxels_[index].push_back(idx);
        }
      }
    }
  }

  /**
   * @brief Get all indices in the voxel containing point p.
   */
  std::vector<int> getIndicesAt(const Eigen::Vector3f &p) const {
    Eigen::Vector3i idx = (p / (float)voxel_size_).array().floor().cast<int>();
    auto it = voxel_to_ids_.find(idx);
    if (it != voxel_to_ids_.end()) {
      return it->second;
    }
    return {};
  }

  /**
   * @brief Get all indices in the exact voxel index (x, y, z).
   */
  std::vector<int> getIndicesAtVoxel(const Eigen::Vector3i &idx) const {
    auto it = voxel_to_ids_.find(idx);
    if (it != voxel_to_ids_.end()) {
      return it->second;
    }
    return {};
  }

  /**
   * @brief Get all voxels associated with a specific index (e.g., Node ID).
   */
  std::vector<Eigen::Vector3i> getVoxelsForIndex(int index) const {
    auto it = id_to_voxels_.find(index);
    if (it != id_to_voxels_.end()) {
      return it->second;
    }
    return {};
  }

  /**
   * @brief Get the center coordinate of a voxel.
   */
  Eigen::Vector3f getVoxelCenter(const Eigen::Vector3i &idx) const {
    return (idx.cast<float>() * (float)voxel_size_) +
           Eigen::Vector3f::Constant((float)voxel_size_ * 0.5f);
  }

  double getVoxelSize() const { return voxel_size_; }

  const std::unordered_map<Eigen::Vector3i, std::vector<int>, Vector3iHash> &
  getVoxelToIdsMap() const {
    return voxel_to_ids_;
  }

  bool isOccupied(const Eigen::Vector3i &idx) const {
    return voxel_to_ids_.find(idx) != voxel_to_ids_.end();
  }

  /**
   * @brief Helper to compute a unique long ID for a voxel index.
   * Format: (x * 10000 + y) * 10000 + z
   * Note: This assumes indices are within reasonable bounds (-10000 to 10000 is
   * safe if handled carefully, but here we assume positive or offset indices
   * for simplicity, or just raw unique mapping). For robustness, we map to
   * positive space if needed, but for now we match the simple logic used
   * elsewhere.
   */
  static long getFlatVoxelId(const Eigen::Vector3i &idx) {
    long x = (long)idx.x() + 1000000L;
    long y = (long)idx.y() + 1000000L;
    long z = (long)idx.z() + 1000000L;
    // Pack into 64-bit long: 21 bits per component (covers +/- 1,000,000 range)
    return (x << 42) | (y << 21) | z;
  }

  static Eigen::Vector3i getIndexFromFlatId(long id) {
    long x = (id >> 42) - 1000000L;
    long y = ((id >> 21) & 0x1FFFFFL) - 1000000L;
    long z = (id & 0x1FFFFFL) - 1000000L;
    return Eigen::Vector3i((int)x, (int)y, (int)z);
  }

private:
  double voxel_size_;
  std::unordered_map<Eigen::Vector3i, std::vector<int>, Vector3iHash>
      voxel_to_ids_;
  std::unordered_map<int, std::vector<Eigen::Vector3i>> id_to_voxels_;
};

} // namespace Analysis
} // namespace GNG
