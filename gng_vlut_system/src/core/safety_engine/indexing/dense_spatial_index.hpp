#pragma once

#include "index_voxel_grid.hpp"
#include "ispatial_index.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace robot_sim {
namespace analysis {

// 隣接セル間で終端オフセットを共有するCSR索引。セル参照時の展開処理なし
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

  // 解像度と作業空間境界による索引初期化
  DenseSpatialIndex(double res, const Eigen::Vector3d &min_bounds,
                    const Eigen::Vector3d &max_bounds)
      : grid_(res), world_min_(min_bounds) {

    configure_grid(res, min_bounds, max_bounds);

    std::cout << "[DenseSpatialIndex] Created grid: " << dims_.transpose()
              << " (" << (offsets_.size() - 1) << " voxels)" << std::endl;
  }

  double getVoxelSize() const override { return grid_.getVoxelSize(); }
  ::GNG::Analysis::IndexVoxelGrid& getGrid() { return grid_; }
  const ::GNG::Analysis::IndexVoxelGrid& getGrid() const { return grid_; }

  Eigen::Vector3d getWorldMin() const override { return world_min_; }

  bool load(const std::string &filename) override {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.is_open())
      return false;

    // 保存済み解像度と境界への設定追従
    uint32_t file_id = 0;
    uint32_t version = 0;
    float file_res = 0.0f;

    ifs.read(reinterpret_cast<char *>(&file_id), sizeof(uint32_t));
    
    // 4文字識別子の照合
    auto hasMatchingIdentifier = [](uint32_t val, const char* s) {
        uint32_t target = (uint32_t)(s[0] << 24 | s[1] << 16 | s[2] << 8 | s[3]);
        return val == target;
    };

    if (hasMatchingIdentifier(file_id, "VLUT")) {
        ifs.read(reinterpret_cast<char *>(&version), sizeof(uint32_t));
        ifs.read(reinterpret_cast<char *>(&file_res), sizeof(float));
        if (!ifs || !std::isfinite(file_res) || file_res <= 0.0f) {return false;}

        if (version >= 2) {
            float min_b[3], max_b[3];
            ifs.read(reinterpret_cast<char *>(min_b), sizeof(float) * 3);
            ifs.read(reinterpret_cast<char *>(max_b), sizeof(float) * 3);

            if (!ifs) {return false;}
            try {
              configure_grid(file_res, Eigen::Vector3d(min_b[0], min_b[1], min_b[2]),
                Eigen::Vector3d(max_b[0], max_b[1], max_b[2]));
            } catch (const std::invalid_argument &) {
              return false;
            } catch (const std::length_error &) {
              return false;
            }
            std::cout << "[DenseSpatialIndex] Adopted File Config: Res=" << grid_.getVoxelSize() << "m, Dims=" << dims_.transpose() << std::endl;
        } else {
            grid_.setVoxelSize(file_res);
            std::cout << "[DenseSpatialIndex] Version 1 VLUT: Adopting resolution " << file_res << "m (using default bounds)" << std::endl;
        }
    } else {
        // 識別子のない保存形式の既存読込経路
        ifs.seekg(0);
        std::cout << "[DenseSpatialIndex] Warning: Loading legacy format. Make sure config.txt matches the file data!" << std::endl;
    }

    size_t total_relations = 0;
    ifs.read(reinterpret_cast<char *>(&total_relations), sizeof(size_t));
    
    if (!ifs || total_relations > 100000000) {
        std::cerr << "[DenseSpatialIndex] ERROR: Suspiciously large relation count: " << total_relations << std::endl;
        return false;
    }
    // 不完全な入力による過大な中間配列確保の防止
    const auto data_pos = ifs.tellg();
    ifs.seekg(0, std::ios::end);
    const auto end_pos = ifs.tellg();
    const std::size_t record_bytes = sizeof(long) + 2 * sizeof(int) + sizeof(float);
    if (data_pos < 0 || end_pos < data_pos ||
      total_relations > static_cast<std::uint64_t>(end_pos - data_pos) / record_bytes) {
      return false;
    }
    ifs.seekg(data_pos);
    std::cout << "[DenseSpatialIndex] Loading " << total_relations << " relations..." << std::endl;

    // 復号済みセル番号と関係情報だけの16 byte中間表現
    struct load_relation {std::uint32_t cell_idx; Relation relation;};
    static_assert(sizeof(load_relation) == 16);
    std::vector<load_relation> raw_buffer(total_relations);
    
    // 各セルの関係数の集計
    std::vector<uint32_t> counts((offsets_.size() - 1), 0);

    for (size_t i = 0; i < total_relations; ++i) {
      long vid = 0;
      ifs.read(reinterpret_cast<char *>(&vid), sizeof(long));
      ifs.read(reinterpret_cast<char *>(&raw_buffer[i].relation.node_id), sizeof(int));
      ifs.read(reinterpret_cast<char *>(&raw_buffer[i].relation.distance), sizeof(float));
      ifs.read(reinterpret_cast<char *>(&raw_buffer[i].relation.link_id), sizeof(int));
      if (!ifs) {return false;}

      Eigen::Vector3i v_idx = grid_.getIndexFromFlatId(vid);
      Eigen::Vector3i g_idx = v_idx - index_offset_;

      if ((g_idx.array() >= 0).all() && (g_idx.array() < dims_.array()).all()) {
        int flat_g_idx = g_idx.x() + g_idx.y() * dims_.x() + g_idx.z() * dims_.x() * dims_.y();
        raw_buffer[i].cell_idx = flat_g_idx;
        counts[flat_g_idx]++;
      } else {
        raw_buffer[i].cell_idx = UINT32_MAX;
      }
      if (i % 5000000 == 0 && i > 0) {
          std::cout << "  Reading file: " << (i*100/total_relations) << "%\r" << std::flush;
      }
    }
    std::cout << "  File read complete. Building index structure..." << std::endl;

    // 終端を含むセル数+1個の累積オフセット。集計配列の書込位置への再利用
    uint32_t current_offset = 0;
    for (size_t i = 0; i < (offsets_.size() - 1); ++i) {
      offsets_[i] = current_offset;
      current_offset += counts[i];
      counts[i] = 0;
    }

    offsets_.back() = current_offset;
    node_data_.assign(current_offset, {0, 0.0f, 0});
    
    // 座標の再復号を省いた関係情報の配置
    for (size_t i = 0; i < total_relations; ++i) {
      const auto flat_g_idx = raw_buffer[i].cell_idx;
      if (flat_g_idx != UINT32_MAX) {
        uint32_t write_pos = offsets_[flat_g_idx] + counts[flat_g_idx];
        node_data_[write_pos] = raw_buffer[i].relation;
        counts[flat_g_idx]++;
      }
    }

    // 空でないセルの距離順整列
    std::cout << "  Sorting active voxels..." << std::endl;
    for (size_t i = 0; i < (offsets_.size() - 1); ++i) {
      const auto h = header_at(i);
      if (h.count > 1) {
        std::sort(node_data_.begin() + h.offset,
                  node_data_.begin() + h.offset + h.count,
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
    Eigen::Vector3i idx = (pos.array() / grid_.getVoxelSize()).floor().cast<int>();
    return grid_.getFlatVoxelId(idx);
  }

  void queryAABB(const Eigen::Vector3d &min_pt, const Eigen::Vector3d &max_pt,
                 std::vector<int> &collision_counts,
                 std::vector<int> &danger_counts, float threshold,
                 int delta) const override {

    Eigen::Vector3i start_idx = ((min_pt - world_min_).array() / grid_.getVoxelSize())
                                    .floor()
                                    .cast<int>()
                                    .cwiseMax(0);
    Eigen::Vector3i end_idx =
        ((max_pt - world_min_).array() / grid_.getVoxelSize()).ceil().cast<int>();

    end_idx = end_idx.cwiseMin(dims_ - Eigen::Vector3i::Ones()).cwiseMax(0);
    start_idx = start_idx.cwiseMin(dims_ - Eigen::Vector3i::Ones());

    for (int z = start_idx.z(); z <= end_idx.z(); ++z) {
      int z_offset = z * dims_.x() * dims_.y();
      for (int y = start_idx.y(); y <= end_idx.y(); ++y) {
        int y_offset = z_offset + y * dims_.x();
        for (int x = start_idx.x(); x <= end_idx.x(); ++x) {
          int flat_idx = y_offset + x;
          const auto &h = header_at(flat_idx);

          if (h.count == 0) {continue;}
          const Relation *rels = node_data_.data() + h.offset;
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
      Eigen::Vector3i v_idx = grid_.getIndexFromFlatId(vid);

      Eigen::Vector3i g_idx = v_idx - index_offset_;

      if ((g_idx.array() < 0).any() || (g_idx.array() >= dims_.array()).any()) {
        continue;
      }

      int flat_idx =
          g_idx.x() + g_idx.y() * dims_.x() + g_idx.z() * dims_.x() * dims_.y();
      const VoxelHeader &h = header_at(flat_idx);
      if (h.count == 0) {continue;}
      const Relation *rels = node_data_.data() + h.offset;
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
        ((point - world_min_).array() / grid_.getVoxelSize()).floor().cast<int>();

    if ((g_idx.array() < 0).any() || (g_idx.array() >= dims_.array()).any()) {
      return {};
    }

    int flat_idx =
        g_idx.x() + g_idx.y() * dims_.x() + g_idx.z() * dims_.x() * dims_.y();

    // Bounds check for flat index
    if (flat_idx < 0 || flat_idx >= (int)(offsets_.size() - 1))
      return {};

    const auto &h = header_at(flat_idx);
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
        grid_.getIndexFromFlatId(voxel_id);
    Eigen::Vector3i g_idx = v_idx - index_offset_;

    if ((g_idx.array() < 0).any() || (g_idx.array() >= dims_.array()).any()) {
      return {};
    }

    int flat_idx =
        g_idx.x() + g_idx.y() * dims_.x() + g_idx.z() * dims_.x() * dims_.y();
    if (flat_idx < 0 || flat_idx >= (int)(offsets_.size() - 1))
      return {};

    const auto &h = header_at(flat_idx);
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
        ((p_voxel - world_min_).array() / grid_.getVoxelSize()).floor().cast<int>();
    return getNodesInVoxelRawByIndex(g_idx);
  }

  std::pair<const Relation *, int> getRelationsInVoxelRaw(long voxel_id) const {
    Eigen::Vector3i v_idx =
        grid_.getIndexFromFlatId(voxel_id);
    Eigen::Vector3i g_idx = v_idx - index_offset_;
    return getRelationsInVoxelRawByIndex(g_idx);
  }

  std::pair<const Relation *, int>
  getRelationsInVoxelRawByIndex(const Eigen::Vector3i &g_idx) const {
    if (g_idx.minCoeff() < 0 || (g_idx.array() >= dims_.array()).any())
      return {nullptr, 0};

    int flat_idx = g_idx.x() + dims_.x() * (g_idx.y() + dims_.y() * g_idx.z());
    if (flat_idx < 0 || flat_idx >= (int)(offsets_.size() - 1))
      return {nullptr, 0};

    const auto &h = header_at(flat_idx);
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

  double getResolution() const { return grid_.getVoxelSize(); }
  const Eigen::Vector3i &getDims() const { return dims_; }

  // 既存の占有関係を持つセルだけの領域マスク。全空間の再ボクセル化なし。
  template <class cell_predicate>
  std::vector<long> select_voxel_ids(cell_predicate has_selected_cell) const {
    std::vector<long> ids;
    for (int z = 0; z < dims_.z(); ++z) {
      for (int y = 0; y < dims_.y(); ++y) {
        for (int x = 0; x < dims_.x(); ++x) {
          const int flat_idx = x + dims_.x() * (y + dims_.y() * z);
          const auto entry = header_at(flat_idx);
          if (entry.count == 0 || node_data_[entry.offset].distance > 1e-6f) {continue;}
          const Eigen::Vector3i cell = index_offset_ + Eigen::Vector3i(x, y, z);
          const Eigen::Vector3d low = cell.cast<double>() * grid_.getVoxelSize();
          const Eigen::Vector3d high = low + Eigen::Vector3d::Constant(grid_.getVoxelSize());
          if (has_selected_cell(low, high)) {ids.push_back(grid_.getFlatVoxelId(cell));}
        }
      }
    }
    return ids;
  }

  // --- Dynamic Update Implementation ---
  void clear() override {
    if (is_static_) {
      std::cout << "[DenseSpatialIndex] Warning: Attempting to clear a static (loaded) index. Operation ignored." << std::endl;
      return;
    }
    // Reset headers and data
    std::fill(offsets_.begin(), offsets_.end(), 0);
    node_data_.clear();
    pending_updates_.clear();
  }

  void insert(int id, const Eigen::Vector3d &pos) override {
    if (is_static_) return;
    // Calculate Voxel ID
    Eigen::Vector3i g_idx =
        ((pos - world_min_).array() / grid_.getVoxelSize()).floor().cast<int>();

    if ((g_idx.array() >= 0).all() && (g_idx.array() < dims_.array()).all()) {
      int flat_g_idx =
          g_idx.x() + g_idx.y() * dims_.x() + g_idx.z() * dims_.x() * dims_.y();

      // Calculate distance from voxel center (approximation)
      Eigen::Vector3d voxel_center =
          world_min_ +
          (g_idx.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * grid_.getVoxelSize();
      float dist = (float)(pos - voxel_center).norm();

      pending_updates_[flat_g_idx].push_back({id, dist, -1});
    }
  }

  void build() override {
    if (is_static_) return;
    node_data_.clear();
    // 追加待ちの関係情報による全セルの累積オフセット再構築
    for (int i = 0; i < (int)(offsets_.size() - 1); ++i) {
      offsets_[i] = static_cast<uint32_t>(node_data_.size());
      auto it = pending_updates_.find(i);
      if (it != pending_updates_.end()) {
        if (it->second.size() > UINT32_MAX - node_data_.size()) {
          throw std::length_error("spatial index relations exceed uint32 range");
        }

        // セル内の距離順整列
        std::sort(it->second.begin(), it->second.end(),
                  [](const Relation &a, const Relation &b) {
                    return a.distance < b.distance;
                  });

        node_data_.insert(node_data_.end(), it->second.begin(),
                          it->second.end());
      }
    }
    offsets_.back() = static_cast<uint32_t>(node_data_.size());
  }

private:
  // 平坦添字のint範囲と、外部座標のint範囲を保つ格子設定
  void configure_grid(double res, const Eigen::Vector3d &min_bounds,
    const Eigen::Vector3d &max_bounds) {
    if (!std::isfinite(res) || res <= 0 || !min_bounds.allFinite() ||
      !max_bounds.allFinite() || (min_bounds.array() > max_bounds.array()).any()) {
      throw std::invalid_argument("invalid spatial index bounds or resolution");
    }
    const Eigen::Array3d dims = ((max_bounds - min_bounds) / res).array().ceil().max(1.0);
    const Eigen::Array3d offset = (min_bounds.array() / res).floor();
    const double max_idx = std::numeric_limits<int>::max();
    if (!dims.isFinite().all() || dims.prod() > max_idx || !offset.isFinite().all() ||
      (offset < std::numeric_limits<int>::min()).any() || (offset > max_idx).any() ||
      (offset + dims - 1.0 > max_idx).any()) {
      throw std::length_error("spatial index grid exceeds int range");
    }
    dims_ = dims.cast<int>();
    index_offset_ = offset.cast<int>();
    world_min_ = min_bounds;
    grid_.setVoxelSize(res);
    offsets_.assign(static_cast<std::size_t>(dims_.prod()) + 1, 0);
  }

  // 隣接セルで共有する終端オフセットからの範囲参照
  VoxelHeader header_at(std::size_t idx) const {
    return {offsets_[idx], offsets_[idx + 1] - offsets_[idx]};
  }

  ::GNG::Analysis::IndexVoxelGrid grid_;
  Eigen::Vector3d world_min_;
  Eigen::Vector3i dims_;
  Eigen::Vector3i index_offset_;
  bool is_static_ = false;

  std::vector<uint32_t> offsets_;
  std::vector<Relation> node_data_;
  std::unordered_map<int, std::vector<Relation>>
      pending_updates_; // For dynamic updates
};

} // namespace analysis
} // namespace robot_sim
