#pragma once

#include "node_classifier.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace GNG {
namespace Analysis {

/**
 * @brief 占有グリッドベースの表面ノード分類器 (Occupancy Grid Surface
 * Classifier)
 *
 * 3D空間をグリッドに分割し、占有されている（ノードが存在する）セルのうち、
 * 「空」の隣接セルを持つものを境界（表面）と判定する。
 * レイキャスティングの死角に依存しない、幾何学的に厳密な表面抽出が可能。
 */
template <typename GNGType>
class OccupancyGridSurfaceClassifier : public INodeClassifier<GNGType> {
public:
  // グリッドの解像度 (メートル単位)
  float grid_resolution = 0.015f; // 1.5cm (for 10,000 nodes)
  // 表面とみなす層の厚さ (1 = 最外殻のみ, 2以上で厚みを持たせる)
  int shell_layers = 1;

  void classify(GNGType &gng) override {
    std::cout
        << "[OccupancySurface] Starting geometric boundary detection (res: "
        << grid_resolution << ", layers: " << shell_layers << ")..."
        << std::endl;

    // 1. ノードのインデックス化
    std::vector<int> active_indices;
    gng.forEachActive([&](int i, const auto &) {
      active_indices.push_back(i);
      const_cast<typename GNGType::NodeType &>(gng.nodeAt(i))
          .status.is_surface = false;
    });

    if (active_indices.empty())
      return;

    // 2. セルごとの占有状況を記録
    struct GridKey {
      int x, y, z;
      bool operator==(const GridKey &other) const {
        return x == other.x && y == other.y && z == other.z;
      }
    };
    struct GridKeyHash {
      std::size_t operator()(const GridKey &k) const {
        return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^
               (std::hash<int>()(k.z) << 1);
      }
    };

    std::unordered_map<GridKey, std::vector<int>, GridKeyHash> grid;

    auto get_key = [&](const Eigen::Vector3f &pos) {
      return GridKey{(int)std::floor(pos.x() / grid_resolution),
                     (int)std::floor(pos.y() / grid_resolution),
                     (int)std::floor(pos.z() / grid_resolution)};
    };

    for (int idx : active_indices) {
      grid[get_key(gng.nodeAt(idx).weight_coord)].push_back(idx);
    }

    // 3. 境界セルの判定
    int surface_node_count = 0;

    for (auto const &[key, nodes] : grid) {
      bool is_boundary_cell = false;

      // 26近傍のうち、1つでも空のセルがあれば「境界ボクセル」
      for (int dx = -1; dx <= 1 && !is_boundary_cell; ++dx) {
        for (int dy = -1; dy <= 1 && !is_boundary_cell; ++dy) {
          for (int dz = -1; dz <= 1 && !is_boundary_cell; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0)
              continue;

            GridKey neighbor_key = {key.x + dx, key.y + dy, key.z + dz};
            if (grid.find(neighbor_key) == grid.end()) {
              is_boundary_cell = true;
            }
          }
        }
      }

      if (is_boundary_cell) {
        for (int node_idx : nodes) {
          // さらに「隣接ノード数」でフィルタリング
          // 3D空間のGNGでは、内部ノードは通常7〜12個以上の隣接を持つ。
          // 表面ノードはトポロジ的に接続が「片側」に偏るため、次数が少なくなる。
          if (gng.getNeighborsAngle(node_idx).size() <= 8) {
            const_cast<typename GNGType::NodeType &>(gng.nodeAt(node_idx))
                .status.is_surface = true;
            surface_node_count++;
          }
        }
      }
    }

    std::cout << "[OccupancySurface] Grid Cells: " << grid.size() << std::endl;
    std::cout << "[OccupancySurface] Detected " << surface_node_count
              << " surface nodes using grid + degree heuristic." << std::endl;
  }
};

} // namespace Analysis
} // namespace GNG
