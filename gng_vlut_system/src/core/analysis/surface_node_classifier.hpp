#pragma once

#include "node_classifier.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace GNG {
namespace Analysis {

/**
 * @brief 表面ノード分類器 (Surface Node Classifier)
 * 重心からのレイキャスティング法を用いて、3D形状の外殻（表面）にあるノードを判定する。
 */
template <typename GNGType>
class SurfaceNodeClassifier : public INodeClassifier<GNGType> {
public:
  // レイキャスティング方向のサンプリング数
  int sample_directions = 500;

  void classify(GNGType &gng) override {
    std::cout << "[SurfaceClassifier] Starting surface node detection..."
              << std::endl;

    int valid_count = 0;
    Eigen::Vector3f actual_origin = origin;

    // Auto-calculate centroid if origin is zero
    if (actual_origin.isZero()) {
      Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
      gng.forEachActive([&](int, const auto &node) {
        centroid += node.weight_coord;
        valid_count++;
      });
      if (valid_count > 0)
        actual_origin = centroid / (float)valid_count;
    } else {
      gng.forEachActive([&](int, const auto &) { valid_count++; });
    }

    if (valid_count == 0)
      return;

    const int max_nodes = (int)gng.getMaxNodeNum();

    // 2. 単位球面上に方向ベクトルを生成 (Fibonacci Sphere Algorithm)
    std::vector<Eigen::Vector3f> directions;
    directions.reserve(sample_directions);
    float phi = std::acos(-1.0f) * (3.0f - std::sqrt(5.0f));

    for (int i = 0; i < sample_directions; ++i) {
      float y = 1.0f - (i / (float)(sample_directions - 1)) * 2.0f;
      float radius = std::sqrt(1.0f - y * y);
      float theta = phi * i;
      directions.emplace_back(std::cos(theta) * radius, y,
                              std::sin(theta) * radius);
    }

    // 3. 各方向について最大射影距離を取得
    std::vector<float> max_projections(sample_directions,
                                       -std::numeric_limits<float>::max());

    for (int d = 0; d < sample_directions; ++d) {
      const auto &dir = directions[d];
      for (int i = 0; i < max_nodes; ++i) {
        const auto &node = gng.nodeAt(i);
        if (node.id != -1 && node.status.active) {
          float proj = (node.weight_coord - actual_origin).dot(dir);
          if (proj > max_projections[d]) {
            max_projections[d] = proj;
          }
        }
      }
    }

    // 4. フラグ設定
    int surface_detected_count = 0;
    for (int i = 0; i < max_nodes; ++i) {
      auto &node = const_cast<typename GNGType::NodeType &>(gng.nodeAt(i));
      if (node.id == -1 || !node.status.active)
        continue;

      node.status.is_surface = false; // Reset

      bool is_on_surface = false;
      Eigen::Vector3f vec = node.weight_coord - actual_origin;

      // Heuristic 1: Raycast projection (Furthest from origin)
      for (int d = 0; d < sample_directions; ++d) {
        float proj = vec.dot(directions[d]);
        if (proj >= max_projections[d] - thickness_threshold) {
          is_on_surface = true;
          break;
        }
      }

      // Heuristic 2: Degree check (Isolated/Edge nodes)
      // Surface nodes often have fewer neighbors in a 3D graph.
      if (!is_on_surface && use_degree_check) {
        if (gng.getNeighborsAngle(node.id).size() <= 3) {
          is_on_surface = true;
        }
      }

      if (is_on_surface) {
        node.status.is_surface = true;
        surface_detected_count++;
      }
    }

    std::cout << "[SurfaceClassifier] Detected " << surface_detected_count
              << " unit surface nodes (thickness: " << thickness_threshold
              << ", degree_check: " << (use_degree_check ? "ON" : "OFF")
              << ") out of " << valid_count << " active nodes." << std::endl;
  }

  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  float thickness_threshold = 0.05f;
  bool use_degree_check = true; // Use node degree as auxiliary hint
};

} // namespace Analysis
} // namespace GNG
