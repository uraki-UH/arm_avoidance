#pragma once

#include "node_classifier.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

namespace GNG {
namespace Analysis {

/**
 * @brief 重心ズレベースの表面ノード分類器 (Centroid Shift Surface Classifier)
 *
 * 各ノードについて、その近傍ノードの幾何学的重心を計算し、元の位置からのズレ（距離）を評価する。
 * 内部にあるノードは周囲をノードに囲まれているため重心ズレが小さくなるが、
 * 表面にあるノードは一方向にのみ近傍が偏るため、重心が「内側」へ大きくズレる特性を利用する。
 */
template <typename GNGType>
class CentroidShiftSurfaceClassifier : public INodeClassifier<GNGType> {
public:
  // 表面と判定するズレのしきい値係数 (平均エッジ長の何倍か)
  float shift_threshold_factor = 0.4f;
  // 表面とみなす「殻」の厚さ (m)
  float shell_thickness = 0.10f; // 10cm
  // レイ方向数
  int sample_directions = 1000;

  void classify(GNGType &gng) override {
    std::cout
        << "[CentroidShift] Starting hybrid topological-geometric detection..."
        << std::endl;

    int max_nodes = (int)gng.getMaxNodeNum();
    std::vector<float> shifts(max_nodes, 0.0f);
    std::vector<float> edge_lengths;

    // 0. 方向ベクトルの生成 (Visibility用)
    std::vector<Eigen::Vector3f> directions;
    float phi = std::acos(-1.0f) * (3.0f - std::sqrt(5.0f));
    for (int i = 0; i < sample_directions; ++i) {
      float y = 1.0f - (i / (float)(sample_directions - 1)) * 2.0f;
      float radius = std::sqrt(1.0f - y * y);
      float theta = phi * i;
      directions.emplace_back(std::cos(theta) * radius, y,
                              std::sin(theta) * radius);
    }

    // 1. 各方向の最大距離を取得 (Outer Shellの外壁を定義)
    std::vector<float> max_projections(sample_directions, -1e9f);
    gng.forEachActive([&](int, const auto &node) {
      for (int d = 0; d < sample_directions; ++d) {
        float proj = node.weight_coord.dot(directions[d]);
        if (proj > max_projections[d])
          max_projections[d] = proj;
      }
    });

    // 2. 各ノードの重心ズレを計算
    int active_count = 0;
    gng.forEachActive([&](int i, const auto &node) {
      active_count++;
      const auto &neighbors = gng.getNeighborsAngle(node.id);
      if (!neighbors.empty()) {
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (int nid : neighbors) {
          const auto &nb = gng.nodeAt(nid);
          centroid += nb.weight_coord;
          edge_lengths.push_back((node.weight_coord - nb.weight_coord).norm());
        }
        centroid /= (float)neighbors.size();
        shifts[i] = (node.weight_coord - centroid).norm();
      }
      const_cast<typename GNGType::NodeType &>(node).status.is_surface = false;
    });

    if (active_count == 0 || edge_lengths.empty())
      return;

    // 3. 判定 (Shift > Threshold AND Close to Max Projection)
    float avg_edge_len =
        std::accumulate(edge_lengths.begin(), edge_lengths.end(), 0.0f) /
        edge_lengths.size();
    float shift_threshold = avg_edge_len * shift_threshold_factor;

    int surface_count = 0;
    for (int i = 0; i < max_nodes; ++i) {
      auto &node = gng.nodeAt(i);
      if (node.id == -1 || !node.status.active)
        continue;

      // a. トポロジー的境界判定
      bool is_topological_boundary = (shifts[i] > shift_threshold);
      if (!is_topological_boundary)
        continue;

      // b. 幾何学的外殻判定 (最外層付近にいるか)
      bool is_visible_outer = false;
      // 複数のレイ方向でチェック (自分の法線方向だけでなく、近くの方向も)
      Eigen::Vector3f pos = node.weight_coord;
      for (int d = 0; d < sample_directions; d += 10) { // 間引いて高速化
        float proj = pos.dot(directions[d]);
        if (proj >= max_projections[d] - shell_thickness) {
          // もしその方向の最先端から shell_thickness 以内にいるなら「外殻領域」
          is_visible_outer = true;
          break;
        }
      }

      if (is_visible_outer) {
        const_cast<typename GNGType::NodeType &>(node).status.is_surface = true;
        surface_count++;
      }
    }

    std::cout << "[CentroidShift] Avg Edge: " << avg_edge_len
              << "m, Shift Thresh: " << shift_threshold << "m" << std::endl;
    std::cout << "[CentroidShift] Detected " << surface_count
              << " hybrid surface nodes." << std::endl;
  }
};

} // namespace Analysis
} // namespace GNG
