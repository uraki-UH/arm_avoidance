#pragma once

#include "node_classifier.hpp"
#include <iostream>
#include <vector>

namespace GNG {
namespace Analysis {

/**
 * @brief 境界ノード分類器 (Boundary Node Classifier)
 * 手法B: 無効ノード・エッジ隣接法 (Invalid Neighbor/Edge Check)
 *
 * 有効なノードのうち、
 * 1. 隣接ノードが無効 (status.active == false) であるもの
 * 2. (オプション) 本来あるはずのエッジが欠損しているもの
 * を境界ノード (is_boundary = true) と判定する。
 */
template <typename GNGType>
class NeighborBasedBoundaryClassifier : public INodeClassifier<GNGType> {
public:
  void classify(GNGType &gng) override {
    std::cout << "[BoundaryClassifier] Starting boundary node detection..."
              << std::endl;

    int boundary_count = 0;
    int valid_count = 0;

    // まずリセット
    gng.forEachActive([&](int /*i*/, const auto &node) {
      const_cast<typename GNGType::NodeType &>(node).status.is_boundary = false;
      valid_count++;
    });

    // 走査
    gng.forEachActive([&](int i, const auto &node) {
      auto &mnode = const_cast<typename GNGType::NodeType &>(node);
      const auto &neighbors = gng.getNeighborsAngle(node.id);
      bool is_boundary = false;
      for (int nid : neighbors) {
        const auto &nb = gng.nodeAt(nid);
        if (nb.id != -1 && !nb.status.active) {
          is_boundary = true;
          break;
        }
      }
      if (!is_boundary) {
        for (int nid : neighbors) {
          if (!gng.isEdgeActive(i, nid, 0) && gng.nodeAt(nid).status.active) {
            is_boundary = true;
            break;
          }
        }
      }
      if (is_boundary) {
        mnode.status.is_boundary = true;
        boundary_count++;
      }
    });

    std::cout << "[BoundaryClassifier] Detected " << boundary_count
              << " boundary nodes out of " << valid_count << " active nodes."
              << std::endl;
  }
};

} // namespace Analysis
} // namespace GNG
