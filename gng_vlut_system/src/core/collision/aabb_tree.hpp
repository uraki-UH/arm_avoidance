#pragma once

#include "collision/collision_detector.hpp"
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <limits>

namespace collision {

/**
 * @brief 境界ボリューム階層（AABB Tree）の実装
 */
class AABBTree {
private:
    struct Node {
        AABB bounds;  // collision_detector.hppの統一AABBを使用
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
        std::vector<int> triangle_indices;
        
        bool isLeaf() const { return left == nullptr && right == nullptr; }
    };

public:
    void build(const std::vector<Triangle>& triangles);
    
    int findClosestTriangle(const Eigen::Vector3d& point, 
                           const std::vector<Triangle>& triangles,
                           double& min_distance) const;
    
    void queryIntersections(const Eigen::Vector3d& capsule_a,
                           const Eigen::Vector3d& capsule_b,
                           double capsule_radius,
                           const std::vector<Triangle>& triangles,
                           std::vector<int>& result_indices) const;

private:
    std::unique_ptr<Node> root_;
    
    std::unique_ptr<Node> buildRecursive(const std::vector<Triangle>& triangles,
                                        std::vector<int> triangle_indices,
                                        int depth = 0);
    
    AABB computeTriangleBounds(const Triangle& tri) const;
    
    void queryRecursive(const Node* node,
                       const AABB& query_bounds,
                       const std::vector<Triangle>& triangles,
                       std::vector<int>& result_indices) const;
    
    void findClosestRecursive(const Node* node,
                             const Eigen::Vector3d& point,
                             const std::vector<Triangle>& triangles,
                             int& closest_triangle,
                             double& min_distance) const;
    
    double pointToAABBDistance(const Eigen::Vector3d& point, const AABB& aabb) const;
};

} // namespace collision