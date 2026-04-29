#include "collision/aabb_tree.hpp"
#include "collision/collision_detector.hpp"
#include <algorithm>
#include <limits>
#include <numeric>  // std::iota用

namespace collision {

void AABBTree::build(const std::vector<Triangle>& triangles) {
    if (triangles.empty()) {
        root_ = nullptr;
        return;
    }
    
    std::vector<int> indices(triangles.size());
    std::iota(indices.begin(), indices.end(), 0);
    
    root_ = buildRecursive(triangles, std::move(indices));
}

std::unique_ptr<AABBTree::Node> AABBTree::buildRecursive(
    const std::vector<Triangle>& triangles,
    std::vector<int> triangle_indices,
    int depth) {
    
    auto node = std::make_unique<Node>();
    
    for (int idx : triangle_indices) {
        AABB tri_bounds = computeTriangleBounds(triangles[idx]);
        node->bounds.expand(tri_bounds);
    }
    
    const int MAX_TRIANGLES_PER_LEAF = 4;
    const int MAX_DEPTH = 20;
    
    if (triangle_indices.size() <= MAX_TRIANGLES_PER_LEAF || depth >= MAX_DEPTH) {
        node->triangle_indices = std::move(triangle_indices);
        return node;
    }
    
    Eigen::Vector3d extent = node->bounds.max - node->bounds.min;
    int split_axis = 0;
    if (extent.y() > extent.x()) split_axis = 1;
    if (extent.z() > extent[split_axis]) split_axis = 2;
    
    double split_value = (node->bounds.min[split_axis] + node->bounds.max[split_axis]) * 0.5;
    
    std::vector<int> left_indices, right_indices;
    
    for (int idx : triangle_indices) {
        const Triangle& tri = triangles[idx];
        double centroid = (tri.v0[split_axis] + tri.v1[split_axis] + tri.v2[split_axis]) / 3.0;
        
        if (centroid < split_value) {
            left_indices.push_back(idx);
        } else {
            right_indices.push_back(idx);
        }
    }
    
    if (left_indices.empty() || right_indices.empty()) {
        node->triangle_indices = std::move(triangle_indices);
        return node;
    }
    
    node->left = buildRecursive(triangles, std::move(left_indices), depth + 1);
    node->right = buildRecursive(triangles, std::move(right_indices), depth + 1);
    
    return node;
}

int AABBTree::findClosestTriangle(const Eigen::Vector3d& point,
                                 const std::vector<Triangle>& triangles,
                                 double& min_distance) const {
    if (!root_) {
        min_distance = std::numeric_limits<double>::max();
        return -1;
    }
    
    int closest_triangle = -1;
    min_distance = std::numeric_limits<double>::max();
    
    findClosestRecursive(root_.get(), point, triangles, closest_triangle, min_distance);
    
    return closest_triangle;
}

void AABBTree::queryIntersections(const Eigen::Vector3d& capsule_a,
                                 const Eigen::Vector3d& capsule_b,
                                 double capsule_radius,
                                 const std::vector<Triangle>& triangles,
                                 std::vector<int>& result_indices) const {
    if (!root_) return;
    
    AABB query_bounds;
    query_bounds.expand(capsule_a);
    query_bounds.expand(capsule_b);
    
    Eigen::Vector3d radius_vec = Eigen::Vector3d::Ones() * capsule_radius;
    query_bounds.min -= radius_vec;
    query_bounds.max += radius_vec;
    
    result_indices.clear();
    queryRecursive(root_.get(), query_bounds, triangles, result_indices);
}

AABB AABBTree::computeTriangleBounds(const Triangle& tri) const {
    AABB bounds;
    bounds.expand(tri.v0);
    bounds.expand(tri.v1);
    bounds.expand(tri.v2);
    return bounds;
}

void AABBTree::queryRecursive(const Node* node,
                             const AABB& query_bounds,
                             const std::vector<Triangle>& triangles,
                             std::vector<int>& result_indices) const {
    if (!node || !node->bounds.intersects(query_bounds)) {
        return;
    }
    
    if (node->isLeaf()) {
        for (int idx : node->triangle_indices) {
            result_indices.push_back(idx);
        }
    } else {
        queryRecursive(node->left.get(), query_bounds, triangles, result_indices);
        queryRecursive(node->right.get(), query_bounds, triangles, result_indices);
    }
}

void AABBTree::findClosestRecursive(const Node* node,
                                   const Eigen::Vector3d& point,
                                   const std::vector<Triangle>& triangles,
                                   int& closest_triangle,
                                   double& min_distance) const {
    if (!node) return;
    
    double aabb_distance = pointToAABBDistance(point, node->bounds);
    if (aabb_distance > min_distance) {
        return;
    }
    
    if (node->isLeaf()) {
        for (int idx : node->triangle_indices) {
            const Triangle& tri = triangles[idx];
            Eigen::Vector3d centroid = (tri.v0 + tri.v1 + tri.v2) / 3.0;
            double distance = (point - centroid).norm();
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_triangle = idx;
            }
        }
    } else {
        findClosestRecursive(node->left.get(), point, triangles, closest_triangle, min_distance);
        findClosestRecursive(node->right.get(), point, triangles, closest_triangle, min_distance);
    }
}

double AABBTree::pointToAABBDistance(const Eigen::Vector3d& point, const AABB& aabb) const {
    Eigen::Vector3d closest = point;
    
    for (int i = 0; i < 3; ++i) {
        if (point[i] < aabb.min[i]) {
            closest[i] = aabb.min[i];
        } else if (point[i] > aabb.max[i]) {
            closest[i] = aabb.max[i];
        }
    }
    
    return (point - closest).norm();
}

} // namespace collision