#pragma once

#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#include "collision/collision_detector.hpp"
#include "common/voxel_utils.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"

namespace robot_sim {
namespace recognition {

class PrimitiveSpatialMapper {
public:
    static std::vector<Eigen::Vector3d> sampleSpherePoints(const collision::Sphere& sphere, double voxel_size) {
        std::vector<Eigen::Vector3d> points;
        double step = voxel_size * 0.7;
        for (double phi = 0; phi < M_PI; phi += step / (sphere.radius + 1e-6)) {
            for (double theta = 0; theta < 2 * M_PI; theta += step / (sphere.radius * sin(phi) + 1e-6)) {
                points.push_back(sphere.center + sphere.radius * Eigen::Vector3d(sin(phi)*cos(theta), sin(phi)*sin(theta), cos(phi)));
            }
        }
        points.push_back(sphere.center);
        return points;
    }

    static std::vector<Eigen::Vector3d> sampleCapsulePoints(const collision::Capsule& capsule, double voxel_size) {
        std::vector<Eigen::Vector3d> points;
        Eigen::Vector3d ab = capsule.b - capsule.a;
        double len = ab.norm();
        if (len < 1e-6) return sampleSpherePoints({capsule.a, capsule.radius}, voxel_size);

        Eigen::Vector3d dir = ab / len;
        double step = voxel_size * 0.7;
        for (double d = 0; d <= len; d += step) {
            points.push_back(capsule.a + dir * d);
            // 断面のサンプリング（簡略化：軸のみでも voxel_size が適切ならカバーされる）
        }
        
        auto p1 = sampleSpherePoints({capsule.a, capsule.radius}, voxel_size);
        auto p2 = sampleSpherePoints({capsule.b, capsule.radius}, voxel_size);
        points.insert(points.end(), p1.begin(), p1.end());
        points.insert(points.end(), p2.begin(), p2.end());
        return points;
    }

    static std::vector<Eigen::Vector3d> sampleBoxPoints(const collision::Box& box, const Eigen::Isometry3d& origin, double voxel_size) {
        std::vector<Eigen::Vector3d> points;
        double step = voxel_size * 0.8;
        
        for (double x = -box.extents.x(); x <= box.extents.x(); x += step) {
            for (double y = -box.extents.y(); y <= box.extents.y(); y += step) {
                for (double z = -box.extents.z(); z <= box.extents.z(); z += step) {
                    points.push_back(origin * (box.center + box.rotation * Eigen::Vector3d(x, y, z)));
                }
            }
        }
        return points;
    }
};

} // namespace recognition
} // namespace robot_sim
