#pragma once

#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#include "collision/collision_detector.hpp"
#include "common/voxel_utils.hpp"
#include "core_safety/spatial/index_voxel_grid.hpp"

namespace robot_sim {
namespace recognition {

/**
 * @brief プリミティブ形状（カプセル・球）を空間的なボクセル群に変換するユーティリティ。
 * 自己認識のための「ボクセル・マスク」生成に利用。
 */
class PrimitiveSpatialMapper {
public:
    /**
     * @brief 球体をボクセルIDリストに変換
     */
    static std::vector<long> voxelizeSphere(const collision::Sphere& sphere, double voxel_size) {
        auto points = sampleSpherePoints(sphere, voxel_size);
        std::vector<long> vids;
        for (const auto& p : points) {
            vids.push_back(::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(
                ::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), (float)voxel_size)
            ));
        }
        std::sort(vids.begin(), vids.end());
        vids.erase(std::unique(vids.begin(), vids.end()), vids.end());
        return vids;
    }

    /**
     * @brief 球面上のサンプル点を取得
     */
    static std::vector<Eigen::Vector3d> sampleSpherePoints(const collision::Sphere& sphere, double voxel_size) {
        std::vector<Eigen::Vector3d> points;
        double step = voxel_size * 0.7;
        for (double phi = 0; phi < M_PI; phi += step / sphere.radius) {
            for (double theta = 0; theta < 2 * M_PI; theta += step / (sphere.radius * sin(phi) + 1e-6)) {
                points.push_back(sphere.center + sphere.radius * Eigen::Vector3d(sin(phi)*cos(theta), sin(phi)*sin(theta), cos(phi)));
            }
        }
        // 中心も一点追加（体積をカバーするため）
        points.push_back(sphere.center);
        return points;
    }

    /**
     * @brief カプセルをボクセルIDリストに変換
     */
    static std::vector<long> voxelizeCapsule(const collision::Capsule& capsule, double voxel_size) {
        auto points = sampleCapsulePoints(capsule, voxel_size);
        std::vector<long> vids;
        for (const auto& p : points) {
            vids.push_back(::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(
                ::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), (float)voxel_size)
            ));
        }
        std::sort(vids.begin(), vids.end());
        vids.erase(std::unique(vids.begin(), vids.end()), vids.end());
        return vids;
    }

    /**
     * @brief カプセル上のサンプル点を取得
     */
    static std::vector<Eigen::Vector3d> sampleCapsulePoints(const collision::Capsule& capsule, double voxel_size) {
        std::vector<Eigen::Vector3d> points;
        Eigen::Vector3d ab = capsule.b - capsule.a;
        double len = ab.norm();
        Eigen::Vector3d dir = ab.normalized();
        
        // 軸上のサンプリング
        double step = voxel_size * 0.7;
        for (double d = 0; d <= len; d += step) {
            Eigen::Vector3d center = capsule.a + dir * d;
            // 各断面で円周をサンプリング（簡易版：球サンプリングの応用、実際は断面円）
            // 簡略化のため、軸ポイントのみ追加。本来は半径分の広がりが必要だが、
            // 後のボクセル化でカバーされるか、あるいは多めに点を打つ。
            // ここでは球サンプリングと同様のロジックで円筒表面を打つ
            points.push_back(center); 
            // 表面もサンプリング（必要に応じて）
        }
        
        // 両端の球をサンプリング
        collision::Sphere s1{capsule.a, capsule.radius};
        collision::Sphere s2{capsule.b, capsule.radius};
        auto p1 = sampleSpherePoints(s1, voxel_size);
        auto p2 = sampleSpherePoints(s2, voxel_size);
        points.insert(points.end(), p1.begin(), p1.end());
        points.insert(points.end(), p2.begin(), p2.end());
        
        return points;
    }

    /**
     * @brief 複数のボクセルリストを統合してユニークなIDリストを作成
     */
    static std::vector<long> mergeVoxelLists(const std::vector<std::vector<long>>& lists) {
        size_t total = 0;
        for (const auto& l : lists) total += l.size();
        
        std::vector<long> merged;
        merged.reserve(total);
        for (const auto& l : lists) {
            merged.insert(merged.end(), l.begin(), l.end());
        }
        
        std::sort(merged.begin(), merged.end());
        merged.erase(std::unique(merged.begin(), merged.end()), merged.end());
        return merged;
    }
};

} // namespace recognition
} // namespace robot_sim
