#pragma once

#include <vector>
#include <Eigen/Dense>
#include "description/stl_loader.hpp"
#include "core_safety/spatial/index_voxel_grid.hpp"

namespace robot_sim {
namespace recognition {

/**
 * @brief メッシュデータをボクセル集合に変換するユーティリティ。
 */
class MeshVoxelizer {
public:
    /**
     * @brief メッシュ表面を占有するボクセルのIDリストを取得する
     * @param mesh 生のメッシュデータ
     * @param transform メッシュの現在のポーズ（世界座標系）
     * @param voxel_size ボクセルサイズ
     */
    static std::vector<long> voxelizeMesh(const simulation::MeshData& mesh, 
                                          const Eigen::Isometry3d& transform,
                                          double voxel_size) {
        auto points = sampleMeshPoints(mesh, transform, voxel_size);
        std::set<long> unique_vids;
        for (const auto& p : points) {
            Eigen::Vector3i idx = GNG::Analysis::IndexVoxelGrid::getIndex(p.cast<float>(), (float)voxel_size);
            unique_vids.insert(GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx));
        }
        return std::vector<long>(unique_vids.begin(), unique_vids.end());
    }

    /**
     * @brief メッシュ表面をサンプリングした3D点群を取得する
     */
    static std::vector<Eigen::Vector3d> sampleMeshPoints(const simulation::MeshData& mesh,
                                                        const Eigen::Isometry3d& transform,
                                                        double voxel_size) {
        if (mesh.vertices.empty() || mesh.indices.empty()) return {};

        std::vector<Eigen::Vector3d> points;
        // 三角形ごとにサンプリング
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            uint32_t i1 = mesh.indices[i];
            uint32_t i2 = mesh.indices[i+1];
            uint32_t i3 = mesh.indices[i+2];

            Eigen::Vector3d v1 = transform * Eigen::Vector3d(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
            Eigen::Vector3d v2 = transform * Eigen::Vector3d(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
            Eigen::Vector3d v3 = transform * Eigen::Vector3d(mesh.vertices[i3*3], mesh.vertices[i3*3+1], mesh.vertices[i3*3+2]);

            // 三角形表面をサンプリング
            sampleTriangle(v1, v2, v3, voxel_size, points);
        }
        return points;
    }

private:
    static void sampleTriangle(const Eigen::Vector3d& v1, 
                              const Eigen::Vector3d& v2, 
                              const Eigen::Vector3d& v3,
                              double voxel_size,
                              std::vector<Eigen::Vector3d>& points) {
        Eigen::Vector3d edge1 = v2 - v1;
        Eigen::Vector3d edge2 = v3 - v1;

        double area = 0.5 * edge1.cross(edge2).norm();
        if (area < 1e-7) return;

        double l1 = edge1.norm();
        double l2 = edge2.norm();
        int n1 = std::max(1, static_cast<int>(l1 / (voxel_size * 0.5)));
        int n2 = std::max(1, static_cast<int>(l2 / (voxel_size * 0.5)));

        for (int i = 0; i <= n1; ++i) {
            for (int j = 0; j <= n2 - i * n2 / n1; ++j) {
                double u = static_cast<double>(i) / n1;
                double v = static_cast<double>(j) / n2;
                if (u + v > 1.0) continue;
                points.push_back(v1 + u * edge1 + v * edge2);
            }
        }
    }
};

} // namespace recognition
} // namespace robot_sim
