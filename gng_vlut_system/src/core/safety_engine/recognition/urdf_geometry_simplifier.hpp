#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <vector>
#include <string>
#include <memory>
#include <iostream>

#include "robot_model/robot_model.hpp"
#include "robot_model/stl_loader.hpp"
#include "collision/collision_detector.hpp"
#include "common/resource_utils.hpp"

namespace robot_sim {
namespace recognition {

/**
 * @brief URDFの複雑なジオメトリを、判定用のシンプルなプリミティブ（カプセル・球）に変換するクラス。
 */
class UrdfGeometrySimplifier {
public:
    struct SimplifiedLink {
        std::string name;
        std::vector<collision::Capsule> capsules;
        std::vector<collision::Sphere> spheres;
        
        // メッシュデータ（高精度用）
        std::vector<simulation::MeshData> meshes;
        
        // リンク原点からのオフセット（URDFのorigin）
        std::vector<Eigen::Isometry3d> capsule_origins;
        std::vector<Eigen::Isometry3d> sphere_origins;
        std::vector<Eigen::Isometry3d> mesh_origins;
    };

    /**
     * @brief リンクの衝突モデルから簡略化形状を生成
     */
    static SimplifiedLink simplifyLink(const simulation::LinkProperties& link) {
        SimplifiedLink res;
        res.name = link.name;

        for (const auto& col : link.collisions) {
            if (col.geometry.type == simulation::GeometryType::MESH) {
                // 1. 生のメッシュを保持
                const std::string resolved_mesh = robot_sim::common::resolvePath(col.geometry.mesh_filename);
                auto mesh = simulation::StlLoader::loadBinaryStl(resolved_mesh);
                if (!mesh.vertices.empty()) {
                    res.meshes.push_back(mesh);
                    res.mesh_origins.push_back(col.origin);

                    // 2. 下位互換性および高速化用のカプセル近似も生成
                    collision::Capsule cap = fitCapsule(mesh);
                    res.capsules.push_back(cap);
                    res.capsule_origins.push_back(col.origin);
                    std::cout << "[Recognition] Loaded mesh link [" << link.name << "] and generated proxy capsule." << std::endl;
                }
            } 
            else if (col.geometry.type == simulation::GeometryType::CYLINDER) {
                collision::Capsule cap;
                double radius = col.geometry.size[0];
                double length = col.geometry.size[1];
                cap.a = Eigen::Vector3d(0, 0, -length * 0.5);
                cap.b = Eigen::Vector3d(0, 0, length * 0.5);
                cap.radius = radius;
                res.capsules.push_back(cap);
                res.capsule_origins.push_back(col.origin);
            }
            else if (col.geometry.type == simulation::GeometryType::SPHERE) {
                collision::Sphere sphere;
                sphere.center = Eigen::Vector3d::Zero();
                sphere.radius = col.geometry.size[0];
                res.spheres.push_back(sphere);
                res.sphere_origins.push_back(col.origin);
            }
        }
        return res;
    }

private:
    /**
     * @brief 点群（メッシュ頂点）にフィットする最小カプセルをPCAで計算
     */
    static collision::Capsule fitCapsule(const simulation::MeshData& mesh) {
        collision::Capsule cap;
        size_t n = mesh.vertices.size() / 3;
        if (n < 2) return cap;

        // 1. 重心の計算
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < n; ++i) {
            centroid += Eigen::Vector3d(mesh.vertices[i*3], mesh.vertices[i*3+1], mesh.vertices[i*3+2]);
        }
        centroid /= static_cast<double>(n);

        // 2. 共分散行列の計算
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < n; ++i) {
            Eigen::Vector3d p(mesh.vertices[i*3], mesh.vertices[i*3+1], mesh.vertices[i*3+2]);
            Eigen::Vector3d diff = p - centroid;
            cov += diff * diff.transpose();
        }

        // 3. 固有値分解で主軸を求める
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        Eigen::Vector3d axis = solver.eigenvectors().col(2).normalized(); // 最大固有値の軸

        // 4. 軸への投影で端点を決定
        double min_t = 0, max_t = 0;
        for (size_t i = 0; i < n; ++i) {
            Eigen::Vector3d p(mesh.vertices[i*3], mesh.vertices[i*3+1], mesh.vertices[i*3+2]);
            double t = (p - centroid).dot(axis);
            min_t = std::min(min_t, t);
            max_t = std::max(max_t, t);
        }

        // 5. 半径の計算（軸からの最大距離）
        double max_r_sq = 0;
        for (size_t i = 0; i < n; ++i) {
            Eigen::Vector3d p(mesh.vertices[i*3], mesh.vertices[i*3+1], mesh.vertices[i*3+2]);
            Eigen::Vector3d proj = centroid + ((p - centroid).dot(axis)) * axis;
            double r_sq = (p - proj).squaredNorm();
            max_r_sq = std::max(max_r_sq, r_sq);
        }

        cap.a = centroid + axis * min_t;
        cap.b = centroid + axis * max_t;
        cap.radius = std::sqrt(max_r_sq);

        return cap;
    }
};

} // namespace recognition
} // namespace robot_sim
