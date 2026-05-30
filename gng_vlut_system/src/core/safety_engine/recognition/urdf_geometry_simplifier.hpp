#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>

#include "robot_model/robot_model.hpp"
#include "robot_model/stl_loader.hpp"
#include "common/resource_utils.hpp"
#include "common/constants.hpp"

namespace robot_sim {
namespace recognition {

/**
 * @brief URDFの複雑なジオメトリを、判定用のシンプルなプリミティブ（カプセル・球）に変換するクラス。
 */
class UrdfGeometrySimplifier {
public:
    struct SimplifiedLink {
        std::string name;
        
        // メッシュデータ（高精度用）
        std::vector<simulation::MeshData> meshes;
        
        // リンク原点からのオフセット（URDFのorigin）
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
                    // URDFのメッシュスケールを適用（ミリメートルからメートルへの変換等を反映）
                    Eigen::Vector3d scale = col.geometry.size;
                    if (scale.norm() < ::robot_sim::common::Constants::GEOM_EPSILON) scale = Eigen::Vector3d(1, 1, 1); // 未設定時のフォールバック
                    
                    for (size_t i = 0; i < mesh.vertices.size() / 3; ++i) {
                        mesh.vertices[i*3+0] *= scale.x();
                        mesh.vertices[i*3+1] *= scale.y();
                        mesh.vertices[i*3+2] *= scale.z();
                    }

                    res.meshes.push_back(mesh);
                    res.mesh_origins.push_back(col.origin);
                }
            } 
        }
        return res;
    }
};

} // namespace recognition
} // namespace robot_sim
