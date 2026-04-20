#pragma once

#include <vector>
#include <map>
#include <memory>

#include "urdf_geometry_simplifier.hpp"
#include "primitive_spatial_mapper.hpp"
#include "mesh_voxelizer.hpp"
#include "kinematics/kinematic_chain.hpp"

namespace robot_sim {
namespace recognition {

/**
 * @brief 自作の自己認識マネージャ。
 * ロボットのリンク形状（URDF）から、リアルタイムの姿勢に応じた「自己ボクセル・マスク」を生成する。
 */
class SelfRecognitionManager {
public:
    SelfRecognitionManager() = default;

    struct CachedLinkVoxels {
        std::string name;
        std::vector<Eigen::Vector3d> local_points;
    };

    /**
     * @brief 初期化：URDFモデルを解析し、全リンクの簡略化形状およびメッシュデータを生成
     */
    void initialize(const simulation::RobotModel& model, 
                    std::shared_ptr<kinematics::KinematicChain> chain,
                    double voxel_size) {
        chain_ = chain;
        voxel_size_ = voxel_size;
        
        simplified_links_.clear();
        link_voxel_caches_.clear();

        for (const auto& pair : model.getLinks()) {
            auto slink = UrdfGeometrySimplifier::simplifyLink(pair.second);
            simplified_links_.push_back(slink);

            // プリビルド：ローカルボクセルキャッシュの構築
            CachedLinkVoxels cache;
            cache.name = slink.name;

            // A. メッシュ（高精度用）
            for (size_t i = 0; i < slink.meshes.size(); ++i) {
                auto points = MeshVoxelizer::sampleMeshPoints(slink.meshes[i], slink.mesh_origins[i], voxel_size_);
                cache.local_points.insert(cache.local_points.end(), points.begin(), points.end());
            }

            // B. カプセル
            for (size_t i = 0; i < slink.capsules.size(); ++i) {
                collision::Capsule local_cap = slink.capsules[i];
                Eigen::Isometry3d total_tf = slink.capsule_origins[i];
                local_cap.a = total_tf * slink.capsules[i].a;
                local_cap.b = total_tf * slink.capsules[i].b;
                auto points = PrimitiveSpatialMapper::sampleCapsulePoints(local_cap, voxel_size_);
                cache.local_points.insert(cache.local_points.end(), points.begin(), points.end());
            }

            // C. 球体
            for (size_t i = 0; i < slink.spheres.size(); ++i) {
                collision::Sphere local_sphere = slink.spheres[i];
                Eigen::Isometry3d total_tf = slink.sphere_origins[i];
                local_sphere.center = total_tf * slink.spheres[i].center;
                auto points = PrimitiveSpatialMapper::sampleSpherePoints(local_sphere, voxel_size_);
                cache.local_points.insert(cache.local_points.end(), points.begin(), points.end());
            }

            link_voxel_caches_.push_back(cache);
        }

        fixed_link_info_ = model.getFixedLinkInfo();
        std::cout << "[Recognition] Initialized " << link_voxel_caches_.size() << " link caches." << std::endl;
    }

    /**
     * @brief 現在の関節角度から、ロボット自身の占有ボクセルIDリスト（Self-Mask）を取得
     * @param joints 関節角度群
     * @param use_precision_mesh（現在はキャッシュ方式のため、初期化時の密度に依存）
     */
    std::vector<long> getSelfVoxelMask(const std::vector<double>& joints, bool /*unused*/ = true) {
        if (!chain_) return {};

        // 1. 全リンクの現在位置をFKで計算
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> j_pos;
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> j_ori;
        chain_->forwardKinematicsAt(joints, j_pos, j_ori);

        std::map<std::string, Eigen::Isometry3d> link_tfs;
        chain_->buildAllLinkTransforms(j_pos, j_ori, fixed_link_info_, link_tfs);

        // 2. キャッシュされたローカル点群を世界座標に変換
        std::vector<long> all_vids;
        for (const auto& cache : link_voxel_caches_) {
            auto it = link_tfs.find(cache.name);
            if (it == link_tfs.end()) continue;

            const Eigen::Isometry3d& link_tf = it->second;
            for (const auto& lp : cache.local_points) {
                Eigen::Vector3d wp = link_tf * lp;
                all_vids.push_back(::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(
                    ::common::geometry::VoxelUtils::worldToVoxel(wp.cast<float>(), (float)voxel_size_)
                ));
            }
        }

        // 3. 重複削除
        std::sort(all_vids.begin(), all_vids.end());
        all_vids.erase(std::unique(all_vids.begin(), all_vids.end()), all_vids.end());
        
        return all_vids;
    }

    /**
     * @brief 手動で追加のプリミティブ（把持物体など）を追加してマスクを生成
     */
    std::vector<long> getObjectMask(const std::vector<collision::Capsule>& caps, 
                                   const std::vector<collision::Sphere>& spheres) {
        std::vector<long> vids;
        for (const auto& c : caps) {
            auto pts = PrimitiveSpatialMapper::sampleCapsulePoints(c, voxel_size_);
            for (const auto& p : pts) vids.push_back(::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), (float)voxel_size_)));
        }
        for (const auto& s : spheres) {
            auto pts = PrimitiveSpatialMapper::sampleSpherePoints(s, voxel_size_);
            for (const auto& p : pts) vids.push_back(::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), (float)voxel_size_)));
        }
        std::sort(vids.begin(), vids.end());
        vids.erase(std::unique(vids.begin(), vids.end()), vids.end());
        return vids;
    }

private:
    std::shared_ptr<kinematics::KinematicChain> chain_;
    double voxel_size_ = 0.02;
    std::vector<UrdfGeometrySimplifier::SimplifiedLink> simplified_links_;
    std::vector<CachedLinkVoxels> link_voxel_caches_;
    std::map<std::string, std::pair<std::string, Eigen::Isometry3d>> fixed_link_info_;
};

} // namespace recognition
} // namespace robot_sim
