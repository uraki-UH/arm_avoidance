#pragma once

#include <vector>
#include <map>
#include <memory>
#include <unordered_set>
#include <algorithm>
#include <iostream>

#include "urdf_geometry_simplifier.hpp"
#include "primitive_spatial_mapper.hpp"
#include "mesh_voxelizer.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "GNG/Analysis/IndexVoxelGrid.hpp"
#include "common/geometry_utils.hpp"

namespace robot_sim {
namespace recognition {

/**
 * @brief 高性能自己認識マネージャ。
 * 「逆変換ルックアップ」方式を採用し、点群の高速フィルタリングを実現する。
 */
class SelfRecognitionManager {
public:
    SelfRecognitionManager() = default;

    struct CachedLinkVoxels {
        std::string name;
        Eigen::Vector3d local_min;
        Eigen::Vector3d local_max;
        std::unordered_set<long> local_vids; // ローカル空間でのO(1)検索用
        std::vector<Eigen::Vector3d> debug_points; // 可視化・デバッグ用
    };

    /**
     * @brief 初期化：URDFモデルを解析し、全リンクのローカルボクセルハッシュを生成
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

            CachedLinkVoxels cache;
            cache.name = slink.name;
            cache.local_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
            cache.local_max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

            auto add_point = [&](const Eigen::Vector3d& p) {
                cache.debug_points.push_back(p);
                long vid = ::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(
                    ::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), (float)voxel_size_)
                );
                cache.local_vids.insert(vid);
                cache.local_min = cache.local_min.cwiseMin(p);
                cache.local_max = cache.local_max.cwiseMax(p);
            };

            // A. メッシュ
            for (size_t i = 0; i < slink.meshes.size(); ++i) {
                auto points = MeshVoxelizer::sampleMeshPoints(slink.meshes[i], slink.mesh_origins[i], voxel_size_);
                for (const auto& p : points) add_point(p);
            }

            // B. カプセル
            for (size_t i = 0; i < slink.capsules.size(); ++i) {
                collision::Capsule local_cap = slink.capsules[i];
                Eigen::Isometry3d total_tf = slink.capsule_origins[i];
                local_cap.a = total_tf * slink.capsules[i].a;
                local_cap.b = total_tf * slink.capsules[i].b;
                auto points = PrimitiveSpatialMapper::sampleCapsulePoints(local_cap, voxel_size_);
                for (const auto& p : points) add_point(p);
            }

            // C. 球体
            for (size_t i = 0; i < slink.spheres.size(); ++i) {
                collision::Sphere local_sphere = slink.spheres[i];
                Eigen::Isometry3d total_tf = slink.sphere_origins[i];
                local_sphere.center = total_tf * slink.spheres[i].center;
                auto points = PrimitiveSpatialMapper::sampleSpherePoints(local_sphere, voxel_size_);
                for (const auto& p : points) add_point(p);
            }

            // AABBを少し太らせる（安全マージン）
            cache.local_min -= Eigen::Vector3d::Constant(voxel_size_);
            cache.local_max += Eigen::Vector3d::Constant(voxel_size_);

            link_voxel_caches_.push_back(cache);
        }

        fixed_link_info_ = model.getFixedLinkInfo();
        std::cout << "[Recognition] Initialized " << link_voxel_caches_.size() << " link caches using Inverse Lookup mode." << std::endl;
    }

    /**
     * @brief [高性能判定] 点群を「自己認識」と「それ以外」に高速に振り分ける
     */
    void filterPointCloud(const std::vector<Eigen::Vector3d>& input_points,
                          const std::vector<double>& joints,
                          std::vector<Eigen::Vector3d>& output_remaining,
                          std::vector<Eigen::Vector3d>& output_self) {
        if (!chain_) {
            output_remaining = input_points;
            return;
        }

        // 1. 各リンクの現在の世界座標AABBと逆行列を事前計算
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> j_pos;
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> j_ori;
        chain_->forwardKinematicsAt(joints, j_pos, j_ori);

        std::map<std::string, Eigen::Isometry3d> link_tfs;
        chain_->buildAllLinkTransforms(j_pos, j_ori, fixed_link_info_, link_tfs);

        struct LinkRuntime {
            Eigen::Isometry3d world_to_local;
            Eigen::Vector3d world_min;
            Eigen::Vector3d world_max;
            const CachedLinkVoxels* cache;
        };
        std::vector<LinkRuntime> runtimes;
        for (const auto& cache : link_voxel_caches_) {
            auto it = link_tfs.find(cache.name);
            if (it == link_tfs.end()) continue;
            
            LinkRuntime rt;
            rt.cache = &cache;
            rt.world_to_local = it->second.inverse();
            
            // 世界座標AABBの算出
            Eigen::Vector3d corners[8];
            for(int i=0; i<8; ++i) {
                Eigen::Vector3d c = cache.local_min;
                if (i & 1) c.x() = cache.local_max.x();
                if (i & 2) c.y() = cache.local_max.y();
                if (i & 4) c.z() = cache.local_max.z();
                corners[i] = it->second * c;
            }
            rt.world_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
            rt.world_max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
            for(int i=0; i<8; ++i) {
                rt.world_min = rt.world_min.cwiseMin(corners[i]);
                rt.world_max = rt.world_max.cwiseMax(corners[i]);
            }
            runtimes.push_back(rt);
        }

        // 2. 点群フィルタリング
        output_remaining.clear();
        output_self.clear();
        output_remaining.reserve(input_points.size());
        
        for (const auto& wp : input_points) {
            bool is_self = false;
            for (const auto& rt : runtimes) {
                // Broad Phase: AABBチェック
                if (wp.x() < rt.world_min.x() || wp.x() > rt.world_max.x() ||
                    wp.y() < rt.world_min.y() || wp.y() > rt.world_max.y() ||
                    wp.z() < rt.world_min.z() || wp.z() > rt.world_max.z()) {
                    continue;
                }

                // Narrow Phase: 逆変換ルックアップ
                Eigen::Vector3d lp = rt.world_to_local * wp;
                long vid = ::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(
                    ::common::geometry::VoxelUtils::worldToVoxel(lp.cast<float>(), (float)voxel_size_)
                );

                if (rt.cache->local_vids.count(vid)) {
                    is_self = true;
                    break;
                }
            }

            if (is_self) {
                output_self.push_back(wp);
            } else {
                output_remaining.push_back(wp);
            }
        }
    }

    /**
     * @brief 可視化用のマスク取得
     */
    std::vector<long> getSelfVoxelMask(const std::vector<double>& joints) {
        if (!chain_) return {};
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> j_pos;
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> j_ori;
        chain_->forwardKinematicsAt(joints, j_pos, j_ori);
        std::map<std::string, Eigen::Isometry3d> link_tfs;
        chain_->buildAllLinkTransforms(j_pos, j_ori, fixed_link_info_, link_tfs);

        std::vector<long> all_vids;
        for (const auto& cache : link_voxel_caches_) {
            auto it = link_tfs.find(cache.name);
            if (it == link_tfs.end()) continue;
            const Eigen::Isometry3d& link_tf = it->second;
            for (const auto& lp : cache.debug_points) {
                Eigen::Vector3d wp = link_tf * lp;
                all_vids.push_back(::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(
                    ::common::geometry::VoxelUtils::worldToVoxel(wp.cast<float>(), (float)voxel_size_)
                ));
            }
        }
        std::sort(all_vids.begin(), all_vids.end());
        all_vids.erase(std::unique(all_vids.begin(), all_vids.end()), all_vids.end());
        return all_vids;
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
