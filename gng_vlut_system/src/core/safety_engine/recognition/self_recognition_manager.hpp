#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <unordered_set>
#include <algorithm>

#include "robot_model/robot_model.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_voxelizer.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "core/common/voxelizer_engine.hpp"
#include "common/voxel_utils.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"


namespace robot_sim {
namespace recognition {

/**
 * @brief 自己認識・ロボット干渉除去の実行管理クラス
 */
class SelfRecognitionManager {
public:
    void initialize(std::shared_ptr<::kinematics::KinematicChain> chain,
                    std::shared_ptr<::simulation::RobotModel> model,
                    const std::vector<::simulation::LinkVoxelData>& voxel_data,
                    double voxel_size) {
        chain_ = chain;
        model_ = model;
        link_data_list_ = voxel_data;
        voxel_size_ = voxel_size;
        grid_.setVoxelSize(voxel_size);
        initial_link_tfs_ = getCurrentLinkTransforms();
        initial_base_voxel_centers_.clear();
        initial_base_voxel_centers_.reserve(link_data_list_.size());
        for (const auto& data : link_data_list_) {
            const auto tf_it = initial_link_tfs_.find(data.name);
            if (tf_it == initial_link_tfs_.end()) {
                initial_base_voxel_centers_.push_back({});
                continue;
            }
            initial_base_voxel_centers_.push_back(
                buildInitialBaseVoxelCenters(data, tf_it->second));
        }
        last_vids_.clear();
        need_mask_update_ = true;
    }

    void updateRobotState(const std::vector<double>& joints) {
        if (!chain_) return;

        // 関節角度の変化をチェックして不要な計算を避ける
        bool changed = false;
        if (last_joints_.size() != joints.size()) {
            changed = true;
        } else {
            for (size_t i = 0; i < joints.size(); ++i) {
                if (std::abs(last_joints_[i] - joints[i]) > 1e-4) {
                    changed = true;
                    break;
                }
            }
        }

        if (changed) {
            chain_->updateKinematics(joints);
            last_joints_ = joints;
            need_mask_update_ = true;
        }
    }

    std::map<std::string, Eigen::Isometry3d> getCurrentLinkTransforms() {
        std::map<std::string, Eigen::Isometry3d> link_tfs;
        if (!chain_ || !model_) return link_tfs;

        // Viewerと同じURDF木構造を唯一のリンク姿勢の基準として使用
        const auto joint_values = chain_->getJointValues();
        std::map<std::string, double> joint_value_hints;
        std::size_t value_idx = 0;
        for (int joint_idx = 0; joint_idx < chain_->getNumJoints(); ++joint_idx) {
            const int dof = chain_->getJointDOF(joint_idx);
            const std::string joint_name = chain_->getJointName(joint_idx);
            if (dof == 1 && value_idx < joint_values.size() && !joint_name.empty()) {
                joint_value_hints[joint_name] = joint_values[value_idx];
            }
            value_idx += static_cast<std::size_t>(std::max(dof, 0));
        }

        // ルートから全関節を展開し、複数アームの部分チェイン由来の
        // 上書きや固定リンクの欠落を防止
        link_tfs[model_->getRootLinkName()] = Eigen::Isometry3d::Identity();
        ::simulation::completeMissingBranchLinkTransforms(
            *model_, joint_value_hints, link_tfs);
        return link_tfs;
    }

    bool isTfChanged(const Eigen::Isometry3d& target_to_base) {
        return !target_to_base.matrix().isApprox(last_target_to_base_.matrix(), 1e-4);
    }

    std::vector<long> getSelfVoxelMask(const Eigen::Isometry3d& target_to_base = Eigen::Isometry3d::Identity()) {
        // target_to_base の変化をチェック
        if (isTfChanged(target_to_base)) {
            last_target_to_base_ = target_to_base;
            need_mask_update_ = true;
        }

        if (!need_mask_update_ && !last_vids_.empty()) {
            return last_vids_;
        }

        auto tfs = getCurrentLinkTransforms();
        if (tfs.empty()) return {};

        // 1. 準備
        size_t total_points = 0;
        struct LinkJob {
            const std::vector<Eigen::Vector3d>* centers;
            Eigen::Isometry3d tf;
        };
        std::vector<LinkJob> jobs;
        jobs.reserve(link_data_list_.size());

        for (std::size_t link_idx = 0; link_idx < link_data_list_.size(); ++link_idx) {
            const auto& data = link_data_list_[link_idx];
            const auto current_tf_it = tfs.find(data.name);
            const auto initial_tf_it = initial_link_tfs_.find(data.name);
            if (current_tf_it != tfs.end() &&
                initial_tf_it != initial_link_tfs_.end() &&
                link_idx < initial_base_voxel_centers_.size()) {
                // target_to_base * current_base_to_link * initial_base_to_link^-1
                jobs.push_back({&initial_base_voxel_centers_[link_idx],
                                target_to_base * current_tf_it->second *
                                    initial_tf_it->second.inverse()});
                total_points += initial_base_voxel_centers_[link_idx].size();
            }
        }

        if (total_points == 0) return {};

        // 2. 座標変換（シングルスレッド）
        auto t1 = std::chrono::high_resolution_clock::now();
        std::vector<long> all_vids;
        all_vids.reserve(total_points * 2);

        for (const auto& job : jobs) {
            appendTransformedVoxelCenters(*job.centers, job.tf, all_vids);
        }
        auto t2 = std::chrono::high_resolution_clock::now();

        size_t before_unique = all_vids.size();
        
        // 3. Radix Sort (O(N)) による高速重複削除
        if (all_vids.size() > 1) {
            ::common::geometry::VoxelUtils::radixSort(all_vids);
            all_vids.erase(std::unique(all_vids.begin(), all_vids.end()), all_vids.end());
        }
        auto t_end = std::chrono::high_resolution_clock::now();

        last_total_points_pre_unique_ = before_unique;
        last_calc_time_ms_ = std::chrono::duration<double, std::milli>(t_end - t1).count();
        
        last_vids_ = all_vids;
        need_mask_update_ = false;
        return all_vids;
    }

    size_t getLastPreUniqueCount() const { return last_total_points_pre_unique_; }
    double getLastCalcTimeMs() const { return last_calc_time_ms_; }

    void filterPointCloud(const std::vector<Eigen::Vector3d>& all_points, const std::vector<double>& joints, 
                          std::vector<Eigen::Vector3d>& filtered, std::vector<Eigen::Vector3d>& self) {
        updateRobotState(joints);
        auto vids = getSelfVoxelMask();
        std::unordered_set<long> vset(vids.begin(), vids.end());
        filtered.clear(); self.clear();
        for (const auto& p : all_points) {
            long vid = grid_.getFlatVoxelId(
                ::common::geometry::VoxelUtils::worldToVoxel(p.template cast<float>(), (float)voxel_size_));
            if (vset.count(vid)) self.push_back(p); else filtered.push_back(p);
        }
    }


    const std::vector<::simulation::LinkVoxelData>& getLinkVoxelDataList() const { return link_data_list_; }
    std::shared_ptr<::kinematics::KinematicChain> getKinematicChain() { return chain_; }
    ::GNG::Analysis::IndexVoxelGrid* getIndexGrid() { return &grid_; }
    const ::GNG::Analysis::IndexVoxelGrid* getIndexGrid() const { return &grid_; }
    double getVoxelSize() const { return voxel_size_; }

    /**
     * @brief リンクごとのボクセルIDリストを取得する
     */
    std::vector<std::vector<long>> getLinkVoxelMasks(const Eigen::Isometry3d& target_to_base = Eigen::Isometry3d::Identity()) {
        auto tfs = getCurrentLinkTransforms();
        std::vector<std::vector<long>> result(link_data_list_.size());
        
        for (size_t i = 0; i < link_data_list_.size(); ++i) {
            const auto& data = link_data_list_[i];
            const auto current_tf_it = tfs.find(data.name);
            const auto initial_tf_it = initial_link_tfs_.find(data.name);
            if (current_tf_it != tfs.end() &&
                initial_tf_it != initial_link_tfs_.end() &&
                i < initial_base_voxel_centers_.size()) {
                const Eigen::Isometry3d initial_base_to_target =
                    target_to_base * current_tf_it->second *
                    initial_tf_it->second.inverse();
                result[i].reserve(initial_base_voxel_centers_[i].size() * 2);
                appendTransformedVoxelCenters(
                    initial_base_voxel_centers_[i], initial_base_to_target,
                    result[i]);
                if (result[i].size() > 1) {
                    ::common::geometry::VoxelUtils::radixSort(result[i]);
                    result[i].erase(std::unique(result[i].begin(), result[i].end()), result[i].end());
                }
            }
        }
        return result;
    }

private:
    std::vector<Eigen::Vector3d> buildInitialBaseVoxelCenters(
        const ::simulation::LinkVoxelData& data,
        const Eigen::Isometry3d& initial_base_to_link) const {
        std::vector<long> initial_voxel_ids;
        if (!data.local_mesh_triangles.empty()) {
            appendTransformedMeshTriangles(
                data.local_mesh_triangles, initial_base_to_link,
                initial_voxel_ids);
        } else {
            appendTransformedVoxelCenters(
                data.local_voxel_centers, initial_base_to_link,
                initial_voxel_ids);
        }
        appendTransformedVoxelCenters(
            data.local_primitive_voxel_centers, initial_base_to_link,
            initial_voxel_ids);
        if (initial_voxel_ids.size() > 1) {
            ::common::geometry::VoxelUtils::radixSort(initial_voxel_ids);
            initial_voxel_ids.erase(
                std::unique(initial_voxel_ids.begin(), initial_voxel_ids.end()),
                initial_voxel_ids.end());
        }

        std::vector<Eigen::Vector3d> initial_centers;
        initial_centers.reserve(initial_voxel_ids.size());
        for (const long voxel_id : initial_voxel_ids) {
            initial_centers.push_back(
                ::common::geometry::VoxelUtils::voxelToWorld(
                    grid_.getIndexFromFlatId(voxel_id),
                    static_cast<float>(voxel_size_)).template cast<double>());
        }
        return initial_centers;
    }

    void appendTransformedVoxelCenters(
        const std::vector<Eigen::Vector3d>& centers,
        const Eigen::Isometry3d& local_to_target,
        std::vector<long>& voxel_ids) const {
        const bool is_grid_aligned =
            ::robot_sim::common::VoxelizerEngine::
                isVoxelGridAlignedTransform(local_to_target);
        for (const auto& center : centers) {
            if (!is_grid_aligned) {
                ::robot_sim::common::VoxelizerEngine::
                    appendTransformedVoxelCell(
                        center, local_to_target, grid_, voxel_ids);
                continue;
            }
            const Eigen::Vector3d target_point = local_to_target * center;
            voxel_ids.push_back(grid_.getFlatVoxelId(
                ::common::geometry::VoxelUtils::worldToVoxel(
                    target_point.template cast<float>(),
                    static_cast<float>(voxel_size_))));
        }
    }

    void appendTransformedMeshTriangles(
        const std::vector<Eigen::Vector3d>& local_triangles,
        const Eigen::Isometry3d& local_to_target,
        std::vector<long>& voxel_ids) const {
        std::vector<Eigen::Vector3d> target_triangles;
        target_triangles.reserve(local_triangles.size());
        for (const auto& vertex : local_triangles) {
            target_triangles.push_back(local_to_target * vertex);
        }

        std::unordered_set<long> mesh_voxel_ids;
        mesh_voxel_ids.reserve(local_triangles.size());
        ::robot_sim::common::VoxelizerEngine::voxelizeMeshTriangles(
            target_triangles, grid_, mesh_voxel_ids);
        voxel_ids.insert(voxel_ids.end(), mesh_voxel_ids.begin(),
                         mesh_voxel_ids.end());
    }

    size_t last_total_points_pre_unique_ = 0;
    double last_calc_time_ms_ = 0;
    Eigen::Isometry3d last_target_to_base_ = Eigen::Isometry3d::Identity();
    std::vector<double> last_joints_;
    std::vector<long> last_vids_;
    bool need_mask_update_ = true;

    std::shared_ptr<::kinematics::KinematicChain> chain_;
    std::shared_ptr<::simulation::RobotModel> model_;
    double voxel_size_;
    std::vector<::simulation::LinkVoxelData> link_data_list_;
    std::map<std::string, Eigen::Isometry3d> initial_link_tfs_;
    std::vector<std::vector<Eigen::Vector3d>> initial_base_voxel_centers_;
    ::GNG::Analysis::IndexVoxelGrid grid_{0.0};
};

} // namespace recognition
} // namespace robot_sim
