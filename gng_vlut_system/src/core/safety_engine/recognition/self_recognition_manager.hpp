#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <unordered_set>
#include <algorithm>

#include "robot_model/robot_model.hpp"
#include "robot_model/robot_voxelizer.hpp"
#include "kinematics/kinematic_chain.hpp"
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

        auto fixed_info = model_->getFixedLinkInfo();
        chain_->buildAllLinkTransforms(
            chain_->getLinkPositions(), 
            chain_->getLinkOrientations(), 
            fixed_info, 
            link_tfs
        );
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

        for (const auto& data : link_data_list_) {
            auto it = tfs.find(data.name);
            if (it != tfs.end()) {
                // target_to_base * base_to_link = target_to_link
                jobs.push_back({&data.local_voxel_centers, target_to_base * it->second});
                total_points += data.local_voxel_centers.size();
            }
        }

        if (total_points == 0) return {};

        // 2. 座標変換（シングルスレッド）
        auto t1 = std::chrono::high_resolution_clock::now();
        std::vector<long> all_vids(total_points);
        size_t offset = 0;

        for (const auto& job : jobs) {
            const auto& centers = *job.centers;
            const auto& tf = job.tf;
            size_t n = centers.size();
            for (size_t i = 0; i < n; ++i) {
                Eigen::Vector3d wp = tf * centers[i];
                all_vids[offset + i] = grid_.getFlatVoxelId(
                    ::common::geometry::VoxelUtils::worldToVoxel(wp.template cast<float>(), (float)voxel_size_));
            }
            offset += n;
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
            auto it = tfs.find(data.name);
            if (it != tfs.end()) {
                Eigen::Isometry3d tf = target_to_base * it->second;
                result[i].reserve(data.local_voxel_centers.size());
                for (const auto& lp : data.local_voxel_centers) {
                    Eigen::Vector3d wp = tf * lp;
                    result[i].push_back(grid_.getFlatVoxelId(
                        ::common::geometry::VoxelUtils::worldToVoxel(wp.template cast<float>(), (float)voxel_size_)));
                }
                if (result[i].size() > 1) {
                    ::common::geometry::VoxelUtils::radixSort(result[i]);
                    result[i].erase(std::unique(result[i].begin(), result[i].end()), result[i].end());
                }
            }
        }
        return result;
    }

private:
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
    ::GNG::Analysis::IndexVoxelGrid grid_{0.0};
};

} // namespace recognition
} // namespace robot_sim
