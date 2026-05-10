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

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

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
            radixSort(all_vids);
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

    struct VisualizationMarkers {
        visualization_msgs::msg::MarkerArray voxels;
        visualization_msgs::msg::MarkerArray aabb;
    };

    VisualizationMarkers getVisualizationMarkers(const std::string& frame_id, double time_sec) {
        VisualizationMarkers msg;
        auto tfs = getCurrentLinkTransforms();
        if (tfs.empty()) return msg;

        int marker_id = 0;
        for (const auto& data : link_data_list_) {
            if (tfs.count(data.name)) {
                const auto& tf = tfs.at(data.name);
                
                visualization_msgs::msg::Marker m;
                m.header.frame_id = frame_id; m.header.stamp.sec = (int32_t)time_sec;
                m.header.stamp.nanosec = static_cast<uint32_t>((time_sec - (double)m.header.stamp.sec) * 1e9);
                m.ns = "self_voxels"; m.id = marker_id++;
                m.type = visualization_msgs::msg::Marker::CUBE_LIST;
                m.scale.x = m.scale.y = m.scale.z = voxel_size_;
                m.color.g = 1.0f; m.color.a = 0.5f; m.pose.orientation.w = 1.0;
                for (const auto& lp : data.local_voxel_centers) {
                    Eigen::Vector3d wp = tf * lp;
                    geometry_msgs::msg::Point p_msg; p_msg.x = wp.x(); p_msg.y = wp.y(); p_msg.z = wp.z();
                    m.points.push_back(p_msg);
                }
                if (!m.points.empty()) msg.voxels.markers.push_back(m);

                visualization_msgs::msg::Marker b;
                b.header = m.header; b.ns = "self_aabb"; b.id = marker_id++;
                b.type = visualization_msgs::msg::Marker::LINE_LIST;
                b.scale.x = 0.002; b.color.r = 1.0f; b.color.g = 0.5f; b.color.a = 0.8f; b.pose.orientation.w = 1.0;
                Eigen::Vector3d corners[8];
                for(int i=0; i<8; ++i) {
                    Eigen::Vector3d c = data.local_min;
                    if(i&1) c.x()=data.local_max.x(); if(i&2) c.y()=data.local_max.y(); if(i&4) c.z()=data.local_max.z();
                    corners[i] = tf * c;
                }
                int ed[12][2]={{0,1},{1,3},{3,2},{2,0},{4,5},{5,7},{7,6},{6,4},{0,4},{1,5},{2,6},{3,7}};
                for(auto& e:ed) {
                    geometry_msgs::msg::Point p1,p2;
                    p1.x=corners[e[0]].x(); p1.y=corners[e[0]].y(); p1.z=corners[e[0]].z();
                    p2.x=corners[e[1]].x(); p2.y=corners[e[1]].y(); p2.z=corners[e[1]].z();
                    b.points.push_back(p1); b.points.push_back(p2);
                }
                msg.aabb.markers.push_back(b);
            }
        }
        return msg;
    }

    VisualizationMarkers getVisualizationMarkers(const std::vector<double>& joints, const std::string& frame_id, double time_sec) {
        updateRobotState(joints);
        return getVisualizationMarkers(frame_id, time_sec);
    }

    const std::vector<::simulation::LinkVoxelData>& getLinkVoxelDataList() const { return link_data_list_; }
    std::shared_ptr<::kinematics::KinematicChain> getKinematicChain() { return chain_; }
    ::GNG::Analysis::IndexVoxelGrid* getIndexGrid() { return &grid_; }
    double getVoxelSize() const { return voxel_size_; }

    void radixSort(std::vector<long>& v) {
        if (v.size() < 2) return;
        if (radix_tmp_.size() < v.size()) radix_tmp_.resize(v.size());
        
        const int bits = 8;
        const int mask = (1 << bits) - 1;
        
        long* src = v.data();
        long* dst = radix_tmp_.data();
        
        // 64bitを8bitずつ常に8回まわす (確定的パフォーマンス)
        for (int shift = 0; shift < 64; shift += bits) {
            size_t count[256] = {0};
            
            // カウント (分岐なし)
            for (size_t i = 0; i < v.size(); ++i) {
                count[(src[i] >> shift) & mask]++;
            }
            
            // オフセット計算
            size_t pos[256];
            pos[0] = 0;
            for (int i = 1; i < 256; i++) pos[i] = pos[i-1] + count[i-1];
            
            // 転送
            for (size_t i = 0; i < v.size(); ++i) {
                dst[pos[(src[i] >> shift) & mask]++] = src[i];
            }
            
            // ポインタ入れ替え
            std::swap(src, dst);
        }
        
        // 8回(偶数回)回した後は必ず src == v.data() に戻っている
    }

private:
    std::vector<long> radix_tmp_;
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
