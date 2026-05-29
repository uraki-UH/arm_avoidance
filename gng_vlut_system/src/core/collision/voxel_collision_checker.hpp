#pragma once

#include "safety_engine/vlut/iself_collision_checker.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/robot_voxelizer.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "collision/environment_collision_checker.hpp"
#include "safety_engine/recognition/self_recognition_manager.hpp"
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <string>
#include <set>
#include <map>

namespace simulation {

/**
 * @brief ロボットをボクセル集合として扱い、衝突判定を行うクラス。
 * 自己干渉および環境障害物との衝突を判定可能。
 */
class VoxelCollisionChecker : public ISelfCollisionChecker {
public:
    VoxelCollisionChecker(const RobotModel& model, 
                          const kinematics::KinematicChain& chain, 
                          double voxel_size,
                          double padding = 0.0);
    ~VoxelCollisionChecker() override = default;

    void updateBodyPoses(
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& positions,
        const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& orientations) override;

    bool checkCollision() override;


    void setEnvironmentCollisionChecker(std::shared_ptr<EnvironmentCollisionChecker> env_checker) {
        env_checker_ = env_checker;
    }


    void setGroundZThreshold(double threshold) {
        ground_z_threshold_ = threshold;
    }


    void setEnableSelfCollision(bool enable) {
        enable_self_collision_ = enable;
    }

    /**
     * @brief 動的な関節値のヒントを設定する
     * mimic 関節や非チェーンリンクの補完に使う。
     */
    void setJointValueHints(const std::map<std::string, double>& joint_values) {
        joint_value_hints_ = joint_values;
        cached_link_voxel_masks_valid_ = false;
    }

    /**
     * @brief 現在姿勢における各リンクの voxel 占有 ID を取得する
     * updateBodyPoses() で更新された current_tfs_ を使うため、
     * SelfRecognitionManager の内部 FK 状態には依存しない。
     */
    std::vector<std::vector<long>> getLinkVoxelMasks() const;

    /**
     * @brief ボクセル化済みリンク情報を取得する
     * 位置検証用に、リンク名と local voxel center を参照できるようにする。
     */
    const std::vector<simulation::LinkVoxelData>& getLinkVoxelDataList() const {
        return manager_.getLinkVoxelDataList();
    }

    /**
     * @brief 現在姿勢のリンク変換を、リンク名付きで取得する
     */
    std::vector<std::pair<std::string, Eigen::Isometry3d>> getCurrentLinkTransforms() const;

    /**
     * @brief 現在姿勢で自己衝突しているリンクペアを列挙する
     * 既に除外済みのペアは含めない。
     */
    std::vector<std::pair<std::string, std::string>> collectSelfCollisionPairs() const;


    void addCollisionExclusion(const std::string& link1, const std::string& link2);

    /**
     * @brief 地面や環境障害物との衝突判定から除外するリンク名を追加する
     * self collision の除外ペアとは独立して扱う。
     */
    void addEnvironmentIgnoreLink(const std::string& link_name);

private:
    const RobotModel& model_;
    const kinematics::KinematicChain& chain_;
    double voxel_size_;
    
    robot_sim::recognition::SelfRecognitionManager manager_;
    std::shared_ptr<EnvironmentCollisionChecker> env_checker_;
    double ground_z_threshold_ = -std::numeric_limits<double>::infinity();
    bool enable_self_collision_ = true;
    std::set<std::string> environment_ignore_links_;
    std::map<std::string, double> joint_value_hints_;

    std::set<std::pair<std::string, std::string>> exclusion_pairs_;
    
    // 現在のリンク姿勢
    std::vector<Eigen::Isometry3d> current_tfs_;
    mutable std::vector<std::vector<long>> cached_link_voxel_masks_;
    mutable bool cached_link_voxel_masks_valid_ = false;

    std::vector<std::vector<long>> computeLinkVoxelMasks() const;
    const std::vector<std::vector<long>>& getCachedLinkVoxelMasks() const;
    void augmentBranchLinkTransforms(std::map<std::string, Eigen::Isometry3d>& link_tfs) const;
    Eigen::Isometry3d computeJointMotionTransform(const simulation::JointProperties& joint,
                                                  double joint_value) const;
    double getJointValueHint(const std::string& joint_name) const;

    bool checkSelfCollision();
    bool checkEnvironmentCollision();
};

} // namespace simulation
