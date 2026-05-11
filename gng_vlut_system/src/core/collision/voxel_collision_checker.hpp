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

    /**
     * @brief 環境チェッカーを設定する
     */
    void setEnvironmentCollisionChecker(std::shared_ptr<EnvironmentCollisionChecker> env_checker) {
        env_checker_ = env_checker;
    }

    /**
     * @brief 床（Z平面）との衝突判定用閾値を設定する
     */
    void setGroundZThreshold(double threshold) {
        ground_z_threshold_ = threshold;
    }

    /**
     * @brief 自己干渉判定を行うかどうかを設定する
     */
    void setEnableSelfCollision(bool enable) {
        enable_self_collision_ = enable;
    }

    /**
     * @brief 自己干渉から除外するペアを追加する
     */
    void addCollisionExclusion(const std::string& link1, const std::string& link2);

private:
    const RobotModel& model_;
    const kinematics::KinematicChain& chain_;
    double voxel_size_;
    
    robot_sim::recognition::SelfRecognitionManager manager_;
    std::shared_ptr<EnvironmentCollisionChecker> env_checker_;
    double ground_z_threshold_ = -std::numeric_limits<double>::infinity();
    bool enable_self_collision_ = true;

    std::set<std::pair<std::string, std::string>> exclusion_pairs_;
    
    // 現在のリンク姿勢
    std::vector<Eigen::Isometry3d> current_tfs_;

    bool checkSelfCollision();
    bool checkEnvironmentCollision();
};

} // namespace simulation
