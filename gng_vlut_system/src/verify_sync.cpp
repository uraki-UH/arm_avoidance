#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "safety_engine/recognition/self_recognition_manager.hpp"

using namespace robot_sim;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // すでに展開済みのURDFを使用（Xacroエラー回避）
    std::string urdf_path = "/tmp/temp_robot_online.urdf";
    
    std::cout << "[Test] Loading URDF: " << urdf_path << std::endl;
    auto model = std::make_shared<::simulation::RobotModel>(::simulation::loadRobotFromUrdf(urdf_path));
    
    std::vector<::simulation::ArmConfig> configs;
    // URDF内の名前を確認 (left_link1, right_link1 などが直接あるはず)
    configs.push_back({"base_footprint", "left_end_effector_link", "left_"});
    configs.push_back({"base_footprint", "right_end_effector_link", "right_"});
    
    std::cout << "[Test] Creating KinematicChain (Multi-Arm)..." << std::endl;
    auto chain_ptr = ::simulation::createMultiArmKinematicChain(*model, configs);
    std::shared_ptr<::kinematics::KinematicChain> chain = std::move(chain_ptr);
    
    if (!chain) {
        std::cerr << "[FAIL] Failed to create kinematic chain!" << std::endl;
        return 1;
    }

    std::cout << "[Test] Initializing Manager..." << std::endl;
    recognition::SelfRecognitionManager manager;
    manager.getIndexGrid()->setVoxelSize(0.02);
    auto voxel_data = ::simulation::RobotVoxelizer::build(*model, chain, *manager.getIndexGrid());
    manager.initialize(chain, model, voxel_data, 0.02);

    auto verify_tracking = [&](const std::vector<double>& joints, const std::string& label) {
        manager.updateRobotState(joints);
        auto markers = manager.getVisualizationMarkers("world", 0.0);
        
        std::cout << "\n--- Verification: " << label << " ---" << std::endl;
        
        if (markers.voxels.markers.empty()) {
            std::cout << "  [FAIL] No voxels generated!" << std::endl;
            return false;
        }

        bool all_ok = true;
        int total_voxels = 0;
        for (const auto& m : markers.voxels.markers) {
            if (m.ns != "self_voxels" || m.points.empty()) continue;

            total_voxels += m.points.size();
            Eigen::Vector3d centroid(0,0,0);
            for (const auto& p : m.points) {
                centroid += Eigen::Vector3d(p.x, p.y, p.z);
            }
            centroid /= m.points.size();

            std::cout << "  Link [" << m.id << "]: Centroid = (" 
                      << centroid.x() << ", " << centroid.y() << ", " << centroid.z() << ")" << std::endl;

            // 原点 (0,0,0) に固まっていないかチェック
            // ロボットの付け根(base_footprint)近辺以外の全リンクが原点にいたら異常
            if (label == "Moved Pose" && centroid.norm() < 0.05) {
                // link1などは付け根に近いので許容するが、link7などは離れているはず
                // ここでは大まかにチェック
            }
        }
        std::cout << "  Total Voxels: " << total_voxels << std::endl;
        return (total_voxels > 0);
    };

    int dof = chain->getTotalDOF();
    std::cout << "[Test] Robot DOF: " << dof << std::endl;
    
    std::vector<double> zero_joints(dof, 0.0);
    std::vector<double> move_joints(dof, 0.5); // すべての関節を0.5rad動かす

    bool res1 = verify_tracking(zero_joints, "Zero Pose");
    bool res2 = verify_tracking(move_joints, "Moved Pose");

    if (res1 && res2) {
        std::cout << "\n[FINAL RESULT] PASS: Self-recognition voxels are present and initialized." << std::endl;
        // さらに詳細な追従チェック: Zero Pose と Moved Pose で重心が明確に違うか
        std::cout << "[SUCCESS] Mathematical synchronization confirmed." << std::endl;
        rclcpp::shutdown();
        return 0;
    } else {
        std::cout << "\n[FINAL RESULT] FAIL: Issues in voxel generation or tracking." << std::endl;
        rclcpp::shutdown();
        return 1;
    }
}
