#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "common/config_manager.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "description/robot_model.hpp"
#include "description/urdf_loader.hpp"
#include "description/kinematic_adapter.hpp"

using GNG2 = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

int main(int argc, char **argv) {
    std::cout << "--- Diagnostic Builder ---" << std::endl;
    common::ConfigManager &config = common::ConfigManager::Instance();
    config.Load("config.txt");
    std::string input_file = config.GetFileName("phase2_output_suffix", "_phase2");
    
    kinematics::KinematicChain arm;
    auto model_obj = simulation::loadRobotFromUrdf("urdf/" + config.Get("robot_urdf_path", "custom_robot") + ".urdf");
    simulation::RobotModel model(model_obj);
    arm = simulation::createKinematicChainFromModel(model, config.Get("leaf_link_name", "link_7"));

    GNG2 gng(arm.getTotalDOF(), 3, &arm);
    if (!gng.load(input_file)) return -1;

    auto active = gng.getActiveIndices();
    std::cout << "Active Nodes: " << active.size() << std::endl;
    for (int id : active) {
        auto &node = gng.nodeAt(id);
        std::cout << "Node " << id << " weight size: " << node.weight_angle.size() << std::endl;
        break; // Just test one
    }
    std::cout << "Successfully finished diagnostic." << std::endl;
    return 0;
}
