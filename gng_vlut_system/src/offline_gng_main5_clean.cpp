// 非activeノードを削除

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "collision/geometric_self_collision_checker.hpp"
#include "common/config_manager.hpp"
#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "core_safety/management/gng_status_providers.hpp"
#include "description/kinematic_adapter.hpp"
#include "description/robot_model.hpp"
#include "description/urdf_loader.hpp"

using GNG2 = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

int main() {
  std::cout << "--- Offline GNG Phase 4: Cleanup Inactive Elements ---"
            << std::endl;

  // 0. Load Configuration
  std::string config_file = "config.txt";
  if (!common::ConfigManager::Instance().Load(config_file)) {
    std::cerr << "Failed to load config file: " << config_file << std::endl;
    return -1;
  }
  auto &config = common::ConfigManager::Instance();

  // Robot Load (Needed for Constructor)
  kinematics::KinematicChain arm;
  simulation::RobotModel *model = nullptr;
  try {
    std::string urdf_path = config.Get("robot_urdf_path", "custom_robot");
    std::string full_urdf = "urdf/" + urdf_path + ".urdf";
    std::string leaf_link = config.Get("leaf_link_name", "link_7");
    auto model_obj = simulation::loadRobotFromUrdf(full_urdf);
    model = new simulation::RobotModel(model_obj);
    arm = simulation::createKinematicChainFromModel(*model, leaf_link);
  } catch (const std::exception &e) {
    return 1;
  }

  GNG2 gng(arm.getTotalDOF(), 3, &arm);
  gng.loadParameters("gng_offline.cfg");

  std::string input_file = config.GetFileName(
      "phase4_input_suffix", "_phase3"); // Default input from phase3
  if (!gng.load(input_file)) {
    std::cerr << "Failed to load " << input_file << std::endl;
    delete model;
    return 1;
  }

  // Refresh coordinates to match current Kinematic Model (EEF Tip)
  gng.refresh_coord_weights();
  std::cout << "[GNG] Refreshed node coordinates to EEF Tip." << std::endl;

  std::cout << "Pruning inactive nodes and edges..." << std::endl;

  gng.removeInactiveElements();

  std::cout << "Calculating node features (Manipulability, etc.)..."
            << std::endl;
  // Register Providers
  auto manip_provider = std::make_shared<
      GNG::ManipulabilityProvider<Eigen::VectorXf, Eigen::Vector3f>>(&arm);
  gng.registerStatusProvider(manip_provider);
  // registerEEProvider if needed...

  // Compute
  gng.triggerBatchUpdates();

  std::string output_file =
      config.GetFileName("phase4_output_suffix", "_phase4");
  gng.save(output_file);
  std::cout << "Saved clean map to " << output_file << std::endl;

  std::cout << "Saved clean map to " << output_file << std::endl;

  // Also export result dat
  std::string exp_id = config.Get("experiment_id", "default_run");
  std::string output_dat =
      config.ConstructPath(config.Get("phase4_output_suffix", "_phase4"));

  std::string dat_file =
      output_file.substr(0, output_file.find_last_of('.')) + "_vis.dat";

  FILE *fp = fopen(dat_file.c_str(), "w");
  if (fp) {
    for (int i = 0; i < (int)gng.getMaxNodeNum(); ++i) {
      const auto &node = gng.nodeAt(i);
      if (node.id == -1)
        continue;
      // Active check not strictly needed if we pruned them, but safe

      fprintf(fp, "%d %f %f %f ", node.id, node.weight_coord.x(),
              node.weight_coord.y(), node.weight_coord.z());
      fprintf(fp, "%d ", (int)node.weight_angle.size());
      for (int j = 0; j < node.weight_angle.size(); ++j)
        fprintf(fp, "%f ", node.weight_angle(j));

      auto neighbors = gng.getNeighborsAngle(node.id);
      fprintf(fp, "%d ", (int)neighbors.size());
      for (int n : neighbors)
        fprintf(fp, "%d ", n);
      fprintf(fp, "\n");
    }
    fclose(fp);
  }
  std::cout << "Exported " << dat_file << std::endl;

  delete model;
  return 0;
}
