#include <iostream>

#include "common/config_manager.hpp"
#include "gng/GrowingNeuralGas_offline.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "simulation/robot/kinematic_adapter.hpp"
#include "simulation/robot/robot_model.hpp"
#include "simulation/robot/urdf_loader.hpp"

using GNG2 = GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f>;

int main(int argc, char **argv) {
  std::cout << "--- Offline GNG Phase 5: Island Pruning (Largest Component) ---"
            << std::endl;

  // 0. Load Configuration
  std::string config_file = (argc > 1) ? argv[1] : "config.txt";
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
    std::cerr << "Error initializing robot: " << e.what() << std::endl;
    return 1;
  }

  GNG2 gng(arm.getTotalDOF(), 3, &arm);
  gng.loadParameters("gng_offline.cfg");

  // Input file: Phase 3 or Phase 4 output?
  // User might have run main5_clean or not.
  // Let's allow specifying input suffix in config.
  std::string input_suffix =
      config.Get("island_pruning_input_suffix", "_phase4");
  std::string input_file =
      config.GetFileName("island_pruning_input", input_suffix);
  if (!gng.load(input_file)) {
    std::cerr << "Failed to load " << input_file << std::endl;
    delete model;
    return 1;
  }

  // Refresh coordinates to match current Kinematic Model (EEF Tip)
  gng.refresh_coord_weights();
  std::cout << "[GNG] Refreshed node coordinates to EEF Tip." << std::endl;

  std::cout << "Extracting largest connected component..." << std::endl;
  gng.pruneToLargestComponent();

  // Optionally remove deactivated elements to keep file small
  bool permanent_removal =
      config.GetBool("island_pruning_permanent_removal", true);
  if (permanent_removal) {
    gng.removeInactiveElements();
  }

  std::string output_suffix =
      config.Get("island_pruning_output_suffix", "_pruned");
  std::string output_file =
      config.GetFileName("island_pruning_output", output_suffix);

  gng.save(output_file);
  std::cout << "Saved pruned map to " << output_file << std::endl;

  delete model;
  return 0;
}
