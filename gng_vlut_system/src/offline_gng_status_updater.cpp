#include "common/config_manager.hpp"
#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "description/kinematic_adapter.hpp"
#include "description/robot_model.hpp"
#include "description/urdf_loader.hpp"
#include "core_safety/management/gng_status_providers.hpp"
#include <iostream>
#include <memory>
#include <string>

#include "spatial/boundary_node_classifier.hpp"
#include "spatial/centroid_shift_surface_classifier.hpp"
#include "spatial/occupancy_grid_surface_classifier.hpp"
#include "spatial/surface_node_classifier.hpp"

int main(int argc, char **argv) {
  // 1. ConfigManager Initialization
  common::ConfigManager &config = common::ConfigManager::Instance();
  if (!config.Load("config.txt")) {
    std::cerr << "[Warning] Could not load config.txt. Using defaults."
              << std::endl;
  }

  // 2. Determine Input File
  std::string input_file;
  if (argc > 1) {
    input_file = argv[1];
  } else {
    // Attempt to read from config, or use a default
    // Using Get method which returns string
    std::string suffix = config.Get("updater_target_suffix", "");
    if (!suffix.empty()) {
      input_file = config.GetFileName("updater_target_suffix", "");
    } else {
      // Fallback: try to find the latest phase output or just use a sensible
      // default
      input_file = config.GetFileName("phase4_output_suffix", "_phase4");
    }
  }

  std::cout << "Target GNG Map: " << input_file << std::endl;

  // 3. Load Robot Model (KinematicChain)
  std::string urdf_path_base = config.Get("robot_urdf_path", "custom_robot");
  std::string urdf_file = "urdf/" + urdf_path_base +
                          ".urdf"; // Assuming urdf/ dir structure from main3

  // NOTE: If ConfigManager::GetFileName returns path with "urdf/" potentially,
  // be careful. offline_gng_main3.cpp:35: std::string full_urdf = "urdf/" +
  // urdf_path + ".urdf";

  kinematics::KinematicChain arm;
  simulation::RobotModel *model =
      nullptr; // keep pointer to delete later if needed, or use smart ptr

  try {
    auto model_obj = simulation::loadRobotFromUrdf(urdf_file);
    model = new simulation::RobotModel(model_obj);
    std::string leaf_link = config.Get("leaf_link_name", "link_7");
    arm = simulation::createKinematicChainFromModel(*model, leaf_link);
    arm.setBase(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
    std::cout << "Successfully loaded robot from: " << urdf_file
              << ", Leaf Link: " << leaf_link << std::endl;
    std::cout << "Successfully loaded robot from: " << urdf_file << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error loading robot: " << e.what() << std::endl;
    // Fallback or exit? Exit is safer for recalculation.
    return -1;
  }

  // 4. Initialize GNG (Offline Version 2)
  // Dimensions are minimal for loading; actual data will be overwritten on load
  GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f> gng(
      arm.getTotalDOF(), 3, &arm);

  if (!gng.load(input_file)) {
    std::cerr << "Error: Failed to load GNG file: " << input_file << std::endl;
    if (model)
      delete model;
    return -1;
  }
  std::cout << "Loaded GNG map with " << "???"
            << " nodes (Internal check needed)" << std::endl;

  // 5. Register Status Providers
  std::cout << "Registering Status Providers..." << std::endl;

  // Manipulability
  auto manip_provider = std::make_shared<
      GNG::ManipulabilityProvider<Eigen::VectorXf, Eigen::Vector3f>>(&arm);
  gng.registerStatusProvider(manip_provider);

  // EE Direction (Stub for now, but good practice)
  auto ee_provider = std::make_shared<
      GNG::EEDirectionProvider<Eigen::VectorXf, Eigen::Vector3f>>(&arm);
  gng.registerStatusProvider(ee_provider);

  // 6. Execute Batch Update (Calculate Features)
  std::cout << "Calculating node features..." << std::endl;
  gng.triggerBatchUpdates();

  // 6.5. Reset all surface flags (since classification is disabled)
  std::cout << "Resetting all surface flags to false..." << std::endl;
  for (size_t i = 0; i < gng.getMaxNodeNum(); ++i) {
    auto &node = gng.nodeAt(i);
    node.status.is_surface = false;
    node.status.is_active_surface = false;
    node.status.is_boundary = false;
  }
  std::cout << "Reset " << gng.getMaxNodeNum() << " nodes." << std::endl;

  // 7. Run Classifiers (Surface & Boundary)
  std::cout << "Running Node Classifiers..." << std::endl;

  // Surface Node Detection (Centroid Shift + Raycast Hybrid)
  // 1. Centroid Shift for topological completeness
  GNG::Analysis::CentroidShiftSurfaceClassifier<decltype(gng)> shift_classifier;
  shift_classifier.shift_threshold_factor = 0.3f; // Slightly more sensitive
  shift_classifier.classify(gng);

  // 2. Thick Shell Raycasting (Optional union if needed, but let's see shift
  // result first) GNG::Analysis::SurfaceNodeClassifier<decltype(gng)>
  // ray_classifier; ray_classifier.classify(gng);

  // Boundary Node Detection - TEMPORARILY DISABLED
  // GNG::Analysis::NeighborBasedBoundaryClassifier<decltype(gng)>
  //     boundary_classifier;
  // boundary_classifier.classify(gng);
  std::cout << "[BoundaryClassifier] SKIPPED (disabled for now)" << std::endl;

  if (gng.save(input_file)) {
    std::cout << "Successfully updated and saved GNG map to: " << input_file
              << std::endl;
  } else {
    std::cerr << "Error: Failed to save updated GNG map." << std::endl;
    if (model)
      delete model;
    return -1;
  }

  if (model)
    delete model;
  return 0;
}
