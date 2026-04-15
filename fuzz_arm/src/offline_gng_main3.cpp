// Note:ODE 非依存　の GNG
// #include <ode/ode.h>

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <vector>

#include "collision/geometric_self_collision_checker.hpp"
#include "common/config_manager.hpp"
#include "gng/GrowingNeuralGas_offline.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "simulation/robot/kinematic_adapter.hpp"
#include "simulation/robot/robot_model.hpp"
#include "simulation/robot/urdf_loader.hpp"
#include "simulation/world/collision_exclusion_setup.hpp"

using GNG2 = GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f>;

int main(int argc, char **argv) {
  // 0. Load Configuration
  std::string config_file = (argc > 1) ? argv[1] : "config.txt";
  if (!common::ConfigManager::Instance().Load(config_file)) {
    std::cerr << "Failed to load config file: " << config_file << std::endl;
    return -1;
  }
  auto &config = common::ConfigManager::Instance();

  // 1. No ODE Setup needed

  // 2. Load Robot
  kinematics::KinematicChain arm;
  simulation::RobotModel *model = nullptr;
  try {
    std::string urdf_path = config.Get("robot_urdf_path", "custom_robot");
    std::string full_urdf = "urdf/" + urdf_path + ".urdf";
    std::string leaf_link = config.Get("leaf_link_name", "link_7");
    auto model_obj = simulation::loadRobotFromUrdf(full_urdf);
    model = new simulation::RobotModel(model_obj);
    arm = simulation::createKinematicChainFromModel(*model, leaf_link);
    arm.setBase(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
  } catch (const std::exception &e) {
    std::cerr << "Error initializing robot arm: " << e.what() << std::endl;
    return 1;
  }

  // 3. Setup Geometric Collision Checker
  // No CollisionManager or ODE world needed
  simulation::GeometricSelfCollisionChecker checker(*model, arm);
  simulation::setupDefaultCollisionExclusions(checker);

  // 5. GNG Initialization
  GNG2 gng(arm.getTotalDOF(), 3, &arm);
  gng.loadParameters("gng_offline.cfg");
  gng.setSelfCollisionChecker(&checker);

  // 4. Sample Generation (Joint space samples)
  std::cout << "Generating samples..." << std::endl;
  const size_t num_samples = gng.getParams().num_samples;
  std::vector<Eigen::VectorXf> angle_samples;
  angle_samples.reserve(num_samples);
  for (size_t i = 0; i < num_samples; ++i) {
    std::vector<double> q_vec = arm.sampleRandomJointValues();
    Eigen::VectorXf q(q_vec.size());
    for (size_t j = 0; j < q_vec.size(); ++j)
      q[j] = static_cast<float>(q_vec[j]);
    angle_samples.push_back(q);
  }

  std::cout << "Stage 1: Initial Exploration without collision awareness..."
            << std::endl;
  gng.setCollisionAware(false);
  gng.gngTrain(angle_samples, gng.getParams().max_iterations);

  // --- Intermediate Cleanup ---
  std::cout << "Intermediate Cleanup: Cleaning up obviously bad nodes..."
            << std::endl;
  gng.strictFilter();

  // --- Stage 2: Refinement (Collision aware) ---
  std::cout << "Stage 2: Refinement WITH collision awareness..." << std::endl;
  gng.setCollisionAware(true);
  gng.gngTrain(angle_samples,
               gng.getParams().refine_iterations); // Increased iterations

  // --- Stage 3: Final Verification ---
  std::cout << "Stage 3: Final strict filtering for safety guarantee..."
            << std::endl;
  gng.strictFilter();

  gng.refresh_coord_weights();

  // --- Stage 4: Coordinate Space Edge Construction ---
  std::cout << "Stage 4: Constructing Coordinate Space Edges..." << std::endl;
  gng.trainCoordEdges(angle_samples, gng.getParams().coord_edge_iterations);

  // 6. Save Results
  std::string result_dat =
      config.ConstructPath(config.Get("phase2_output_suffix", "_phase2")) +
      ".dat";
  // Remove .bin.dat if ConstructPath adds extension, but ConstructPath adds
  // .bin usually. Let's use GetFileName logic but manual for .dat
  std::string exp_id = config.Get("experiment_id", "default_run");
  std::string suffix = config.Get("phase2_output_suffix", "_phase2");
  std::string data_dir = config.Get("data_directory", ".");
  if (!data_dir.empty() && data_dir.back() != '/')
    data_dir += "/";

  std::string result_file_dat = data_dir + exp_id + suffix + ".dat";
  std::string result_file_bin = data_dir + exp_id + suffix + ".bin";

  std::cout << "Saving results to " << result_file_dat << " and "
            << result_file_bin << "..." << std::endl;
  FILE *fp = fopen(result_file_dat.c_str(), "w");
  if (fp) {
    for (int i = 0; i < (int)gng.getMaxNodeNum(); ++i) {
      const auto &node = gng.nodeAt(i);
      if (node.id == -1)
        continue;

      fprintf(fp, "%d %f %f %f ", node.id, node.weight_coord.x(),
              node.weight_coord.y(), node.weight_coord.z());
      fprintf(fp, "%d ", (int)node.weight_angle.size());
      for (int j = 0; j < node.weight_angle.size(); ++j)
        fprintf(fp, "%f ", node.weight_angle(j));

      // Get neighbors
      const auto &neighbors = gng.getNeighborsAngle(node.id);
      fprintf(fp, "%d ", (int)neighbors.size());
      for (int n : neighbors)
        fprintf(fp, "%d ", n);
      fprintf(fp, "\n");
    }
    fclose(fp);
  }

  gng.save(result_file_bin);

  // Cleanup
  delete model;

  std::cout << "GNG Offline Learning (Geometric Phase 2) complete."
            << std::endl;
  return 0;
}
