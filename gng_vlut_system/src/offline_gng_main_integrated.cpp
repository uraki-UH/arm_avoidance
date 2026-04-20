#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

#include "collision/composite_collision_checker.hpp"
#include "collision/environment_collision_checker.hpp"
#include "collision/geometric_self_collision_checker.hpp"
#include "common/config_manager.hpp"
#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "description/kinematic_adapter.hpp"
#include "description/robot_model.hpp"
#include "description/urdf_loader.hpp"
#include "collision/collision_exclusion_setup.hpp"

using GNG2 = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

int main(int argc, char **argv) {
  // 0. Load Configuration
  std::string config_file = (argc > 1) ? argv[1] : "config.txt";
  auto &config = common::ConfigManager::Instance();
  if (!config.Load(config_file)) {
    std::cerr << "Failed to load config file: " << config_file << std::endl;
    return -1;
  }

  // 1. Robot Setup
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

  // 2. Setup Collision Checkers
  auto self_checker =
      std::make_shared<simulation::GeometricSelfCollisionChecker>(*model, arm);
  simulation::setupDefaultCollisionExclusions(*self_checker);

  auto env_checker =
      std::make_shared<simulation::EnvironmentCollisionChecker>();

  // 床 (Ground) の追加
  double ground_z = config.GetDouble("ground_z_threshold", 0.0);
  // EnvironmentCollisionChecker は土台 (base_link/stand_linkなど)
  // を自動的に判定対象外にするため、正確な高さで配置可能。
  env_checker->addBoxObstacle("ground",
                              Eigen::Vector3d(0, 0, ground_z - 0.05), // 中心
                              Eigen::Matrix3d::Identity(),
                              Eigen::Vector3d(5.0, 5.0, 0.05)); // half-extents

  // 必要に応じて他の障害物を追加可能
  // env_checker->addBoxObstacle("table", ...);

  auto composite_checker =
      std::make_shared<simulation::CompositeCollisionChecker>();
  composite_checker->setSelfCollisionChecker(self_checker);
  composite_checker->setEnvironmentCollisionChecker(env_checker);

  // 3. GNG Initialization
  int gng_dim = config.GetInt("gng_dimension", arm.getTotalDOF());
  std::cout << "GNG Training Dimension: " << gng_dim << " (Total Robot DOF: " << arm.getTotalDOF() << ")" << std::endl;
  GNG2 gng(gng_dim, 3, &arm);
  gng.loadParameters("gng_offline.cfg");
  gng.setSelfCollisionChecker(composite_checker.get());

  // Set stats log path
  std::string exp_id = config.Get("experiment_id", "default_run");
  std::string data_dir = "gng_results/";
  if (!data_dir.empty() && data_dir.back() != '/')
    data_dir += "/";
  gng.setStatsLogPath(data_dir + exp_id + "_distance_stats.dat");

  // 4. Training
  std::cout << "Stage 1: Initial Exploration (Free Space)..." << std::endl;
  gng.setCollisionAware(false);
  gng.gngTrainOnTheFly(gng.getParams().max_iterations);

  std::cout << "Intermediate Cleanup..." << std::endl;
  gng.strictFilter();

  std::cout << "Stage 2: Refinement (Self + Environment Collision Aware)..."
            << std::endl;
  gng.setCollisionAware(true);
  gng.gngTrainOnTheFly(gng.getParams().refine_iterations);

  std::cout << "Stage 3: Final Verification..." << std::endl;
  gng.strictFilter();
  gng.refresh_coord_weights();

  std::cout << "Stage 4: Coordinate Space Edge Construction (On-the-fly)..." << std::endl;
  gng.trainCoordEdgesOnTheFly(gng.getParams().coord_edge_iterations);

  // 6. Save
  std::string suffix = config.Get("phase2_output_suffix", "_phase2");
  std::string result_file_dat = data_dir + exp_id + suffix + ".dat";
  std::string result_file_bin = data_dir + exp_id + suffix + ".bin";

  std::cout << "Saving results to " << result_file_dat << " and "
            << result_file_bin << "..." << std::endl;

  // --- 1. Text Data (.dat) for Visualization/Diagnosis ---
  FILE *fp = fopen(result_file_dat.c_str(), "w");
  if (fp) {
    for (int i = 0; i < (int)gng.getMaxNodeNum(); ++i) {
      const auto &node = gng.nodeAt(i);
      if (node.id == -1)
        continue;

      // Coordinate weights (3D)
      fprintf(fp, "%d %f %f %f ", node.id, node.weight_coord.x(),
              node.weight_coord.y(), node.weight_coord.z());

      // Joint weights (DOF)
      fprintf(fp, "%d ", (int)node.weight_angle.size());
      for (int j = 0; j < node.weight_angle.size(); ++j)
        fprintf(fp, "%f ", node.weight_angle(j));

      // Neighbors (Topology)
      const auto &neighbors = gng.getNeighborsAngle(node.id);
      fprintf(fp, "%d ", (int)neighbors.size());
      for (int n : neighbors)
        fprintf(fp, "%d ", n);
      fprintf(fp, "\n");
    }
    fclose(fp);
    std::cout << ".dat file saved." << std::endl;
  } else {
    std::cerr << "Failed to open " << result_file_dat << " for writing."
              << std::endl;
  }

  // --- 2. Binary Data (.bin) for Phased Loading ---
  if (gng.save(result_file_bin)) {
    std::cout << ".bin file saved successfully." << std::endl;
  } else {
    std::cerr << "Failed to save .bin file." << std::endl;
  }

  std::cout << "GNG Integrated Offline Training complete." << std::endl;

  delete model;
  return 0;
}
