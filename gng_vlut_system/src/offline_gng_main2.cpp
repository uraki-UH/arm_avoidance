// 自己干渉判定ODE使用のGNG

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <ode/ode.h>
#include <vector>

#include "collision/ode/ode_robot_collision_model.hpp"
#include "common/config_manager.hpp"
#include "common/resource_utils.hpp"
#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "description/geometry_management.hpp"
#include "description/kinematic_adapter.hpp"
// Added
#include "description/ode/ode_collision_manager.hpp"
#include "description/urdf_loader.hpp"

using GNG2 = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

int main() {
  // 1. ODE Setup (Minimal for collision checking)
  dInitODE();
  dWorldID world = dWorldCreate();
  dSpaceID space = dHashSpaceCreate(0);
  dJointGroupID contactgroup = dJointGroupCreate(0);

  // 2. Load Robot
  // 0. Load Configuration
  common::ConfigManager &config = common::ConfigManager::Instance();
  std::string config_file = robot_sim::common::resolvePath("config.txt");
  if (!config.Load(config_file)) {
    std::cerr << "[Warning] Could not load " << config_file << "." << std::endl;
  }

  kinematics::KinematicChain arm;
  simulation::RobotModel *model = nullptr;
  try {
    std::string urdf_name = config.Get("robot_urdf_path", "custom_robot");
    std::string full_urdf = "urdf/" + urdf_name + ".urdf";
    std::string leaf_link = config.Get("leaf_link_name", "link_7");

    std::cout << "Loading Robot: " << full_urdf << " (Leaf: " << leaf_link
              << ")" << std::endl;

    auto model_obj = simulation::loadRobotFromUrdf(full_urdf);
    model = new simulation::RobotModel(model_obj);
    arm = simulation::createKinematicChainFromModel(*model, leaf_link);
    arm.setBase(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
  } catch (const std::exception &e) {
    std::cerr << "Error initializing robot arm: " << e.what() << std::endl;
    return 1;
  }

  // 3. Setup Collision Checker
  auto mesh_cache =
      std::make_unique<robot_sim::simulation::MeshCache>(); // Added
  auto collision_manager =
      std::make_unique<simulation::CollisionManager>(world, contactgroup);
  simulation::OdeRobotCollisionModel checker(*model, world, space,
                                             collision_manager.get(), arm,
                                             mesh_cache.get()); // Updated

  // 4. GNG Initialization
  int gng_dim = config.GetInt("gng_dimension", arm.getTotalDOF());
  std::cout << "GNG Training Dimension: " << gng_dim << " (Total Robot DOF: " << arm.getTotalDOF() << ")" << std::endl;
  GNG2 gng(gng_dim, 3, &arm);
  gng.loadParameters("gng_offline.cfg");
  gng.setSelfCollisionChecker(&checker);

  // 5. Training
  std::cout << "Stage 1: Initial Exploration without collision awareness..."
            << std::endl;
  gng.setCollisionAware(false);
  gng.gngTrainOnTheFly(gng.getParams().max_iterations);

  // --- Intermediate Cleanup ---
  std::cout << "Intermediate Cleanup: Cleaning up obviously bad nodes..."
            << std::endl;
  gng.strictFilter();

  // --- Stage 2: Refinement (Collision aware) ---
  std::cout << "Stage 2: Refinement WITH collision awareness..." << std::endl;
  gng.setCollisionAware(true);
  gng.gngTrainOnTheFly(gng.getParams().refine_iterations);

  // --- Stage 3: Final Verification ---
  std::cout << "Stage 3: Final strict filtering for safety guarantee..."
            << std::endl;
  gng.strictFilter();

  gng.refresh_coord_weights();

  // --- Stage 4: Coordinate Space Edge Construction ---
  std::cout << "Stage 4: Constructing Coordinate Space Edges (On-the-fly)..." << std::endl;
  gng.trainCoordEdgesOnTheFly(gng.getParams().coord_edge_iterations);

  // 6. Save Results
  std::string exp_id = config.Get("experiment_id", "default_run");
  std::string suffix = config.Get("phase2_output_suffix", "_phase2");
  std::string data_dir = config.Get("data_directory", "gng_results");
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
  dJointGroupDestroy(contactgroup);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();

  std::cout << "GNG Offline Learning (Ver. 2) complete." << std::endl;
  return 0;
}
