#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "collision/collision_detector.hpp"
#include "collision/geometric_self_collision_checker.hpp"
#include "common/config_manager.hpp"
#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "description/kinematic_adapter.hpp"
#include "description/robot_model.hpp"
#include "description/urdf_loader.hpp"
#include "collision/collision_exclusion_setup.hpp"

using GNG2 = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

int main() {
  std::cout << "--- Offline GNG Phase 3: Environment Collision Check ---"
            << std::endl;

  // 0. Load Configuration
  std::string config_file = "config.txt";
  if (!common::ConfigManager::Instance().Load(config_file)) {
    std::cerr << "Failed to load config file: " << config_file << std::endl;
    return -1;
  }
  auto &config = common::ConfigManager::Instance();

  // 1. Robot Setup
  kinematics::KinematicChain arm;
  simulation::RobotModel *model = nullptr;
  try {
    std::string urdf_path = config.Get("robot_urdf_path", "custom_robot");
    std::string full_urdf =
        "urdf/" + urdf_path +
        ".urdf"; // Assuming config gives name w/o extension or path?
    // Wait, prev code was "urdf/custom_robot.urdf". Config says "custom_robot".
    // I should probably conform to config.txt comment "robot_urdf_path =
    // custom_robot"
    std::string leaf_link = config.Get("leaf_link_name", "link_7");
    auto model_obj = simulation::loadRobotFromUrdf(full_urdf);
    model = new simulation::RobotModel(model_obj);
    arm = simulation::createKinematicChainFromModel(*model, leaf_link);
    arm.setBase(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
  } catch (const std::exception &e) {
    std::cerr << "Error initializing robot: " << e.what() << std::endl;
    return 1;
  }

  simulation::GeometricSelfCollisionChecker checker(*model, arm);

  // 3. Load GNG Map (V3)
  GNG2 gng(arm.getTotalDOF(), 3, &arm);
  gng.loadParameters("gng_offline.cfg");
  gng.setSelfCollisionChecker(&checker);

  // デフォルトの衝突除外ペアを設定
  simulation::setupDefaultCollisionExclusions(checker);

  std::string input_file = config.GetFileName("phase3_input_suffix", "_phase2");
  if (!gng.load(input_file)) {
    std::cerr << "Failed to load " << input_file << std::endl;
    delete model;
    return 1;
  }

  // Refresh coordinates to match current Kinematic Model (EEF Tip)
  gng.refresh_coord_weights();
  std::cout << "[GNG] Refreshed node coordinates to EEF Tip." << std::endl;

  // 4. Iterate Nodes and Check Collision
  std::cout << "Checking nodes against environment..." << std::endl;
  int nodes_deactivated = 0;
  int edges_deactivated = 0;

  std::vector<double> q_vec(arm.getTotalDOF());
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      pos_buf;
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
      ori_buf;

  for (int i = 0; i < (int)gng.getMaxNodeNum(); ++i) {
    auto &node = gng.nodeAt(i);
    if (node.id == -1)
      continue;

    bool active = node.status.active; // Start with existing status
    if (!active)
      continue; // Already inactive

    // Check against environment
    arm.forwardKinematicsAt(node.weight_angle, q_vec, pos_buf, ori_buf);
    checker.updateBodyPoses(pos_buf, ori_buf);
    const auto &robot_objs = checker.getCollisionObjects();

    bool colliding = false;
    int debug_print_count = 0;
    for (size_t obj_idx = 0; obj_idx < robot_objs.size(); ++obj_idx) {
      const auto &rob_obj = robot_objs[obj_idx];

      // SKIP BASE LINK (or fixed links) from Environment Collision
      if (rob_obj.is_fixed_to_base) {
        continue;
      }

      // 衝突除外ペアをチェック（地面は "base_link" として扱う）
      std::string link_name = checker.getLinkNameForObject(obj_idx);

      if (checker.shouldSkipCollision(link_name, "base_link")) {
        continue;
      }

      // Check against Ground Plane (Z < 0)
      bool local_col = false;
      double ground_z = config.GetDouble("ground_z_threshold", 0.0);

      if (rob_obj.type == collision::SelfCollisionChecker::ShapeType::BOX) {
        // Project Box extents onto Z axis
        double z_extent_proj = 0.0;
        for (int k = 0; k < 3; ++k) {
          z_extent_proj +=
              std::abs(rob_obj.box.rotation(2, k)) * rob_obj.box.extents(k);
        }
        if ((rob_obj.box.center.z() - z_extent_proj) < ground_z)
          local_col = true;

      } else if (rob_obj.type ==
                 collision::SelfCollisionChecker::ShapeType::CAPSULE) {
        double min_z = std::min(rob_obj.capsule.a.z(), rob_obj.capsule.b.z());
        if ((min_z - rob_obj.capsule.radius) < ground_z)
          local_col = true;

      } else if (rob_obj.type ==
                 collision::SelfCollisionChecker::ShapeType::SPHERE) {
        if ((rob_obj.sphere.center.z() - rob_obj.sphere.radius) < ground_z)
          local_col = true;
      }

      if (local_col) {
        colliding = true;
        break;
      }
    }

    if (colliding) {
      gng.setNodeActive(node.id, false);
      active = false;

      // Immediately deactivate all connected edges
      auto neighbors = gng.getNeighborsAngle(node.id);
      for (int n : neighbors) {
        gng.setEdgeActive(node.id, n, false, 0);
        edges_deactivated++;
      }

      nodes_deactivated++;
    }
  }
  std::cout << "Nodes deactivated: " << nodes_deactivated << std::endl;

  // 5. Iterate Edges and Check Collision (Path)
  std::cout << "Checking edges against environment..." << std::endl;

  for (int i = 0; i < (int)gng.getMaxNodeNum(); ++i) {
    if (gng.nodeAt(i).id == -1)
      continue;
    if (!gng.nodeAt(i).status.active)
      continue; // Skip inactive source nodes

    // Iterate neighbors (Angle Space)
    auto neighbors = gng.getNeighborsAngle(i);
    for (int n : neighbors) {
      if (i < n) { // Check once per pair
        if (!gng.nodeAt(n).status.active)
          continue; // Skip inactive target nodes

        // Both nodes active, check Path Collision
        // Steps = 5
        int steps = 5;
        bool path_colliding = false;
        Eigen::VectorXf q1 = gng.nodeAt(i).weight_angle;
        Eigen::VectorXf q2 = gng.nodeAt(n).weight_angle;

        for (int s = 1; s < steps;
             ++s) { // Start 1, End Steps-1 to avoid checking nodes again
          float t = (float)s / steps;
          Eigen::VectorXf q_interp = q1 + t * (q2 - q1);

          // Check collision for q_interp
          arm.forwardKinematicsAt(q_interp, q_vec, pos_buf, ori_buf);
          checker.updateBodyPoses(pos_buf, ori_buf);
          const auto &r_objs = checker.getCollisionObjects();

          bool col = false;
          double ground_z = config.GetDouble("ground_z_threshold", 0.0);
          for (const auto &r_obj : r_objs) {
            if (r_obj.is_fixed_to_base)
              continue; // Skip Base Link

            // 衝突除外ペアをチェック
            std::string l_name = checker.getLinkNameForObject(r_obj.id);
            if (checker.shouldSkipCollision(l_name, "base_link")) {
              continue;
            }

            if (r_obj.type == collision::SelfCollisionChecker::ShapeType::BOX) {
              double z_extent_proj = 0.0;
              for (int k = 0; k < 3; ++k)
                z_extent_proj +=
                    std::abs(r_obj.box.rotation(2, k)) * r_obj.box.extents(k);
              if ((r_obj.box.center.z() - z_extent_proj) < ground_z)
                col = true;
            } else if (r_obj.type ==
                       collision::SelfCollisionChecker::ShapeType::CAPSULE) {
              double min_z = std::min(r_obj.capsule.a.z(), r_obj.capsule.b.z());
              if ((min_z - r_obj.capsule.radius) < ground_z)
                col = true;
            } else if (r_obj.type ==
                       collision::SelfCollisionChecker::ShapeType::SPHERE) {
              if ((r_obj.sphere.center.z() - r_obj.sphere.radius) < ground_z)
                col = true;
            }
            if (col)
              break;
          }
          if (col) {
            path_colliding = true;
            break;
          }
        }

        if (path_colliding) {
          gng.setEdgeActive(i, n, false, 0);
          edges_deactivated++;
        }
      }
    }
  }
  std::cout << "Edges deactivated: " << edges_deactivated << std::endl;

  // Save
  std::string output_file =
      config.GetFileName("phase3_output_suffix", "_phase3");
  gng.save(output_file);
  std::cout << "Saved " << output_file << std::endl;

  delete model;
  return 0;
}
