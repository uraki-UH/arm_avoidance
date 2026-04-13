#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <ode/ode.h>

#include "common/config_manager.hpp"
#include "common/geometry_utils.hpp"
#include "gng/GrowingNeuralGas_offline2.hpp"
#include "simulation/robot/kinematic_adapter.hpp"
#include "simulation/robot/robot_collision_model.hpp"
#include "simulation/robot/urdf_loader.hpp"
#include "simulation/robot/ode/ode_robot_builder.hpp"
#include "simulation/robot/ode/ode_collision_manager.hpp"
#include "simulation/robot/ode/ode_robot_sim.hpp"
#include "simulation/robot/geometry_management.hpp"
#include "spatial/index_voxel_grid.hpp"

// Qualify names instead of global 'using namespace'
using namespace GNG;

// --- Data Structures ---

struct SpatialRelation {
  int node_id;
  int link_id;
  float distance;
};

// --- Helper Functions ---

// Reposition a body and its geoms based on KinematicChain link poses
void updateComponentPoses(const std::string& name, 
                         ::simulation::OdeRobotComponent& comp,
                         const Eigen::Isometry3d& link_tf,
                         const Eigen::Vector3d& com_offset) {
    if (!comp.body_id) return;

    // The body pose is the link pose shifted by the COM offset (in link frame)
    Eigen::Vector3d body_pos = link_tf * com_offset;
    
    dBodySetPosition(comp.body_id, body_pos.x(), body_pos.y(), body_pos.z());
    Eigen::Quaterniond q(link_tf.rotation());
    dQuaternion dq = { (dReal)q.w(), (dReal)q.x(), (dReal)q.y(), (dReal)q.z() };
    dBodySetQuaternion(comp.body_id, dq);
}

// --- Main ---

int main(int argc, char **argv) {
  dInitODE2(0);
  ::common::ConfigManager::Instance().Load("config.txt");
  auto &config = ::common::ConfigManager::Instance();

  std::string gng_file;
  if (argc > 1) {
    gng_file = argv[1];
  } else {
    std::string data_dir = config.Get("data_directory", "gng_results");
    std::string exp_id = config.Get("experiment_id", "default_experiment");
    std::string phase2_suffix = config.Get("online_input_suffix", "_final");
    gng_file = data_dir + "/" + exp_id + phase2_suffix + ".bin";
    std::cout << "[Headless Analyzer] No input file specified. Using default: "
              << gng_file << std::endl;
  }

  std::string output_file = "gng_spatial_correlation.bin";
  double spatial_map_resolution = config.GetDouble("spatial_map_resolution", 0.02);
  double danger_threshold = config.GetDouble("danger_proximity_threshold", 0.005);

  std::string urdf_path = (argc > 2) ? argv[2] : config.Get("robot_urdf_path", "simple_3dof_arm");
  std::string full_urdf = "urdf/" + urdf_path + ".urdf";

  // 1. Setup ODE headless world
  dWorldID world = dWorldCreate();
  dSpaceID space = dHashSpaceCreate(0);
  
  // 2. Load Robot
  ::simulation::RobotModel* robot_model = nullptr;
  try {
    auto model_obj = ::simulation::loadRobotFromUrdf(full_urdf);
    robot_model = new ::simulation::RobotModel(model_obj);
  } catch (...) {
    std::cerr << "[Error] Failed to load robot URDF: " << full_urdf << std::endl;
    return 1;
  }

  std::string leaf_link = config.Get("leaf_link_name", "link_3");
  ::kinematics::KinematicChain chain = ::simulation::createKinematicChainFromModel(*robot_model, leaf_link);

  // 3. Build ODE Robot
  ::simulation::MeshCache mesh_cache;
  ::simulation::CollisionManager collision_manager(world, nullptr);
  ::simulation::OdeRobotBuilder builder(world, space, &collision_manager, &mesh_cache);
  
  auto components = builder.build(*robot_model, 0x1, 0x0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), true);
  
  // Store COM offsets
  std::map<std::string, Eigen::Vector3d> com_offsets;
  for (const auto& pair : robot_model->getLinks()) {
      com_offsets[pair.first] = pair.second.inertial.origin.translation();
  }

  // 4. Load GNG
  auto gng = std::make_unique<GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f>>(
      chain.getTotalDOF(), 3, &chain);
  if (!gng->load(gng_file)) {
    std::cerr << "[Error] Failed to load GNG: " << gng_file << std::endl;
    return 1;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  // --- Phase: Accurate Voxel Conflict Detection ---
  std::cout << "[Step 1/1] Computing Accurate Voxel Proximity using ODE..." << std::endl;

  int max_nodes = (int)gng->getMaxNodeNum();
  std::unordered_map<long, std::vector<SpatialRelation>> voxel_relations;
  
  // Temporary Voxel Geom (Box)
  dGeomID voxel_geom = dCreateBox(space, 
                     spatial_map_resolution + 2.0 * danger_threshold, 
                     spatial_map_resolution + 2.0 * danger_threshold, 
                     spatial_map_resolution + 2.0 * danger_threshold);

  int dof = chain.getTotalDOF();
  
  for (int i = 0; i < max_nodes; ++i) {
    const auto &node = gng->nodeAt(i);
    if (node.id == -1 || !node.status.active) continue;

    // A. Update Robot Posture
    Eigen::VectorXd q_slice = node.weight_angle.cast<double>().head(std::min((int)node.weight_angle.size(), dof));
    chain.updateKinematics(q_slice);
    
    auto positions = chain.getLinkPositions();
    auto orientations = chain.getLinkOrientations();

    std::map<std::string, Eigen::Isometry3d> link_tfs_map;
    chain.buildAllLinkTransforms(positions, orientations, robot_model->getFixedLinkInfo(), link_tfs_map);

    // B. Update ODE Bodies
    for (auto& pair : components) {
       const std::string& name = pair.first;
       ::simulation::OdeRobotComponent& comp = pair.second;
       auto it = link_tfs_map.find(name);
       if (it != link_tfs_map.end()) {
           updateComponentPoses(name, comp, it->second, com_offsets[name]);
       }
    }

    // C. Collision Check per Link
    int link_uid = 0;
    for (auto& pair : components) {
        ::simulation::OdeRobotComponent& comp = pair.second;
        if (comp.geom_ids.empty()) continue;
        
        dReal aabb[6];
        bool first = true;
        for (dGeomID g : comp.geom_ids) {
            dReal g_aabb[6];
            dGeomGetAABB(g, g_aabb);
            if (first) {
                for (int j=0; j<6; ++j) aabb[j] = g_aabb[j];
                first = false;
            } else {
                aabb[0] = std::min(aabb[0], g_aabb[0]); aabb[1] = std::max(aabb[1], g_aabb[1]);
                aabb[2] = std::min(aabb[2], g_aabb[2]); aabb[3] = std::max(aabb[3], g_aabb[3]);
                aabb[4] = std::min(aabb[4], g_aabb[4]); aabb[5] = std::max(aabb[5], g_aabb[5]);
            }
        }

        Eigen::Vector3i min_v = (Eigen::Vector3d(aabb[0], aabb[2], aabb[4]) / spatial_map_resolution).array().floor().cast<int>();
        Eigen::Vector3i max_v = (Eigen::Vector3d(aabb[1], aabb[3], aabb[5]) / spatial_map_resolution).array().ceil().cast<int>();

        for (int vx = min_v.x(); vx <= max_v.x(); ++vx) {
            for (int vy = min_v.y(); vy <= max_v.y(); ++vy) {
                for (int vz = min_v.z(); vz <= max_v.z(); ++vz) {
                    Eigen::Vector3i v_idx(vx, vy, vz);
                    Eigen::Vector3d v_center = (v_idx.cast<double>() + Eigen::Vector3d::Constant(0.5)) * spatial_map_resolution;
                    
                    dGeomSetPosition(voxel_geom, v_center.x(), v_center.y(), v_center.z());
                    
                    bool col = false;
                    for (dGeomID g : comp.geom_ids) {
                        dContact contact[1];
                        if (dCollide(voxel_geom, g, 1, &contact[0].geom, sizeof(dContact)) > 0) {
                            col = true;
                            break;
                        }
                    }

                    if (col) {
                        long vid = GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(v_idx);
                        auto &rels = voxel_relations[vid];
                        bool found = false;
                        for (auto &rel : rels) {
                            if (rel.node_id == node.id) {
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            rels.push_back({ (int)node.id, link_uid, 0.0f });
                        }
                    }
                }
            }
        }
        link_uid++;
    }

    if (i % 100 == 0) {
      std::cout << "  Analyzed Node " << i << "/" << max_nodes << " ("
                << voxel_relations.size() << " active voxels)\r" << std::flush;
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  std::cout << "\nAccurate Analysis Complete. Total Active Voxels: " << voxel_relations.size() << std::endl;
  std::cout << "Execution Time: " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "s" << std::endl;

  // 5. Save Results
  std::ofstream ofs(output_file, std::ios::binary);
  if (!ofs) return 1;

  size_t total_relations_out = 0;
  for (const auto &kv : voxel_relations) total_relations_out += kv.second.size();

  ofs.write((char *)&total_relations_out, sizeof(size_t));
  for (const auto &kv : voxel_relations) {
    long vid = kv.first;
    for (const auto &rel : kv.second) {
      ofs.write((char *)&vid, sizeof(long));
      ofs.write((char *)&rel.node_id, sizeof(int));
      ofs.write((char *)&rel.distance, sizeof(float));
      ofs.write((char *)&rel.link_id, sizeof(int));
    }
  }
  ofs.close();

  std::cout << "Saved " << total_relations_out << " relations successfully." << std::endl;
  
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
  
  return 0;
}
