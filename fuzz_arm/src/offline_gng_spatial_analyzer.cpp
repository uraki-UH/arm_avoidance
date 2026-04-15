#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <set>
#include <ode/ode.h>

#include "common/config_manager.hpp"
#include "gng/GrowingNeuralGas_offline.hpp"
#include "simulation/robot/kinematic_adapter.hpp"
#include "simulation/robot/urdf_loader.hpp"
#include "simulation/robot/ode/ode_robot_builder.hpp"
#include "simulation/robot/ode/ode_robot_sim.hpp"
#include "simulation/robot/geometry_management.hpp"
#include "spatial/index_voxel_grid.hpp"

using namespace GNG;
using robot_sim::simulation::MeshCache; 

// --- Data Structures ---
struct RawRelation {
  long vid;
  int node_id;
  int link_id;

  bool operator<(const RawRelation& other) const {
    if (vid != other.vid) return vid < other.vid;
    return node_id < other.node_id;
  }
  bool operator==(const RawRelation& other) const {
    return vid == other.vid && node_id == other.node_id;
  }
};

// --- Main ---
int main(int argc, char **argv) {
  dInitODE2(0);
  dWorldID world = dWorldCreate();
  dSpaceID space = dHashSpaceCreate(0);
  ::common::ConfigManager::Instance().Load("config.txt");
  auto &config = ::common::ConfigManager::Instance();

  std::string gng_file;
  if (argc > 1) {
    gng_file = argv[1];
  } else {
    std::string data_dir = config.Get("data_directory", "gng_results");
    std::string exp_id = config.Get("experiment_id", "default_experiment");
    std::string phase2_suffix = config.Get("online_input_suffix", "_final");
    gng_file = data_dir + "/" + exp_id + "/" + exp_id + phase2_suffix + ".bin";
  }

  double spatial_map_resolution = config.GetDouble("spatial_map_resolution", 0.02);
  std::string urdf_path = (argc > 2) ? argv[2] : config.Get("robot_urdf_path", "simple_3dof_arm");
  std::string full_urdf = "urdf/" + urdf_path + ".urdf";

  // 1. Setup Robot & Chain
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

  ::simulation::MeshCache mesh_cache;
  ::simulation::CollisionManager collision_manager(world, nullptr);
  ::simulation::OdeRobotBuilder builder(world, space, &collision_manager, &mesh_cache);
  
  // 2. Pre-processing: Voxelize Each Link in its LOCAL Coordinate System
  std::cout << "[Pre-process] Voxelizing links in local coordinate system..." << std::endl;
  std::vector<std::string> link_names;
  std::vector<std::vector<Eigen::Vector3d>> link_local_voxels;
  
  for (const auto& pair : robot_model->getLinks()) {
      const std::string& name = pair.first;
      const auto& link = pair.second;
      std::vector<Eigen::Vector3d> local_centers;
      std::set<long> occupied_local_vids;

      for (const auto& coll : link.collisions) {
          if (coll.geometry.type == simulation::GeometryType::MESH) {
              auto entry = mesh_cache.getMesh(coll.geometry.mesh_filename, coll.geometry.size);
              if (entry && !entry->original_vertices.empty()) {
                  Eigen::Vector3d min_corner = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
                  Eigen::Vector3d max_corner = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
                  for (size_t k = 0; k + 2 < entry->original_vertices.size(); k += 3) {
                      Eigen::Vector3d v_raw(entry->original_vertices[k], entry->original_vertices[k+1], entry->original_vertices[k+2]);
                      min_corner = min_corner.cwiseMin(v_raw);
                      max_corner = max_corner.cwiseMax(v_raw);
                  }
                  
                  // Solid volume voxelization of the Mesh AABB
                  for (double x = min_corner.x(); x <= max_corner.x() + spatial_map_resolution; x += spatial_map_resolution) {
                      for (double y = min_corner.y(); y <= max_corner.y() + spatial_map_resolution; y += spatial_map_resolution) {
                          for (double z = min_corner.z(); z <= max_corner.z() + spatial_map_resolution; z += spatial_map_resolution) {
                              Eigen::Vector3d v_local = coll.origin * Eigen::Vector3d(std::min(x, max_corner.x()), std::min(y, max_corner.y()), std::min(z, max_corner.z()));
                              Eigen::Vector3i v_idx = (v_local / spatial_map_resolution).array().floor().cast<int>();
                              long vid = GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(v_idx);
                              if (occupied_local_vids.insert(vid).second) {
                                  local_centers.push_back((v_idx.cast<double>() + Eigen::Vector3d::Constant(0.5)) * spatial_map_resolution);
                              }
                          }
                      }
                  }
              }
          } else if (coll.geometry.type == simulation::GeometryType::BOX) {
              // Precise Box Voxelization
              Eigen::Vector3d size = coll.geometry.size;
              Eigen::Vector3d half = size * 0.5;
              for (double x = -half.x(); x <= half.x() + spatial_map_resolution; x += spatial_map_resolution) {
                  for (double y = -half.y(); y <= half.y() + spatial_map_resolution; y += spatial_map_resolution) {
                      for (double z = -half.z(); z <= half.z() + spatial_map_resolution; z += spatial_map_resolution) {
                          Eigen::Vector3d v_local = coll.origin * Eigen::Vector3d(std::min(x, half.x()), std::min(y, half.y()), std::min(z, half.z()));
                          Eigen::Vector3i v_idx = (v_local / spatial_map_resolution).array().floor().cast<int>();
                          long vid = GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(v_idx);
                          if (occupied_local_vids.insert(vid).second) {
                              local_centers.push_back((v_idx.cast<double>() + Eigen::Vector3d::Constant(0.5)) * spatial_map_resolution);
                          }
                      }
                  }
              }
          }
      }
      if (!local_centers.empty()) {
          link_names.push_back(name);
          link_local_voxels.push_back(local_centers);
          std::cout << "  Link [" << name << "]: Voxelized into " << local_centers.size() << " local cells." << std::endl;
      }
  }

  // 3. Main Loop: Analysis using Local Voxel Sets
  auto gng = std::make_unique<GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f>>(
      chain.getTotalDOF(), 3, &chain);
  if (!gng->load(gng_file)) { return 1; }

  int max_nodes = (int)gng->getMaxNodeNum();
  int dof = (int)chain.getTotalDOF();
  std::vector<RawRelation> raw_relations;
  raw_relations.reserve(max_nodes * 500); // Heuristic allocation

  auto start_time = std::chrono::high_resolution_clock::now();
  std::cout << "[Analysis] Processing " << max_nodes << " nodes using local rasterization..." << std::endl;

  for (int i = 0; i < max_nodes; ++i) {
    const auto &node = gng->nodeAt(i);
    if (node.id == -1 || !node.status.active) continue;

    Eigen::VectorXd q_slice = node.weight_angle.cast<double>().head(std::min((int)node.weight_angle.size(), dof));
    chain.updateKinematics(q_slice);
    
    auto positions = chain.getLinkPositions();
    auto orientations = chain.getLinkOrientations();
    std::map<std::string, Eigen::Isometry3d> link_tfs_map;
    chain.buildAllLinkTransforms(positions, orientations, robot_model->getFixedLinkInfo(), link_tfs_map);

    for (size_t l = 0; l < link_names.size(); ++l) {
        const std::string& link_name = link_names[l];
        if (link_tfs_map.find(link_name) == link_tfs_map.end()) continue;
        const Eigen::Isometry3d& link_tf = link_tfs_map.at(link_name);

        for (const auto& v_local : link_local_voxels[l]) {
            Eigen::Vector3d v_world = link_tf * v_local;
            Eigen::Vector3i v_idx = (v_world / spatial_map_resolution).array().floor().cast<int>();
            long vid = GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(v_idx);
            if (vid != -1) {
                raw_relations.push_back({ vid, (int)node.id, (int)l });
            }
        }
    }

    if (i % 1000 == 0) {
      std::cout << "  Analyzed Node " << i << "/" << max_nodes << " (Relations: " << raw_relations.size() << ")\r" << std::flush;
    }
  }

  // 4. Final Aggregation & Save
  std::cout << "\n[Post-process] Sorting and Unifying " << raw_relations.size() << " relations..." << std::endl;
  std::sort(raw_relations.begin(), raw_relations.end());
  raw_relations.erase(std::unique(raw_relations.begin(), raw_relations.end()), raw_relations.end());

  std::cout << "Analysis Complete. Unique Relations: " << raw_relations.size() << std::endl;
  auto end_time = std::chrono::high_resolution_clock::now();
  std::cout << "Execution Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << "ms" << std::endl;

  std::ofstream ofs("gng_spatial_correlation.bin", std::ios::binary);
  if (ofs) {
      size_t total = raw_relations.size();
      ofs.write((char *)&total, sizeof(size_t));
      for (const auto &rel : raw_relations) {
        ofs.write((char *)&rel.vid, sizeof(long));
        ofs.write((char *)&rel.node_id, sizeof(int));
        float dummy_dist = 0.0f;
        ofs.write((char *)&dummy_dist, sizeof(float));
        ofs.write((char *)&rel.link_id, sizeof(int));
      }
      ofs.close();
      std::cout << "Saved results to gng_spatial_correlation.bin" << std::endl;
  }

  dCloseODE();
  return 0;
}
