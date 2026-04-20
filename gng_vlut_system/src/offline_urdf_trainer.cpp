#include <Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <atomic>
#include <set>

#include "collision/composite_collision_checker.hpp"
#include "collision/environment_collision_checker.hpp"
#include "collision/geometric_self_collision_checker.hpp"
#include "common/config_manager.hpp"
#include "common/resource_utils.hpp"
#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "description/kinematic_adapter.hpp"
#include "description/robot_model.hpp"
#include "description/urdf_loader.hpp"
#include "core_safety/management/gng_geometric_self_collision_provider.hpp"
#include "core_safety/management/gng_status_providers.hpp"
#include "core_safety/spatial/index_voxel_grid.hpp"

#ifdef USE_FCL
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#endif

using GNG2 = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

#ifdef USE_FCL
// --- Fast Triangle-Box Overlap Test (Akenine-Möller) ---
static bool planeBoxOverlap(const fcl::Vector3d& normal, const fcl::Vector3d& vert, const fcl::Vector3d& maxbox) {
    fcl::Vector3d vmin, vmax;
    for (int q = 0; q < 3; q++) {
        double v = vert[q];
        if (normal[q] > 0.0) { vmin[q] = -maxbox[q] - v; vmax[q] = maxbox[q] - v; }
        else { vmin[q] = maxbox[q] - v; vmax[q] = -maxbox[q] - v; }
    }
    if (normal.dot(vmin) > 0.0) return false;
    if (normal.dot(vmax) >= 0.0) return true;
    return false;
}

#define AXISTEST_X01(a, b, fa, fb) \
    p0 = a * v0.y() - b * v0.z(); p2 = a * v2.y() - b * v2.z(); \
    if (p0 < p2) { min = p0; max = p2; } else { min = p2; max = p0; } \
    rad = fa * boxhalfsize.y() + fb * boxhalfsize.z(); \
    if (min > rad || max < -rad) return false;

#define AXISTEST_X2(a, b, fa, fb) \
    p0 = a * v0.y() - b * v0.z(); p1 = a * v1.y() - b * v1.z(); \
    if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; } \
    rad = fa * boxhalfsize.y() + fb * boxhalfsize.z(); \
    if (min > rad || max < -rad) return false;

#define AXISTEST_Y02(a, b, fa, fb) \
    p0 = -a * v0.x() + b * v0.z(); p2 = -a * v2.x() + b * v2.z(); \
    if (p0 < p2) { min = p0; max = p2; } else { min = p2; max = p0; } \
    rad = fa * boxhalfsize.x() + fb * boxhalfsize.z(); \
    if (min > rad || max < -rad) return false;

#define AXISTEST_Y1(a, b, fa, fb) \
    p0 = -a * v0.x() + b * v0.z(); p1 = -a * v1.x() + b * v1.z(); \
    if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; } \
    rad = fa * boxhalfsize.x() + fb * boxhalfsize.z(); \
    if (min > rad || max < -rad) return false;

#define AXISTEST_Z12(a, b, fa, fb) \
    p1 = a * v1.x() - b * v1.y(); p2 = a * v2.x() - b * v2.y(); \
    if (p1 < p2) { min = p1; max = p2; } else { min = p2; max = p1; } \
    rad = fa * boxhalfsize.x() + fb * boxhalfsize.y(); \
    if (min > rad || max < -rad) return false;

#define AXISTEST_Z0(a, b, fa, fb) \
    p0 = a * v0.x() - b * v0.y(); p1 = a * v1.x() - b * v1.y(); \
    if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; } \
    rad = fa * boxhalfsize.x() + fb * boxhalfsize.y(); \
    if (min > rad || max < -rad) return false;

static bool triBoxOverlap(const fcl::Vector3d& boxcenter, const fcl::Vector3d& boxhalfsize, const fcl::Vector3d triverts[3]) {
    fcl::Vector3d v0 = triverts[0] - boxcenter, v1 = triverts[1] - boxcenter, v2 = triverts[2] - boxcenter;
    fcl::Vector3d e0 = v1 - v0, e1 = v2 - v1, e2 = v0 - v2;
    double min, max, p0, p1, p2, rad, fex, fey, fez;
    fex = std::abs(e0.x()); fey = std::abs(e0.y()); fez = std::abs(e0.z());
    AXISTEST_X01(e0.z(), e0.y(), fez, fey); AXISTEST_Y02(e0.z(), e0.x(), fez, fex); AXISTEST_Z12(e0.y(), e0.x(), fey, fex);
    fex = std::abs(e1.x()); fey = std::abs(e1.y()); fez = std::abs(e1.z());
    AXISTEST_X01(e1.z(), e1.y(), fez, fey); AXISTEST_Y02(e1.z(), e1.x(), fez, fex); AXISTEST_Z0(e1.y(), e1.x(), fey, fex);
    fex = std::abs(e2.x()); fey = std::abs(e2.y()); fez = std::abs(e2.z());
    AXISTEST_X2(e2.z(), e2.y(), fez, fey); AXISTEST_Y1(e2.z(), e2.x(), fez, fex); AXISTEST_Z12(e2.y(), e2.x(), fey, fex);
    fcl::Vector3d bmin = -boxhalfsize, bmax = boxhalfsize;
    if (std::max({v0.x(), v1.x(), v2.x()}) < bmin.x() || std::min({v0.x(), v1.x(), v2.x()}) > bmax.x()) return false;
    if (std::max({v0.y(), v1.y(), v2.y()}) < bmin.y() || std::min({v0.y(), v1.y(), v2.y()}) > bmax.y()) return false;
    if (std::max({v0.z(), v1.z(), v2.z()}) < bmin.z() || std::min({v0.z(), v1.z(), v2.z()}) > bmax.z()) return false;
    fcl::Vector3d normal = e0.cross(e1);
    return planeBoxOverlap(normal, v0, bmax);
}
#endif

static void initGngParameters(GNG2 &gng) {
  auto &config = common::ConfigManager::Instance();
  GNG::GngParameters p;
  gng.setParams(p);
  gng.loadParameters("gng_offline.cfg");
}

static void initStatusProviders(GNG2 &gng,
                                simulation::CompositeCollisionChecker *checker,
                                kinematics::KinematicChain *arm) {
  // Register additional status providers (Metadata for simulation)
  gng.registerStatusProvider(
      std::make_shared<
          GNG::GeometricSelfCollisionProvider<Eigen::VectorXf, Eigen::Vector3f>>(
          checker, arm));
  gng.registerStatusProvider(
      std::make_shared<GNG::ManipulabilityProvider<Eigen::VectorXf,
                                                   Eigen::Vector3f>>(arm));
  gng.registerStatusProvider(
      std::make_shared<GNG::EEDirectionProvider<Eigen::VectorXf,
                                                Eigen::Vector3f>>(arm));
}

int main(int argc, char **argv) {
  std::cout << "--- Standalone URDF-based Unified GNG/VLUT Pipeline ---" << std::endl;

  // 0. Load Configuration
  std::string config_file = "config.txt";

  // Check if first arg is a config file (.txt)
  if (argc > 1 && std::string(argv[1]).find(".txt") != std::string::npos) {
    config_file = argv[1];
  }

  auto &config = common::ConfigManager::Instance();
  if (!config.Load(config_file)) {
    std::cerr << "[Error] Failed to load config file: " << config_file << std::endl;
    return -1;
  }

  bool vlut_only = false;
  // Parse Overrides
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--urdf" && i + 1 < argc) {
      config.Set("robot_urdf_path", argv[++i]);
    } else if (arg == "--id" && i + 1 < argc) {
      config.Set("experiment_id", argv[++i]);
    } else if (arg == "--res" && i + 1 < argc) {
      config.Set("spatial_map_resolution", argv[++i]);
    } else if (arg == "--leaf" && i + 1 < argc) {
      config.Set("leaf_link_name", argv[++i]);
    } else if (arg == "--vlut-only") {
      vlut_only = true;
    }
  }

    // 1. Robot Setup
    kinematics::KinematicChain arm;
    simulation::RobotModel *model = nullptr;
    try {
        std::string urdf_arg = config.Get("robot_urdf_path", "custom_robot");
        std::string full_urdf = robot_sim::common::resolvePath(urdf_arg);

        if (!std::filesystem::exists(full_urdf)) {
             // Try common extensions if needed
             if (std::filesystem::exists(robot_sim::common::resolvePath(urdf_arg + ".urdf"))) {
                 full_urdf = robot_sim::common::resolvePath(urdf_arg + ".urdf");
             } else if (std::filesystem::exists(robot_sim::common::resolvePath(urdf_arg + ".xacro"))) {
                 full_urdf = robot_sim::common::resolvePath(urdf_arg + ".xacro");
             }
        }

        if (!std::filesystem::exists(full_urdf) || std::filesystem::is_directory(full_urdf)) {
            throw std::runtime_error("Could not find robot model for: " + urdf_arg);
        }

        // Manage temporary URDF if using xacro
        bool is_xacro = (full_urdf.find(".xacro") != std::string::npos);
        std::string temp_urdf = (std::filesystem::temp_directory_path() / "temp_robot.urdf").string();
        
        if (is_xacro) {
            std::cout << "[Robot] Xacro detected. Expanding macros..." << std::endl;
            std::string cmd = "xacro " + full_urdf + " > " + temp_urdf;
            std::cout << "[Robot] Running: " << cmd << std::endl;
            if (std::system(cmd.c_str()) != 0) {
                std::cerr << "[Error] Failed to run xacro command. Make sure xacro is installed." << std::endl;
                return -1;
            }
            full_urdf = temp_urdf;
        }

    std::string leaf_link = config.Get("leaf_link_name", "link_7");
    
    auto model_obj = simulation::loadRobotFromUrdf(full_urdf);
    model = new simulation::RobotModel(model_obj);
    arm = simulation::createKinematicChainFromModel(*model, leaf_link);
    arm.setBase(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
    std::cout << "[Robot] Loaded: " << full_urdf << ", DOF: " << arm.getTotalDOF() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "[Error] Robot fail: " << e.what() << std::endl; return 1;
  }

  // 2. Setup Checkers
  auto self_checker = std::make_shared<simulation::GeometricSelfCollisionChecker>(*model, arm);
  auto env_checker = std::make_shared<simulation::EnvironmentCollisionChecker>();
  double ground_z = config.GetDouble("ground_z_threshold", 0.0);
  env_checker->addBoxObstacle("ground", Eigen::Vector3d(0, 0, ground_z - 0.05),
                              Eigen::Matrix3d::Identity(), Eigen::Vector3d(10.0, 10.0, 0.05));
#ifdef USE_FCL
  self_checker->setStrictMode(true);
#endif

  auto composite_checker = std::make_shared<simulation::CompositeCollisionChecker>();
  composite_checker->setSelfCollisionChecker(self_checker);
  composite_checker->setEnvironmentCollisionChecker(env_checker);
  
  // 3. GNG Setup
  int gng_dim = config.GetInt("gng_dimension", arm.getTotalDOF());
  GNG2 gng(gng_dim, 3, &arm);

  initGngParameters(gng);
  gng.setSelfCollisionChecker(composite_checker.get());
  initStatusProviders(gng, composite_checker.get(), &arm);

  std::string exp_id = config.Get("experiment_id", "standalone_train");
  std::string data_dir = "gng_results/";
  gng.setStatsLogPath(data_dir + exp_id + "_distance_stats.dat");

  // 4. Training Steps
  std::string out_bin = config.GetFileName("phase2_output_suffix", "_phase2");

  if (vlut_only) {
      std::cout << "[Step 0] Skipping GNG training. Loading existing map: " << out_bin << std::endl;
      if (!gng.load(out_bin)) {
          std::cerr << "[Error] Failed to load GNG map for VLUT reconstruction." << std::endl;
          return -1;
      }
  } else {
      std::cout << "[Step 1] Initial Exploration..." << std::endl;
      gng.setCollisionAware(false); gng.gngTrainOnTheFly(gng.getParams().max_iterations);
      
      std::cout << "[Step 2] Intermediate Filter..." << std::endl;
      gng.strictFilter();
      
      std::cout << "[Step 3] Refinement (Self-Collision Aware)..." << std::endl;
      gng.setCollisionAware(true); gng.gngTrainOnTheFly(gng.getParams().refine_iterations);
      
      std::cout << "[Step 4] Final Verification..." << std::endl;
      gng.strictFilter(); gng.refresh_coord_weights();
      
      std::cout << "[Step 5] Coordinate Space Edge construction..." << std::endl;
      gng.trainCoordEdgesOnTheFly(gng.getParams().coord_edge_iterations);

      // Metadata update for finale
      gng.triggerBatchUpdates();
  }

  // 5. High-Fidelity Voxelization (VLUT Generation)
#ifdef USE_FCL
  struct VRel { long vid; int nid; int lid; };
  std::vector<VRel> v_rels;
  double res = config.GetDouble("spatial_map_resolution", 0.02);
  fcl::Vector3d box_half_size(res * 0.5, res * 0.5, res * 0.5);
  auto active_ids = gng.getActiveIndices();
  int processed = 0;

  std::cout << "[Step 6] Pre-voxelizing Robot Links..." << std::endl;
  
  // Cache for local voxel centers of each link
  struct LocalVoxelCloud { std::vector<fcl::Vector3d> centers; };
  std::map<int, LocalVoxelCloud> link_voxel_clouds;

  const auto& objects = self_checker->getCollisionObjects();
  for (size_t i = 0; i < objects.size(); ++i) {
      if (objects[i].type != collision::SelfCollisionChecker::ShapeType::MESH) {
          // Add basic support for primitives if needed, but primary focus is MESH
          // For now, we skip non-mesh links in VLUT to focus on the main body
          continue;
      }
      auto fcl_obj = self_checker->getFCLObject(i);
      if (!fcl_obj || !fcl_obj->collisionGeometry()) continue;
      auto mesh = dynamic_cast<const fcl::BVHModel<fcl::OBBRSS<double>>*>(fcl_obj->collisionGeometry().get());
      if (!mesh || !mesh->vertices || !mesh->tri_indices) continue;

      LocalVoxelCloud& cloud = link_voxel_clouds[i];
      fcl::Vector3d mesh_min, mesh_max;
      mesh_min.setConstant(std::numeric_limits<double>::infinity());
      mesh_max.setConstant(-std::numeric_limits<double>::infinity());
      for (int vidx = 0; vidx < mesh->num_vertices; ++vidx) {
          mesh_min = mesh_min.cwiseMin(mesh->vertices[vidx]);
          mesh_max = mesh_max.cwiseMax(mesh->vertices[vidx]);
      }

      Eigen::Vector3i b_min = (mesh_min / res).array().floor().cast<int>().matrix() - Eigen::Vector3i::Ones();
      Eigen::Vector3i b_max = (mesh_max / res).array().ceil().cast<int>().matrix() + Eigen::Vector3i::Ones();

      for (int vx = b_min.x(); vx <= b_max.x(); ++vx) {
          for (int vy = b_min.y(); vy <= b_max.y(); ++vy) {
              for (int vz = b_min.z(); vz <= b_max.z(); ++vz) {
                  fcl::Vector3d box_center = (Eigen::Vector3i(vx, vy, vz).cast<double>() + Eigen::Vector3d::Constant(0.5)) * res;
                  bool occupied = false;
                  for (int t = 0; t < mesh->num_tris; ++t) {
                      const fcl::Triangle& tri = mesh->tri_indices[t];
                      fcl::Vector3d v[3] = { mesh->vertices[tri[0]], mesh->vertices[tri[1]], mesh->vertices[tri[2]] };
                      if (triBoxOverlap(box_center, box_half_size, v)) {
                          occupied = true; break;
                      }
                  }
                  if (occupied) cloud.centers.push_back(box_center);
              }
          }
      }
      std::cout << "  Link [" << i << "] (" << objects[i].name << ") voxelized: " 
                << cloud.centers.size() << " voxels. Local Z-range: [" << mesh_min.z() << ", " << mesh_max.z() << "]" << std::endl;
  }

  std::cout << "[Step 6] Building Spatial Index (Voxel Cloud Transformation)..." << std::endl;
  v_rels.reserve(active_ids.size() * 500); // Pre-reserve to avoid reallocations
  std::unordered_set<long> seen; 

  // --- Track Actual Reachable Bounds ---
  fcl::Vector3d actual_min(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  fcl::Vector3d actual_max(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());

  for (int nid : active_ids) {
      auto &node = gng.nodeAt(nid);
      Eigen::VectorXd q = node.weight_angle.template cast<double>().head(std::min((int)node.weight_angle.size(), arm.getTotalDOF()));
      arm.updateKinematics(q);
      self_checker->updateBodyPoses(arm.getLinkPositions(), arm.getLinkOrientations());
      
      for (auto const& [link_idx, cloud] : link_voxel_clouds) {
          fcl::Transform3d tf = self_checker->getFCLObject(link_idx)->getTransform();
          seen.clear(); // Re-use memory

          for (const auto& local_p : cloud.centers) {
              fcl::Vector3d world_p = tf * local_p;
              
              // Update actual bounds
              actual_min = actual_min.cwiseMin(world_p);
              actual_max = actual_max.cwiseMax(world_p);

              Eigen::Vector3i idx = (world_p / res).array().floor().cast<int>();
              long vid = ::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);
              if (seen.insert(vid).second) {
                  v_rels.push_back({ vid, nid, link_idx });
              }
          }
      }
      processed++;
      if (processed % 100 == 0 || processed == (int)active_ids.size()) {
          std::cout << "  Analyzed Node " << processed << "/" << active_ids.size() << " (" << v_rels.size() << " rels)\r" << std::flush;
      }
  }
  std::cout << std::endl;

  // Calculate Relative Margin (10% of the detected range, min 2cm)
  fcl::Vector3d range = actual_max - actual_min;
  fcl::Vector3d margin = range * 0.1;
  for (int i = 0; i < 3; ++i) if (margin[i] < 0.02) margin[i] = 0.02;

  actual_min -= margin;
  actual_max += margin;
  
  Eigen::Vector3i final_dims = ((actual_max - actual_min) / res).array().ceil().cast<int>().cwiseMax(1);
  std::cout << "[Step 6] Final Voxel Grid: " << final_dims.transpose() 
            << " (" << (long)final_dims.x() * final_dims.y() * final_dims.z() << " total voxels)" << std::endl;
  std::cout << "[Step 6] Workspace Bounds: Min(" << actual_min.transpose() << "), Max(" << actual_max.transpose() << ")" << std::endl;
#endif

  // 6. Save Everything
  std::filesystem::path folder = std::filesystem::path(out_bin).parent_path();
  std::filesystem::create_directories(folder);

  // Backup config
  try { std::filesystem::copy_file(config_file, folder / "config.txt", std::filesystem::copy_options::overwrite_existing); } catch(...) {}

  // Save GNG Map
  if (gng.save(out_bin)) { std::cout << "[Success] GNG saved to: " << out_bin << std::endl; }

  // Save VLUT
#ifdef USE_FCL
  std::string vlut_file = (folder / "gng_spatial_correlation.bin").string();
  std::ofstream ofs(vlut_file, std::ios::binary);
  if (ofs) {
      // --- Add Self-Describing Header ---
      // Convert a 4-character string to a 32-bit binary identifier (e.g., "VLUT")
      auto pack4CharsToUint32 = [](const char* s) {
          return (uint32_t)(s[0] << 24 | s[1] << 16 | s[2] << 8 | s[3]);
      };
      uint32_t file_id = pack4CharsToUint32("VLUT");
      uint32_t version = 2;        // Updated version to include bounds
      float save_res = (float)res;
      float min_b[3] = { (float)actual_min.x(), (float)actual_min.y(), (float)actual_min.z() };
      float max_b[3] = { (float)actual_max.x(), (float)actual_max.y(), (float)actual_max.z() };
      
      ofs.write((char *)&file_id, sizeof(uint32_t));
      ofs.write((char *)&version, sizeof(uint32_t));
      ofs.write((char *)&save_res, sizeof(float));
      ofs.write((char *)min_b, sizeof(float) * 3);
      ofs.write((char *)max_b, sizeof(float) * 3);

      std::sort(v_rels.begin(), v_rels.end(), [](const auto &a, const auto &b){
          return (a.vid != b.vid) ? a.vid < b.vid : a.nid < b.nid;
      });
      size_t total = v_rels.size(); ofs.write((char *)&total, sizeof(size_t));
      float d0 = 0.0f;
      for (const auto &rel : v_rels) {
          ofs.write((char *)&rel.vid, sizeof(long)); ofs.write((char *)&rel.nid, sizeof(int));
          ofs.write((char *)&d0, sizeof(float)); ofs.write((char *)&rel.lid, sizeof(int));
      }
      std::cout << "[Success] VLUT saved to: " << vlut_file << " (Res: " << res << "m)" << std::endl;
  }
#endif

  delete model;
  return 0;
}
