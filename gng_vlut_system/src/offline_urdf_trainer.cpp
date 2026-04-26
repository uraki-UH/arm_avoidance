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

#include "rclcpp/rclcpp.hpp"

#include "collision/composite_collision_checker.hpp"
#include "collision/environment_collision_checker.hpp"
#include "collision/geometric_self_collision_checker.hpp"
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



class OfflineUrdfTrainerNode : public rclcpp::Node {
public:
  OfflineUrdfTrainerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("offline_urdf_trainer", options) {
    RCLCPP_INFO(this->get_logger(), "--- Standalone URDF-based Unified GNG/VLUT Pipeline ---");

    // 0. Declare and Get Parameters
    experiment_id_ = this->declare_parameter<std::string>("experiment_id", "standalone_train");
    data_directory_ = robot_sim::common::resolveDataPath(
        this->declare_parameter<std::string>("data_directory", "gng_results"));
    robot_urdf_path_ = this->declare_parameter<std::string>("robot_urdf_path", "temp_robot.urdf");
    ground_z_threshold_ = this->declare_parameter<double>("ground_z_threshold", 0.0);
    leaf_link_name_ = this->declare_parameter<std::string>("leaf_link_name", "end_effector_link");
    gng_dimension_ = this->declare_parameter<int>("gng_dimension", 6);
    spatial_map_resolution_ = this->declare_parameter<double>("spatial_map_resolution", 0.02);
    sensing_resolution_ = this->declare_parameter<double>("sensing_resolution", 0.02);
    arm_cache_resolution_ = this->declare_parameter<double>("arm_cache_resolution", 0.008);
    danger_threshold_ = this->declare_parameter<double>("danger_threshold", 0.025);
    vlut_only_ = this->declare_parameter<bool>("vlut_only", false);

    // GNG Parameters (nested under gng_params)
    gng_params_.lambda = this->declare_parameter<int>("gng_params.lambda", 50);
    gng_params_.max_node_num = this->declare_parameter<int>("gng_params.max_node_num", 10000);
    gng_params_.num_samples = this->declare_parameter<int>("gng_params.num_samples", 1000000);
    gng_params_.max_iterations = this->declare_parameter<int>("gng_params.max_iterations", 1000000);
    gng_params_.refine_iterations = this->declare_parameter<int>("gng_params.refine_iterations", 100000);
    gng_params_.coord_edge_iterations = this->declare_parameter<int>("gng_params.coord_edge_iterations", 200000);
    gng_params_.learn_rate_s1 = this->declare_parameter<double>("gng_params.learn_rate_s1", 0.08);
    gng_params_.learn_rate_s2 = this->declare_parameter<double>("gng_params.learn_rate_s2", 0.008);
    gng_params_.max_edge_age = this->declare_parameter<int>("gng_params.max_edge_age", 500);
    gng_params_.alpha = this->declare_parameter<double>("gng_params.alpha", 0.5);
    gng_params_.beta = this->declare_parameter<double>("gng_params.beta", 0.0005);
    gng_params_.n_best_candidates = this->declare_parameter<int>("gng_params.n_best_candidates", 4);
    gng_params_.ais_threshold = this->declare_parameter<double>("gng_params.ais_threshold", 1.0);
    gng_params_.start_node_num = this->declare_parameter<int>("gng_params.start_node_num", 2);

    // Log parameters for verification
    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  experiment_id: %s", experiment_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  robot_urdf_path: %s", robot_urdf_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  spatial_map_resolution: %f", spatial_map_resolution_);
    RCLCPP_INFO(this->get_logger(), "  vlut_only: %s", vlut_only_ ? "true" : "false");
  } // Constructor

  void run_training_pipeline() {
    // 1. Robot Setup
    kinematics::KinematicChain arm;
    simulation::RobotModel *model = nullptr;
    try {
        std::string resolved_path = robot_sim::common::resolvePath(robot_urdf_path_);

        if (resolved_path.empty() || !std::filesystem::exists(resolved_path) || std::filesystem::is_directory(resolved_path)) {
            // Try common extensions as a fallback
            std::string with_xacro = robot_sim::common::resolvePath(robot_urdf_path_ + ".xacro");
            if (!with_xacro.empty() && std::filesystem::exists(with_xacro)) {
                resolved_path = with_xacro;
            } else {
                std::string with_urdf = robot_sim::common::resolvePath(robot_urdf_path_ + ".urdf");
                if (!with_urdf.empty() && std::filesystem::exists(with_urdf)) {
                    resolved_path = with_urdf;
                }
            }
        }

        if (resolved_path.empty() || !std::filesystem::exists(resolved_path)) {
            throw std::runtime_error("Could not find robot model for: " + robot_urdf_path_);
        }

        RCLCPP_INFO(this->get_logger(), "[Robot] Loading from resolved path: %s", resolved_path.c_str());
        auto model_obj = simulation::loadRobotFromUrdf(resolved_path);
    model = new simulation::RobotModel(model_obj);
    arm = simulation::createKinematicChainFromModel(*model, leaf_link_name_);
    arm.setBase(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    RCLCPP_INFO(this->get_logger(), "[Robot] Loaded: %s, DOF: %d", full_urdf.c_str(), arm.getTotalDOF());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "[Error] Robot setup failed: %s", e.what());
    if (model) delete model;
    throw; // Re-throw to indicate failure
  }

  // 2. Setup Checkers
  auto self_checker = std::make_shared<simulation::GeometricSelfCollisionChecker>(*model, arm);
  auto env_checker = std::make_shared<simulation::EnvironmentCollisionChecker>();
  env_checker->addBoxObstacle("ground", Eigen::Vector3d(0, 0, ground_z_threshold_ - 0.05),
                              Eigen::Matrix3d::Identity(), Eigen::Vector3d(10.0, 10.0, 0.05));
#ifdef USE_FCL
  self_checker->setStrictMode(true);
#endif

  auto composite_checker = std::make_shared<simulation::CompositeCollisionChecker>();
  composite_checker->setSelfCollisionChecker(self_checker);
  composite_checker->setEnvironmentCollisionChecker(env_checker);
  
  // 3. GNG Setup
  GNG2 gng(gng_dimension_, 3, &arm);

  // Initialize GNG parameters from ROS 2 parameters
  gng.setParams(gng_params_);

  gng.setSelfCollisionChecker(composite_checker.get());
  // Initialize Status Providers
  gng.registerStatusProvider(
      std::make_shared<
          GNG::GeometricSelfCollisionProvider<Eigen::VectorXf, Eigen::Vector3f>>(
          composite_checker.get(), &arm));
  gng.registerStatusProvider(
      std::make_shared<GNG::ManipulabilityProvider<Eigen::VectorXf,
                                                   Eigen::Vector3f>>(&arm));
  gng.registerStatusProvider(
      std::make_shared<GNG::EEDirectionProvider<Eigen::VectorXf,
                                                Eigen::Vector3f>>(&arm));

  std::filesystem::path output_dir = std::filesystem::path(data_directory_) / experiment_id_;
  gng.setStatsLogPath((output_dir / (experiment_id_ + "_distance_stats.dat")).string());

  // Define standard file paths
  std::string gng_file_path = (output_dir / "gng.bin").string();

  // 4. Training Steps
  if (vlut_only_) {
      RCLCPP_INFO(this->get_logger(), "[Step 0] Skipping GNG training. Loading existing map: %s", gng_file_path.c_str());
      if (!gng.load(gng_file_path)) {
          RCLCPP_ERROR(this->get_logger(), "[Error] Failed to load GNG map for VLUT reconstruction from %s.", gng_file_path.c_str());
          if (model) delete model;
          throw std::runtime_error("Failed to load GNG map.");
      }
  } else {
      RCLCPP_INFO(this->get_logger(), "[Step 1] Initial Exploration...");
      gng.setCollisionAware(false); gng.gngTrainOnTheFly(gng_params_.max_iterations);
      
      RCLCPP_INFO(this->get_logger(), "[Step 2] Intermediate Filter...");
      gng.strictFilter();
      
      RCLCPP_INFO(this->get_logger(), "[Step 3] Refinement (Self-Collision Aware)...");
      gng.setCollisionAware(true); gng.gngTrainOnTheFly(gng_params_.refine_iterations);
      
      RCLCPP_INFO(this->get_logger(), "[Step 4] Final Verification...");
      gng.strictFilter(); gng.refresh_coord_weights();
      
      RCLCPP_INFO(this->get_logger(), "[Step 5] Coordinate Space Edge construction...");
      gng.trainCoordEdgesOnTheFly(gng_params_.coord_edge_iterations);

      // Metadata update for finale
      gng.triggerBatchUpdates();
  }

  // 5. High-Fidelity Voxelization (VLUT Generation)
#ifdef USE_FCL
  struct VRel { long vid; int nid; int lid; };
  std::vector<VRel> v_rels;
  double res = spatial_map_resolution_;
  fcl::Vector3d box_half_size(res * 0.5, res * 0.5, res * 0.5);
  auto active_ids = gng.getActiveIndices();
  int processed = 0;

  RCLCPP_INFO(this->get_logger(), "[Step 6] Pre-voxelizing Robot Links...");
  
  // Cache for local voxel centers of each link
  struct LocalVoxelCloud { std::vector<fcl::Vector3d> centers; };
  std::map<int, LocalVoxelCloud> link_voxel_clouds;

  const auto& objects = self_checker->getCollisionObjects();
  for (size_t i = 0; i < objects.size(); ++i) {
      if (objects[i].type != collision::SelfCollisionChecker::ShapeType::MESH) {
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
      RCLCPP_INFO(this->get_logger(), "  Link [%zu] (%s) voxelized: %zu voxels. Local Z-range: [%f, %f]",
                  i, objects[i].name.c_str(), cloud.centers.size(), mesh_min.z(), mesh_max.z());
  }

  RCLCPP_INFO(this->get_logger(), "[Step 6] Building Spatial Index (Voxel Cloud Transformation)...");
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
          RCLCPP_INFO(this->get_logger(), "  Analyzed Node %d/%zu (%zu rels)", processed, active_ids.size(), v_rels.size());
      }
  }
  RCLCPP_INFO(this->get_logger(), "");

  // Calculate Relative Margin (10% of the detected range, min 2cm)
  fcl::Vector3d range = actual_max - actual_min;
  fcl::Vector3d margin = range * 0.1;
  for (int i = 0; i < 3; ++i) if (margin[i] < 0.02) margin[i] = 0.02;

  actual_min -= margin;
  actual_max += margin;
  
  Eigen::Vector3i final_dims = ((actual_max - actual_min) / res).array().ceil().cast<int>().cwiseMax(1);
  RCLCPP_INFO(this->get_logger(), "[Step 6] Final Voxel Grid: [%ld, %ld, %ld] (%ld total voxels)", 
            (long)final_dims.x(), (long)final_dims.y(), (long)final_dims.z(), (long)final_dims.x() * final_dims.y() * final_dims.z());
  RCLCPP_INFO(this->get_logger(), "[Step 6] Workspace Bounds: Min(%f, %f, %f), Max(%f, %f, %f)", 
            actual_min.x(), actual_min.y(), actual_min.z(), actual_max.x(), actual_max.y(), actual_max.z());
#endif

  // 6. Save Everything
  std::filesystem::path output_dir_path = std::filesystem::path(data_directory_) / experiment_id_;
  std::filesystem::create_directories(output_dir_path);

  // Save GNG Map
  if (gng.save(gng_file_path)) { RCLCPP_INFO(this->get_logger(), "[Success] GNG saved to: %s", gng_file_path.c_str()); }
  else { RCLCPP_ERROR(this->get_logger(), "[Error] Failed to save GNG to: %s", gng_file_path.c_str()); }

  // Save VLUT
#ifdef USE_FCL
  std::string vlut_file_path = (output_dir_path / "vlut.bin").string();
  std::ofstream ofs(vlut_file_path, std::ios::binary);
  if (ofs) {
      // --- Add Self-Describing Header ---
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
      RCLCPP_INFO(this->get_logger(), "[Success] VLUT saved to: %s (Res: %f m)", vlut_file_path.c_str(), res);
  }
  else { RCLCPP_ERROR(this->get_logger(), "[Error] Failed to save VLUT to: %s", vlut_file_path.c_str()); }
#endif

  delete model;
}

private:
  // Member variables to store parameters
  std::string experiment_id_;
  std::string data_directory_;
  std::string robot_urdf_path_;
  double ground_z_threshold_;
  std::string leaf_link_name_;
  int gng_dimension_;
  double spatial_map_resolution_;
  double sensing_resolution_;
  double arm_cache_resolution_;
  double danger_threshold_;
  bool vlut_only_ = false;

  // GNG Parameters struct
  GNG::GngParameters gng_params_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  // Pass command-line arguments to the node options
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<OfflineUrdfTrainerNode>(options);

  try {
    node->run_training_pipeline();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Caught exception during training pipeline: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }
  
  RCLCPP_INFO(node->get_logger(), "Training pipeline finished successfully.");
  rclcpp::shutdown();
  return 0;
}
