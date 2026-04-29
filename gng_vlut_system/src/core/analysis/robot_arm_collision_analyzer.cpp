#include "collision/fcl/fcl_collision_detector.hpp"
#include "common/resource_utils.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"

#include <Eigen/Dense>
#include <fcl/narrowphase/distance.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

// Helper to create FCL Collision Objects from simulation::Geometry and
// transform
std::shared_ptr<fcl::CollisionObject<double>>
createFCLCollisionObject(const simulation::Geometry &geom,
                         const Eigen::Isometry3d &transform) {
  fcl::Transform3d fcl_tf;
  Eigen::Vector3d eigen_translation = transform.translation();
  fcl_tf.translation() =
      static_cast<fcl::Vector3d>(collision::toFCL(eigen_translation));

  Eigen::Matrix3d eigen_linear = transform.linear();
  fcl_tf.linear() = static_cast<fcl::Matrix3d>(collision::toFCL(eigen_linear));

  if (geom.type == simulation::GeometryType::BOX) {
    auto fcl_box = std::make_shared<fcl::Box<double>>(
        geom.size.x(), geom.size.y(), geom.size.z());
    return std::make_shared<fcl::CollisionObject<double>>(fcl_box, fcl_tf);
  } else if (geom.type == simulation::GeometryType::CYLINDER) {
    // Note: FCL Capsule is length along Z. URDF cylinder length is
    // geom.size.y(). We need to decide if we want to model URDF cylinder as FCL
    // cylinder or FCL capsule. For simplicity, let's use FCL Cylinder which is
    // more direct if aligned. FCL Cylinder is along Z-axis. URDF cylinder
    // usually also along Z.
    auto fcl_cylinder =
        std::make_shared<fcl::Cylinder<double>>(geom.size.x(), geom.size.y());
    return std::make_shared<fcl::CollisionObject<double>>(fcl_cylinder, fcl_tf);
  } else if (geom.type == simulation::GeometryType::SPHERE) {
    auto fcl_sphere = std::make_shared<fcl::Sphere<double>>(geom.size.x());
    return std::make_shared<fcl::CollisionObject<double>>(fcl_sphere, fcl_tf);
  } else if (geom.type == simulation::GeometryType::MESH) {
    // FCL requires a BVHModel for meshes
    auto fcl_mesh = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();

    // This part assumes that the mesh vertices are relative to the link origin
    // and need to be transformed by 'transform'.
    // However, simulation::Geometry only stores filename, not actual vertices.
    // To properly handle meshes here, we would need to load the mesh data
    // (e.g., from an STL/OBJ file) and then transform its vertices.
    // This is a complex task and beyond the immediate scope of calculating SDF
    // for simple primitives.
    // For now, if we encounter a mesh, we'll return a null object or throw.
    std::cerr << "Warning: Mesh collision geometry not fully supported for "
                 "SDF calculation yet."
              << std::endl;
    return nullptr; // Or throw an exception
  }
  return nullptr; // Unknown geometry type
}

// Struct to hold link information for FCL
struct FCLModelLink {
  std::string name;
  std::vector<std::shared_ptr<fcl::CollisionObject<double>>> collision_objects;
};

// Function to get FCL collision objects for all links of a robot model
std::vector<FCLModelLink>
getFCLCollisionObjectsForRobot(const simulation::RobotModel &model,
                               const kinematics::KinematicChain &chain) {
  std::vector<FCLModelLink> fcl_links;

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      link_positions;
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
      link_orientations;
  chain.forwardKinematicsAt(chain.getJointValues(), link_positions,
                            link_orientations);

  // Map link names to their segment index in KinematicChain
  std::map<std::string, int> link_name_to_segment_idx;
  // Use getNumJoints() as segments count, as each joint adds a segment
  for (int i = 0; i < chain.getNumJoints(); ++i) {
    link_name_to_segment_idx[chain.getLinkName(i)] = i;
  }

  for (const auto &[link_name, link_props] : model.getLinks()) {
    FCLModelLink fcl_link;
    fcl_link.name = link_name;

    // Find the corresponding segment in the kinematic chain
    if (link_name_to_segment_idx.count(link_name) == 0) {
      // This link is not part of the active kinematic chain (e.g., base link,
      // fixed links that are not ancestors of the leaf).
      // For the root link, its transform is Identity (assuming base_position
      // was Zero).
      if (link_name == model.getRootLinkName()) {
        Eigen::Isometry3d root_transform =
            Eigen::Isometry3d::Identity(); // Assuming base transform is
                                           // identity
        // No need to set base_position/orientation from chain as it's private
        // and handled by setBase. If createKinematicChainFromModel is called
        // with a non-zero base_position, it applies to the first segment. For
        // the *true* root link before any segments, we can assume identity
        // unless explicitly passed.

        for (const auto &col_prop : link_props.collisions) {
          Eigen::Isometry3d geom_transform = root_transform * col_prop.origin;
          auto obj =
              createFCLCollisionObject(col_prop.geometry, geom_transform);
          if (obj) {
            fcl_link.collision_objects.push_back(obj);
          }
        }
      }
      // If not in the active chain and not root, skip for now.
      continue;
    }

    int segment_idx = link_name_to_segment_idx[link_name];
    Eigen::Isometry3d link_transform = Eigen::Isometry3d::Identity();
    link_transform.translation() = link_positions[segment_idx];
    link_transform.rotate(
        link_orientations[segment_idx]); // Correct way to assign rotation from
                                         // Quaternion

    for (const auto &col_prop : link_props.collisions) {
      // The collision geometry's origin is relative to the link's frame.
      Eigen::Isometry3d geom_transform = link_transform * col_prop.origin;
      auto obj = createFCLCollisionObject(col_prop.geometry, geom_transform);
      if (obj) {
        fcl_link.collision_objects.push_back(obj);
      }
    }
    if (!fcl_link.collision_objects.empty()) {
      fcl_links.push_back(fcl_link);
    }
  }
  return fcl_links;
}

int main(int argc, char *argv[]) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0]
              << " <urdf_path_arm1> <log_path_arm1> <urdf_path_arm2> "
                 "<log_path_arm2>"
              << std::endl;
    return 1;
  }

  std::string urdf_path_arm1 = argv[1];
  std::string log_path_arm1 = argv[2];
  std::string urdf_path_arm2 = argv[3];
  std::string log_path_arm2 = argv[4];

  // Output file name
  std::string output_filename = "sdf_analysis.csv";
  std::ofstream output_file(output_filename);
  if (!output_file.is_open()) {
    std::cerr << "Error: Could not open output file: " << output_filename
              << std::endl;
    return 1;
  }
  output_file << "Timestamp,MinSDF,CollidingLink1,CollidingLink2\n";

  // 1. Load Robot Models and Kinematic Chains
  simulation::RobotModel model1 = simulation::loadRobotFromUrdf(urdf_path_arm1);
  kinematics::KinematicChain chain1 =
      simulation::createKinematicChainFromModel(model1);

  simulation::RobotModel model2 = simulation::loadRobotFromUrdf(urdf_path_arm2);
  kinematics::KinematicChain chain2 =
      simulation::createKinematicChainFromModel(model2);

  // 2. Parse Log Files
  std::ifstream log_file1(log_path_arm1);
  std::ifstream log_file2(log_path_arm2);

  if (!log_file1.is_open()) {
    std::cerr << "Error: Could not open log file: " << log_path_arm1
              << std::endl;
    return 1;
  }
  if (!log_file2.is_open()) {
    std::cerr << "Error: Could not open log file: " << log_path_arm2
              << std::endl;
    return 1;
  }

  std::string line1, line2;
  // Skip headers
  std::getline(log_file1, line1);
  std::getline(log_file2, line2);

  while (std::getline(log_file1, line1) && std::getline(log_file2, line2)) {
    std::stringstream ss1(line1);
    std::stringstream ss2(line2);

    double timestamp1, timestamp2;
    std::string robot_name1, robot_name2;
    int joint_count1, joint_count2;
    std::string temp_str; // For reading "JointValues..." part

    // Parse arm1 log
    std::getline(ss1, temp_str, ','); // Timestamp
    timestamp1 = std::stod(temp_str);
    std::getline(ss1, robot_name1, ','); // RobotName
    std::getline(ss1, temp_str, ',');    // JointCount
    joint_count1 = std::stoi(temp_str);

    std::vector<double> joint_values1(joint_count1);
    for (int i = 0; i < joint_count1; ++i) {
      std::getline(ss1, temp_str, ',');
      joint_values1[i] = std::stod(temp_str);
    }
    // Skip BaseX, BaseY, BaseZ
    std::getline(ss1, temp_str, ',');
    std::getline(ss1, temp_str, ',');
    std::getline(ss1, temp_str, ',');

    // Parse arm2 log
    std::getline(ss2, temp_str, ','); // Timestamp
    timestamp2 = std::stod(temp_str);
    std::getline(ss2, robot_name2, ','); // RobotName
    std::getline(ss2, temp_str, ',');    // JointCount
    joint_count2 = std::stoi(temp_str);

    std::vector<double> joint_values2(joint_count2);
    for (int i = 0; i < joint_count2; ++i) {
      std::getline(ss2, temp_str, ',');
      joint_values2[i] = std::stod(temp_str);
    }
    // Skip BaseX, BaseY, BaseZ
    std::getline(ss2, temp_str, ',');
    std::getline(ss2, temp_str, ',');
    std::getline(ss2, temp_str, ',');

    // Ensure timestamps are roughly synchronized (or handle as needed)
    if (std::abs(timestamp1 - timestamp2) > 1e-4) {
      std::cerr << "Warning: Timestamps in log files are not synchronized at "
                << timestamp1 << " and " << timestamp2 << std::endl;
      // For simplicity, we'll use timestamp1 for output, but a more robust
      // solution might interpolate or resynchronize.
    }

    // Set joint values to kinematic chains
    chain1.setJointValues(joint_values1);
    chain2.setJointValues(joint_values2);

    // Generate FCL collision objects for current configuration
    std::vector<FCLModelLink> fcl_arm1_links =
        getFCLCollisionObjectsForRobot(model1, chain1);
    std::vector<FCLModelLink> fcl_arm2_links =
        getFCLCollisionObjectsForRobot(model2, chain2);

    double min_signed_distance = std::numeric_limits<double>::max();
    std::string colliding_link1_name = "N/A";
    std::string colliding_link2_name = "N/A";

    // Compute minimum signed distance between all pairs of links
    for (const auto &arm1_link : fcl_arm1_links) {
      for (const auto &arm2_link : fcl_arm2_links) {
        for (const auto &obj1 : arm1_link.collision_objects) {
          for (const auto &obj2 : arm2_link.collision_objects) {
            if (!obj1 || !obj2)
              continue; // Skip if mesh not supported yet

            fcl::DistanceRequest<double> dist_request;
            fcl::DistanceResult<double> dist_result;
            fcl::distance(obj1.get(), obj2.get(), dist_request, dist_result);

            double current_dist = dist_result.min_distance;
            double current_signed_dist = current_dist;

            // Check for collision to determine sign if distance is near zero
            if (current_dist < 1e-6) { // Arbitrary small threshold
              fcl::CollisionRequest<double> col_request;
              fcl::CollisionResult<double> col_result;
              fcl::collide(obj1.get(), obj2.get(), col_request, col_result);
              if (col_result.isCollision()) {
                // FCL collision result doesn't directly give penetration depth
                // for all cases in a simple collide() call without contacts.
                // If collision, we set distance to negative, with a placeholder
                // for depth. A more robust solution would involve computing
                // contact points to get accurate penetration depth.
                current_signed_dist =
                    -0.001; // Indicate collision, placeholder value
              }
            }

            if (current_signed_dist < min_signed_distance) {
              min_signed_distance = current_signed_dist;
              colliding_link1_name = arm1_link.name;
              colliding_link2_name = arm2_link.name;
            }
          }
        }
      }
    }
    output_file << std::fixed << std::setprecision(6) << timestamp1 << ","
                << min_signed_distance << "," << colliding_link1_name << ","
                << colliding_link2_name << "\n";
  }

  output_file.close();
  log_file1.close();
  log_file2.close();

  std::cout << "SDF analysis complete. Results saved to " << output_filename
            << std::endl;

  return 0;
}
