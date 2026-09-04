#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "collision/geometric_self_collision_checker.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include "visualization/visualization_gng.hpp"

namespace {

using CellIndex = std::array<std::int32_t, 3>;

double halton(std::uint64_t index, std::uint32_t base) {
  double value = 0.0;
  double factor = 1.0;
  while (index > 0) {
    factor /= static_cast<double>(base);
    value += factor * static_cast<double>(index % base);
    index /= base;
  }
  return value;
}

std::vector<double> makeInitialJointValues(
    const std::vector<std::pair<double, double>> &joint_limits,
    std::uint64_t sample_index) {
  static constexpr std::array<std::uint32_t, 16> bases{
      2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53};
  std::vector<double> values;
  values.reserve(joint_limits.size());
  for (std::size_t index = 0; index < joint_limits.size(); ++index) {
    const auto &[min_value, max_value] = joint_limits[index];
    const double ratio = halton(sample_index, bases[index % bases.size()]);
    values.push_back(min_value + ratio * (max_value - min_value));
  }
  return values;
}

std::vector<std::pair<double, double>> collectJointLimits(
    const simulation::RobotModel &model,
    const kinematics::KinematicChain &chain) {
  std::vector<std::pair<double, double>> limits;
  for (int joint_index = 0; joint_index < chain.getNumJoints(); ++joint_index) {
    const int dof = chain.getJointDOF(joint_index);
    if (dof <= 0) {
      continue;
    }
    const auto *joint = model.getJoint(chain.getJointName(joint_index));
    const bool has_limits = joint && joint->has_limits &&
                            std::isfinite(joint->limits.lower) &&
                            std::isfinite(joint->limits.upper) &&
                            joint->limits.lower < joint->limits.upper;
    for (int dof_index = 0; dof_index < dof; ++dof_index) {
      limits.emplace_back(has_limits ? joint->limits.lower : -M_PI,
                          has_limits ? joint->limits.upper : M_PI);
    }
  }
  if (limits.empty() || limits.size() !=
                            static_cast<std::size_t>(chain.getTotalDOF())) {
    throw std::runtime_error("reachability voxel joint limits are invalid");
  }
  return limits;
}

class ReachabilityVoxelBuilderNode : public rclcpp::Node {
 public:
  ReachabilityVoxelBuilderNode()
      : Node("reachability_voxel_builder") {
    std::string robot_urdf_path =
        declare_parameter<std::string>("robot_urdf_path", "");
    if (robot_urdf_path.empty()) {
      robot_urdf_path = declare_parameter<std::string>("urdf_path", "");
    }
    const std::string resource_root_dir =
        declare_parameter<std::string>("resource_root_dir", "");
    const std::string mesh_root_dir =
        declare_parameter<std::string>("mesh_root_dir", "");
    const std::string profile_name =
        declare_parameter<std::string>("reachability_voxel.profile_name", "left_arm");
    const std::string root_link = declare_parameter<std::string>(
        "reachability_voxel.root_link", "");
    const std::string eef_link = declare_parameter<std::string>(
        "reachability_voxel.eef_link", "");
    const std::string output_path = declare_parameter<std::string>(
        "reachability_voxel.output_path", "reachability_voxel_map.bin");
    const std::string frame_id = declare_parameter<std::string>(
        "reachability_voxel.frame_id", "base_link");
    const double voxel_size =
        declare_parameter<double>("reachability_voxel.voxel_size", 0.05);
    const Eigen::Vector3d min_corner(
        declare_parameter<double>("reachability_voxel.min_x",
                                  declare_parameter<double>("gng_params.min_x", -0.1)),
        declare_parameter<double>("reachability_voxel.min_y",
                                  declare_parameter<double>("gng_params.min_y", -1.0)),
        declare_parameter<double>("reachability_voxel.min_z",
                                  declare_parameter<double>("gng_params.min_z", -1.0)));
    const Eigen::Vector3d max_corner(
        declare_parameter<double>("reachability_voxel.max_x",
                                  declare_parameter<double>("gng_params.max_x", 0.5)),
        declare_parameter<double>("reachability_voxel.max_y",
                                  declare_parameter<double>("gng_params.max_y", 1.0)),
        declare_parameter<double>("reachability_voxel.max_z",
                                  declare_parameter<double>("gng_params.max_z", 1.0)));
    const int max_sample_count = declare_parameter<int>(
        "reachability_voxel.max_sample_count", 200000);
    const int max_no_new_voxel_samples = declare_parameter<int>(
        "reachability_voxel.max_no_new_voxel_samples", 50000);
    const bool enable_self_collision = declare_parameter<bool>(
        "reachability_voxel.enable_self_collision", true);
    const auto collision_exclusions =
        declare_parameter<std::vector<std::string>>(
            "collision.self_collision_exclusion_pairs", std::vector<std::string>{});

    if (robot_urdf_path.empty() || voxel_size <= 0.0 ||
        (max_corner.array() <= min_corner.array()).any() ||
        max_sample_count < 1 || max_no_new_voxel_samples < 1) {
      throw std::invalid_argument("reachability voxel parameter is invalid");
    }

    const std::string profile_prefix = "gng.profiles." + profile_name + ".";
    const std::string resolved_root = root_link.empty()
                                          ? declare_parameter<std::string>(
                                                profile_prefix + "root", "")
                                          : root_link;
    const std::string resolved_eef = eef_link.empty()
                                         ? declare_parameter<std::string>(
                                               profile_prefix + "eef", "")
                                         : eef_link;
    if (resolved_eef.empty()) {
      throw std::invalid_argument("reachability voxel EEF link is required");
    }

    const simulation::RobotModel model = simulation::loadRobotFromUrdf(
        robot_urdf_path, resource_root_dir, mesh_root_dir);
    const simulation::ArmConfig arm_config{resolved_root, resolved_eef,
                                            profile_name};
    auto chain = simulation::createMultiArmKinematicChain(model, {arm_config});
    const auto joint_limits = collectJointLimits(model, *chain);
    std::unique_ptr<simulation::GeometricSelfCollisionChecker> self_collision_checker;
    if (enable_self_collision) {
      self_collision_checker =
          std::make_unique<simulation::GeometricSelfCollisionChecker>(model, *chain);
      for (const auto &entry : collision_exclusions) {
        const auto separator = entry.find_first_of("|:,");
        if (separator == std::string::npos) {
          continue;
        }
        const std::string first = entry.substr(0, separator);
        const std::string second = entry.substr(separator + 1);
        if (!first.empty() && !second.empty()) {
          self_collision_checker->addCollisionExclusion(first, second);
        }
      }
    }

    const Eigen::Array3i grid_size =
        ((max_corner - min_corner) / voxel_size).array().ceil().cast<int>();
    const std::uint64_t target_count = static_cast<std::uint64_t>(grid_size.x()) *
                                       static_cast<std::uint64_t>(grid_size.y()) *
                                       static_cast<std::uint64_t>(grid_size.z());
    if (target_count == 0 || target_count > 65535U) {
      throw std::invalid_argument(
          "reachability voxel grid must contain 1 to 65535 target cells");
    }

    std::map<CellIndex, robot_sim::visualization::VisualizationGngStaticNode>
        reachable_nodes;
    std::uint64_t collision_reject_count = 0;
    std::uint64_t no_new_voxel_sample_count = 0;
    std::uint64_t sample_count = 0;
    for (std::uint64_t sample_index = 1;
         sample_index <= static_cast<std::uint64_t>(max_sample_count) &&
         no_new_voxel_sample_count <
             static_cast<std::uint64_t>(max_no_new_voxel_samples);
         ++sample_index) {
      const std::vector<double> joint_values =
          makeInitialJointValues(joint_limits, sample_index);
      chain->updateKinematics(joint_values);
      ++sample_count;
      const Eigen::Vector3d position = chain->getEEFPosition();
      if ((position.array() < min_corner.array()).any() ||
          (position.array() >= max_corner.array()).any()) {
        ++no_new_voxel_sample_count;
        continue;
      }
      const Eigen::Array3i cell_index =
          ((position - min_corner) / voxel_size).array().floor().cast<int>();
      const CellIndex cell{cell_index.x(), cell_index.y(), cell_index.z()};
      if (reachable_nodes.count(cell) > 0) {
        ++no_new_voxel_sample_count;
        continue;
      }
      if (self_collision_checker) {
        self_collision_checker->updateBodyPoses(
            chain->getLinkPositions(), chain->getLinkOrientations());
        if (self_collision_checker->checkCollision()) {
          ++collision_reject_count;
          ++no_new_voxel_sample_count;
          continue;
        }
      }
      Eigen::Vector3d normal =
          chain->getEEFOrientation() * Eigen::Vector3d::UnitZ();
      if (normal.norm() <= 1e-9) {
        normal = Eigen::Vector3d::UnitZ();
      } else {
        normal.normalize();
      }
      robot_sim::visualization::VisualizationGngStaticNode node;
      node.position = (min_corner + voxel_size *
          Eigen::Vector3d(static_cast<double>(cell[0]) + 0.5,
                          static_cast<double>(cell[1]) + 0.5,
                          static_cast<double>(cell[2]) + 0.5)).cast<float>();
      node.normal = normal.cast<float>();
      node.label = 1;
      node.representative_joint_angle = Eigen::Map<const Eigen::VectorXd>(
          joint_values.data(), static_cast<Eigen::Index>(joint_values.size()))
                                              .cast<float>();
      reachable_nodes.emplace(cell, std::move(node));
      no_new_voxel_sample_count = 0;
    }

    robot_sim::visualization::VisualizationGngStaticModel map_model;
    map_model.joint_angle_dimension = static_cast<std::uint32_t>(joint_limits.size());
    map_model.nodes.reserve(reachable_nodes.size());
    std::map<CellIndex, std::uint32_t> node_ids;
    for (const auto &[cell, node] : reachable_nodes) {
      node_ids.emplace(cell, static_cast<std::uint32_t>(map_model.nodes.size()));
      map_model.nodes.push_back(node);
    }
    const std::array<CellIndex, 3> neighbor_steps{
        CellIndex{1, 0, 0}, CellIndex{0, 1, 0}, CellIndex{0, 0, 1}};
    for (const auto &[cell, source_id] : node_ids) {
      for (const auto &step : neighbor_steps) {
        const CellIndex neighbor{cell[0] + step[0], cell[1] + step[1],
                                 cell[2] + step[2]};
        const auto target_it = node_ids.find(neighbor);
        if (target_it != node_ids.end()) {
          map_model.edges.emplace_back(source_id, target_it->second);
        }
      }
    }
    std::string error;
    if (!map_model.save(output_path, &error)) {
      throw std::runtime_error(error);
    }
    RCLCPP_INFO(
        get_logger(),
        "Reachability voxel map: frame=%s profile=%s target_cells=%llu reachable_cells=%zu sample_count=%llu collision_reject=%llu voxel_size=%.4f output=%s",
        frame_id.c_str(), profile_name.c_str(),
        static_cast<unsigned long long>(target_count), map_model.nodes.size(),
        static_cast<unsigned long long>(sample_count),
        static_cast<unsigned long long>(collision_reject_count), voxel_size,
        output_path.c_str());
  }
};

}  // 無名名前空間

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    std::make_shared<ReachabilityVoxelBuilderNode>();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "reachability_voxel_builder: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
