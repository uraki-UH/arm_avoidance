#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/resource_utils.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/robot_voxelizer.hpp"
#include "robot_model/urdf_loader.hpp"
#include "robot_model/voxel_spherizer.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"

namespace {

std::vector<std::string> splitCommaSeparated(const std::string &text) {
  std::vector<std::string> items;
  std::stringstream ss(text);
  std::string token;
  while (std::getline(ss, token, ',')) {
    auto begin = token.find_first_not_of(" \t");
    auto end = token.find_last_not_of(" \t");
    if (begin == std::string::npos) {
      continue;
    }
    items.push_back(token.substr(begin, end - begin + 1));
  }
  return items;
}

std::vector<std::string> collectAllLinkNames(const simulation::RobotModel &model) {
  std::vector<std::string> names;
  names.reserve(model.getLinks().size());
  for (const auto &kv : model.getLinks()) {
    names.push_back(kv.first);
  }
  return names;
}

std::string defaultRobotDescriptionFile() {
  const std::string pkg_share = ament_index_cpp::get_package_share_directory("gng_vlut_system");
  try {
    return ament_index_cpp::get_package_share_directory("topoarm_description") +
           "/urdf/topo_dual_arm.urdf.xacro";
  } catch (...) {
    return pkg_share + "/urdf/topoarm_description/urdf/topo_dual_arm.urdf.xacro";
  }
}

} // namespace

class VoxelSpherizerPreviewNode : public rclcpp::Node {
public:
  explicit VoxelSpherizerPreviewNode(const rclcpp::NodeOptions &options)
      : Node("voxel_spherizer_preview", options) {
    declare_parameter<std::string>("robot_description_file", defaultRobotDescriptionFile());
    declare_parameter<std::string>("resource_root_dir", "");
    declare_parameter<std::string>("mesh_root_dir", "");
    declare_parameter<std::string>("voxel_link_names", "");
    declare_parameter<double>("voxel_size", 0.01);
    declare_parameter<double>("voxel_padding", 0.0);
    declare_parameter<int>("max_spheres", 64);
    declare_parameter<int>("min_points_per_sphere", 12);
    declare_parameter<double>("min_gain_ratio", 0.15);
    declare_parameter<bool>("verbose", true);

    const std::string robot_description_file = get_parameter("robot_description_file").as_string();
    const std::string resource_root_dir = get_parameter("resource_root_dir").as_string();
    const std::string mesh_root_dir = get_parameter("mesh_root_dir").as_string();
    const std::string voxel_link_names_param = get_parameter("voxel_link_names").as_string();
    const double voxel_size = get_parameter("voxel_size").as_double();
    const double voxel_padding = get_parameter("voxel_padding").as_double();

    simulation::VoxelSphereFitOptions fit_options;
    fit_options.max_spheres = static_cast<std::size_t>(
        std::max<int64_t>(1, get_parameter("max_spheres").as_int()));
    fit_options.min_points_per_sphere = static_cast<std::size_t>(
        std::max<int64_t>(1, get_parameter("min_points_per_sphere").as_int()));
    fit_options.min_gain_ratio = get_parameter("min_gain_ratio").as_double();
    fit_options.verbose = get_parameter("verbose").as_bool();

    const std::string resolved_urdf_path = robot_sim::common::resolvePath(robot_description_file);
    if (resolved_urdf_path.empty()) {
      throw std::runtime_error("Failed to resolve robot_description_file: " + robot_description_file);
    }

    simulation::RobotModel model = simulation::loadRobotFromUrdf(
        resolved_urdf_path, resource_root_dir, mesh_root_dir);

    std::vector<std::string> target_links = splitCommaSeparated(voxel_link_names_param);
    if (target_links.empty()) {
      target_links = collectAllLinkNames(model);
    }

    GNG::Analysis::IndexVoxelGrid grid(voxel_size);
    const auto voxel_data = simulation::RobotVoxelizer::build(
        model, target_links, grid, {}, voxel_padding);

    const auto sphere_sets = simulation::VoxelSpherizer::fitLinks(voxel_data, voxel_size, fit_options);

    std::size_t total_voxels = 0;
    std::size_t total_spheres = 0;
    for (const auto &entry : sphere_sets) {
      total_voxels += entry.voxel_count;
      total_spheres += entry.spheres.size();
      if (entry.spheres.empty()) {
        RCLCPP_INFO(get_logger(), "[VoxelSpherizerPreview] %s: voxels=%zu spheres=0", entry.link_name.c_str(), entry.voxel_count);
        continue;
      }

      double min_radius = std::numeric_limits<double>::max();
      double max_radius = 0.0;
      for (const auto &sphere : entry.spheres) {
        min_radius = std::min(min_radius, sphere.radius);
        max_radius = std::max(max_radius, sphere.radius);
      }

      RCLCPP_INFO(
          get_logger(),
          "[VoxelSpherizerPreview] %s: voxels=%zu spheres=%zu radius_range=[%.4f, %.4f]",
          entry.link_name.c_str(), entry.voxel_count, entry.spheres.size(), min_radius, max_radius);

      if (fit_options.verbose) {
        const std::size_t preview_count = std::min<std::size_t>(3, entry.spheres.size());
        for (std::size_t i = 0; i < preview_count; ++i) {
          const auto &sphere = entry.spheres[i];
          RCLCPP_INFO(
              get_logger(),
              "  sphere[%zu] center=(%.4f, %.4f, %.4f) radius=%.4f",
              i, sphere.center.x(), sphere.center.y(), sphere.center.z(), sphere.radius);
        }
      }
    }

    RCLCPP_INFO(
        get_logger(),
        "[VoxelSpherizerPreview] summary: links=%zu voxels=%zu spheres=%zu voxel_size=%.4f",
        sphere_sets.size(), total_voxels, total_spheres, voxel_size);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<VoxelSpherizerPreviewNode>(rclcpp::NodeOptions());
    (void)node;
  } catch (const std::exception &e) {
    std::cerr << "[VoxelSpherizerPreview] fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
