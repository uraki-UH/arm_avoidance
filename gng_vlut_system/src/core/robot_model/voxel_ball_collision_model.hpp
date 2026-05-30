#pragma once

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

#include "safety_engine/indexing/index_voxel_grid.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/robot_voxelizer.hpp"
#include "robot_model/voxel_spherizer.hpp"
#include "robot_model/voxel_ball_collision_config.hpp"

namespace simulation {

inline std::vector<std::string>
collectCollisionLinkNames(const RobotModel &model) {
  std::vector<std::string> link_names;
  link_names.reserve(model.getLinks().size());
  for (const auto &[name, props] : model.getLinks()) {
    if (!props.collisions.empty()) {
      link_names.push_back(name);
    }
  }
  return link_names;
}

inline RobotModel buildVoxelBallCollisionModel(
    const RobotModel &model,
    const std::vector<std::string> &voxel_link_names,
    const VoxelBallCollisionConfig &config,
    std::vector<VoxelSphereLinkResult> *out_sphere_links = nullptr) {
  std::vector<std::string> target_links = voxel_link_names;
  if (target_links.empty()) {
    target_links = collectCollisionLinkNames(model);
  }

  const double voxel_size = std::max(1e-12, config.voxel_size);
  const double voxel_padding = std::max(0.0, config.voxel_padding);

  GNG::Analysis::IndexVoxelGrid grid(voxel_size);
  const auto voxel_data =
      RobotVoxelizer::build(model, target_links, grid, {}, voxel_padding);
  const auto sphere_links =
      VoxelSpherizer::fitLinks(voxel_data, voxel_size, config.fit_options);

  if (out_sphere_links) {
    *out_sphere_links = sphere_links;
  }

  std::unordered_set<std::string> replace_links(target_links.begin(), target_links.end());
  RobotModel simplified;
  simplified.setName(model.getName());
  simplified.setRootLinkName(model.getRootLinkName());

  for (const auto &[name, link] : model.getLinks()) {
    LinkProperties copied = link;
    if (replace_links.count(name) > 0) {
      const auto sphere_it = std::find_if(
          sphere_links.begin(), sphere_links.end(),
          [&](const VoxelSphereLinkResult &entry) { return entry.link_name == name; });
      if (sphere_it != sphere_links.end() && !sphere_it->spheres.empty()) {
        copied.collisions.clear();
        copied.collisions.reserve(sphere_it->spheres.size());
        for (std::size_t i = 0; i < sphere_it->spheres.size(); ++i) {
          const auto &sphere = sphere_it->spheres[i];
          if (sphere.radius <= 0.0) {
            continue;
          }
          Collision col;
          col.name = name + "_voxel_sphere_" + std::to_string(i);
          col.origin = Eigen::Isometry3d::Identity();
          col.origin.translation() = sphere.center;
          col.geometry.type = GeometryType::SPHERE;
          col.geometry.size = Eigen::Vector3d(sphere.radius, 0.0, 0.0);
          copied.collisions.push_back(col);
        }
      }
    }
    simplified.addLink(copied);
  }

  for (const auto &[name, joint] : model.getJoints()) {
    simplified.addJoint(joint);
  }

  return simplified;
}

} // namespace simulation
