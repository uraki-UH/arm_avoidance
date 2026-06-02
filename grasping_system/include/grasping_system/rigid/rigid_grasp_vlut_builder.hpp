#pragma once

#include <grasping_system/rigid/rigid_grasp_model.hpp>
#include <grasping_system/rigid/rigid_grasp_vlut.hpp>
#include <grasping_system/voxel/grasp_voxel_model.hpp>
#include <voxel_indexing.hpp>

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace grasping_system::rigid
{

struct RigidGraspVlutSeed
{
  int gng_node_id{-1};
  geometry_msgs::msg::Pose eef_pose_in_world{};
  geometry_msgs::msg::Pose object_pose_in_world{};
  geometry_msgs::msg::Pose object_pose_in_eef{};
  double traversal_cost{0.0};
};

class RigidGraspVlutBuilder
{
public:
  static geometry_msgs::msg::Pose composePose(const geometry_msgs::msg::Pose &parent,
                                              const geometry_msgs::msg::Pose &child)
  {
    tf2::Transform parent_tf;
    tf2::Transform child_tf;
    tf2::fromMsg(parent, parent_tf);
    tf2::fromMsg(child, child_tf);
    const tf2::Transform combined = parent_tf * child_tf;

    geometry_msgs::msg::Pose pose;
    pose.position.x = combined.getOrigin().x();
    pose.position.y = combined.getOrigin().y();
    pose.position.z = combined.getOrigin().z();
    pose.orientation = tf2::toMsg(combined.getRotation());
    return pose;
  }

  RigidGraspVlut build(const voxel::GraspVoxelModel &voxel_model,
                       const std::vector<RigidGraspVlutSeed> &seeds) const
  {
    RigidGraspVlut vlut;
    vlut.indexing() = voxel_model.indexing();

    const double voxel_size = voxel_model.indexing().voxel_size;
    if (voxel_size <= 0.0 || seeds.empty() || voxel_model.cells().empty()) {
      return vlut;
    }

    std::unordered_map<int, RigidGraspVlutEntry> entry_map;
    std::vector<std::pair<long, int>> assignments;
    assignments.reserve(voxel_model.cells().size() * seeds.size());

    for (const auto &seed : seeds) {
      auto [it, inserted] = entry_map.emplace(seed.gng_node_id, RigidGraspVlutEntry{});
      auto &entry = it->second;
      if (inserted) {
        entry.gng_node_id = seed.gng_node_id;
        entry.eef_pose_in_world = seed.eef_pose_in_world;
        entry.object_pose_in_eef = seed.object_pose_in_eef;
        entry.traversal_cost = seed.traversal_cost;
      }

      const geometry_msgs::msg::Pose object_pose_in_world =
        composePose(seed.eef_pose_in_world, seed.object_pose_in_eef);
      entry.object_pose_in_world = object_pose_in_world;

      tf2::Transform object_tf;
      tf2::fromMsg(object_pose_in_world, object_tf);

      for (const auto &kv : voxel_model.cells()) {
        const auto &cell = kv.second;
        if (!cell.active || cell.occupancy <= 0.0) {
          continue;
        }

        const auto local_center =
          voxel_indexing_common::VoxelIndexingSchema::indexToWorldCenter(cell.key, voxel_size);
        const tf2::Vector3 local_point(local_center[0], local_center[1], local_center[2]);
        const tf2::Vector3 world_point = object_tf * local_point;
        const auto world_index = voxel_indexing_common::VoxelIndexingSchema::worldToIndex(
          world_point.x(), world_point.y(), world_point.z(), voxel_size);
        const long flat_voxel_id = static_cast<long>(vlut.indexing().pack(world_index));
        assignments.emplace_back(flat_voxel_id, seed.gng_node_id);
      }
    }

    std::sort(assignments.begin(), assignments.end(),
              [](const auto &a, const auto &b) {
                if (a.first != b.first) {
                  return a.first < b.first;
                }
                return a.second < b.second;
              });
    assignments.erase(std::unique(assignments.begin(), assignments.end()), assignments.end());

    for (const auto &[voxel_id, node_id] : assignments) {
      entry_map[node_id].related_voxel_ids.push_back(voxel_id);
    }

    std::vector<int> node_ids;
    node_ids.reserve(entry_map.size());
    for (const auto &kv : entry_map) {
      node_ids.push_back(kv.first);
    }
    std::sort(node_ids.begin(), node_ids.end());

    for (int node_id : node_ids) {
      auto &entry = entry_map[node_id];
      auto &voxels = entry.related_voxel_ids;
      std::sort(voxels.begin(), voxels.end());
      voxels.erase(std::unique(voxels.begin(), voxels.end()), voxels.end());
      vlut.addEntry(entry);
    }

    return vlut;
  }
};

}  // namespace grasping_system::rigid
