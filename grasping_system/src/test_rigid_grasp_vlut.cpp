#include <rigid/rigid_grasp_vlut_builder.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <vector>

using grasping_system::rigid::RigidGraspVlutBuilder;
using grasping_system::rigid::RigidGraspVlutSeed;

int main()
{
  int seed_count = 1000;
  if (const char *env = std::getenv("GRASP_VLUT_SEED_COUNT")) {
    int parsed = std::atoi(env);
    if (parsed > 0) {
      seed_count = parsed;
    }
  }

  grasping_system::voxel::GraspVoxelModel model;
  model.indexing().voxel_size = 0.02;
  model.indexing().x_shift = 42;
  model.indexing().y_shift = 21;
  model.indexing().z_shift = 0;
  model.indexing().offset = 1000000;

  // 8cm cube in the object frame, centered at the origin.
  // With 2cm voxels this becomes a 4x4x4 block.
  for (int x = -2; x < 2; ++x) {
    for (int y = -2; y < 2; ++y) {
      for (int z = -2; z < 2; ++z) {
      grasping_system::voxel::VoxelCell cell;
        cell.key = {x, y, z};
        cell.occupancy = 1.0;
        cell.active = true;
        model.addCell(cell);
      }
    }
  }

  geometry_msgs::msg::Pose eef_pose;
  eef_pose.position.x = 0.30;
  eef_pose.position.y = -0.05;
  eef_pose.position.z = 0.40;
  eef_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose object_in_eef;
  object_in_eef.orientation.w = 1.0;

  std::vector<RigidGraspVlutSeed> seeds;
  seeds.reserve(static_cast<std::size_t>(seed_count));
  for (int i = 0; i < seed_count; ++i) {
    RigidGraspVlutSeed seed;
    seed.gng_node_id = i;
    seed.eef_pose_in_world = eef_pose;
    seed.eef_pose_in_world.position.x += 0.0005 * static_cast<double>(i % 10);
    seed.eef_pose_in_world.position.y += 0.0005 * static_cast<double>((i / 10) % 10);
    seed.eef_pose_in_world.position.z += 0.0005 * static_cast<double>((i / 100) % 10);
    seed.object_pose_in_eef = object_in_eef;
    seed.traversal_cost = 1.25 + 0.001 * static_cast<double>(i);
    seeds.push_back(seed);
  }

  RigidGraspVlutBuilder builder;
  auto vlut = builder.build(model, seeds);

  std::cout << "VLUT built\n";
  std::cout << "  entries: " << vlut.size() << "\n";
  if (const auto *entry = vlut.findByNodeId(seed_count - 1)) {
    std::cout << "  node_id: " << entry->gng_node_id << "\n";
    std::cout << "  object_pose_in_world: "
              << entry->object_pose_in_world.position.x << ", "
              << entry->object_pose_in_world.position.y << ", "
              << entry->object_pose_in_world.position.z << "\n";
    std::cout << "  related_voxel_ids: " << entry->related_voxel_ids.size() << "\n";
    for (std::size_t i = 0; i < entry->related_voxel_ids.size() && i < 10; ++i) {
      std::cout << "    voxel[" << i << "] = " << entry->related_voxel_ids[i] << "\n";
    }
  } else {
    std::cout << "  last node not found\n";
    return 1;
  }

  const auto *vox_nodes = vlut.findNodesByVoxel(vlut.entries().front().related_voxel_ids.front());
  if (vox_nodes) {
    std::cout << "  inverse lookup node count for first voxel: " << vox_nodes->size() << "\n";
  }

  return 0;
}
