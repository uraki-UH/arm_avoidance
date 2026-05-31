#include <grasping_system/rigid/rigid_grasp_vlut_builder.hpp>

#include "safety_engine/gng/GrowingNeuralGas.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace
{

geometry_msgs::msg::Pose makePose(const Eigen::Vector3f &position,
                                  const Eigen::Quaternionf &orientation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

grasping_system::voxel::GraspVoxelModel makeCubeModel(double edge_m, double voxel_size)
{
  grasping_system::voxel::GraspVoxelModel model;
  model.indexing().voxel_size = voxel_size;
  model.indexing().x_shift = 42;
  model.indexing().y_shift = 21;
  model.indexing().z_shift = 0;
  model.indexing().offset = 1000000;

  const int half_cells = static_cast<int>(edge_m / voxel_size / 2.0 + 0.5);
  for (int x = -half_cells; x < half_cells; ++x) {
    for (int y = -half_cells; y < half_cells; ++y) {
      for (int z = -half_cells; z < half_cells; ++z) {
        grasping_system::voxel::VoxelCell cell;
        cell.key = {x, y, z};
        cell.occupancy = 1.0;
        cell.active = true;
        model.addCell(cell);
      }
    }
  }
  return model;
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc < 2 || argv[1] == nullptr) {
    std::cerr << "Usage: gng_grasp_vlut_preview <gng_bin_path> [seed_limit]\n";
    return 1;
  }

  const std::string gng_path = argv[1];
  int seed_limit = -1;
  if (argc >= 3 && argv[2] != nullptr) {
    seed_limit = std::atoi(argv[2]);
  }

  using GngType = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
  auto gng = std::make_shared<GngType>(7, 3, nullptr);
  if (!gng->load(gng_path)) {
    std::cerr << "Failed to load GNG: " << gng_path << "\n";
    return 1;
  }

  grasping_system::voxel::GraspVoxelModel model = makeCubeModel(0.08, 0.02);
  grasping_system::rigid::RigidGraspVlutBuilder builder;

  std::vector<grasping_system::rigid::RigidGraspVlutSeed> seeds;
  seeds.reserve(static_cast<std::size_t>(gng->getMaxNodeNum()));

  int visited = 0;
  gng->forEachActiveValid([&](int id, const auto &node) {
    if (seed_limit >= 0 && visited >= seed_limit) {
      return;
    }
    grasping_system::rigid::RigidGraspVlutSeed seed;
    seed.gng_node_id = id;
    seed.eef_pose_in_world = makePose(node.weight_coord, node.status.ee_orientation);
    seed.object_pose_in_eef.orientation.w = 1.0;
    seed.traversal_cost = node.error_angle;
    seeds.push_back(seed);
    ++visited;
  });

  const auto t0 = std::chrono::steady_clock::now();
  auto vlut = builder.build(model, seeds);
  const auto t1 = std::chrono::steady_clock::now();
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

  std::cout << "GNG loaded: " << gng_path << "\n";
  std::cout << "  active_valid_nodes: " << seeds.size() << "\n";
  std::cout << "  vlut_entries: " << vlut.size() << "\n";
  std::cout << "  build_elapsed_ms: " << elapsed_ms << "\n";

  if (!seeds.empty()) {
    const auto &last_seed = seeds.back();
    if (const auto *entry = vlut.findByNodeId(last_seed.gng_node_id)) {
      std::cout << "  last_node_id: " << entry->gng_node_id << "\n";
      std::cout << "  last_object_pose_in_world: "
                << entry->object_pose_in_world.position.x << ", "
                << entry->object_pose_in_world.position.y << ", "
                << entry->object_pose_in_world.position.z << "\n";
      std::cout << "  last_related_voxel_ids: " << entry->related_voxel_ids.size() << "\n";
      if (!entry->related_voxel_ids.empty()) {
        const auto *nodes = vlut.findNodesByVoxel(entry->related_voxel_ids.front());
        std::cout << "  inverse_lookup_node_count(first_voxel): "
                  << (nodes ? nodes->size() : 0) << "\n";
      }
    }
  }

  return 0;
}
