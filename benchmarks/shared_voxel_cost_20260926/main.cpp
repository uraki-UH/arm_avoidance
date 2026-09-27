#include "world.cpp"
#include "fuzzy_voxel_grid/voxel_grid_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  const auto threads = std::getenv("SHARED_BENCH_THREADS");
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), threads ? std::atoi(threads) : 2);
  auto world = std::make_shared<robot_sim::bridge::WorldIndexToVoxelNode>(rclcpp::NodeOptions());
  executor.add_node(world);
  std::shared_ptr<fuzzy_voxel_grid::VoxelGridNode> fvg;
  if (std::getenv("SHARED_BENCH_FVG")[0] == '1') {
    fvg = std::make_shared<fuzzy_voxel_grid::VoxelGridNode>();
    executor.add_node(fvg);
  }
  executor.spin();
  rclcpp::shutdown();
}
