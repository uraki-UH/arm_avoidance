#include <memory>

#include "fuzzy_voxel_grid/voxel_grid_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fuzzy_voxel_grid::VoxelGridNode>());
    rclcpp::shutdown();
    return 0;
}