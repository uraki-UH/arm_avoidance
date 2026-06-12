#include <ais_gng/topological_grid/topological_grid_node.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fuzzrobo::topological_grid::TopologicalGridNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

