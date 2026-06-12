#include <ais_gng/topological_query/topological_query_node.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fuzzrobo::topological_query::TopologicalQueryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

