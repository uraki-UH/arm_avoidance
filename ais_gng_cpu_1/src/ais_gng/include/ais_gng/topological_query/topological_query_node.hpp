#pragma once

#include <ais_gng/topological_query/topological_reachability.hpp>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace fuzzrobo::topological_query
{

class TopologicalQueryNode : public rclcpp::Node
{
public:
  explicit TopologicalQueryNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void mapCallback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg);
  void publishResult(
    const ais_gng_msgs::msg::TopologicalMap &map,
    const std::vector<ReachableNode> &reachable_nodes);

  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr node_ids_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string summary_topic_;
  std::string relation_mode_text_;
  QueryOptions query_options_;
  uint8_t semantic_label_value_ = ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE;
};

}  // namespace fuzzrobo::topological_query

