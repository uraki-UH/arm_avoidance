#include <ais_gng/topological_query/topological_query_node.hpp>

#include <algorithm>
#include <sstream>

namespace fuzzrobo::topological_query
{

TopologicalQueryNode::TopologicalQueryNode(const rclcpp::NodeOptions &options)
: Node("topological_query_node", options)
{
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/topological_map");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/reachable_handle_nodes");
  summary_topic_ = this->declare_parameter<std::string>("summary_topic", "/reachable_handle_nodes/summary");
  relation_mode_text_ = this->declare_parameter<std::string>("relation_mode", "graph_edges");
  semantic_label_value_ = static_cast<uint8_t>(
    std::max<int64_t>(0, this->declare_parameter<int64_t>("semantic_label", 1)));
  query_options_.max_euclidean_distance = this->declare_parameter<double>("max_euclidean_distance", 0.5);
  const auto max_hops_param = this->declare_parameter<int64_t>("max_hops", -1);
  query_options_.max_hops = max_hops_param < 0
    ? std::numeric_limits<std::size_t>::max()
    : static_cast<std::size_t>(std::max<int64_t>(1, max_hops_param));
  query_options_.include_seed_nodes = this->declare_parameter<bool>("include_seed_nodes", true);

  if (const auto mode = relationModeFromString(relation_mode_text_)) {
    query_options_.relation_mode = *mode;
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown relation_mode='%s', falling back to graph_edges",
      relation_mode_text_.c_str());
    query_options_.relation_mode = RelationMode::GraphEdges;
  }

  node_ids_pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>(output_topic_, 10);
  summary_pub_ = this->create_publisher<std_msgs::msg::String>(summary_topic_, 10);
  map_sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
    input_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&TopologicalQueryNode::mapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "TopologicalQueryNode ready: input=%s output=%s relation=%s dist=%.3f hops=%zu semantic=%u",
    input_topic_.c_str(),
    output_topic_.c_str(),
    toString(query_options_.relation_mode).c_str(),
    query_options_.max_euclidean_distance,
    query_options_.max_hops,
    semantic_label_value_);
}

void TopologicalQueryNode::mapCallback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const auto reachable_nodes = queryReachableNodesBySemanticLabel(
    *msg,
    semantic_label_value_,
    query_options_);
  publishResult(*msg, reachable_nodes);
}

void TopologicalQueryNode::publishResult(
  const ais_gng_msgs::msg::TopologicalMap &map,
  const std::vector<ReachableNode> &reachable_nodes)
{
  std_msgs::msg::UInt32MultiArray ids_msg;
  ids_msg.data.reserve(reachable_nodes.size());
  for (const auto &entry : reachable_nodes) {
    if (entry.node_index >= map.nodes.size()) {
      continue;
    }
    ids_msg.data.push_back(static_cast<uint32_t>(entry.node_index));
  }
  node_ids_pub_->publish(ids_msg);

  std_msgs::msg::String summary_msg;
  std::ostringstream oss;
  oss << "{";
  oss << "\"node_count\":" << ids_msg.data.size() << ",";
  oss << "\"relation_mode\":\"" << toString(query_options_.relation_mode) << "\",";
  oss << "\"max_euclidean_distance\":" << query_options_.max_euclidean_distance << ",";
  oss << "\"semantic_label\":" << static_cast<int>(semantic_label_value_) << ",";
  oss << "\"node_ids\":[";
  for (std::size_t i = 0; i < ids_msg.data.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << ids_msg.data[i];
  }
  oss << "]";
  oss << "}";
  summary_msg.data = oss.str();
  summary_pub_->publish(summary_msg);
}

}  // namespace fuzzrobo::topological_query
