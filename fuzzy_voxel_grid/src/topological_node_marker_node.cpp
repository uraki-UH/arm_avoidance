#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "ais_gng_msgs/msg/topological_map.hpp"

class TopologicalNodeMarkerNode : public rclcpp::Node
{
public:
  TopologicalNodeMarkerNode()
  : Node("topological_node_marker_node")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/topological_map");
    marker_topic_ = this->declare_parameter<std::string>("marker_topic", "/selected_topological_node_marker");
    selected_index_ = this->declare_parameter<int>("selected_index", 0);
    marker_scale_ = this->declare_parameter<double>("marker_scale", 0.3);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);

    sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      input_topic_,
      10,
      std::bind(&TopologicalNodeMarkerNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
  {
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header = msg->header;
    delete_marker.ns = "topological_node_marker";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(delete_marker);

    if (selected_index_ < 0 || static_cast<size_t>(selected_index_) >= msg->nodes.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "selected_index=%d is out of range. nodes.size()=%zu",
        selected_index_, msg->nodes.size());
      return;
    }

    const auto & node = msg->nodes[static_cast<size_t>(selected_index_)];

    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "topological_node_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = node.pos.x;
    marker.pose.position.y = node.pos.y;
    marker.pose.position.z = node.pos.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = marker_scale_;
    marker.scale.y = marker_scale_;
    marker.scale.z = marker_scale_;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker_pub_->publish(marker);
  }

  std::string input_topic_;
  std::string marker_topic_;
  int selected_index_;
  double marker_scale_;

  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopologicalNodeMarkerNode>());
  rclcpp::shutdown();
  return 0;
}