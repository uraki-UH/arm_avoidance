#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <ais_gng_msgs/msg/topological_map.hpp>

#include "visualization/visualization_gng.hpp"

namespace {

geometry_msgs::msg::Point32 toPoint32(const Eigen::Vector3f &value) {
  geometry_msgs::msg::Point32 point;
  point.x = value.x();
  point.y = value.y();
  point.z = value.z();
  return point;
}

class VisualizationGngStaticNode : public rclcpp::Node {
 public:
  VisualizationGngStaticNode()
      : Node("visualization_gng_static_node") {
    const std::string model_path = declare_parameter<std::string>("model_path", "");
    const std::string topic_name =
        declare_parameter<std::string>("topic_name", "topological_map_vis");
    const std::string frame_id = declare_parameter<std::string>("frame_id", "base_link");
    if (model_path.empty()) {
      throw std::invalid_argument("model_path is required");
    }

    robot_sim::visualization::VisualizationGngStaticModel model;
    std::string error;
    if (!model.load(model_path, &error)) {
      throw std::runtime_error(error);
    }

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    publisher_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
        topic_name, qos);

    ais_gng_msgs::msg::TopologicalMap message;
    message.header.stamp = now();
    message.header.frame_id = frame_id;
    message.nodes.reserve(model.nodes.size());
    for (std::size_t index = 0; index < model.nodes.size(); ++index) {
      const auto &source = model.nodes[index];
      ais_gng_msgs::msg::TopologicalNode node;
      node.id = static_cast<std::uint16_t>(index);
      node.pos = toPoint32(source.position);
      node.normal = toPoint32(source.normal);
      node.label = source.label;
      message.nodes.push_back(std::move(node));
    }
    message.edges.reserve(model.edges.size() * 2U);
    for (const auto &[source, target] : model.edges) {
      if (source >= model.nodes.size() || target >= model.nodes.size() ||
          source == target) {
        continue;
      }
      message.edges.push_back(static_cast<std::uint16_t>(source));
      message.edges.push_back(static_cast<std::uint16_t>(target));
    }
    publisher_->publish(message);
    RCLCPP_INFO(get_logger(),
                "Published standalone static map: nodes=%zu edges=%zu topic=%s",
                message.nodes.size(), message.edges.size() / 2U,
                publisher_->get_topic_name());
  }

 private:
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr publisher_;
};

}  // 無名名前空間

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VisualizationGngStaticNode>());
  } catch (const std::exception &error) {
    std::cerr << "visualization_gng_static_node: " << error.what() << '\n';
  }
  rclcpp::shutdown();
  return 0;
}
