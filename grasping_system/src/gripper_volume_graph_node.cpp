#include <graph/gripper_volume_graph_builder.hpp>
#include <graph/gripper_volume_topological_map.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace grasping_system::nodes
{

class GripperVolumeGraphNode : public rclcpp::Node
{
public:
  GripperVolumeGraphNode()
  : Node("gripper_volume_graph_node")
  {
    const std::string output_topic =
      declare_parameter<std::string>("output_topic", "gripper_volume_topological_map");
    frame_id_ = declare_parameter<std::string>("frame_id", "tool0");
    shape_ = declare_parameter<std::string>("shape", "box");
    dimensions_ = declare_parameter<std::vector<double>>(
      "dimensions", {0.08, 0.04, 0.10});
    center_ = declare_parameter<std::vector<double>>(
      "center", {0.0, 0.0, 0.05});
    orientation_xyzw_ = declare_parameter<std::vector<double>>(
      "orientation_xyzw", {0.0, 0.0, 0.0, 1.0});
    resolution_ = declare_parameter<double>("resolution", 0.01);
    label_ = declare_parameter<int>("label", 0);
    semantic_label_ = declare_parameter<int>("semantic_label", 0);

    publisher_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
      output_topic, rclcpp::QoS(1).reliable().transient_local());
    publishGraph();
  }

private:
  void publishGraph()
  {
    try {
      if (frame_id_.empty()) {
        throw std::invalid_argument("frame_id must not be empty");
      }
      if (dimensions_.size() != 3U || center_.size() != 3U ||
        orientation_xyzw_.size() != 4U)
      {
        throw std::invalid_argument(
                "dimensions and center need 3 values; orientation_xyzw needs 4 values");
      }
      if (label_ < 0 || label_ > 255 || semantic_label_ < 0 || semantic_label_ > 255) {
        throw std::invalid_argument("label values must be in the uint8 range [0, 255]");
      }

      graph::GripperVolumeGraphSpec spec;
      spec.shape = graph::parseGripperVolumeShape(shape_);
      std::copy_n(dimensions_.begin(), 3, spec.dimensions.begin());
      spec.resolution = resolution_;
      spec.pose_in_frame.position.x = center_[0];
      spec.pose_in_frame.position.y = center_[1];
      spec.pose_in_frame.position.z = center_[2];
      spec.pose_in_frame.orientation.x = orientation_xyzw_[0];
      spec.pose_in_frame.orientation.y = orientation_xyzw_[1];
      spec.pose_in_frame.orientation.z = orientation_xyzw_[2];
      spec.pose_in_frame.orientation.w = orientation_xyzw_[3];

      const auto volume = graph::GripperVolumeGraphBuilder::build(spec);
      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = frame_id_;
      const auto map = graph::toTopologicalMap(
        volume, header, static_cast<std::uint8_t>(label_),
        static_cast<std::uint8_t>(semantic_label_));
      publisher_->publish(map);
      RCLCPP_INFO(
        get_logger(),
        "Published gripper volume graph: frame=%s shape=%s nodes=%zu edges=%zu topic=%s",
        frame_id_.c_str(), shape_.c_str(), map.nodes.size(), map.edges.size() / 2U,
        publisher_->get_topic_name());
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Failed to build gripper volume graph: %s", error.what());
    }
  }

  std::string frame_id_;
  std::string shape_;
  std::vector<double> dimensions_;
  std::vector<double> center_;
  std::vector<double> orientation_xyzw_;
  double resolution_{0.01};
  int label_{0};
  int semantic_label_{0};
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr publisher_;
};

}  // namespace grasping_system::nodes

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grasping_system::nodes::GripperVolumeGraphNode>());
  rclcpp::shutdown();
  return 0;
}
