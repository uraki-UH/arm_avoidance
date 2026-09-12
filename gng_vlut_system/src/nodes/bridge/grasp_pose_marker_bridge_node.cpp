#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>


#include <arrow_visualization/arrow_marker.hpp>

namespace robot_sim::bridge
{

class GraspPoseMarkerBridgeNode : public rclcpp::Node
{
public:
  explicit GraspPoseMarkerBridgeNode(const rclcpp::NodeOptions &options)
  : Node("grasp_pose_marker_bridge_node", options)
  {
    declare_parameter<std::string>("input_topic", "/pose_array");
    declare_parameter<std::string>("output_topic", "/grasp_pose_markers");
    declare_parameter<std::string>("marker_namespace", "grasp_pose");
    declare_parameter<double>("arrow_length", 0.04);
    declare_parameter<double>("shaft_diameter", 0.006);
    declare_parameter<double>("head_diameter", 0.008);
    declare_parameter<double>("color_a", 1.0);
    declare_parameter<double>("color_r", 1.0);
    declare_parameter<double>("color_g", 0.2);
    declare_parameter<double>("color_b", 0.2);
    declare_parameter<std::string>("anchor", "tail");
    declare_parameter<bool>("enable_transverse_axes", true);
    declare_parameter<int>("primary_axis_idx", 0);
    declare_parameter<double>("primary_axis_sign", -1.0);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    marker_namespace_ = get_parameter("marker_namespace").as_string();
    arrow_length_ = std::max(0.0001, get_parameter("arrow_length").as_double());
    shaft_diameter_ = std::max(0.0001, get_parameter("shaft_diameter").as_double());
    head_diameter_ = std::max(0.0001, get_parameter("head_diameter").as_double());
    color_a_ = std::clamp(get_parameter("color_a").as_double(), 0.0, 1.0);
    const int primary_axis_idx = get_parameter("primary_axis_idx").as_int();
    if (primary_axis_idx < 0 || primary_axis_idx > 2) {
      throw std::invalid_argument("primary_axis_idx must be 0 (X), 1 (Y), or 2 (Z)");
    }
    primary_axis_idx_ = static_cast<std::size_t>(primary_axis_idx);
    primary_axis_sign_ = get_parameter("primary_axis_sign").as_double() < 0.0 ? -1.0 : 1.0;

    subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      input_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&GraspPoseMarkerBridgeNode::poseArrayCallback, this, std::placeholders::_1));


    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      output_topic_, rclcpp::QoS(1).reliable().transient_local());
    publishDeleteAll();

    RCLCPP_INFO(
      get_logger(),
      "GraspPoseMarkerBridgeNode initialized. input=%s output=%s arrow_length=%.4f axis=%zu sign=%.0f",
      input_topic_.c_str(), output_topic_.c_str(), arrow_length_, primary_axis_idx_,
      primary_axis_sign_);
  }

private:
  void publishDeleteAll()
  {
    visualization_msgs::msg::MarkerArray out;
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    out.markers.push_back(std::move(marker));
    publisher_->publish(std::move(out));
  }

  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray out;
    arrow_visualization::pose_arrow_style options;
    options.marker_namespace = marker_namespace_;
    options.primary.length = arrow_length_;
    options.primary_axis_idx = primary_axis_idx_;
    options.primary_axis_sign = primary_axis_sign_;
    options.helper_axis_length_ratio = 0.5;
    options.primary.shaft_diameter = shaft_diameter_;
    options.primary.head_diameter = head_diameter_;
    options.primary.color.a = color_a_;
    options.primary.anchor = get_parameter("anchor").as_string();
    options.enable_transverse_axes = get_parameter("enable_transverse_axes").as_bool();
    options.primary.color.r = get_parameter("color_r").as_double();
    options.primary.color.g = get_parameter("color_g").as_double();
    options.primary.color.b = get_parameter("color_b").as_double();
    options.primary.head_length = std::min(arrow_length_, head_diameter_ * 1.5);
    out = arrow_visualization::make_pose_arrows(*msg, options);
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    out.markers.insert(out.markers.begin(), clear);
    publisher_->publish(std::move(out));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string marker_namespace_;
  double arrow_length_ = 0.04;
  double shaft_diameter_ = 0.006;
  double head_diameter_ = 0.008;
  double color_a_ = 1.0;
  std::size_t primary_axis_idx_ = 0U;
  double primary_axis_sign_ = -1.0;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

}  // namespace robot_sim::bridge

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::GraspPoseMarkerBridgeNode)

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::bridge::GraspPoseMarkerBridgeNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
