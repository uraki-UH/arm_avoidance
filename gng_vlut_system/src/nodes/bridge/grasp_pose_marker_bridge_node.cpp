#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <gng_control_msgs/msg/grasp_candidate_array.hpp>
#include <arrow_visualization/arrow_marker.hpp>
#include <algorithm>
#include <arrow_visualization/subscription.hpp>
#include <arrow_visualization/state_colors.hpp>
#include <stdexcept>

namespace robot_sim::bridge
{
class GraspPoseMarkerBridgeNode : public rclcpp::Node
{
public:
  explicit GraspPoseMarkerBridgeNode(const rclcpp::NodeOptions &options)
  : Node("grasp_pose_marker_bridge_node", options)
  {
    const auto input_type = declare_parameter<std::string>("input_type", "pose_array");
    const bool has_candidates = input_type == "grasp_candidates";
    if (!has_candidates && input_type != "pose_array") {
      throw std::invalid_argument("input_type must be pose_array or grasp_candidates");
    }
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", has_candidates ? "/grasp_pose_cands" : "/pose_array");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/grasp_pose_markers");
    style_.marker_namespace = declare_parameter<std::string>("marker_namespace", "grasp_pose");
    style_.primary.length = declare_parameter<double>("arrow_length", 0.12);
    style_.primary.shaft_diameter = declare_parameter<double>("shaft_diameter", 0.006);
    style_.primary.head_diameter = declare_parameter<double>("head_diameter", 0.012);
    style_.primary.head_length = declare_parameter<double>(
      "head_length", std::min(style_.primary.length, style_.primary.head_diameter * 1.5));
    style_.primary.color.r = declare_parameter<double>("color_r", 0.0);
    style_.primary.color.g = declare_parameter<double>("color_g", 0.8);
    style_.primary.color.b = declare_parameter<double>("color_b", 0.2);
    style_.primary.color.a = declare_parameter<double>("color_a", 1.0);
    style_.primary.anchor = declare_parameter<std::string>("anchor", "tail");
    style_.enable_transverse_axes = declare_parameter<bool>("enable_transverse_axes", true);
    style_.helper_axis_length_ratio = declare_parameter<double>("helper_axis_length_ratio", 0.5);
    const int primary_axis_idx = declare_parameter<int>("primary_axis_idx", has_candidates ? 2 : 0);
    if (primary_axis_idx < 0 || primary_axis_idx > 2) {
      throw std::invalid_argument("primary_axis_idx must be 0 (X), 1 (Y), or 2 (Z)");
    }
    style_.primary_axis_idx = static_cast<std::size_t>(primary_axis_idx);
    style_.primary_axis_sign = declare_parameter<double>("primary_axis_sign", has_candidates ? 1.0 : -1.0);
    enable_state_colors_ = declare_parameter<bool>("enable_state_colors", true);
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_topic, qos);
    if (has_candidates) {
      subscription_ = arrow_visualization::subscribe_adaptive<gng_control_msgs::msg::GraspCandidateArray>(
        *this, input_topic, [this](gng_control_msgs::msg::GraspCandidateArray::ConstSharedPtr msg) {
          geometry_msgs::msg::PoseArray poses;
          poses.header = msg->header;
          poses.poses.reserve(msg->candidates.size());
          for (const auto &candidate : msg->candidates) poses.poses.push_back(candidate.pose);
          publish_markers(poses, msg.get());
        });
    } else {
      subscription_ = arrow_visualization::subscribe_adaptive<geometry_msgs::msg::PoseArray>(
        *this, input_topic, [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
          publish_markers(*msg);
        });
    }
    publish_markers(geometry_msgs::msg::PoseArray{});
    RCLCPP_INFO(get_logger(), "input=%s (%s) output=%s", input_topic.c_str(),
      input_type.c_str(), output_topic.c_str());
  }

private:
  void publish_markers(const geometry_msgs::msg::PoseArray &poses,
    const gng_control_msgs::msg::GraspCandidateArray *candidates = nullptr)
  {
    auto out = arrow_visualization::make_pose_arrows(poses, style_);
    if (candidates) {
      for (auto &marker : out.markers) {
        const auto &candidate = candidates->candidates[marker.id / 3];
        // uint32候補IDのnamespace保持。Marker IDは軸番号
        marker.ns += "/" + std::to_string(candidate.id);
        marker.id %= 3;
        if (enable_state_colors_ && static_cast<std::size_t>(marker.id) == style_.primary_axis_idx) {
          marker.color = arrow_visualization::state_color(candidate.state, marker.color.a);
        }
      }
    }
    visualization_msgs::msg::Marker clear;
    clear.header = poses.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    out.markers.insert(out.markers.begin(), clear);
    publisher_->publish(std::move(out));
  }

  arrow_visualization::pose_arrow_style style_;
  bool enable_state_colors_ = true;
  std::shared_ptr<void> subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};
}  // robot_sim::bridge 名前空間

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::GraspPoseMarkerBridgeNode)

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_sim::bridge::GraspPoseMarkerBridgeNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
