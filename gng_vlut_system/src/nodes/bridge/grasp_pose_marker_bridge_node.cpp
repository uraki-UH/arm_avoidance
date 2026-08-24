#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "common/grasp_pose_utils.hpp"

namespace robot_sim::bridge
{

class GraspPoseMarkerBridgeNode : public rclcpp::Node
{
public:
  explicit GraspPoseMarkerBridgeNode(const rclcpp::NodeOptions &options)
  : Node("grasp_pose_marker_bridge_node", options)
  {
    declare_parameter<std::string>("input_topic", "/grasp_pose_cands");
    declare_parameter<std::string>("score_topic", "/grasp_pose_cand_scores");
    declare_parameter<std::string>("output_topic", "/grasp_pose_markers");
    declare_parameter<std::string>("marker_namespace", "grasp_pose");
    declare_parameter<double>("arrow_length", 0.04);
    declare_parameter<double>("shaft_diameter", 0.006);
    declare_parameter<double>("head_diameter", 0.008);
    declare_parameter<double>("color_a", 1.0);

    input_topic_ = get_parameter("input_topic").as_string();
    score_topic_ = get_parameter("score_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    marker_namespace_ = get_parameter("marker_namespace").as_string();
    arrow_length_ = std::max(0.0001, get_parameter("arrow_length").as_double());
    shaft_diameter_ = std::max(0.0001, get_parameter("shaft_diameter").as_double());
    head_diameter_ = std::max(0.0001, get_parameter("head_diameter").as_double());
    color_a_ = std::clamp(get_parameter("color_a").as_double(), 0.0, 1.0);

    subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      input_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&GraspPoseMarkerBridgeNode::poseArrayCallback, this, std::placeholders::_1));
    score_subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      score_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&GraspPoseMarkerBridgeNode::scoreCallback, this, std::placeholders::_1));

    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      output_topic_, rclcpp::QoS(1).reliable().transient_local());
    publishDeleteAll();

    RCLCPP_INFO(
      get_logger(),
      "GraspPoseMarkerBridgeNode initialized. input=%s output=%s count_arrow_length=%.4f",
      input_topic_.c_str(), output_topic_.c_str(), arrow_length_);
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

  std::string markerNamespace(std::size_t marker_idx) const
  {
    static constexpr std::array<const char *, 3> axis_names{{"x", "y", "z"}};
    return marker_namespace_ + "/" + axis_names[marker_idx % axis_names.size()];
  }

  void scoreCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scores_ = msg->data;
  }

  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray out;
    robot_sim::common::grasp::PoseAxisMarkerOptions options;
    options.marker_namespace = marker_namespace_;
    options.primary_axis_length = arrow_length_;
    options.primary_axis_idx = 0U;
    options.primary_axis_sign = -1.0;
    options.helper_axis_length_ratio = 0.5;
    options.shaft_diameter = shaft_diameter_;
    options.head_diameter = head_diameter_;
    options.alpha = color_a_;
    out = robot_sim::common::grasp::buildPoseAxisMarkerArray(*msg, options);
    const std::size_t marker_num = out.markers.size();
    for (std::size_t marker_idx = marker_num; marker_idx < last_marker_num_; ++marker_idx) {
      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = markerNamespace(marker_idx);
      marker.id = static_cast<int>(marker_idx);
      marker.action = visualization_msgs::msg::Marker::DELETE;
      out.markers.push_back(std::move(marker));
    }
    last_marker_num_ = marker_num;
    publisher_->publish(std::move(out));
  }

  std::string input_topic_;
  std::string score_topic_;
  std::string output_topic_;
  std::string marker_namespace_;
  double arrow_length_ = 0.04;
  double shaft_diameter_ = 0.006;
  double head_diameter_ = 0.008;
  double color_a_ = 1.0;
  std::size_t last_marker_num_ = 0;
  std::mutex mutex_;
  std::vector<float> latest_scores_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr score_subscription_;
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
