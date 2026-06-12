#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace robot_sim::bridge
{

class GraspPoseMarkerBridgeNode : public rclcpp::Node
{
public:
  explicit GraspPoseMarkerBridgeNode(const rclcpp::NodeOptions &options)
  : Node("grasp_pose_marker_bridge_node", options)
  {
    declare_parameter<std::string>("input_topic", "/grasp_pose_candidates");
    declare_parameter<std::string>("score_topic", "/grasp_pose_scores");
    declare_parameter<std::string>("output_topic", "/grasp_pose_markers");
    declare_parameter<std::string>("marker_namespace", "grasp_pose");
    declare_parameter<double>("arrow_length", 0.12);
    declare_parameter<double>("shaft_diameter", 0.01);
    declare_parameter<double>("head_diameter", 0.02);
    declare_parameter<double>("color_r", 0.0);
    declare_parameter<double>("color_g", 0.8);
    declare_parameter<double>("color_b", 0.2);
    declare_parameter<double>("color_a", 1.0);
    declare_parameter<bool>("use_score_color", true);
    declare_parameter<double>("score_min", 0.0);
    declare_parameter<double>("score_max", 1.0);
    declare_parameter<double>("low_score_r", 0.1);
    declare_parameter<double>("low_score_g", 0.2);
    declare_parameter<double>("low_score_b", 1.0);
    declare_parameter<double>("high_score_r", 1.0);
    declare_parameter<double>("high_score_g", 0.2);
    declare_parameter<double>("high_score_b", 0.1);

    input_topic_ = get_parameter("input_topic").as_string();
    score_topic_ = get_parameter("score_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    marker_namespace_ = get_parameter("marker_namespace").as_string();
    arrow_length_ = std::max(0.0001, get_parameter("arrow_length").as_double());
    shaft_diameter_ = std::max(0.0001, get_parameter("shaft_diameter").as_double());
    head_diameter_ = std::max(0.0001, get_parameter("head_diameter").as_double());
    color_r_ = std::clamp(get_parameter("color_r").as_double(), 0.0, 1.0);
    color_g_ = std::clamp(get_parameter("color_g").as_double(), 0.0, 1.0);
    color_b_ = std::clamp(get_parameter("color_b").as_double(), 0.0, 1.0);
    color_a_ = std::clamp(get_parameter("color_a").as_double(), 0.0, 1.0);
    use_score_color_ = get_parameter("use_score_color").as_bool();
    score_min_ = get_parameter("score_min").as_double();
    score_max_ = get_parameter("score_max").as_double();
    low_score_r_ = std::clamp(get_parameter("low_score_r").as_double(), 0.0, 1.0);
    low_score_g_ = std::clamp(get_parameter("low_score_g").as_double(), 0.0, 1.0);
    low_score_b_ = std::clamp(get_parameter("low_score_b").as_double(), 0.0, 1.0);
    high_score_r_ = std::clamp(get_parameter("high_score_r").as_double(), 0.0, 1.0);
    high_score_g_ = std::clamp(get_parameter("high_score_g").as_double(), 0.0, 1.0);
    high_score_b_ = std::clamp(get_parameter("high_score_b").as_double(), 0.0, 1.0);

    subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GraspPoseMarkerBridgeNode::poseArrayCallback, this, std::placeholders::_1));
    score_subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      score_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&GraspPoseMarkerBridgeNode::scoreCallback, this, std::placeholders::_1));

    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      output_topic_, rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(
      get_logger(),
      "GraspPoseMarkerBridgeNode initialized. input=%s output=%s count_arrow_length=%.4f",
      input_topic_.c_str(), output_topic_.c_str(), arrow_length_);
  }

private:
  void scoreCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scores_ = msg->data;
  }

  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray out;
    out.markers.reserve(msg->poses.size());

    std::vector<float> scores;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      scores = latest_scores_;
    }

    for (std::size_t i = 0; i < msg->poses.size(); ++i) {
      const auto &pose = msg->poses[i];
      const float score = i < scores.size() ? scores[i] : 0.0f;
      const auto [r, g, b] = scoreToRgb(score);

      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = marker_namespace_;
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      const double px = pose.position.x;
      const double py = pose.position.y;
      const double pz = pose.position.z;
      const double qx = pose.orientation.x;
      const double qy = pose.orientation.y;
      const double qz = pose.orientation.z;
      const double qw = pose.orientation.w;

      const Eigen::Quaterniond q(qw, qx, qy, qz);
      const Eigen::Vector3d direction = (q * Eigen::Vector3d::UnitX()).normalized();
      const Eigen::Vector3d origin(px, py, pz);
      const Eigen::Vector3d tip = origin + direction * arrow_length_;

      marker.pose.position.x = origin.x();
      marker.pose.position.y = origin.y();
      marker.pose.position.z = origin.z();
      marker.pose.orientation.w = 1.0;
      marker.points.resize(2);
      marker.points[0].x = origin.x();
      marker.points[0].y = origin.y();
      marker.points[0].z = origin.z();
      marker.points[1].x = tip.x();
      marker.points[1].y = tip.y();
      marker.points[1].z = tip.z();
      marker.scale.x = arrow_length_;
      marker.scale.y = shaft_diameter_;
      marker.scale.z = head_diameter_;
      marker.color.r = static_cast<float>(use_score_color_ ? r : color_r_);
      marker.color.g = static_cast<float>(use_score_color_ ? g : color_g_);
      marker.color.b = static_cast<float>(use_score_color_ ? b : color_b_);
      marker.color.a = static_cast<float>(color_a_);
      marker.frame_locked = false;
      out.markers.push_back(std::move(marker));
    }

    publisher_->publish(std::move(out));

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published grasp pose markers: input=%zu output=%zu frame=%s",
      msg->poses.size(), msg->poses.size(), msg->header.frame_id.c_str());
  }

  std::tuple<double, double, double> scoreToRgb(float score) const
  {
    const double denom = std::max(1e-6, score_max_ - score_min_);
    const double t = std::clamp((static_cast<double>(score) - score_min_) / denom, 0.0, 1.0);
    const double r = low_score_r_ + (high_score_r_ - low_score_r_) * t;
    const double g = low_score_g_ + (high_score_g_ - low_score_g_) * t;
    const double b = low_score_b_ + (high_score_b_ - low_score_b_) * t;
    return {r, g, b};
  }

  std::string input_topic_;
  std::string score_topic_;
  std::string output_topic_;
  std::string marker_namespace_;
  double arrow_length_ = 0.12;
  double shaft_diameter_ = 0.01;
  double head_diameter_ = 0.02;
  double color_r_ = 0.0;
  double color_g_ = 0.8;
  double color_b_ = 0.2;
  double color_a_ = 1.0;
  bool use_score_color_ = true;
  double score_min_ = 0.0;
  double score_max_ = 1.0;
  double low_score_r_ = 0.1;
  double low_score_g_ = 0.2;
  double low_score_b_ = 1.0;
  double high_score_r_ = 1.0;
  double high_score_g_ = 0.2;
  double high_score_b_ = 0.1;

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
