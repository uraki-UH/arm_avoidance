#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "common/grasp_pose_utils.hpp"

namespace robot_sim::bridge
{

class GraspPoseDummyPublisherNode : public rclcpp::Node
{
public:
  explicit GraspPoseDummyPublisherNode(const rclcpp::NodeOptions &options)
  : Node("grasp_pose_dummy_publisher_node", options)
  {
    declare_parameter<std::string>("pose_topic", "/grasp_pose_candidates");
    declare_parameter<std::string>("score_topic", "/grasp_pose_scores");
    declare_parameter<std::string>("marker_topic", "/grasp_pose_markers");
    declare_parameter<std::string>("frame_id", "world");
    declare_parameter<double>("publish_rate_hz", 1.0);
    declare_parameter<int>("candidate_count", 6);
    declare_parameter<double>("center_x", 0.0);
    declare_parameter<double>("center_y", 0.0);
    declare_parameter<double>("center_z", 0.15);
    declare_parameter<double>("spread_x", 0.05);
    declare_parameter<double>("spread_y", 0.05);
    declare_parameter<double>("spread_z", 0.0);
    declare_parameter<double>("base_yaw_deg", 0.0);

    pose_topic_ = get_parameter("pose_topic").as_string();
    score_topic_ = get_parameter("score_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    publish_rate_hz_ = std::max(0.1, get_parameter("publish_rate_hz").as_double());
    candidate_count_ = std::max(1, static_cast<int>(get_parameter("candidate_count").as_int()));
    center_x_ = get_parameter("center_x").as_double();
    center_y_ = get_parameter("center_y").as_double();
    center_z_ = get_parameter("center_z").as_double();
    spread_x_ = get_parameter("spread_x").as_double();
    spread_y_ = get_parameter("spread_y").as_double();
    spread_z_ = get_parameter("spread_z").as_double();
    base_yaw_deg_ = get_parameter("base_yaw_deg").as_double();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      pose_topic_, rclcpp::QoS(1).reliable().transient_local());
    score_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      score_topic_, rclcpp::QoS(1).reliable().transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1).reliable().transient_local());

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GraspPoseDummyPublisherNode::publishDummyPoses, this));

    RCLCPP_INFO(
      get_logger(),
      "GraspPoseDummyPublisherNode initialized. pose=%s score=%s frame=%s count=%d rate=%.2f",
      pose_topic_.c_str(),
      score_topic_.c_str(),
      frame_id_.c_str(),
      candidate_count_,
      publish_rate_hz_);
  }

private:
  void publishDummyPoses()
  {
    constexpr double kPi = 3.14159265358979323846;
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = now();
    pose_array.header.frame_id = frame_id_;
    pose_array.poses.reserve(static_cast<std::size_t>(candidate_count_));

    std_msgs::msg::Float32MultiArray scores;
    scores.data.reserve(static_cast<std::size_t>(candidate_count_));

    const double yaw_base = base_yaw_deg_ * kPi / 180.0;
    const double angle_step = candidate_count_ > 0 ? (2.0 * kPi / static_cast<double>(candidate_count_)) : 0.0;

    for (int i = 0; i < candidate_count_; ++i) {
      const double theta = yaw_base + static_cast<double>(i) * angle_step;
      const double x = center_x_ + spread_x_ * std::cos(theta);
      const double y = center_y_ + spread_y_ * std::sin(theta);
      const double z = center_z_ + std::sin(theta * 0.5) * spread_z_;
      const Eigen::Vector3d position(x, y, z);
      const Eigen::Vector3d center(center_x_, center_y_, center_z_);
      const Eigen::Vector3d inward = robot_sim::common::grasp::safeNormalize(center - position, Eigen::Vector3d::UnitX());
      const Eigen::Quaterniond orientation = robot_sim::common::grasp::makeApproachOrientation(inward);

      geometry_msgs::msg::Pose pose;
      pose.position.x = position.x();
      pose.position.y = position.y();
      pose.position.z = position.z();
      pose.orientation.x = orientation.x();
      pose.orientation.y = orientation.y();
      pose.orientation.z = orientation.z();
      pose.orientation.w = orientation.w();

      pose_array.poses.push_back(pose);
      const double radial_bias = 1.0 - std::abs(std::sin(theta));
      scores.data.push_back(static_cast<float>(0.5 + 0.5 * radial_bias));
    }

    pose_pub_->publish(pose_array);
    score_pub_->publish(scores);
    marker_pub_->publish(robot_sim::common::grasp::buildPoseAxisMarkerArray(pose_array));
  }

  std::string pose_topic_;
  std::string score_topic_;
  std::string marker_topic_;
  std::string frame_id_;
  double publish_rate_hz_ = 1.0;
  int candidate_count_ = 6;
  double center_x_ = 0.0;
  double center_y_ = 0.0;
  double center_z_ = 0.15;
  double spread_x_ = 0.05;
  double spread_y_ = 0.05;
  double spread_z_ = 0.0;
  double base_yaw_deg_ = 0.0;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr score_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_sim::bridge

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::GraspPoseDummyPublisherNode)

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::bridge::GraspPoseDummyPublisherNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
