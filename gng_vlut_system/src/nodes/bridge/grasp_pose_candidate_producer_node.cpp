#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "common/voxel_utils.hpp"
#include "common/grasp_pose_utils.hpp"
#include "core/common/constants.hpp"
#include "safety_engine/indexing/voxel_id_codec.hpp"

namespace robot_sim::bridge
{

class GraspPoseCandidateProducerNode : public rclcpp::Node
{
public:
  explicit GraspPoseCandidateProducerNode(const rclcpp::NodeOptions &options)
  : Node("grasp_pose_candidate_producer_node", options),
    codec_(::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE)
  {
    declare_parameter<std::string>("input_topic", "/topo_points");
    declare_parameter<std::string>("pose_topic", "/grasp_pose_candidates");
    declare_parameter<std::string>("score_topic", "/grasp_pose_scores");
    declare_parameter<std::string>("target_frame_id", "world");
    declare_parameter<double>("voxel_size", ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
    declare_parameter<int>("x_shift", 42);
    declare_parameter<int>("y_shift", 21);
    declare_parameter<int>("z_shift", 0);
    declare_parameter<int>("offset", 1000000);
    declare_parameter<int>("max_candidates", 256);
    declare_parameter<int>("min_points_per_voxel", 1);
    declare_parameter<double>("approach_offset", 0.06);

    input_topic_ = get_parameter("input_topic").as_string();
    pose_topic_ = get_parameter("pose_topic").as_string();
    score_topic_ = get_parameter("score_topic").as_string();
    target_frame_id_ = get_parameter("target_frame_id").as_string();
    codec_.setVoxelSize(get_parameter("voxel_size").as_double());
    codec_.setIndexingParams(
      get_parameter("x_shift").as_int(),
      get_parameter("y_shift").as_int(),
      get_parameter("z_shift").as_int(),
      static_cast<long>(get_parameter("offset").as_int()));
    max_candidates_ = std::max<std::size_t>(
      1U, static_cast<std::size_t>(get_parameter("max_candidates").as_int()));
    min_points_per_voxel_ = std::max<std::size_t>(
      1U, static_cast<std::size_t>(get_parameter("min_points_per_voxel").as_int()));
    approach_offset_ = std::max(0.0, get_parameter("approach_offset").as_double());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GraspPoseCandidateProducerNode::pointCloudCallback, this, std::placeholders::_1));

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      pose_topic_, rclcpp::QoS(1).reliable().transient_local());
    score_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      score_topic_, rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(
      get_logger(),
      "GraspPoseCandidateProducerNode initialized. input=%s pose=%s score=%s target_frame=%s voxel_size=%.4f",
      input_topic_.c_str(), pose_topic_.c_str(), score_topic_.c_str(), target_frame_id_.c_str(),
      codec_.voxelSize());
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received point cloud with empty frame_id.");
      return;
    }

    Eigen::Isometry3d source_to_target = Eigen::Isometry3d::Identity();
    const std::string source_frame = msg->header.frame_id;
    if (!target_frame_id_.empty() && target_frame_id_ != source_frame) {
      try {
        const auto tf_msg = tf_buffer_->lookupTransform(
          target_frame_id_, source_frame, tf2::TimePointZero);
        source_to_target = tf2::transformToEigen(tf_msg.transform);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TF lookup failed target='%s' source='%s': %s. Falling back to source frame.",
          target_frame_id_.c_str(), source_frame.c_str(), ex.what());
        source_to_target = Eigen::Isometry3d::Identity();
      }
    }

    std::unordered_map<long, robot_sim::common::grasp::VoxelObservation> voxel_stats;
    voxel_stats.reserve(static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    std::size_t input_count = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const Eigen::Vector3d p_source(*iter_x, *iter_y, *iter_z);
      const Eigen::Vector3d p_target = source_to_target * p_source;
      const auto idx = ::common::geometry::VoxelUtils::worldToVoxel(
        p_target.cast<float>(), static_cast<float>(codec_.voxelSize()));
      const long vid = codec_.toFlatId(idx);
      auto &stats = voxel_stats[vid];
      ++stats.count;
      stats.sum += p_target;
      ++input_count;
    }

    const std::vector<robot_sim::common::grasp::Candidate> candidates =
      robot_sim::common::grasp::buildSimpleCandidates(
        voxel_stats, codec_, min_points_per_voxel_, max_candidates_, approach_offset_);

    if (candidates.empty()) {
      publishEmpty(msg->header);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No grasp candidates from point cloud: input=%zu", input_count);
      return;
    }

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = msg->header;
    pose_array.header.frame_id = target_frame_id_.empty() ? source_frame : target_frame_id_;
    pose_array.poses.reserve(candidates.size());

    std_msgs::msg::Float32MultiArray scores;
    scores.data.reserve(candidates.size());

    for (const auto &candidate : candidates) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = candidate.position.x();
      pose.position.y = candidate.position.y();
      pose.position.z = candidate.position.z();
      pose.orientation.x = candidate.orientation.x();
      pose.orientation.y = candidate.orientation.y();
      pose.orientation.z = candidate.orientation.z();
      pose.orientation.w = candidate.orientation.w();

      pose_array.poses.push_back(pose);
      scores.data.push_back(static_cast<float>(candidate.score));
    }

    pose_publisher_->publish(pose_array);
    score_publisher_->publish(scores);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published grasp pose candidates: input=%zu candidates=%zu frame=%s",
      input_count, candidates.size(), pose_array.header.frame_id.c_str());

    if (!candidates.empty()) {
      std::ostringstream oss;
      oss << "Top grasp candidates:";
      const std::size_t preview_count = std::min<std::size_t>(3U, candidates.size());
      for (std::size_t i = 0; i < preview_count; ++i) {
        const auto &candidate = candidates[i];
        oss << " ["
            << i
            << "] vid=" << candidate.voxel_id
            << " score=" << std::fixed << std::setprecision(3) << candidate.score
            << " pos=(" << std::setprecision(3)
            << candidate.position.x() << ","
            << candidate.position.y() << ","
            << candidate.position.z() << ")"
            << " count=" << candidate.count
            << " neigh=" << candidate.occupied_neighbors;
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", oss.str().c_str());
    }
  }

  void publishEmpty(const std_msgs::msg::Header &header)
  {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = header;
    pose_array.header.frame_id = target_frame_id_.empty() ? header.frame_id : target_frame_id_;
    pose_publisher_->publish(pose_array);

    std_msgs::msg::Float32MultiArray scores;
    score_publisher_->publish(scores);
  }

  std::string input_topic_;
  std::string pose_topic_;
  std::string score_topic_;
  std::string target_frame_id_;
  std::size_t max_candidates_ = 256;
  std::size_t min_points_per_voxel_ = 1;
  double approach_offset_ = 0.06;

  robot_sim::analysis::VoxelIdCodec codec_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr score_publisher_;
};

}  // namespace robot_sim::bridge

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::GraspPoseCandidateProducerNode)

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::bridge::GraspPoseCandidateProducerNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
