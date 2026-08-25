#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "core/common/constants.hpp"
#include "nodes/bridge/reachability_voxel_accumulator.hpp"
#include "safety_engine/indexing/voxel_id_codec.hpp"

namespace robot_sim::bridge
{

class PointCloudVoxelBridgeNode : public rclcpp::Node
{
public:
  explicit PointCloudVoxelBridgeNode(const rclcpp::NodeOptions &options)
  : Node("pointcloud_voxel_bridge_node", options),
    codec_(::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE)
  {
    declare_parameter<std::string>("input_topic", "/points");
    declare_parameter<std::string>("output_topic", "/voxel_ids");
    declare_parameter<std::string>("source_frame_id", "");
    declare_parameter<std::string>("target_frame_id", "world");
    declare_parameter<double>("voxel_size", ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
    declare_parameter<int>("x_shift", 42);
    declare_parameter<int>("y_shift", 21);
    declare_parameter<int>("z_shift", 0);
    declare_parameter<long>("offset", 1000000L);
    declare_parameter<bool>("enable_reachability_filter", false);
    declare_parameter<double>("min_reachability_x", -0.1);
    declare_parameter<double>("max_reachability_x", 0.5);
    declare_parameter<double>("min_reachability_y", -1.0);
    declare_parameter<double>("max_reachability_y", 1.0);
    declare_parameter<double>("min_reachability_z", -1.0);
    declare_parameter<double>("max_reachability_z", 1.0);
    declare_parameter<double>("reachability_margin_x", 0.2);
    declare_parameter<double>("reachability_margin_y", 0.2);
    declare_parameter<double>("reachability_margin_z", 0.2);
    declare_parameter<int>("max_dense_voxel_num", 8000000);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    source_frame_id_ = get_parameter("source_frame_id").as_string();
    target_frame_id_ = get_parameter("target_frame_id").as_string();

    const double voxel_size = get_parameter("voxel_size").as_double();

    codec_.setVoxelSize(voxel_size);
    codec_.setIndexingParams(
      get_parameter("x_shift").as_int(),
      get_parameter("y_shift").as_int(),
      get_parameter("z_shift").as_int(),
      static_cast<long>(get_parameter("offset").as_int()));

    reachability_bounds_.enable_filter =
      get_parameter("enable_reachability_filter").as_bool();
    reachability_bounds_.min_corner = Eigen::Vector3d(
      get_parameter("min_reachability_x").as_double(),
      get_parameter("min_reachability_y").as_double(),
      get_parameter("min_reachability_z").as_double());
    reachability_bounds_.max_corner = Eigen::Vector3d(
      get_parameter("max_reachability_x").as_double(),
      get_parameter("max_reachability_y").as_double(),
      get_parameter("max_reachability_z").as_double());
    reachability_bounds_.margin = Eigen::Vector3d(
      get_parameter("reachability_margin_x").as_double(),
      get_parameter("reachability_margin_y").as_double(),
      get_parameter("reachability_margin_z").as_double());
    try {
      reachability_bounds_.validate();
    } catch (const std::invalid_argument &ex) {
      throw rclcpp::exceptions::InvalidParametersException(ex.what());
    }
    const auto max_dense_voxel_num = static_cast<std::size_t>(
      std::max<std::int64_t>(0, get_parameter("max_dense_voxel_num").as_int()));
    voxel_accumulator_ = std::make_unique<reachability_voxel_accumulator>(
      codec_, reachability_bounds_, max_dense_voxel_num);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudVoxelBridgeNode::pointCloudCallback, this, std::placeholders::_1));

    publisher_ = create_publisher<voxel_msgs::msg::Voxel>(
      output_topic_, rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(
      get_logger(),
      "PointCloudVoxelBridgeNode initialized. input=%s output=%s target_frame=%s voxel_size=%.4f reachability_filter=%s margin=(%.3f, %.3f, %.3f) accumulator=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      target_frame_id_.c_str(), codec_.voxelSize(),
      reachability_bounds_.enable_filter ? "enabled" : "disabled",
      reachability_bounds_.margin.x(), reachability_bounds_.margin.y(),
      reachability_bounds_.margin.z(),
      voxel_accumulator_->uses_dense_bitmap() ? "dense_bitmap" : "hash");
  }

private:
  std::string resolveSourceFrameId(const sensor_msgs::msg::PointCloud2 &msg) const
  {
    if (!source_frame_id_.empty()) {
      return source_frame_id_;
    }
    if (msg.header.frame_id.empty() || msg.header.frame_id == "map") {
      return "world";
    }
    if (!msg.header.frame_id.empty()) {
      return msg.header.frame_id;
    }
    return "world";
  }

  bool shouldSkipPointCloudTransform(
    const std::string &source_frame,
    const sensor_msgs::msg::PointCloud2 &) const
  {
    if (target_frame_id_.empty() || source_frame == target_frame_id_) {
      return true;
    }
    return false;
  }

  bool lookupPointCloudTransform(
    const std::string &source_frame,
    const sensor_msgs::msg::PointCloud2 &msg,
    geometry_msgs::msg::TransformStamped &tf_msg,
    bool &used_latest_fallback) const
  {
    const bool has_cloud_stamp =
      msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0;
    used_latest_fallback = false;

    try {
      tf_msg = has_cloud_stamp
        ? tf_buffer_->lookupTransform(
            target_frame_id_,
            source_frame,
            rclcpp::Time(msg.header.stamp))
        : tf_buffer_->lookupTransform(
            target_frame_id_,
            source_frame,
            tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException &) {
      if (has_cloud_stamp) {
        try {
          tf_msg = tf_buffer_->lookupTransform(
            target_frame_id_,
            source_frame,
            tf2::TimePointZero);
          used_latest_fallback = true;
          return true;
        } catch (const tf2::TransformException &) {
          return false;
        }
      }
      return false;
    }
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto processing_start = std::chrono::steady_clock::now();
    const bool missing_frame_id = msg->header.frame_id.empty();
    const std::string source_frame = resolveSourceFrameId(*msg);
    const bool has_cloud_stamp =
      msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;

    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received point cloud with empty frame_id and no fallback source frame could be selected.");
      return;
    }

    if (missing_frame_id || msg->header.frame_id == "map") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received point cloud with placeholder/empty frame_id '%s'. Treating it as source frame '%s' from the origin.",
        msg->header.frame_id.c_str(), source_frame.c_str());
    } else if (!source_frame_id_.empty() && source_frame_id_ != msg->header.frame_id) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Overriding point cloud frame '%s' with configured source frame '%s'.",
        msg->header.frame_id.c_str(), source_frame.c_str());
    }

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Received point cloud from frame '%s'. Target frame is '%s'.",
      source_frame.c_str(), target_frame_id_.c_str());

    Eigen::Isometry3d source_to_target = Eigen::Isometry3d::Identity();
    try {
      if (!shouldSkipPointCloudTransform(source_frame, *msg)) {
        geometry_msgs::msg::TransformStamped tf_msg;
        bool used_latest_fallback = false;
        if (!lookupPointCloudTransform(source_frame, *msg, tf_msg, used_latest_fallback)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "TF lookup failed target='%s' source='%s'. Skipping voxelization for this cloud.",
            target_frame_id_.c_str(), source_frame.c_str());
          return;
        }
        source_to_target = tf2::transformToEigen(tf_msg.transform);
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Successfully looked up transform from '%s' to '%s'%s.",
          source_frame.c_str(), target_frame_id_.c_str(),
          used_latest_fallback ? " using latest fallback" : "");
      }
    }
    catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF lookup failed target='%s' source='%s': %s. Skipping voxelization for this cloud.",
        target_frame_id_.c_str(), source_frame.c_str(), ex.what());
      return;
    }
    voxel_accumulator_->begin_frame(static_cast<std::size_t>(msg->width) * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      voxel_accumulator_->add_point(Eigen::Vector3d(*ix, *iy, *iz), source_to_target);
    }
    const auto stats = voxel_accumulator_->stats();
    const auto &voxel_ids = voxel_accumulator_->finish_voxel_ids();

    std_msgs::msg::Header header = msg->header;
    header.frame_id = target_frame_id_.empty() ? source_frame : target_frame_id_;
    if (!has_cloud_stamp) {
      header.stamp = now();
    }

    auto out = codec_.makeMessage(header, voxel_ids);
    publisher_->publish(std::move(out));
    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_start).count();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published voxel ids: input=%zu accepted=%zu outside=%zu nonfinite=%zu output=%zu processing_ms=%.3f frame=%s",
      stats.input_point_count, stats.accepted_point_count, stats.outside_point_count,
      stats.nonfinite_point_count, voxel_ids.size(), processing_ms, header.frame_id.c_str());
  }

private:
  std::string input_topic_;
  std::string output_topic_;
  std::string source_frame_id_;
  std::string target_frame_id_;
  reachability_bounds reachability_bounds_;

  robot_sim::analysis::VoxelIdCodec codec_;
  std::unique_ptr<reachability_voxel_accumulator> voxel_accumulator_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr publisher_;
};

}  // robot_sim::bridge namespace終端

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::PointCloudVoxelBridgeNode)

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::bridge::PointCloudVoxelBridgeNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
