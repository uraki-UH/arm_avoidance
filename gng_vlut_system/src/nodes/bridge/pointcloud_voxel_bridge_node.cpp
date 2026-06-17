#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "common/voxel_utils.hpp"
#include "core/common/constants.hpp"
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

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudVoxelBridgeNode::pointCloudCallback, this, std::placeholders::_1));

    publisher_ = create_publisher<voxel_msgs::msg::Voxel>(
      output_topic_, rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(
      get_logger(),
      "PointCloudVoxelBridgeNode initialized. input=%s output=%s target_frame=%s voxel_size=%.4f",
      input_topic_.c_str(), output_topic_.c_str(),
      target_frame_id_.c_str(), codec_.voxelSize());
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

  static Eigen::Vector3d centroidOfPoints(
    const std::vector<Eigen::Vector3f> &points)
  {
    if (points.empty()) {
      return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto &p : points) {
      sum += p.cast<double>();
    }
    return sum / static_cast<double>(points.size());
  }

  Eigen::Vector3d centroidOfVoxelCenters(
    const std::vector<long> &voxel_ids) const
  {
    if (voxel_ids.empty()) {
      return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const long flat_id : voxel_ids) {
      const auto idx = codec_.toIndex(flat_id);
      const auto center = voxel_idx::VoxelIndexingSchema::indexToWorldCenter(
        voxel_idx::VoxelIndex{idx.x(), idx.y(), idx.z()},
        codec_.voxelSize());
      sum += Eigen::Vector3d(center[0], center[1], center[2]);
    }
    return sum / static_cast<double>(voxel_ids.size());
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const bool missing_frame_id = msg->header.frame_id.empty();
    const std::string source_frame = resolveSourceFrameId(*msg);
    const bool has_cloud_stamp =
      msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;

    sensor_msgs::msg::PointCloud2 transformed_msg = *msg;

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

    RCLCPP_INFO(
      get_logger(),
      "Received point cloud from frame '%s'. Target frame is '%s'.",
      source_frame.c_str(), target_frame_id_.c_str());

    std::vector<Eigen::Vector3f> source_points;
    source_points.reserve(static_cast<size_t>(msg->width) * msg->height);
    {
      sensor_msgs::PointCloud2ConstIterator<float> sx(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> sy(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> sz(*msg, "z");
      for (; sx != sx.end(); ++sx, ++sy, ++sz) {
        source_points.emplace_back(*sx, *sy, *sz);
      }
    }

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
        const Eigen::Isometry3d T = tf2::transformToEigen(tf_msg.transform);

        sensor_msgs::PointCloud2ConstIterator<float> src_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> src_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> src_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<float> dst_x(transformed_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> dst_y(transformed_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> dst_z(transformed_msg, "z");

        for (; src_x != src_x.end(); ++src_x, ++src_y, ++src_z, ++dst_x, ++dst_y, ++dst_z) {
          const Eigen::Vector3d p_source(*src_x, *src_y, *src_z);
          const Eigen::Vector3d p_target = T * p_source;
          *dst_x = static_cast<float>(p_target.x());
          *dst_y = static_cast<float>(p_target.y());
          *dst_z = static_cast<float>(p_target.z());
        }
        RCLCPP_INFO(
          get_logger(),
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
    std::vector<Eigen::Vector3f> points;
    points.reserve(static_cast<size_t>(transformed_msg.width) * transformed_msg.height);

    sensor_msgs::PointCloud2ConstIterator<float> ix(transformed_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(transformed_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(transformed_msg, "z");

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      points.emplace_back(*ix, *iy, *iz);
    }

    RCLCPP_INFO(
      get_logger(),
      "Processed %zu points from input point cloud.",
      points.size());

    const auto voxel_ids = codec_.voxelize(points);

    const Eigen::Vector3d source_centroid = centroidOfPoints(source_points);
    const Eigen::Vector3d voxel_centroid_target = centroidOfVoxelCenters(voxel_ids);
    if (!voxel_ids.empty() && !target_frame_id_.empty() && source_frame != target_frame_id_) {
      try {
        geometry_msgs::msg::TransformStamped tf_msg;
        bool used_latest_fallback = false;
        if (!lookupPointCloudTransform(source_frame, *msg, tf_msg, used_latest_fallback)) {
          throw tf2::TransformException("lookup failed with both stamped and latest transforms");
        }
        const Eigen::Isometry3d T_source_to_target = tf2::transformToEigen(tf_msg.transform);
        const Eigen::Vector3d voxel_centroid_source =
          T_source_to_target.inverse() * voxel_centroid_target;
        const double centroid_error = (source_centroid - voxel_centroid_source).norm();

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Centroid check source=(%.4f, %.4f, %.4f) voxel_source=(%.4f, %.4f, %.4f) error=%.6f%s",
          source_centroid.x(), source_centroid.y(), source_centroid.z(),
          voxel_centroid_source.x(), voxel_centroid_source.y(), voxel_centroid_source.z(),
          centroid_error,
          used_latest_fallback ? " (latest tf fallback)" : "");
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Centroid check skipped because TF lookup failed: %s",
          ex.what());
      }
    }

    std_msgs::msg::Header header = transformed_msg.header;
    header.frame_id = target_frame_id_.empty() ? source_frame : target_frame_id_;
    if (!has_cloud_stamp) {
      header.stamp = now();
    }

    auto out = codec_.makeMessage(header, voxel_ids);
    publisher_->publish(std::move(out));

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published voxel ids: input=%zu output=%zu frame=%s",
      points.size(), voxel_ids.size(), header.frame_id.c_str());
  }

private:
  std::string input_topic_;
  std::string output_topic_;
  std::string source_frame_id_;
  std::string target_frame_id_;

  robot_sim::analysis::VoxelIdCodec codec_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr publisher_;
};

}  // namespace robot_sim::bridge

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
