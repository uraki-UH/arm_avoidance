#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class pointcloud_tf_transformer_node : public rclcpp::Node {
public:
  pointcloud_tf_transformer_node()
  : Node("pointcloud_tf_transformer_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/lidar_points");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/lidar_points_world");
    target_frame_ = declare_parameter<std::string>("target_frame", "world");

    if (input_topic.empty() || output_topic.empty() || target_frame_.empty()) {
      throw std::invalid_argument("PointCloud2変換用のtopicまたはtarget_frameが空です");
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { transform_pointcloud(msg); });

    RCLCPP_INFO(
      get_logger(), "PointCloud2 TF変換を開始: %s -> %s、target_frame=%s",
      input_topic.c_str(), output_topic.c_str(), target_frame_.c_str());
  }

private:
  void transform_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "入力PointCloud2のframe_idが空です");
      return;
    }

    try {
      const auto transform_stamped = tf_buffer_.lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero);
      tf2::Transform transform;
      tf2::fromMsg(transform_stamped.transform, transform);

      auto transformed_msg = *msg;
      sensor_msgs::PointCloud2Iterator<float> iter_x(transformed_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(transformed_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(transformed_msg, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const tf2::Vector3 point(*iter_x, *iter_y, *iter_z);
        const auto transformed_point = transform * point;
        *iter_x = static_cast<float>(transformed_point.x());
        *iter_y = static_cast<float>(transformed_point.y());
        *iter_z = static_cast<float>(transformed_point.z());
      }
      transformed_msg.header.frame_id = target_frame_;
      publisher_->publish(transformed_msg);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "PointCloud2のTF変換待ち: %s", error.what());
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "PointCloud2のxyz読み取り失敗: %s", error.what());
    }
  }

  std::string target_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_tf_transformer_node>());
  rclcpp::shutdown();
  return 0;
}
