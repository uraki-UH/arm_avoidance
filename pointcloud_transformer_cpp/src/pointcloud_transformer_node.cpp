#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Core>
#include "pointcloud_transformer_cpp/geometry_utils.hpp"

class PointCloudTransformerNode : public rclcpp::Node {
public:
  PointCloudTransformerNode() : Node("pointcloud_transformer") {
    // 1. Declare parameters with neutral defaults (actual values should come from YAML)
    this->declare_parameter<double>("x", 0.0);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<double>("z", 0.0);
    this->declare_parameter<double>("roll", 0.0);
    this->declare_parameter<double>("pitch", 0.0);
    this->declare_parameter<double>("yaw", 0.0);
    this->declare_parameter<std::string>("target_frame", "base_link");
    this->declare_parameter<std::string>("input_topic", "/camera/camera/depth/color/points");
    this->declare_parameter<std::string>("output_topic", "/camera/transformed_points");

    // 2. Initial transform calculation
    update_transform();

    // 3. Setup dynamic parameter update callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        // Trigger update if any relevant parameter is changed
        update_transform();
        return result;
      });

    // 4. Create Pub/Sub using parameter values
    std::string in_topic = get_parameter("input_topic").as_string();
    std::string out_topic = get_parameter("output_topic").as_string();

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, 10);
    
    rclcpp::QoS qos(1);
    qos.best_effort();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic, qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
        
        // Use cached rotation matrix and translation vector
        sensor_msgs::PointCloud2Iterator<float> iter_x(*out_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*out_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*out_msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
          double px = *iter_x;
          double py = *iter_y;
          double pz = *iter_z;
          
          *iter_x = static_cast<float>(R_(0,0)*px + R_(0,1)*py + R_(0,2)*pz + t_.x());
          *iter_y = static_cast<float>(R_(1,0)*px + R_(1,1)*py + R_(1,2)*pz + t_.y());
          *iter_z = static_cast<float>(R_(2,0)*px + R_(2,1)*py + R_(2,2)*pz + t_.z());
        }

        out_msg->header.stamp = msg->header.stamp;
        out_msg->header.frame_id = target_frame_;

        pub_->publish(*out_msg);
      });

    RCLCPP_INFO(this->get_logger(), "Initialized with input: %s, output: %s, target: %s", 
                in_topic.c_str(), out_topic.c_str(), target_frame_.c_str());
  }

private:
  void update_transform() {
    double x = get_parameter("x").as_double();
    double y = get_parameter("y").as_double();
    double z = get_parameter("z").as_double();
    double roll = get_parameter("roll").as_double();
    double pitch = get_parameter("pitch").as_double();
    double yaw = get_parameter("yaw").as_double();
    target_frame_ = get_parameter("target_frame").as_string();

    Eigen::Matrix4d T = pointcloud_transformer_cpp::utils::make_raw_matrix(x, y, z, roll, pitch, yaw);
    R_ = T.block<3,3>(0,0);
    t_ = T.block<3,1>(0,3);

    RCLCPP_INFO(this->get_logger(), "Transform updated from parameters.");
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Cached transform data
  Eigen::Matrix3d R_;
  Eigen::Vector3d t_;
  std::string target_frame_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformerNode>());
  rclcpp::shutdown();
  return 0;
}
