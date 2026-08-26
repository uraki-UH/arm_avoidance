#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Core>
#include <cstdint>
#include "pointcloud_transformer_cpp/geometry_utils.hpp"

class PointCloudTransformerNode : public rclcpp::Node {
public:
  PointCloudTransformerNode() : Node("pointcloud_transformer") {
    // 1. ループで一括宣言
    for (const auto & name : {"x", "y", "z", "roll", "pitch", "yaw"}) {
      this->declare_parameter<double>(name, 0.0);
    }
    this->declare_parameter<std::string>("target_frame", "base_link");
    this->declare_parameter<std::string>("input_topic", "/camera/camera/depth/color/points");
    this->declare_parameter<std::string>("output_topic", "/camera/transformed_points");
    this->declare_parameter<std::int64_t>("input_queue_depth", 1);
    this->declare_parameter<bool>("reliable_input", false);

    // 2. 初期計算
    update_transform();

    // 3. パラメータ更新コールバック
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {
        update_transform();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        return result;
      });

    // 4. Pub/Sub 作成
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(get_parameter("output_topic").as_string(), 10);
    
    const auto configured_queue_depth = get_parameter("input_queue_depth").as_int();
    const auto input_queue_depth = static_cast<std::size_t>(
      configured_queue_depth > 0 ? configured_queue_depth : 1);
    rclcpp::QoS qos(input_queue_depth);
    if (get_parameter("reliable_input").as_bool()) {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("input_topic").as_string(), qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
        sensor_msgs::PointCloud2Iterator<float> iter_x(*out_msg, "x"), iter_y(*out_msg, "y"), iter_z(*out_msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
          double px = *iter_x, py = *iter_y, pz = *iter_z;
          *iter_x = static_cast<float>(R_(0,0)*px + R_(0,1)*py + R_(0,2)*pz + t_.x());
          *iter_y = static_cast<float>(R_(1,0)*px + R_(1,1)*py + R_(1,2)*pz + t_.y());
          *iter_z = static_cast<float>(R_(2,0)*px + R_(2,1)*py + R_(2,2)*pz + t_.z());
        }
        out_msg->header.stamp = msg->header.stamp;
        out_msg->header.frame_id = target_frame_;
        pub_->publish(*out_msg);
      });
  }

private:
  void update_transform() {
    // 短いヘルパーを定義して1行で渡す
    auto get_d = [this](const std::string & n) { return get_parameter(n).as_double(); };
    
    Eigen::Matrix4d T = pointcloud_transformer_cpp::utils::make_raw_matrix(
      get_d("x"), get_d("y"), get_d("z"), get_d("roll"), get_d("pitch"), get_d("yaw")
    );

    R_ = T.block<3,3>(0,0);
    t_ = T.block<3,1>(0,3);
    target_frame_ = get_parameter("target_frame").as_string();

    RCLCPP_INFO(this->get_logger(), "Transform updated.");
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
