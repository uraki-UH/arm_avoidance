#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>

#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GazeboEntityTfFollowerNode : public rclcpp::Node {
public:
  GazeboEntityTfFollowerNode() : Node("gazebo_entity_tf_follower_node") {
    entity_name_ = declare_parameter<std::string>("entity_name", "");
    source_frame_ = declare_parameter<std::string>("source_frame", "");
    reference_frame_ = declare_parameter<std::string>("reference_frame", "world");
    service_name_ = declare_parameter<std::string>("service_name", "/gazebo/set_entity_state");
    update_hz_ = std::max(1.0, declare_parameter<double>("update_hz", 20.0));

    if (entity_name_.empty()) {
      throw std::runtime_error("gazebo_entity_tf_follower_node: entity_name is required");
    }
    if (source_frame_.empty()) {
      throw std::runtime_error("gazebo_entity_tf_follower_node: source_frame is required");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    client_ = create_client<gazebo_msgs::srv::SetEntityState>(service_name_);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_hz_),
      std::bind(&GazeboEntityTfFollowerNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Gazebo TF follower ready: entity=%s source_frame=%s reference_frame=%s service=%s",
      entity_name_.c_str(),
      source_frame_.c_str(),
      reference_frame_.c_str(),
      service_name_.c_str());
  }

private:
  void timerCallback() {
    if (request_in_flight_.load(std::memory_order_relaxed)) {
      return;
    }

    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Waiting for Gazebo service %s",
        service_name_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(reference_frame_, source_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "TF lookup failed for %s -> %s: %s",
        reference_frame_.c_str(),
        source_frame_.c_str(),
        ex.what());
      return;
    }

    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = entity_name_;
    request->state.reference_frame = reference_frame_;
    request->state.pose.position.x = tf_msg.transform.translation.x;
    request->state.pose.position.y = tf_msg.transform.translation.y;
    request->state.pose.position.z = tf_msg.transform.translation.z;
    request->state.pose.orientation = tf_msg.transform.rotation;
    request->state.twist = geometry_msgs::msg::Twist();

    request_in_flight_.store(true, std::memory_order_relaxed);
    auto callback = [this](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture future) {
      request_in_flight_.store(false, std::memory_order_relaxed);
      try {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Gazebo rejected entity state update for %s",
            entity_name_.c_str());
        }
      } catch (const std::exception &ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Gazebo state update failed for %s: %s",
          entity_name_.c_str(),
          ex.what());
      }
    };

    client_->async_send_request(request, callback);
  }

  std::string entity_name_;
  std::string source_frame_;
  std::string reference_frame_;
  std::string service_name_;
  double update_hz_{20.0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> request_in_flight_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboEntityTfFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
