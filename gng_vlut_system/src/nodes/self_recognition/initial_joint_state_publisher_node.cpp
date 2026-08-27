#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

class InitialJointStatePublisherNode : public rclcpp::Node {
public:
  InitialJointStatePublisherNode() : Node("initial_joint_state_publisher_node") {
    const auto description = declare_parameter<std::string>("robot_description", "");
    const auto topic = declare_parameter<std::string>("joint_state_topic", "joint_states");

    const auto model = urdf::parseURDF(description);
    if (!model) {
      throw std::runtime_error("Failed to parse robot_description for initial joint state");
    }

    for (const auto &[name, joint] : model->joints_) {
      if (joint && joint->type != urdf::Joint::FIXED) {
        joint_names_.push_back(name);
      }
    }
    if (joint_names_.empty()) {
      throw std::runtime_error("Robot description has no movable joints");
    }

    publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        topic, rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publishOnce(); });
  }

private:
  void publishOnce() {
    // Wait for robot_state_publisher so the one-shot seed is not lost at startup.
    if (publisher_->get_subscription_count() == 0) {
      return;
    }

    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    state.name = joint_names_;
    state.position.assign(joint_names_.size(), 0.0);
    publisher_->publish(state);
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Published zero initial state for %zu joints on %s",
                joint_names_.size(), publisher_->get_topic_name());
  }

  std::vector<std::string> joint_names_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialJointStatePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
