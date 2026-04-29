#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <mutex>
#include <string>
#include <utility>

class JointStateMuxNode : public rclcpp::Node {
public:
  JointStateMuxNode()
  : Node("joint_state_mux_node") {
    // Parameters
    target_topic_ = declare_parameter<std::string>("target_topic", "target_joint_states");
    sim_topic_ = declare_parameter<std::string>("sim_topic", "sim_joint_states");
    real_topic_ = declare_parameter<std::string>("real_topic", "real_joint_states");
    output_topic_ = declare_parameter<std::string>("output_topic", "joint_states");
    active_source_ = declare_parameter<std::string>("active_source", "sim");
    publish_hz_ = declare_parameter<double>("publish_hz", 20.0);

    // Publisher
    output_pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, rclcpp::SystemDefaultsQoS());

    // Subscriptions
    target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        target_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_target_ = *msg;
          has_target_ = true;
        });

    sim_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        sim_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_sim_ = *msg;
          has_sim_ = true;
        });

    real_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        real_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_real_ = *msg;
          has_real_ = true;
        });

    // Timer
    const auto period_ms = static_cast<int>(1000.0 / std::max(1.0, publish_hz_));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&JointStateMuxNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
                "JointStateMuxNode initialized.\n"
                " - Target: %s\n"
                " - Sim:    %s\n"
                " - Real:   %s\n"
                " - Output: %s\n"
                " - Active: %s",
                target_topic_.c_str(), sim_topic_.c_str(), real_topic_.c_str(), 
                output_topic_.c_str(), active_source_.c_str());
  }

private:
  void timerCallback() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Periodically update active_source from parameters
    get_parameter("active_source", active_source_);

    sensor_msgs::msg::JointState out_msg;
    bool has_data = false;

    if (active_source_ == "target") {
      if (has_target_) { out_msg = latest_target_; has_data = true; }
    } else if (active_source_ == "real") {
      if (has_real_) { out_msg = latest_real_; has_data = true; }
    } else if (active_source_ == "sim") {
      if (has_sim_) { out_msg = latest_sim_; has_data = true; }
    }

    if (has_data) {
      out_msg.header.stamp = this->now();
      output_pub_->publish(out_msg);
    }
  }

  // Configuration
  std::string target_topic_, sim_topic_, real_topic_, output_topic_;
  std::string active_source_;
  double publish_hz_;

  // State
  sensor_msgs::msg::JointState latest_target_, latest_sim_, latest_real_;
  bool has_target_ = false;
  bool has_sim_ = false;
  bool has_real_ = false;

  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr output_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_, sim_sub_, real_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateMuxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
