#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <map>

class JointStateMuxNode : public rclcpp::Node {
public:
  JointStateMuxNode() : Node("joint_state_mux_node") {
    const std::string output_topic = declare_parameter<std::string>("output_topic", "joint_states");
    active_source_ = declare_parameter<std::string>("active_source", "sim");
    const double hz = declare_parameter<double>("publish_hz", 20.0);

    const std::map<std::string, std::string> source_topics = {
      {"target", declare_parameter<std::string>("target_topic", "target_joint_states")},
      {"sim",    declare_parameter<std::string>("sim_topic", "sim_joint_states")},
      {"real",   declare_parameter<std::string>("real_topic", "real_joint_states")}
    };

    output_pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic, 10);

    for (const auto& [name, topic] : source_topics) {
      sources_[name].sub = create_subscription<sensor_msgs::msg::JointState>(
        topic, 10, [this, name](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          sources_[name].msg = *msg;
          sources_[name].has_data = true;
        });
    }

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
        [this]() {
          std::lock_guard<std::mutex> lock(mutex_);
          get_parameter("active_source", active_source_);
          if (sources_.count(active_source_) && sources_[active_source_].has_data) {
            auto out_msg = sources_[active_source_].msg;
            out_msg.header.stamp = now();
            output_pub_->publish(out_msg);
          }
        });

    RCLCPP_INFO(get_logger(), "JointStateMuxNode: %s source active", active_source_.c_str());
  }

private:
  struct Source {
    sensor_msgs::msg::JointState msg;
    bool has_data = false;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub;
  };

  std::map<std::string, Source> sources_;
  std::string active_source_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateMuxNode>());
  rclcpp::shutdown();
  return 0;
}
