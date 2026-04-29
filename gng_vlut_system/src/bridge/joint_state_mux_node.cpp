#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace {

enum class JointStateSource {
  kRviz,
  kReal,
  kLatest,
};

JointStateSource parseSourceMode(const std::string &mode) {
  if (mode == "real") {
    return JointStateSource::kReal;
  }
  if (mode == "latest") {
    return JointStateSource::kLatest;
  }
  return JointStateSource::kRviz;
}

const char *sourceModeName(JointStateSource mode) {
  switch (mode) {
    case JointStateSource::kReal:
      return "real";
    case JointStateSource::kLatest:
      return "latest";
    case JointStateSource::kRviz:
    default:
      return "rviz";
  }
}

bool isLaterStamp(const builtin_interfaces::msg::Time &lhs,
                  const builtin_interfaces::msg::Time &rhs) {
  if (lhs.sec != rhs.sec) {
    return lhs.sec > rhs.sec;
  }
  return lhs.nanosec > rhs.nanosec;
}

}  // namespace

class JointStateMuxNode : public rclcpp::Node {
public:
  JointStateMuxNode()
  : Node("joint_state_mux_node") {
    rviz_topic_ = declare_parameter<std::string>("rviz_topic", "joint_states_rviz");
    real_topic_ = declare_parameter<std::string>("real_topic", "joint_states_real");
    output_topic_ = declare_parameter<std::string>("output_topic", "joint_states");
    active_source_ = declare_parameter<std::string>("active_source", "rviz");
    publish_hz_ = declare_parameter<double>("publish_hz", 20.0);

    output_pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, rclcpp::SystemDefaultsQoS());

    rviz_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        rviz_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_rviz_ = *msg;
          has_rviz_ = true;
        });

    real_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        real_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_real_ = *msg;
          has_real_ = true;
        });

    const auto period_ms = static_cast<int>(1000.0 / std::max(1.0, publish_hz_));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&JointStateMuxNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
                "joint_state_mux_node started: rviz_topic=%s real_topic=%s output_topic=%s active_source=%s",
                rviz_topic_.c_str(), real_topic_.c_str(), output_topic_.c_str(), active_source_.c_str());
  }

private:
  std::optional<sensor_msgs::msg::JointState> selectLatestLocked() const {
    const JointStateSource mode = parseSourceMode(active_source_);
    switch (mode) {
      case JointStateSource::kReal:
        return has_real_ ? std::optional<sensor_msgs::msg::JointState>(latest_real_) : std::nullopt;
      case JointStateSource::kLatest:
        if (!has_rviz_ && !has_real_) {
          return std::nullopt;
        }
        if (!has_rviz_) {
          return latest_real_;
        }
        if (!has_real_) {
          return latest_rviz_;
        }
        return isLaterStamp(latest_real_.header.stamp, latest_rviz_.header.stamp)
                   ? latest_real_
                   : latest_rviz_;
      case JointStateSource::kRviz:
      default:
        return has_rviz_ ? std::optional<sensor_msgs::msg::JointState>(latest_rviz_) : std::nullopt;
    }
  }

  void timerCallback() {
    const std::string current_mode = get_parameter("active_source").as_string();
    if (current_mode != active_source_) {
      active_source_ = current_mode;
      RCLCPP_INFO(get_logger(), "Active source switched to %s", sourceModeName(parseSourceMode(active_source_)));
    }

    sensor_msgs::msg::JointState msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto selected = selectLatestLocked();
      if (!selected.has_value()) {
        return;
      }
      msg = std::move(selected.value());
    }

    msg.header.stamp = now();
    output_pub_->publish(msg);
  }

  std::string rviz_topic_;
  std::string real_topic_;
  std::string output_topic_;
  std::string active_source_ = "rviz";
  double publish_hz_ = 20.0;

  mutable std::mutex mutex_;
  bool has_rviz_ = false;
  bool has_real_ = false;
  sensor_msgs::msg::JointState latest_rviz_;
  sensor_msgs::msg::JointState latest_real_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr output_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr rviz_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr real_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateMuxNode>());
  rclcpp::shutdown();
  return 0;
}
