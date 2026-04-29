#include <algorithm>
#include <cctype>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace {

enum class JointStateSource {
  kSim,
  kReal,
  kTest,
  kLatest,
  kPriority,
};

JointStateSource parseSourceMode(const std::string &mode) {
  if (mode == "sim") {
    return JointStateSource::kSim;
  }
  if (mode == "real") {
    return JointStateSource::kReal;
  }
  if (mode == "test") {
    return JointStateSource::kTest;
  }
  if (mode == "latest") {
    return JointStateSource::kLatest;
  }
  return JointStateSource::kPriority;
}

const char *sourceModeName(JointStateSource mode) {
  switch (mode) {
    case JointStateSource::kSim:
      return "sim";
    case JointStateSource::kReal:
      return "real";
    case JointStateSource::kTest:
      return "test";
    case JointStateSource::kLatest:
      return "latest";
    case JointStateSource::kPriority:
    default:
      return "priority";
  }
}

std::string trimCopy(const std::string& value) {
  size_t begin = 0;
  while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }
  size_t end = value.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(begin, end - begin);
}

}  // namespace

class JointStateMuxNode : public rclcpp::Node {
public:
  JointStateMuxNode()
  : Node("joint_state_mux_node") {
    sim_topic_ = declare_parameter<std::string>("sim_topic", "/joint_states_sim");
    real_topic_ = declare_parameter<std::string>("real_topic", "/joint_states_real");
    test_topic_ = declare_parameter<std::string>("test_topic", "/joint_states_test");
    output_topic_ = declare_parameter<std::string>("output_topic", "/joint_states");
    active_source_ = declare_parameter<std::string>("active_source", "priority");
    source_priority_ = declare_parameter<std::vector<std::string>>(
        "source_priority", std::vector<std::string>{"real", "test", "sim"});
    max_age_sec_ = declare_parameter<double>("max_age_sec", 0.5);
    publish_hz_ = declare_parameter<double>("publish_hz", 20.0);

    output_pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, rclcpp::SystemDefaultsQoS());

    sim_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        sim_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_sim_ = *msg;
          has_sim_ = true;
          sim_received_at_ = this->now();
        });

    real_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        real_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_real_ = *msg;
          has_real_ = true;
          real_received_at_ = this->now();
        });

    test_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        test_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_test_ = *msg;
          has_test_ = true;
          test_received_at_ = this->now();
        });

    const auto period_ms = static_cast<int>(1000.0 / std::max(1.0, publish_hz_));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&JointStateMuxNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
                "joint_state_mux_node started: sim_topic=%s real_topic=%s test_topic=%s output_topic=%s active_source=%s",
                sim_topic_.c_str(), real_topic_.c_str(), test_topic_.c_str(), output_topic_.c_str(), active_source_.c_str());
  }

private:
  struct SourceSlot {
    bool has_value{false};
    sensor_msgs::msg::JointState msg;
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
  };

  bool isFresh(const rclcpp::Time& received_at) const {
    if (max_age_sec_ <= 0.0) {
      return true;
    }
    const auto age = (now() - received_at).seconds();
    return age >= 0.0 && age <= max_age_sec_;
  }

  static std::optional<std::pair<std::string, SourceSlot>> getSlotByName(
      const std::string& source,
      const SourceSlot& sim,
      const SourceSlot& real,
      const SourceSlot& test) {
    const std::string normalized = trimCopy(source);
    if (normalized == "sim") {
      return std::make_pair(std::string("sim"), sim);
    }
    if (normalized == "real") {
      return std::make_pair(std::string("real"), real);
    }
    if (normalized == "test") {
      return std::make_pair(std::string("test"), test);
    }
    return std::nullopt;
  }

  std::optional<std::pair<std::string, sensor_msgs::msg::JointState>> selectLatestLocked() const {
    const SourceSlot sim{has_sim_, latest_sim_, sim_received_at_};
    const SourceSlot real{has_real_, latest_real_, real_received_at_};
    const SourceSlot test{has_test_, latest_test_, test_received_at_};
    const JointStateSource mode = parseSourceMode(active_source_);
    switch (mode) {
      case JointStateSource::kSim:
        if (sim.has_value && isFresh(sim.received_at)) {
          return std::make_pair(std::string("sim"), latest_sim_);
        }
        return std::nullopt;
      case JointStateSource::kReal:
        if (real.has_value && isFresh(real.received_at)) {
          return std::make_pair(std::string("real"), latest_real_);
        }
        return std::nullopt;
      case JointStateSource::kTest:
        if (test.has_value && isFresh(test.received_at)) {
          return std::make_pair(std::string("test"), latest_test_);
        }
        return std::nullopt;
      case JointStateSource::kLatest:
        {
          std::optional<std::pair<std::string, sensor_msgs::msg::JointState>> candidate;
          rclcpp::Time candidate_time{0, 0, RCL_ROS_TIME};
          const SourceSlot slots[] = {sim, real, test};
          const char* names[] = {"sim", "real", "test"};
          for (size_t i = 0; i < 3; ++i) {
            if (!slots[i].has_value || !isFresh(slots[i].received_at)) {
              continue;
            }
            if (!candidate.has_value() || slots[i].received_at > candidate_time) {
              candidate = std::make_pair(std::string(names[i]), slots[i].msg);
              candidate_time = slots[i].received_at;
            }
          }
          return candidate;
        }
      case JointStateSource::kPriority:
      default:
        for (const auto& source : source_priority_) {
          auto selected = getSlotByName(source, sim, real, test);
          if (!selected.has_value()) {
            continue;
          }
          const auto& [name, slot] = *selected;
          if (slot.has_value && isFresh(slot.received_at)) {
            return std::make_pair(name, slot.msg);
          }
        }
        return std::nullopt;
    }
  }

  void timerCallback() {
    sensor_msgs::msg::JointState msg;
    std::string selected_source;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto selected = selectLatestLocked();
      if (!selected.has_value()) {
        return;
      }
      selected_source = selected->first;
      msg = std::move(selected->second);
    }

    msg.header.stamp = now();
    output_pub_->publish(msg);
    if (selected_source != last_selected_source_) {
      last_selected_source_ = selected_source;
      RCLCPP_INFO(get_logger(), "Selected joint state source: %s -> %s", active_source_.c_str(), selected_source.c_str());
    }
  }

  std::string sim_topic_;
  std::string real_topic_;
  std::string test_topic_;
  std::string output_topic_;
  std::string active_source_ = "priority";
  std::vector<std::string> source_priority_;
  double max_age_sec_ = 0.5;
  double publish_hz_ = 20.0;
  std::string last_selected_source_;

  mutable std::mutex mutex_;
  bool has_sim_ = false;
  bool has_real_ = false;
  bool has_test_ = false;
  sensor_msgs::msg::JointState latest_sim_;
  sensor_msgs::msg::JointState latest_real_;
  sensor_msgs::msg::JointState latest_test_;
  rclcpp::Time sim_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time real_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time test_received_at_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr output_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr real_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr test_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateMuxNode>());
  rclcpp::shutdown();
  return 0;
}
