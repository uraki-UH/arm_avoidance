#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/common/constants.hpp"
#include "gng_control_msgs/msg/joint_control_claim.hpp"

class JointStateMuxNode : public rclcpp::Node {
public:
  JointStateMuxNode() : Node("joint_state_mux_node") {
    output_topic_ = declare_parameter<std::string>("output_topic", "target_joint_states");
    publish_hz_ = std::max(1.0, declare_parameter<double>(
                                   "publish_hz", ::robot_sim::common::Constants::DEFAULT_UPDATE_HZ));
    hold_last_output_ = declare_parameter<bool>("hold_last_output", true);
    claim_topic_ = declare_parameter<std::string>("claim_topic", "control_claims");

    const std::string ns_raw = std::string(get_namespace());
    const std::string ns = ns_raw.empty() ? std::string{} : (ns_raw.front() == '/' ? ns_raw.substr(1) : ns_raw);
    const std::string resolved_claim_topic = resolveTopic(ns, claim_topic_);

    output_pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, 10);
    claim_bus_sub_ = create_subscription<gng_control_msgs::msg::JointControlClaim>(
        resolved_claim_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this, ns](const gng_control_msgs::msg::JointControlClaim::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          const std::string command_topic = resolveTopic(ns, msg->command_topic);
          if (command_topic.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Ignoring claim with empty command_topic on %s", claim_topic_.c_str());
            return;
          }

          auto & src = sources_[command_topic];
          src.command_topic = command_topic;
          src.claim_msg = *msg;
          src.has_claim = true;

          if (!src.command_sub) {
            src.command_sub = create_subscription<sensor_msgs::msg::JointState>(
                command_topic, 10,
                [this, command_topic](const sensor_msgs::msg::JointState::SharedPtr cmd_msg) {
                  std::lock_guard<std::mutex> lock(mutex_);
                  auto & dst = sources_[command_topic];
                  dst.command_topic = command_topic;
                  dst.command_msg = *cmd_msg;
                  dst.has_command = true;
                });
          }
        });

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz_)),
        [this]() { this->update(); });

    RCLCPP_INFO(
        get_logger(),
        "JointStateMuxNode ready. namespace=%s output=%s claim_bus=%s publish_hz=%.1f hold_last_output=%d",
        get_namespace(), output_topic_.c_str(), resolved_claim_topic.c_str(),
        publish_hz_, hold_last_output_ ? 1 : 0);
  }

private:
  struct SourceState {
    std::string command_topic;
    sensor_msgs::msg::JointState command_msg;
    gng_control_msgs::msg::JointControlClaim claim_msg;
    bool has_command = false;
    bool has_claim = false;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub;
  };

  static std::unordered_map<std::string, double> buildMap(
      const sensor_msgs::msg::JointState & msg)
  {
    std::unordered_map<std::string, double> out;
    out.reserve(msg.name.size());
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      if (i < msg.position.size()) {
        out[msg.name[i]] = msg.position[i];
      }
    }
    return out;
  }

  static std::vector<std::string> uniqueMerge(
      const std::vector<std::string> & a,
      const std::vector<std::string> & b)
  {
    std::vector<std::string> out = a;
    std::unordered_set<std::string> seen(a.begin(), a.end());
    for (const auto & name : b) {
      if (seen.insert(name).second) {
        out.push_back(name);
      }
    }
    return out;
  }

  static std::string resolveTopic(const std::string & robot_ns, const std::string & topic) {
    if (topic.empty()) {
      return {};
    }
    if (!topic.empty() && topic.front() == '/') {
      return topic;
    }
    if (robot_ns.empty()) {
      return "/" + topic;
    }
    return "/" + robot_ns + "/" + topic;
  }

  const std::vector<std::string> & effectiveClaimJoints(const SourceState & src,
                                                        std::vector<std::string> & scratch) const
  {
    if (src.has_claim && !src.claim_msg.joint_names.empty()) {
      return src.claim_msg.joint_names;
    }
    if (src.has_command) {
      scratch = src.command_msg.name;
      return scratch;
    }
    scratch.clear();
    return scratch;
  }

  int effectivePriority(const SourceState & src) const {
    return src.has_claim ? src.claim_msg.priority : 0;
  }

  int effectiveMode(const SourceState & src) const {
    return src.has_claim ? src.claim_msg.mode : static_cast<int>(gng_control_msgs::msg::JointControlClaim::MODE_EXCLUSIVE);
  }

  bool effectiveEnabled(const SourceState & src) const {
    return src.has_claim ? src.claim_msg.enabled : true;
  }

  void publishLocked() {
    std::unordered_map<std::string, double> current_values;
    std::unordered_map<std::string, int> chosen_priority;
    std::unordered_map<std::string, int> chosen_mode;
    std::vector<std::string> all_joints;
    all_joints.reserve(128);

    if (have_output_ && hold_last_output_) {
      current_values = buildMap(latest_output_);
      all_joints = latest_output_.name;
    }

    for (const auto &[command_topic, src] : sources_) {
      (void)command_topic;
      if (src.has_command) {
        all_joints = uniqueMerge(all_joints, src.command_msg.name);
      }
    }

    if (all_joints.empty()) {
      return;
    }

    for (const auto &[command_topic, src] : sources_) {
      (void)command_topic;
      if (!src.has_command || !effectiveEnabled(src)) {
        continue;
      }

      std::vector<std::string> scratch;
      const auto & claim_joints = effectiveClaimJoints(src, scratch);
      if (claim_joints.empty()) {
        continue;
      }

      const auto src_map = buildMap(src.command_msg);
      const int src_priority = effectivePriority(src);
      const int src_mode = effectiveMode(src);

      for (const auto & joint_name : claim_joints) {
        const auto cmd_it = src_map.find(joint_name);
        if (cmd_it == src_map.end()) {
          continue;
        }

        const auto cur_it = chosen_priority.find(joint_name);
        const bool take = [&]() {
          if (cur_it == chosen_priority.end()) {
            return true;
          }
          const int cur_mode = chosen_mode[joint_name];
          const bool src_exclusive =
              (src_mode == gng_control_msgs::msg::JointControlClaim::MODE_EXCLUSIVE);
          const bool cur_exclusive =
              (cur_mode == gng_control_msgs::msg::JointControlClaim::MODE_EXCLUSIVE);
          if (src_exclusive != cur_exclusive) {
            return src_exclusive;
          }
          if (src_priority != cur_it->second) {
            return src_priority > cur_it->second;
          }
          return false;
        }();

        if (take) {
          current_values[joint_name] = cmd_it->second;
          chosen_priority[joint_name] = src_priority;
          chosen_mode[joint_name] = src_mode;
        }
      }
    }

    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    out.name = all_joints;
    out.position.resize(out.name.size(), 0.0);
    out.velocity.resize(out.name.size(), 0.0);
    out.effort.resize(out.name.size(), 0.0);

    for (std::size_t i = 0; i < out.name.size(); ++i) {
      const auto it = current_values.find(out.name[i]);
      out.position[i] = (it != current_values.end()) ? it->second : 0.0;
    }

    latest_output_ = out;
    have_output_ = true;
    output_pub_->publish(out);
  }

  void update() {
    std::lock_guard<std::mutex> lock(mutex_);
    publishLocked();
  }

  std::string output_topic_;
  std::string claim_topic_;
  double publish_hz_ = 50.0;
  bool hold_last_output_ = true;

  std::map<std::string, SourceState> sources_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr output_pub_;
  rclcpp::Subscription<gng_control_msgs::msg::JointControlClaim>::SharedPtr claim_bus_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  sensor_msgs::msg::JointState latest_output_;
  bool have_output_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateMuxNode>());
  rclcpp::shutdown();
  return 0;
}
