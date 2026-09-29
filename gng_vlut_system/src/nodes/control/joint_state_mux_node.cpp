#include "core/control/joint_command_mux.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <gng_control_msgs/msg/joint_control_claim.hpp>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <regex>

// Viewer・Gazebo・実機で共通の関節単位仲裁
class joint_state_mux_node : public rclcpp::Node {
public:
  joint_state_mux_node() : Node("joint_state_mux_node") {
    const auto names = declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    mux_ = std::make_unique<mux_type>(std::set<std::string>(names.begin(), names.end()));
    const auto alias_names = declare_parameter<std::vector<std::string>>("alias_names", std::vector<std::string>{});
    const auto alias_parents = declare_parameter<std::vector<std::string>>("alias_parents", std::vector<std::string>{});
    const auto alias_multipliers = declare_parameter<std::vector<double>>("alias_multipliers", std::vector<double>{});
    const auto alias_offsets = declare_parameter<std::vector<double>>("alias_offsets", std::vector<double>{});
    if (alias_names.size() != alias_parents.size() || alias_names.size() != alias_multipliers.size() ||
        alias_names.size() != alias_offsets.size()) throw std::invalid_argument("mimic設定の配列長が不正です");
    for (std::size_t idx = 0; idx < alias_names.size(); ++idx) {
      if (!mux_->set_alias(alias_names[idx], {alias_parents[idx], alias_multipliers[idx], alias_offsets[idx]})) {
        throw std::invalid_argument("mimic設定が不正です: " + alias_names[idx]);
      }
    }
    const auto output_topic = declare_parameter<std::string>("output_topic", "target_joint_states");
    const auto active_topic = declare_parameter<std::string>("active_topic", "active_joint_commands");
    const auto claim_topic = declare_parameter<std::string>("claim_topic", "control_claims");
    const auto publish_hz = declare_parameter<double>("publish_hz", 50.0);
    // 従来launchのパラメータ名との互換性
    enable_hold_last_output_ = declare_parameter<bool>("hold_last_output", true);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 1.0);
    if (!std::isfinite(publish_hz) || publish_hz < 1.0 || publish_hz > 1000.0 ||
        !std::isfinite(command_timeout_sec_) || command_timeout_sec_ < 0.0 ||
        resolve_topic(output_topic) == resolve_topic(active_topic)) {
      throw std::invalid_argument("muxの周期・有効期間・出力先が不正です");
    }
    output_topic_ = resolve_topic(output_topic);
    active_topic_ = resolve_topic(active_topic);
    output_pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic, 10);
    active_pub_ = create_publisher<sensor_msgs::msg::JointState>(active_topic, 10);
    const auto topics = declare_parameter<std::vector<std::string>>(
        "source_topics", {"joint_commands", "gripper_commands", "leader_joint_states"});
    const auto priorities = declare_parameter<std::vector<int64_t>>("source_priorities", {50, 200, 100});
    const auto timeouts = declare_parameter<std::vector<double>>("source_timeouts_sec", {0.0, 0.0, 0.5});
    const auto grippers = declare_parameter<std::vector<std::string>>(
        "gripper_joint_names", {"L_gripper_joint", "R_gripper_joint"});
    if (topics.size() != priorities.size() || topics.size() != timeouts.size()) {
      throw std::invalid_argument("指令元設定の配列長が一致しません");
    }
    for (std::size_t idx = 0; idx < topics.size(); ++idx) {
      mux_type::claim settings;
      settings.priority = static_cast<int>(priorities[idx]);
      settings.command_timeout_sec = timeouts[idx];
      if (topics[idx] == "gripper_commands") {
        if (grippers.empty()) continue;
        settings.joints = {grippers.begin(), grippers.end()};
      }
      const auto topic = resolve_topic(topics[idx]);
      if (!can_subscribe(topic) || !mux_->set_claim(topic, settings)) {
        throw std::invalid_argument("指令元の登録が不正です: " + topic);
      }
      source_timeouts_[topic] = timeouts[idx];
      subscribe(topic);
    }
    claim_sub_ = create_subscription<gng_control_msgs::msg::JointControlClaim>(
        claim_topic, rclcpp::QoS(100).reliable().transient_local(),
        [this](gng_control_msgs::msg::JointControlClaim::ConstSharedPtr msg) {
          const auto topic = resolve_topic(msg->command_topic);
          mux_type::claim settings;
          settings.joints = {msg->joint_names.begin(), msg->joint_names.end()};
          settings.priority = msg->priority;
          settings.is_exclusive = msg->mode == msg->MODE_EXCLUSIVE;
          settings.enable_source = msg->enabled;
          const auto timeout = source_timeouts_.find(topic);
          settings.command_timeout_sec = timeout == source_timeouts_.end()
              ? command_timeout_sec_ : timeout->second;
          if (msg->mode > msg->MODE_EXCLUSIVE || !can_subscribe(topic) ||
              !mux_->set_claim(topic, settings)) {
            RCLCPP_WARN(get_logger(), "制御権要求を拒否しました: %s", topic.c_str());
            return;
          }
          subscribe(topic);
        });
    for (const auto &[service_name, source_topic] : std::map<std::string, std::string>{
             {"joint_control/release_manual", "joint_commands"},
             {"joint_control/release_gripper", "gripper_commands"}}) {
      release_services_.push_back(create_service<std_srvs::srv::Trigger>(
          service_name, [this, source_topic](
              const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            response->success = mux_->clear_command(resolve_topic(source_topic));
            response->message = response->success ? "保持目標を解除しました" : "指令元が未登録です";
          }));
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_hz), [this]() {
      const auto result = mux_->resolve(monotonic_sec(), enable_hold_last_output_);
      output_pub_->publish(message(result.held));
      active_pub_->publish(message(result.active));
    });
  }

private:
  using mux_type = robot_sim::control::joint_command_mux;
  static double monotonic_sec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  std::string resolve_topic(const std::string &topic) const {
    if (topic.empty()) return {};
    if (topic.front() == '/') return topic;
    const std::string robot_namespace = get_namespace();
    return (robot_namespace == "/" ? robot_namespace : robot_namespace + "/") + topic;
  }
  bool can_subscribe(const std::string &topic) const {
    static const std::regex pattern("^/([A-Za-z_][A-Za-z0-9_]*)(/[A-Za-z_][A-Za-z0-9_]*)*$");
    return std::regex_match(topic, pattern) && topic != output_topic_ && topic != active_topic_;
  }
  void subscribe(const std::string &topic) {
    if (subscriptions_.count(topic)) return;
    subscriptions_[topic] = create_subscription<sensor_msgs::msg::JointState>(
        topic, 10, [this, topic](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          if (!mux_->set_command(topic, msg->name, msg->position, monotonic_sec())) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                "関節指令を拒否しました: %s", topic.c_str());
          }
        });
  }
  sensor_msgs::msg::JointState message(const std::map<std::string, double> &values) {
    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    for (const auto &[name, value] : values) {
      out.name.push_back(name);
      out.position.push_back(value);
    }
    return out;
  }
  std::unique_ptr<mux_type> mux_;
  bool enable_hold_last_output_ = true;
  double command_timeout_sec_ = 1.0;
  std::string output_topic_, active_topic_;
  std::map<std::string, double> source_timeouts_;
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> subscriptions_;
  rclcpp::Subscription<gng_control_msgs::msg::JointControlClaim>::SharedPtr claim_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr output_pub_, active_pub_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> release_services_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joint_state_mux_node>());
  rclcpp::shutdown();
  return 0;
}
