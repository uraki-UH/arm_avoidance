#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace robot_sim::bridge
{
namespace
{

using json = nlohmann::json;

bool isTopicToken(const std::string &value)
{
  if (value.empty() || !std::isalpha(static_cast<unsigned char>(value.front()))) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) || character == '_';
  });
}

std::string trim(std::string value)
{
  const auto is_space = [](unsigned char character) {return std::isspace(character);};
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), [&](char character) {
    return !is_space(static_cast<unsigned char>(character));
  }));
  value.erase(std::find_if(value.rbegin(), value.rend(), [&](char character) {
    return !is_space(static_cast<unsigned char>(character));
  }).base(), value.end());
  return value;
}

}  // namespace

class ObjectHypothesisSummonNode : public rclcpp::Node
{
public:
  explicit ObjectHypothesisSummonNode(const rclcpp::NodeOptions &options)
  : Node("object_hypothesis_summon_node", options)
  {
    template_ids_ = declare_parameter<std::vector<std::string>>(
      "template_ids", std::vector<std::string>{});
    const std::string initial_template_id = declare_parameter<std::string>("initial_template_id", "");
    state_topic_ = declare_parameter<std::string>(
      "state_topic", "/object_hypothesis/summon_state");
    selection_topic_ = declare_parameter<std::string>(
      "selection_topic", "/object_hypothesis/select");
    map_topic_ = declare_parameter<std::string>(
      "map_topic", "/object_hypothesis/topological_map");
    const double switch_interval_sec = declare_parameter<double>("switch_interval_sec", 5.0);
    const auto random_seed = declare_parameter<std::int64_t>("random_seed", 0);

    normalizeTemplateIds();
    if (template_ids_.empty()) {
      throw std::runtime_error("template_idsが空です。");
    }
    if (state_topic_.empty() || selection_topic_.empty() || map_topic_.empty() ||
      !std::isfinite(switch_interval_sec) || switch_interval_sec < 0.0)
    {
      throw std::runtime_error("召喚topicまたはswitch_interval_secが不正です。");
    }
    if (!initial_template_id.empty() && !containsTemplate(initial_template_id)) {
      throw std::runtime_error("initial_template_idがtemplate_idsにありません: " + initial_template_id);
    }

    if (random_seed == 0) {
      std::random_device device;
      random_engine_.seed(
        (static_cast<std::uint64_t>(device()) << 32U) ^ static_cast<std::uint64_t>(device()));
    } else {
      random_engine_.seed(static_cast<std::uint64_t>(random_seed));
    }

    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    state_publisher_ = create_publisher<std_msgs::msg::String>(state_topic_, state_qos);
    selection_subscription_ = create_subscription<std_msgs::msg::String>(
      selection_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&ObjectHypothesisSummonNode::onSelection, this, std::placeholders::_1));

    if (switch_interval_sec > 0.0) {
      const auto period = std::chrono::duration<double>(switch_interval_sec);
      switch_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ObjectHypothesisSummonNode::summonRandom, this));
    }

    if (initial_template_id.empty()) {
      summonRandom();
    } else {
      summon(initial_template_id, "initial");
    }

    RCLCPP_INFO(
      get_logger(),
      "物体仮説召喚準備: templates=%zu map=%s state=%s select=%s interval=%.3fs",
      template_ids_.size(), map_topic_.c_str(), state_topic_.c_str(), selection_topic_.c_str(),
      switch_interval_sec);
  }

private:
  void normalizeTemplateIds()
  {
    std::vector<std::string> normalized;
    normalized.reserve(template_ids_.size());
    std::unordered_set<std::string> seen;
    for (const std::string &template_id : template_ids_) {
      if (!isTopicToken(template_id)) {
        throw std::runtime_error("不正なtemplate_idです: " + template_id);
      }
      if (seen.insert(template_id).second) {
        normalized.push_back(template_id);
      }
    }
    template_ids_ = std::move(normalized);
  }

  bool containsTemplate(const std::string &template_id) const
  {
    return std::find(template_ids_.begin(), template_ids_.end(), template_id) != template_ids_.end();
  }

  void summonRandom()
  {
    std::size_t index = 0U;
    if (template_ids_.size() > 1U && !current_template_id_.empty()) {
      const auto current = std::find(
        template_ids_.begin(), template_ids_.end(), current_template_id_);
      const std::size_t current_index = static_cast<std::size_t>(
        std::distance(template_ids_.begin(), current));
      std::uniform_int_distribution<std::size_t> distribution(0U, template_ids_.size() - 2U);
      index = distribution(random_engine_);
      if (index >= current_index) {
        ++index;
      }
    } else if (template_ids_.size() > 1U) {
      std::uniform_int_distribution<std::size_t> distribution(0U, template_ids_.size() - 1U);
      index = distribution(random_engine_);
    }
    summon(template_ids_[index], "random_placeholder");
  }

  void summon(const std::string &template_id, const char *source)
  {
    current_template_id_ = template_id;
    ++sequence_;
    std::uniform_real_distribution<double> score_distribution(0.50, 1.0);
    std::uniform_real_distribution<double> yaw_distribution(0.0, 360.0);

    std_msgs::msg::String state;
    state.data = json{
      {"schema_version", 1},
      {"kind", "object_hypothesis_summon"},
      {"state", "confirmed"},
      {"confirmed", true},
      {"source", source},
      {"template_id", current_template_id_},
      {"hypothesis_id", "placeholder_" + std::to_string(sequence_)},
      {"score", score_distribution(random_engine_)},
      {"yaw_deg", yaw_distribution(random_engine_)},
      {"sequence", sequence_},
      {"map_topic", map_topic_},
      {"available_template_ids", template_ids_}
    }.dump();
    state_publisher_->publish(state);
    RCLCPP_INFO(
      get_logger(), "物体仮説召喚: template=%s source=%s sequence=%llu",
      current_template_id_.c_str(), source,
      static_cast<unsigned long long>(sequence_));
  }

  void onSelection(const std_msgs::msg::String::SharedPtr message)
  {
    std::string requested = trim(message->data);
    if (!requested.empty() && requested.front() == '{') {
      try {
        requested = json::parse(requested).value("template_id", "");
      } catch (const json::exception &error) {
        RCLCPP_WARN(get_logger(), "召喚ID JSONの解析失敗: %s", error.what());
        return;
      }
    }
    if (!containsTemplate(requested)) {
      RCLCPP_WARN(get_logger(), "未登録の召喚IDです: %s", requested.c_str());
      return;
    }
    summon(requested, "manual");
    if (switch_timer_) {
      switch_timer_->reset();
    }
  }

  std::vector<std::string> template_ids_;
  std::string current_template_id_;
  std::string state_topic_;
  std::string selection_topic_;
  std::string map_topic_;
  std::uint64_t sequence_ = 0U;
  std::mt19937_64 random_engine_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selection_subscription_;
  rclcpp::TimerBase::SharedPtr switch_timer_;
};

}  // namespace robot_sim::bridge

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_sim::bridge::ObjectHypothesisSummonNode>(
      rclcpp::NodeOptions()));
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("object_hypothesis_summon"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
