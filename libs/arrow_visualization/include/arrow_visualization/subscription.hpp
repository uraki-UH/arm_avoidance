#pragma once
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>

namespace arrow_visualization {
// 既知の全送信元に接続可能なQoS。未検出時はbest_effort・volatile
inline rclcpp::QoS subscription_qos(rclcpp::Node &node, const std::string &topic) {
  const auto publishers = node.get_publishers_info_by_topic(topic);
  auto qos = rclcpp::QoS(1).best_effort();
  if (!publishers.empty()) {
    if (std::all_of(publishers.begin(), publishers.end(), [](const auto &publisher) {
        return publisher.qos_profile().get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE;
      })) qos.reliable();
    if (std::all_of(publishers.begin(), publishers.end(), [](const auto &publisher) {
        return publisher.qos_profile().get_rmw_qos_profile().durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
      })) qos.transient_local();
  }
  return qos;
}

// 送信元の遅着・再起動・混在に追従する購読。所有権の解放で監視も終了
template<class message_type, class callback_type>
std::shared_ptr<void> subscribe_adaptive(rclcpp::Node &node, const std::string &topic, callback_type callback) {
  struct subscription_state {
    typename rclcpp::Subscription<message_type>::SharedPtr subscription;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::QoS qos{1};
  };
  auto state = std::make_shared<subscription_state>();
  auto refresh = [&node, topic, callback, weak = std::weak_ptr<subscription_state>(state)] {
    const auto current = weak.lock();
    if (!current) return;
    const auto qos = subscription_qos(node, topic);
    if (current->subscription && current->qos.reliability() == qos.reliability() &&
        current->qos.durability() == qos.durability()) return;
    current->subscription.reset();
    current->qos = qos;
    current->subscription = node.create_subscription<message_type>(topic, qos, callback);
  };
  refresh();
  state->timer = node.create_wall_timer(std::chrono::milliseconds(500), refresh);
  return state;
}
}  // arrow_visualization 名前空間
