#include <chrono>
#include <map>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

// HumbleのPython APIで欠落する配信元GIDを含む、TFの所有者と固定内部姿勢の観測
class sensor_calibration_tf_observer : public rclcpp::Node {
public:
  sensor_calibration_tf_observer() : Node("sensor_calibration_tf_observer") {
    publisher_ = create_publisher<std_msgs::msg::String>(
      "/sensor_calibration/tf_sources", rclcpp::QoS(1).transient_local());
    static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).transient_local(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        observe(*message, info, true);
      });
    dynamic_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        observe(*message, info, false);
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { publish(); });
  }

private:
  static std::string publisher_id(const uint8_t * data) {
    constexpr char digits[] = "0123456789abcdef";
    std::string result;
    for (size_t idx = 0; idx < RMW_GID_STORAGE_SIZE; ++idx) {
      result.push_back(digits[data[idx] >> 4]);
      result.push_back(digits[data[idx] & 15]);
    }
    return result;
  }

  void observe(const tf2_msgs::msg::TFMessage & message, const rclcpp::MessageInfo & info, bool is_static) {
    const auto id = publisher_id(info.get_rmw_message_info().publisher_gid.data);
    for (const auto & transform : message.transforms) {
      if (transform.child_frame_id.empty()) { continue; }
      nlohmann::json record = {{"parent_frame", transform.header.frame_id},
        {"child_frame", transform.child_frame_id}, {"publisher_id", id}, {"is_static", is_static}};
      if (is_static) {
        const auto & value = transform.transform;
        record["pos"] = {value.translation.x, value.translation.y, value.translation.z};
        record["quat"] = {value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w};
      }
      // 時変TFの位置列の保持・転送なし。所有者と接続先だけの観測
      records_[id + ":" + transform.child_frame_id] = std::move(record);
    }
  }

  void publish() {
    nlohmann::json publishers = nlohmann::json::object();
    for (const auto & topic : {"/tf", "/tf_static"}) {
      for (const auto & endpoint : get_publishers_info_by_topic(topic)) {
        const auto & id = endpoint.endpoint_gid();
        std::string name = endpoint.node_namespace();
        if (name.empty() || name.back() != '/') { name += '/'; }
        publishers[publisher_id(id.data())] = name + endpoint.node_name();
      }
    }
    nlohmann::json records = nlohmann::json::array();
    for (auto it = records_.begin(); it != records_.end();) {
      if (!publishers.contains(it->second["publisher_id"].get<std::string>())) {
        it = records_.erase(it);
      } else {
        records.push_back(it->second);
        ++it;
      }
    }
    std_msgs::msg::String message;
    message.data = nlohmann::json{{"publishers", publishers}, {"records", records}}.dump();
    publisher_->publish(message);
  }

  std::map<std::string, nlohmann::json> records_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_, dynamic_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_calibration_tf_observer>());
  rclcpp::shutdown();
  return 0;
}
