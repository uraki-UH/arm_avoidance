#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamixel_communicator.h"
#include "dynamixel_handler_msgs/msg/dynamixel_present.hpp"
#include "rclcpp/rclcpp.hpp"

// Pingと現在角度の同期読取りのみ。起動・終了時を含むサーボ設定書込みなし
class dynamixel_state_reader : public rclcpp::Node {
public:
  dynamixel_state_reader() : Node("dynamixel_state_reader") {
    device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    const auto baudrate = declare_parameter<int>("baudrate", 1000000);
    const auto publish_hz = declare_parameter<double>("publish_hz", 30.0);
    const auto output_topic = declare_parameter<std::string>("output_topic", "/dynamixel/state/present");
    const auto joint_ids = declare_parameter<std::vector<int64_t>>("joint_ids", std::vector<int64_t>{});
    if (baudrate <= 0 || !std::isfinite(publish_hz) || publish_hz <= 0.0 ||
        publish_hz > 100.0 || joint_ids.empty() || joint_ids.size() > 100 || output_topic.empty()) {
      throw std::invalid_argument("通信速度・周期・ID・出力トピックの設定不正");
    }
    std::set<int64_t> unique_ids;
    for (const auto id : joint_ids) {
      if (id < 0 || id > 252 || !unique_ids.insert(id).second) {
        throw std::invalid_argument("サーボIDの範囲外または重複");
      }
      joint_ids_.push_back(static_cast<uint8_t>(id));
    }
    communicator_ = std::make_unique<DynamixelCommunicator>(device_name_.c_str(), baudrate, 16);
    try {
      if (!communicator_->OpenPort()) {
        throw std::runtime_error("USBポートのオープン失敗: " + device_name_);
      }
      for (const auto id : joint_ids_) {
        if (!communicator_->Ping(id)) {
          throw std::runtime_error("サーボ応答なし: ID " + std::to_string(id));
        }
        const auto model = communicator_->ping_id_model_map_last_read().at(id);
        if (dynamixel_series(model) != SERIES_X) {
          throw std::runtime_error("読取り対象外の機種: ID " + std::to_string(id));
        }
        models_[id] = model;
      }
      publisher_ = create_publisher<dynamixel_handler_msgs::msg::DynamixelPresent>(output_topic, 10);
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_hz),
                                [this]() { read_positions(); });
      RCLCPP_INFO(get_logger(), "読取り専用: %s, %ld bps, %zu台, %.1f Hz",
                  device_name_.c_str(), static_cast<long>(baudrate), joint_ids_.size(), publish_hz);
    } catch (...) {
      communicator_->ClosePort();
      throw;
    }
  }

  ~dynamixel_state_reader() override {
    communicator_->ClosePort();
  }

private:
  void read_positions() {
    const auto values = communicator_->SyncRead(AddrX::present_position, joint_ids_);
    // 欠測時の全姿勢配信停止。過去値による実測値の補完なし
    if (values.size() != joint_ids_.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                          "関節角度の欠測: %zu/%zu台", values.size(), joint_ids_.size());
      return;
    }
    dynamixel_handler_msgs::msg::DynamixelPresent msg;
    for (const auto id : joint_ids_) {
      const auto value = values.find(id);
      if (value == values.end()) {
        return;
      }
      msg.id_list.push_back(id);
      // 既存handlerと共通の中心基準角度。Wizardの表示角とは基準点が異なる場合あり
      msg.position_deg.push_back(
          AddrX::present_position.pulse2val(value->second, models_.at(id)) * 180.0 / std::acos(-1.0));
    }
    publisher_->publish(msg);
  }

  std::string device_name_;
  std::unique_ptr<DynamixelCommunicator> communicator_;
  std::vector<uint8_t> joint_ids_;
  std::map<uint8_t, uint16_t> models_;
  rclcpp::Publisher<dynamixel_handler_msgs::msg::DynamixelPresent>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<dynamixel_state_reader>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_state_reader"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
