#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <initializer_list>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

namespace robot_sim::common {

// 内部統計: pc=[入力点数, ms]、vxl=[環境, 自己除去マスク, 除去数, ms]、emap=[安全, 衝突, 注意, ms]
// 時間は各処理のsteady_clock実測値。点群・グラフ本体の追加転送なし
class viewer_status_reporter {
public:
  viewer_status_reporter(rclcpp::Node &node, const std::string &stage) {
    if (node.declare_parameter("enable_viewer_status", false)) {
      publisher_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
          "viewer_status/" + stage, rclcpp::QoS(1).best_effort());
    }
  }

  void report(std::initializer_list<double> values) {
    if (!publisher_) return;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_publish_ < std::chrono::milliseconds(200)) return;
    std_msgs::msg::Float64MultiArray message;
    message.data = values;
    publisher_->publish(message);
    last_publish_ = now;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  std::chrono::steady_clock::time_point last_publish_{};
};

// 段階ごとの最新観測。異なる段階間のフレーム同期・時間合算なし
class viewer_status_snapshot {
public:
  using clock = std::chrono::steady_clock;

  void update(std::size_t stage, const std::vector<double> &values,
              clock::time_point now = clock::now()) {
    if (stage >= data_.size() || values.size() != (stage == 0 ? 2U : 4U)) return;
    for (std::size_t idx = 0; idx < values.size(); ++idx) {
      if (!std::isfinite(values[idx]) || values[idx] < 0.0 ||
          (idx + 1 < values.size() && std::floor(values[idx]) != values[idx])) return;
    }
    data_[stage] = values;
    received_[stage] = now;
  }

  std::string format(clock::time_point now = clock::now()) const {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    const std::array<std::string, 3> titles{"PCl: ", " | Vxl: Env ", " | Emap: Safe "};
    const std::array<std::array<std::string, 2>, 3> fields{{{{"", ""}}, {{" Self ", " Dup "}}, {{" Coll ", " Dang "}}}};
    for (std::size_t stage = 0; stage < data_.size(); ++stage) {
      const bool is_fresh = !data_[stage].empty() && now - received_[stage] <= std::chrono::seconds(3);
      out << titles[stage];
      const std::size_t num_counts = stage == 0 ? 1U : 3U;
      for (std::size_t idx = 0; idx < num_counts; ++idx) {
        if (idx) out << fields[stage][idx - 1];
        if (is_fresh) out << std::fixed << std::setprecision(0) << data_[stage][idx];
        else out << "--";
      }
      out << " (";
      if (is_fresh) out << std::fixed << std::setprecision(2) << data_[stage].back();
      else out << "--";
      out << " ms)";
    }
    return out.str();
  }

private:
  std::array<std::vector<double>, 3> data_;
  std::array<clock::time_point, 3> received_{};
};

// 既存Viewerブリッジ内での1 Hz集約。購読とタイマーは既定callback groupによる直列実行
class viewer_status_summary {
public:
  explicit viewer_status_summary(rclcpp::Node &node) {
    if (!node.declare_parameter("enable_viewer_status", false)) return;
    const std::array<std::string, 3> stages{"pc", "vxl", "emap"};
    for (std::size_t idx = 0; idx < stages.size(); ++idx) {
      subscriptions_[idx] = node.create_subscription<std_msgs::msg::Float64MultiArray>(
          "viewer_status/" + stages[idx], rclcpp::QoS(1).best_effort(),
          [this, idx](std_msgs::msg::Float64MultiArray::ConstSharedPtr message) {
            snapshot_.update(idx, message->data);
          });
    }
    const auto logger = node.get_logger();
    timer_ = node.create_wall_timer(std::chrono::seconds(1), [this, logger]() {
      RCLCPP_INFO(logger, "%s", snapshot_.format().c_str());
    });
  }

private:
  viewer_status_snapshot snapshot_;
  std::array<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr, 3> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_sim::common
