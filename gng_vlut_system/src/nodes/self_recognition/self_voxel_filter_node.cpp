#include <rclcpp/rclcpp.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/common/constants.hpp"
#include "safety_engine/vlut/voxel_processor.hpp"

namespace robot_sim::self_recognition {

namespace {

std::string normalizeFrameId(const std::string &frame_id) {
  const auto first = frame_id.find_first_not_of('/');
  return first == std::string::npos ? std::string{} : frame_id.substr(first);
}

std::vector<long> uniqueSorted(std::vector<long> values) {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
  return values;
}

}  // namespace

class SelfVoxelFilterNode : public rclcpp::Node {
public:
  SelfVoxelFilterNode()
  : Node("self_voxel_filter_node"),
    processor_(::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE) {
    declare_parameter<std::string>(
        "self_recognition.raw_environment_voxel_topic", "roi_voxel_ids_raw");
    declare_parameter<std::string>(
        "self_recognition.filtered_environment_voxel_topic", "roi_voxel_ids");
    declare_parameter<std::string>("self_recognition.mask_topic", "self_voxel");
    declare_parameter<double>("self_recognition.self_exclusion_inflation", 0.02);
    declare_parameter<double>("self_recognition.max_self_mask_age_sec", 0.5);
    declare_parameter<double>("self_recognition.voxel_size_match_th", 1e-9);

    raw_environment_voxel_topic_ = get_parameter(
        "self_recognition.raw_environment_voxel_topic").as_string();
    filtered_environment_voxel_topic_ = get_parameter(
        "self_recognition.filtered_environment_voxel_topic").as_string();
    self_voxel_topic_ = get_parameter("self_recognition.mask_topic").as_string();
    self_exclusion_inflation_ = std::max(
        0.0, get_parameter("self_recognition.self_exclusion_inflation").as_double());
    max_self_mask_age_sec_ = std::max(
        0.0, get_parameter("self_recognition.max_self_mask_age_sec").as_double());
    voxel_size_match_th_ = std::max(
        0.0, get_parameter("self_recognition.voxel_size_match_th").as_double());

    if (raw_environment_voxel_topic_ == filtered_environment_voxel_topic_) {
      throw std::runtime_error(
          "自己ボクセルフィルタの入力topicと出力topicには異なる名前が必要");
    }

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    self_sub_ = create_subscription<voxel_msgs::msg::Voxel>(
        self_voxel_topic_, qos,
        std::bind(&SelfVoxelFilterNode::selfVoxelCallback, this, std::placeholders::_1));
    environment_sub_ = create_subscription<voxel_msgs::msg::Voxel>(
        raw_environment_voxel_topic_, qos,
        std::bind(
            &SelfVoxelFilterNode::environmentVoxelCallback, this,
            std::placeholders::_1));
    filtered_pub_ = create_publisher<voxel_msgs::msg::Voxel>(
        filtered_environment_voxel_topic_, qos);

    RCLCPP_INFO(
        get_logger(),
        "自己ボクセルフィルタ起動: environment_input=%s environment_output=%s "
        "self_mask=%s inflation=%.4f max_mask_age=%.3f",
        raw_environment_voxel_topic_.c_str(), filtered_environment_voxel_topic_.c_str(),
        self_voxel_topic_.c_str(), self_exclusion_inflation_, max_self_mask_age_sec_);
  }

private:
  struct MaskSnapshot {
    std::unordered_set<int64_t> excluded_ids;
    std::string frame_id;
    float voxel_size{0.0F};
    int32_t x_shift{0};
    int32_t y_shift{0};
    int32_t z_shift{0};
    int64_t offset{0};
    std::chrono::steady_clock::time_point received_at;
  };

  bool hasMatchingSchema(
      const voxel_msgs::msg::Voxel &environment,
      const MaskSnapshot &mask) const {
    return normalizeFrameId(environment.header.frame_id) == mask.frame_id &&
           std::abs(static_cast<double>(environment.voxel_size) - mask.voxel_size) <=
               voxel_size_match_th_ &&
           environment.x_shift == mask.x_shift &&
           environment.y_shift == mask.y_shift &&
           environment.z_shift == mask.z_shift &&
           environment.offset == mask.offset;
  }

  void selfVoxelCallback(const voxel_msgs::msg::Voxel::SharedPtr msg) {
    if (msg->voxel_size <= 0.0F) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "自己ボクセルマスクのvoxel_sizeが不正なため更新を無視: %.9g",
          static_cast<double>(msg->voxel_size));
      return;
    }

    processor_.getGrid().setVoxelSize(msg->voxel_size);
    processor_.getGrid().setIndexingParams(
        msg->x_shift, msg->y_shift, msg->z_shift, msg->offset);

    std::vector<long> excluded(msg->data.begin(), msg->data.end());
    if (self_exclusion_inflation_ > 0.0 && !excluded.empty()) {
      auto inflated = processor_.dilate(
          excluded, static_cast<float>(self_exclusion_inflation_));
      excluded.insert(excluded.end(), inflated.begin(), inflated.end());
    }
    excluded = uniqueSorted(std::move(excluded));

    auto snapshot = std::make_shared<MaskSnapshot>();
    snapshot->excluded_ids.reserve(excluded.size());
    for (const long voxel_id : excluded) {
      snapshot->excluded_ids.insert(static_cast<int64_t>(voxel_id));
    }
    snapshot->frame_id = normalizeFrameId(msg->header.frame_id);
    snapshot->voxel_size = msg->voxel_size;
    snapshot->x_shift = msg->x_shift;
    snapshot->y_shift = msg->y_shift;
    snapshot->z_shift = msg->z_shift;
    snapshot->offset = msg->offset;
    snapshot->received_at = std::chrono::steady_clock::now();

    {
      std::lock_guard<std::mutex> lock(mask_mutex_);
      mask_snapshot_ = std::move(snapshot);
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "自己ボクセルマスク更新: source=%zu exclusion=%zu frame=%s voxel_size=%.6g",
        msg->data.size(), excluded.size(), msg->header.frame_id.c_str(),
        static_cast<double>(msg->voxel_size));
  }

  void environmentVoxelCallback(const voxel_msgs::msg::Voxel::SharedPtr msg) {
    std::shared_ptr<const MaskSnapshot> mask;
    {
      std::lock_guard<std::mutex> lock(mask_mutex_);
      mask = mask_snapshot_;
    }

    if (!mask) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "自己ボクセルマスク受信前のため環境ボクセル更新を抑止");
      return;
    }

    const double mask_age_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - mask->received_at).count();
    if (max_self_mask_age_sec_ > 0.0 && mask_age_sec > max_self_mask_age_sec_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "自己ボクセルマスクが古いため環境ボクセル更新を抑止: age=%.3f sec",
          mask_age_sec);
      return;
    }

    if (!hasMatchingSchema(*msg, *mask)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "環境ボクセルと自己マスクのgrid定義不一致により更新を抑止: "
          "environment_frame=%s self_frame=%s environment_size=%.9g self_size=%.9g",
          msg->header.frame_id.c_str(), mask->frame_id.c_str(),
          static_cast<double>(msg->voxel_size), static_cast<double>(mask->voxel_size));
      return;
    }

    voxel_msgs::msg::Voxel filtered = *msg;
    filtered.data.clear();
    filtered.data.reserve(msg->data.size());
    const bool has_aligned_labels = msg->labels.size() == msg->data.size();
    filtered.labels.clear();
    if (has_aligned_labels) {
      filtered.labels.reserve(msg->labels.size());
    }

    for (std::size_t idx = 0; idx < msg->data.size(); ++idx) {
      if (mask->excluded_ids.count(msg->data[idx]) != 0U) {
        continue;
      }
      filtered.data.push_back(msg->data[idx]);
      if (has_aligned_labels) {
        filtered.labels.push_back(msg->labels[idx]);
      }
    }
    filtered_pub_->publish(filtered);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "環境自己ボクセル除外: input=%zu output=%zu removed=%zu",
        msg->data.size(), filtered.data.size(), msg->data.size() - filtered.data.size());
  }

  std::string raw_environment_voxel_topic_;
  std::string filtered_environment_voxel_topic_;
  std::string self_voxel_topic_;
  double self_exclusion_inflation_{0.02};
  double max_self_mask_age_sec_{0.5};
  double voxel_size_match_th_{1e-9};
  robot_sim::analysis::VoxelProcessor processor_;
  std::mutex mask_mutex_;
  std::shared_ptr<const MaskSnapshot> mask_snapshot_;
  rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr self_sub_;
  rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr environment_sub_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr filtered_pub_;
};

}  // namespace robot_sim::self_recognition

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_sim::self_recognition::SelfVoxelFilterNode>());
  rclcpp::shutdown();
  return 0;
}
