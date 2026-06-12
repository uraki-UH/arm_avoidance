#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/int64_multi_array.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/voxel_utils.hpp"
#include "core/common/constants.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"
#include "safety_engine/vlut/voxel_processor.hpp"
#include <voxel_idx.hpp>

namespace robot_sim::bridge {
namespace {

std::vector<std::string> splitTopicsCsv(const std::string &text) {
  std::vector<std::string> topics;
  std::stringstream ss(text);
  std::string token;
  while (std::getline(ss, token, ',')) {
    const auto begin = token.find_first_not_of(" \t");
    if (begin == std::string::npos) {
      continue;
    }
    const auto end = token.find_last_not_of(" \t");
    topics.push_back(token.substr(begin, end - begin + 1));
  }
  return topics;
}

bool sameSchema(
    const voxel_idx::VoxelIndexingSchema &lhs,
    const voxel_idx::VoxelIndexingSchema &rhs) {
  return lhs.matches(rhs);
}

voxel_idx::VoxelIndexingSchema schemaFromMsg(
    const voxel_msgs::msg::Voxel &msg,
    double fallback_voxel_size) {
  voxel_idx::VoxelIndexingSchema schema;
  schema.x_shift = msg.x_shift;
  schema.y_shift = msg.y_shift;
  schema.z_shift = msg.z_shift;
  schema.offset = msg.offset;
  schema.voxel_size = msg.voxel_size > 0.0f
      ? static_cast<double>(msg.voxel_size)
      : fallback_voxel_size;
  return schema;
}

std::vector<long> uniqueSorted(std::vector<long> values) {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
  return values;
}

}  // namespace

class VoxelTopicMergerNode : public rclcpp::Node {
public:
  explicit VoxelTopicMergerNode(const rclcpp::NodeOptions &options)
    : Node("voxel_topic_merger_node", options),
    processor_(::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE) {
    declare_parameter<std::string>("input_topics", "/topo_voxel_ids");
    declare_parameter<std::string>("output_voxel_topic", "voxel_merged");
    declare_parameter<std::string>("occupied_voxels_topic", "occupied_voxels");
    declare_parameter<std::string>("danger_voxels_topic", "danger_voxels");
    declare_parameter<std::string>("target_frame_id", "");
    declare_parameter<double>("target_voxel_size", 0.0);
    declare_parameter<int>("target_x_shift", 42);
    declare_parameter<int>("target_y_shift", 21);
    declare_parameter<int>("target_z_shift", 0);
    declare_parameter<std::int64_t>("target_offset", 1000000L);
    declare_parameter<double>("danger_inflation", 0.0);
    declare_parameter<double>("publish_hz", 10.0);

    const auto csv_topics = splitTopicsCsv(get_parameter("input_topics").as_string());
    input_topics_ = csv_topics;
    if (input_topics_.empty()) {
      input_topics_.push_back("/topo_voxel_ids");
    }

    output_voxel_topic_ = get_parameter("output_voxel_topic").as_string();
    occupied_voxels_topic_ = get_parameter("occupied_voxels_topic").as_string();
    danger_voxels_topic_ = get_parameter("danger_voxels_topic").as_string();
    target_frame_id_ = get_parameter("target_frame_id").as_string();
    target_voxel_size_ = std::max(0.0, get_parameter("target_voxel_size").as_double());
    target_x_shift_ = get_parameter("target_x_shift").as_int();
    target_y_shift_ = get_parameter("target_y_shift").as_int();
    target_z_shift_ = get_parameter("target_z_shift").as_int();
    target_offset_ = get_parameter("target_offset").as_int();
    danger_inflation_ = std::max(0.0, get_parameter("danger_inflation").as_double());
    publish_hz_ = std::max(0.1, get_parameter("publish_hz").as_double());

    if (target_voxel_size_ > 0.0) {
      output_schema_.voxel_size = target_voxel_size_;
      output_schema_.x_shift = target_x_shift_;
      output_schema_.y_shift = target_y_shift_;
      output_schema_.z_shift = target_z_shift_;
      output_schema_.offset = target_offset_;
      has_output_schema_ = true;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    merged_voxel_pub_ = create_publisher<voxel_msgs::msg::Voxel>(
        output_voxel_topic_, rclcpp::QoS(1).reliable().transient_local());
    occupied_pub_ = create_publisher<std_msgs::msg::Int64MultiArray>(
        occupied_voxels_topic_, rclcpp::QoS(1).reliable().transient_local());
    danger_pub_ = create_publisher<std_msgs::msg::Int64MultiArray>(
        danger_voxels_topic_, rclcpp::QoS(1).reliable().transient_local());

    const auto sub_qos = rclcpp::QoS(1).reliable().transient_local();
    for (const auto &topic : input_topics_) {
      subscriptions_.push_back(create_subscription<voxel_msgs::msg::Voxel>(
          topic, sub_qos,
          [this, topic](const voxel_msgs::msg::Voxel::SharedPtr msg) {
            onVoxelMsg(topic, *msg);
          }));
    }

    const auto publish_period_ms = std::max(1, static_cast<int>(1000.0 / publish_hz_));
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_period_ms),
        std::bind(&VoxelTopicMergerNode::publishLatest, this));

    RCLCPP_INFO(
        get_logger(),
        "VoxelTopicMergerNode initialized. topics=%zu output=%s occupied=%s danger=%s target_frame=%s target_voxel_size=%.4f danger_inflation=%.4f",
        input_topics_.size(), output_voxel_topic_.c_str(), occupied_voxels_topic_.c_str(),
        danger_voxels_topic_.c_str(), target_frame_id_.c_str(), target_voxel_size_,
        danger_inflation_);
  }

private:
  struct TopicState {
    voxel_msgs::msg::Voxel msg;
    bool ready = false;
  };

  static std_msgs::msg::Int64MultiArray makeInt64Msg(const std::vector<long> &values) {
    std_msgs::msg::Int64MultiArray out;
    out.data.reserve(values.size());
    for (long v : values) {
      out.data.push_back(static_cast<int64_t>(v));
    }
    return out;
  }

  void onVoxelMsg(const std::string &topic, const voxel_msgs::msg::Voxel &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_by_topic_[topic] = TopicState{msg, true};
    if (!has_output_schema_) {
      output_schema_ = schemaFromMsg(msg, ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
      if (output_schema_.voxel_size <= 0.0) {
        output_schema_.voxel_size = ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE;
      }
      if (target_frame_id_.empty()) {
        output_frame_id_ = msg.header.frame_id;
      }
      processor_.getGrid().setVoxelSize(output_schema_.voxel_size);
      processor_.getGrid().setIndexingParams(
          output_schema_.x_shift, output_schema_.y_shift, output_schema_.z_shift,
          static_cast<long>(output_schema_.offset));
      has_output_schema_ = true;
    }
    dirty_ = true;
  }

  std::vector<long> convertToOutputIds(
      const voxel_msgs::msg::Voxel &msg,
      const voxel_idx::VoxelIndexingSchema &source_schema,
      const std::string &source_frame,
      const std::string &canonical_frame) const {
    if (msg.data.empty()) {
      return {};
    }

    const bool direct_passthrough =
        sameSchema(source_schema, output_schema_) &&
        (canonical_frame.empty() || source_frame.empty() || source_frame == canonical_frame);
    if (direct_passthrough) {
      std::vector<long> ids;
      ids.reserve(msg.data.size());
      for (auto id : msg.data) {
        ids.push_back(static_cast<long>(id));
      }
      return uniqueSorted(std::move(ids));
    }

    if (source_schema.voxel_size <= 0.0 || output_schema_.voxel_size <= 0.0) {
      return {};
    }

    Eigen::Isometry3d source_to_canonical = Eigen::Isometry3d::Identity();
    if (!canonical_frame.empty() && !source_frame.empty() && source_frame != canonical_frame) {
      try {
        const auto tf_msg = tf_buffer_->lookupTransform(
            canonical_frame, source_frame, tf2::TimePointZero);
        source_to_canonical = tf2::transformToEigen(tf_msg.transform);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(
            get_logger(),
            "TF lookup failed for voxel merge: target='%s' source='%s' error=%s. Topic skipped.",
            canonical_frame.c_str(), source_frame.c_str(), ex.what());
        return {};
      }
    }

    GNG::Analysis::IndexVoxelGrid source_grid(source_schema.voxel_size);
    source_grid.setIndexingParams(
        source_schema.x_shift, source_schema.y_shift, source_schema.z_shift,
        static_cast<long>(source_schema.offset));

    GNG::Analysis::IndexVoxelGrid target_grid(output_schema_.voxel_size);
    target_grid.setIndexingParams(
        output_schema_.x_shift, output_schema_.y_shift, output_schema_.z_shift,
        static_cast<long>(output_schema_.offset));

    std::vector<long> ids;
    ids.reserve(msg.data.size());
    for (int64_t raw_id : msg.data) {
      const auto idx = source_grid.getIndexFromFlatId(static_cast<long>(raw_id));
      const Eigen::Vector3d source_center =
          ((idx.cast<double>().array() + 0.5) * source_schema.voxel_size).matrix();
      const Eigen::Vector3d canonical_center = source_to_canonical * source_center;
      const Eigen::Vector3i target_idx = ::common::geometry::VoxelUtils::worldToVoxel(
          canonical_center.cast<float>(), static_cast<float>(output_schema_.voxel_size));
      ids.push_back(target_grid.getFlatVoxelId(target_idx));
    }

    return uniqueSorted(std::move(ids));
  }

  void publishLatest() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!dirty_ || !has_output_schema_) {
      return;
    }

    std::vector<long> merged_ids;
    merged_ids.reserve(1024);
    const std::string canonical_frame = !target_frame_id_.empty() ? target_frame_id_ : output_frame_id_;
    if (output_frame_id_.empty()) {
      output_frame_id_ = canonical_frame;
    }

    for (const auto &topic : input_topics_) {
      auto it = latest_by_topic_.find(topic);
      if (it == latest_by_topic_.end() || !it->second.ready) {
        continue;
      }
      const auto source_schema = schemaFromMsg(it->second.msg, output_schema_.voxel_size);
      const auto ids = convertToOutputIds(
          it->second.msg, source_schema, it->second.msg.header.frame_id, canonical_frame);
      merged_ids.insert(merged_ids.end(), ids.begin(), ids.end());
    }

    merged_ids = uniqueSorted(std::move(merged_ids));

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = canonical_frame;

    voxel_msgs::msg::Voxel merged_msg;
    merged_msg.header = header;
    merged_msg.voxel_size = static_cast<float>(output_schema_.voxel_size);
    merged_msg.x_shift = output_schema_.x_shift;
    merged_msg.y_shift = output_schema_.y_shift;
    merged_msg.z_shift = output_schema_.z_shift;
    merged_msg.offset = output_schema_.offset;
    merged_msg.data.reserve(merged_ids.size());
    for (long id : merged_ids) {
      merged_msg.data.push_back(static_cast<int64_t>(id));
    }

    processor_.getGrid().setVoxelSize(output_schema_.voxel_size);
    processor_.getGrid().setIndexingParams(
        output_schema_.x_shift, output_schema_.y_shift, output_schema_.z_shift,
        static_cast<long>(output_schema_.offset));

    const auto danger_ids = danger_inflation_ > 0.0
        ? processor_.dilate(merged_ids, static_cast<float>(danger_inflation_))
        : std::vector<long>{};

    merged_voxel_pub_->publish(std::move(merged_msg));
    occupied_pub_->publish(makeInt64Msg(merged_ids));
    danger_pub_->publish(makeInt64Msg(danger_ids));
    dirty_ = false;

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Merged voxel topics=%zu occupied=%zu danger=%zu frame=%s voxel_size=%.4f",
        input_topics_.size(), merged_ids.size(), danger_ids.size(), canonical_frame.c_str(),
        output_schema_.voxel_size);
  }

  std::vector<std::string> input_topics_;
  std::string output_voxel_topic_;
  std::string occupied_voxels_topic_;
  std::string danger_voxels_topic_;
  std::string target_frame_id_;
  std::string output_frame_id_;
  double target_voxel_size_ = 0.0;
  int target_x_shift_ = 42;
  int target_y_shift_ = 21;
  int target_z_shift_ = 0;
  std::int64_t target_offset_ = 1000000L;
  double danger_inflation_ = 0.0;
  double publish_hz_ = 10.0;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr merged_voxel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr occupied_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr danger_pub_;
  std::vector<rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, TopicState> latest_by_topic_;
  robot_sim::analysis::VoxelProcessor processor_;
  voxel_idx::VoxelIndexingSchema output_schema_;
  bool has_output_schema_ = false;
  bool dirty_ = false;
};

}  // namespace robot_sim::bridge

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::VoxelTopicMergerNode)

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::bridge::VoxelTopicMergerNode>(
      rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
