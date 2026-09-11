#pragma once

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <gng_control_msgs/msg/grasp_candidate_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace grasping_system::candidate
{

// 候補生成と状態更新の単一配信元。生成方式間で共有する位置到達性評価
class grasp_candidate_publisher
{
  using candidate_msg = gng_control_msgs::msg::GraspCandidate;
  using array_msg = gng_control_msgs::msg::GraspCandidateArray;
  using cell = std::array<double, 3>;

public:
  grasp_candidate_publisher(
    rclcpp::Node &node, const std::string &topic,
    std::function<void(const array_msg &)> on_publish = {})
  : node_(node), tf_buffer_(node.get_clock()), tf_listener_(tf_buffer_, &node, true),
    on_publish_(std::move(on_publish))
  {
    const auto map_topic = node.declare_parameter<std::string>("reachability_map_topic", "");
    latest_.voxel_size = node.declare_parameter<double>("reachability_voxel_size", 0.05);
    const auto origin = node.declare_parameter<std::vector<double>>(
      "reachability_voxel_origin", {0.0, 0.0, 0.0});
    const double publish_hz = node.declare_parameter<double>("reachability_publish_hz", 5.0);
    const double tf_timeout_sec = node.declare_parameter<double>("reachability_tf_timeout_sec", 0.05);
    if (!std::isfinite(latest_.voxel_size) || latest_.voxel_size <= 0.0 ||
      origin.size() != 3 || !std::all_of(origin.begin(), origin.end(),
      [](double value) { return std::isfinite(value); }) ||
      !std::isfinite(publish_hz) || publish_hz <= 0.0 ||
      !std::isfinite(tf_timeout_sec) || tf_timeout_sec < 0.0)
    {
      throw std::invalid_argument("到達セルの寸法・原点・更新周期・TF待機時間が不正です");
    }
    tf_timeout_ = rclcpp::Duration::from_seconds(tf_timeout_sec);
    latest_.voxel_origin.x = origin[0];
    latest_.voxel_origin.y = origin[1];
    latest_.voxel_origin.z = origin[2];
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    publisher_ = node.create_publisher<array_msg>(topic, qos);
    if (!map_topic.empty()) {
      map_sub_ = node.create_subscription<ais_gng_msgs::msg::TopologicalMap>(map_topic, qos,
        [this](ais_gng_msgs::msg::TopologicalMap::ConstSharedPtr map) {
          cells_.clear();
          const bool has_frame_changed = map_frame_ != map->header.frame_id;
          map_frame_ = map->header.frame_id;
          for (const auto &entry : map->nodes) {
            geometry_msgs::msg::Point point;
            point.x = entry.pos.x; point.y = entry.pos.y; point.z = entry.pos.z;
            if (is_finite(point)) cells_.insert(voxel_cell(point));
          }
          evaluate(has_frame_changed);
        });
      timer_ = node.create_wall_timer(std::chrono::duration<double>(1.0 / publish_hz),
        [this]() { evaluate(false); });
    }
  }

  void publish(array_msg msg)
  {
    latest_.header = std::move(msg.header);
    latest_.tcp_frame = std::move(msg.tcp_frame);
    latest_.candidates = std::move(msg.candidates);
    ++latest_.update_id;
    evaluate(true);
  }

private:
  static bool is_finite(const geometry_msgs::msg::Point &p)
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }

  cell voxel_cell(const geometry_msgs::msg::Point &p) const
  {
    const auto &o = latest_.voxel_origin;
    const double size = latest_.voxel_size;
    return {std::floor((p.x - o.x) / size), std::floor((p.y - o.y) / size),
      std::floor((p.z - o.z) / size)};
  }

  void evaluate(bool has_changed)
  {
    if (latest_.update_id == 0) return;
    geometry_msgs::msg::TransformStamped transform;
    bool has_transform = !map_frame_.empty() && !latest_.header.frame_id.empty();
    const bool is_same_frame = map_frame_ == latest_.header.frame_id;
    if (has_transform && !is_same_frame && !latest_.candidates.empty()) {
      try {
        const auto candidate_time = rclcpp::Time(latest_.header.stamp);
        if (candidate_time.nanoseconds() == 0) {
          transform = tf_buffer_.lookupTransform(
            map_frame_, latest_.header.frame_id, tf2::TimePointZero);
        } else {
          // 専用TF受信スレッドによる観測時刻の変換待機。最新TFへの代替なし
          transform = tf_buffer_.lookupTransform(
            map_frame_, latest_.header.frame_id, candidate_time, tf_timeout_);
        }
      } catch (const tf2::TransformException &error) {
        RCLCPP_DEBUG(node_.get_logger(), "Reachability TF unavailable: %s", error.what());
        has_transform = false;
      }
    }
    for (auto &candidate : latest_.candidates) {
      auto state = candidate_msg::UNKNOWN;
      const auto &q = candidate.pose.orientation;
      const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
      if (has_transform && is_finite(candidate.pose.position) && std::isfinite(norm) && norm > 1e-12) {
        geometry_msgs::msg::Point point = candidate.pose.position;
        if (!is_same_frame) tf2::doTransform(candidate.pose.position, point, transform);
        if (is_finite(point)) state = cells_.count(voxel_cell(point)) ? candidate_msg::INSIDE : candidate_msg::OUTSIDE;
      }
      has_changed = has_changed || candidate.state != state;
      candidate.state = state;
    }
    if (!has_changed) return;
    latest_.evaluation_header.frame_id = map_frame_;
    latest_.evaluation_header.stamp = node_.now();
    publisher_->publish(latest_);
    // 状態のみの更新も含めた、同一候補集合に対する表示側への通知
    if (on_publish_) on_publish_(latest_);
  }

  rclcpp::Node &node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Duration tf_timeout_{0, 0};
  array_msg latest_;
  std::string map_frame_;
  std::set<cell> cells_;
  rclcpp::Publisher<array_msg>::SharedPtr publisher_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::function<void(const array_msg &)> on_publish_;
};

}  // grasping_system::candidate 名前空間
