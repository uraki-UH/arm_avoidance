#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "goal_node_selection.hpp"

namespace robot_sim::planning {

class topological_map_goal_selector_node : public rclcpp::Node {
 public:
  topological_map_goal_selector_node() : Node("topological_map_goal_selector_node"), tf_buffer_(get_clock()) {
    options_.num_candidates = static_cast<std::size_t>(std::max<int64_t>(1, declare_parameter("candidate_count", 8)));
    options_.allow_collision = !declare_parameter("non_collision_only", true);
    options_.orientation_weight = declare_parameter("orientation_weight", 0.25);
    options_.manipulability_weight = declare_parameter("manipulability_weight", 0.25);
    const auto goal_update_hz = declare_parameter("goal_update_hz", 5.0);
    if (!std::isfinite(goal_update_hz) || goal_update_hz <= 0.0)
      throw std::invalid_argument("goal_update_hzには正の有限値が必要です");
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    output_pub_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
        declare_parameter("output_topic", "/selected_Tmap"), qos);
    ids_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
        declare_parameter("goal_candidate_ids_topic", "/selected_goal_candidate_ids"), qos);
    ids_pub_->publish(std_msgs::msg::Int32MultiArray{});
    // 受信時は最新スナップショットのみ保持。目標更新周期への計算集約。
    map_sub_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
        declare_parameter("topological_map_topic", "/ToPoDualArm/Tmap_static"), qos,
        [this](ais_gng_msgs::msg::TopologicalMap::ConstSharedPtr msg) { map_ = std::move(msg); });
    candidates_sub_ = create_subscription<gng_control_msgs::msg::GraspCandidateArray>(
        declare_parameter("candidate_topic", "/grasp_pose_cands"), qos,
        [this](gng_control_msgs::msg::GraspCandidateArray::ConstSharedPtr msg) { candidates_ = std::move(msg); });
    const auto feature_topic = declare_parameter("node_feature_topic", "/ToPoDualArm/topological_node_features");
    if (!feature_topic.empty() && options_.manipulability_weight > 0.0) {
      features_sub_ = create_subscription<ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray>(
          feature_topic, rclcpp::QoS(1).reliable(),
          [this](ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray::ConstSharedPtr msg) { features_ = std::move(msg); });
    }
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    // 候補が静止していてもTF変化を反映。待機timeoutなしの最新TF参照。
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / goal_update_hz), [this] {
      if (!candidates_) return;
      auto selected = select_goal_nodes(map_.get(), *candidates_, features_.get(), options_,
          [this](const std::string &target, const std::string &source) -> std::optional<tf2::Transform> {
            try {
              tf2::Transform transform;
              tf2::fromMsg(tf_buffer_.lookupTransform(target, source, tf2::TimePointZero).transform, transform);
              return transform;
            } catch (const tf2::TransformException &error) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                  "目標選択のTF取得失敗: %s", error.what());
              return std::nullopt;
            }
          });
      output_pub_->publish(selected.map);
      std_msgs::msg::Int32MultiArray ids;
      ids.data = std::move(selected.ids);
      ids_pub_->publish(ids);
    });
  }

 private:
  goal_selection_options options_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  ais_gng_msgs::msg::TopologicalMap::ConstSharedPtr map_;
  gng_control_msgs::msg::GraspCandidateArray::ConstSharedPtr candidates_;
  ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray::ConstSharedPtr features_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ids_pub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Subscription<gng_control_msgs::msg::GraspCandidateArray>::SharedPtr candidates_sub_;
  rclcpp::Subscription<ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray>::SharedPtr features_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // 計画目標選択の名前空間

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_sim::planning::topological_map_goal_selector_node>());
  rclcpp::shutdown();
  return 0;
}
