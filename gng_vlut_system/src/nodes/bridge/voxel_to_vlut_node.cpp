#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <mutex>
#include <unordered_set>
#include <vector>

#include "core/common/constants.hpp"
#include "common/voxel_utils.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"
#include "safety_engine/vlut/voxel_processor.hpp"

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace robot_sim::bridge {

class VoxelToVlutNode : public rclcpp::Node {
public:
  explicit VoxelToVlutNode(const rclcpp::NodeOptions &options)
  : Node("voxel_to_vlut_node", options),
    grid_(::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE),
    processor_(::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE) {
    declare_parameter<std::string>("input_topic", "/self_recognition/voxel_mask");
    declare_parameter<std::string>("occupied_voxels_topic", "/occupied_voxels");
    declare_parameter<std::string>("danger_voxels_topic", "/danger_voxels");
    declare_parameter<std::string>("target_frame_id", "world");
    declare_parameter<double>("danger_inflation", ::robot_sim::common::Constants::DEFAULT_SAFETY_MARGIN);
    declare_parameter<double>("output_voxel_size", ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
    declare_parameter<double>("publish_hz", 30.0);
    declare_parameter<double>("identity_transform_translation_th", 1e-9);
    declare_parameter<double>("identity_transform_rotation_th", 1e-9);
    declare_parameter<double>("voxel_size_match_th", 1e-9);

    input_topic_ = get_parameter("input_topic").as_string();
    occupied_topic_ = get_parameter("occupied_voxels_topic").as_string();
    danger_topic_ = get_parameter("danger_voxels_topic").as_string();
    target_frame_id_ = get_parameter("target_frame_id").as_string();
    danger_inflation_ = std::max(0.0, get_parameter("danger_inflation").as_double());
    output_voxel_size_ = std::max(1e-6, get_parameter("output_voxel_size").as_double());
    identity_transform_translation_th_ = std::max(
        0.0, get_parameter("identity_transform_translation_th").as_double());
    identity_transform_rotation_th_ = std::max(
        0.0, get_parameter("identity_transform_rotation_th").as_double());
    voxel_size_match_th_ = std::max(
        0.0, get_parameter("voxel_size_match_th").as_double());
    const double hz = std::max(0.1, get_parameter("publish_hz").as_double());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    mask_sub_ = create_subscription<voxel_msgs::msg::Voxel>(
        input_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&VoxelToVlutNode::maskCallback, this, std::placeholders::_1));

    occupied_pub_ = create_publisher<std_msgs::msg::Int64MultiArray>(
        occupied_topic_, rclcpp::QoS(1).reliable().transient_local());
    danger_pub_ = create_publisher<std_msgs::msg::Int64MultiArray>(
        danger_topic_, rclcpp::QoS(1).reliable().transient_local());

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
        std::bind(&VoxelToVlutNode::publishLatest, this));

    graph_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        [this]() {
          RCLCPP_INFO(
              get_logger(),
              "Topic graph: input pubs=%zu occupied pubs=%zu danger pubs=%zu",
              count_publishers(input_topic_),
              count_publishers(occupied_topic_),
              count_publishers(danger_topic_));
        });

    RCLCPP_INFO(get_logger(),
                "VoxelToVlutNode initialized. input=%s occupied=%s danger=%s target_frame=%s inflation=%.4f output_voxel_size=%.4f",
                input_topic_.c_str(), occupied_topic_.c_str(),
                danger_topic_.c_str(), target_frame_id_.c_str(), danger_inflation_, output_voxel_size_);
  }

private:
  static std::vector<long> uniqueSorted(std::vector<long> values) {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
    return values;
  }

  bool isIdentityTransform(const Eigen::Isometry3d &source_to_target) const {
    return source_to_target.translation().norm() <= identity_transform_translation_th_ &&
           (source_to_target.linear() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() <=
               identity_transform_rotation_th_;
  }

  bool hasSameVoxelSize(double input_voxel_size) const {
    return std::abs(input_voxel_size - output_voxel_size_) <= voxel_size_match_th_;
  }

  std::vector<long> computeDangerShell(const std::vector<long> &occupied) {
    if (occupied.empty() || danger_inflation_ <= 0.0) {
      return {};
    }

    std::vector<long> occupied_sorted = uniqueSorted(occupied);
    std::vector<long> danger = processor_.dilate(
        occupied_sorted, static_cast<float>(danger_inflation_));

    std::vector<long> shell;
    shell.reserve(danger.size());
    std::set_difference(danger.begin(), danger.end(),
                        occupied_sorted.begin(), occupied_sorted.end(),
                        std::back_inserter(shell));
    return shell;
  }

  std::vector<long> convertVoxelIds(
      const std::vector<long> &input_ids,
      double input_voxel_size,
      double output_voxel_size,
      const Eigen::Isometry3d &source_to_target) const {
    if (input_ids.empty()) {
      return {};
    }

    GNG::Analysis::IndexVoxelGrid input_grid(input_voxel_size);
    input_grid.setIndexingParams(grid_.getXShift(), grid_.getYShift(), grid_.getZShift(), grid_.getOffset());

    GNG::Analysis::IndexVoxelGrid output_grid(output_voxel_size);
    output_grid.setIndexingParams(grid_.getXShift(), grid_.getYShift(), grid_.getZShift(), grid_.getOffset());

    std::vector<long> output_ids;
    output_ids.reserve(input_ids.size());
    for (long vid : input_ids) {
      const Eigen::Vector3i input_idx = input_grid.getIndexFromFlatId(vid);
      const Eigen::Vector3f center_source =
          (input_idx.cast<float>() + Eigen::Vector3f::Constant(0.5f)) *
          static_cast<float>(input_voxel_size);
      const Eigen::Vector3f center =
          (source_to_target * center_source.cast<double>()).cast<float>();
      const Eigen::Vector3i output_idx =
          ::common::geometry::VoxelUtils::worldToVoxel(center, static_cast<float>(output_voxel_size));
      output_ids.push_back(output_grid.getFlatVoxelId(output_idx));
    }

    return uniqueSorted(std::move(output_ids));
  }

  static std_msgs::msg::Int64MultiArray makeMsg(
      const std::vector<long> &values) {
    std_msgs::msg::Int64MultiArray msg;
    msg.data.reserve(values.size());
    for (long v : values) {
      msg.data.push_back(static_cast<int64_t>(v));
    }
    return msg;
  }

  void maskCallback(const voxel_msgs::msg::Voxel::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    const double input_voxel_size = msg->voxel_size > 0.0f
                                        ? static_cast<double>(msg->voxel_size)
                                        : grid_.getVoxelSize();
    const std::string source_frame = msg->header.frame_id.empty() ? target_frame_id_ : msg->header.frame_id;
    Eigen::Isometry3d source_to_target = Eigen::Isometry3d::Identity();
    if (!target_frame_id_.empty() && target_frame_id_ != source_frame) {
      try {
        const auto ts = tf_buffer_->lookupTransform(target_frame_id_, source_frame, tf2::TimePointZero);
        source_to_target = tf2::transformToEigen(ts.transform);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "TF lookup failed for voxel baking: target='%s' source='%s' error=%s. Skipping voxel baking for this message.",
            target_frame_id_.c_str(), source_frame.c_str(), ex.what());
        return;
      }
    }

    grid_.setVoxelSize(output_voxel_size_);
    grid_.setIndexingParams(msg->x_shift, msg->y_shift, msg->z_shift, msg->offset);
    processor_.getGrid().setVoxelSize(output_voxel_size_);
    processor_.getGrid().setIndexingParams(msg->x_shift, msg->y_shift, msg->z_shift, msg->offset);

    const bool can_reuse_voxel_ids =
        isIdentityTransform(source_to_target) && hasSameVoxelSize(input_voxel_size);
    std::vector<long> input_ids(msg->data.begin(), msg->data.end());
    latest_occupied_ = can_reuse_voxel_ids
                           ? uniqueSorted(std::move(input_ids))
                           : convertVoxelIds(
                               input_ids, input_voxel_size, output_voxel_size_, source_to_target);
    latest_danger_ = computeDangerShell(latest_occupied_);
    has_new_data_ = true;

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Mask received: input_voxel_size=%.4f input=%zu occupied=%zu danger=%zu source_frame=%s target_frame=%s voxel_id_reuse=%s",
        input_voxel_size, msg->data.size(), latest_occupied_.size(),
        latest_danger_.size(), source_frame.c_str(), target_frame_id_.c_str(),
        can_reuse_voxel_ids ? "true" : "false");
  }

  void publishLatest() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_new_data_) {
      return;
    }

    occupied_pub_->publish(makeMsg(latest_occupied_));
    danger_pub_->publish(makeMsg(latest_danger_));
    has_new_data_ = false;
  }

  std::string input_topic_;
  std::string occupied_topic_;
  std::string danger_topic_;
  std::string target_frame_id_ = "world";
  double danger_inflation_ = 0.0;
  double output_voxel_size_ = ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE;
  double identity_transform_translation_th_ = 1e-9;
  double identity_transform_rotation_th_ = 1e-9;
  double voxel_size_match_th_ = 1e-9;

  GNG::Analysis::IndexVoxelGrid grid_;
  robot_sim::analysis::VoxelProcessor processor_;
  std::vector<long> latest_occupied_;
  std::vector<long> latest_danger_;
  bool has_new_data_ = false;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr mask_sub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr occupied_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr danger_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
};

} // namespace robot_sim::bridge

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::VoxelToVlutNode)

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<robot_sim::bridge::VoxelToVlutNode>(
          rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
