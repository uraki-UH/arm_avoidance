#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <algorithm>
#include <chrono>
#include <iterator>
#include <mutex>
#include <unordered_set>
#include <vector>

#include "common/constants.hpp"
#include "common/voxel_utils.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"
#include "safety_engine/vlut/voxel_processor.hpp"

namespace robot_sim::bridge {

class SelfRecognitionVoxelBridgeNode : public rclcpp::Node {
public:
  explicit SelfRecognitionVoxelBridgeNode(const rclcpp::NodeOptions &options)
  : Node("self_recognition_voxel_bridge_node", options),
    grid_(::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE),
    processor_(::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE) {
    declare_parameter<std::string>("input_topic", "/self_recognition/voxel_mask");
    declare_parameter<std::string>("occupied_voxels_topic", "/occupied_voxels");
    declare_parameter<std::string>("danger_voxels_topic", "/danger_voxels");
    declare_parameter<double>("danger_inflation", ::robot_sim::common::Constants::DEFAULT_SAFETY_MARGIN);
    declare_parameter<double>("output_voxel_size", ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
    declare_parameter<double>("publish_hz", 30.0);

    input_topic_ = get_parameter("input_topic").as_string();
    input_topic_relative_ = input_topic_.rfind('/') == 0
                                ? input_topic_.substr(1)
                                : input_topic_;
    occupied_topic_ = get_parameter("occupied_voxels_topic").as_string();
    danger_topic_ = get_parameter("danger_voxels_topic").as_string();
    danger_inflation_ = std::max(0.0, get_parameter("danger_inflation").as_double());
    output_voxel_size_ = std::max(1e-6, get_parameter("output_voxel_size").as_double());
    const double hz = std::max(0.1, get_parameter("publish_hz").as_double());

    mask_sub_ = create_subscription<voxel_msgs::msg::Voxel>(
        input_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&SelfRecognitionVoxelBridgeNode::maskCallback, this, std::placeholders::_1));
    if (input_topic_relative_ != input_topic_) {
      mask_sub_relative_ = create_subscription<voxel_msgs::msg::Voxel>(
          input_topic_relative_, rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&SelfRecognitionVoxelBridgeNode::maskCallback, this, std::placeholders::_1));
    }

    occupied_pub_ = create_publisher<std_msgs::msg::Int64MultiArray>(
        occupied_topic_, rclcpp::QoS(1).reliable().transient_local());
    danger_pub_ = create_publisher<std_msgs::msg::Int64MultiArray>(
        danger_topic_, rclcpp::QoS(1).reliable().transient_local());

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
        std::bind(&SelfRecognitionVoxelBridgeNode::publishLatest, this));

    graph_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        [this]() {
          RCLCPP_INFO(
              get_logger(),
              "Topic graph: input pubs(abs)=%zu input pubs(rel)=%zu occupied pubs=%zu danger pubs=%zu",
              count_publishers(input_topic_),
              count_publishers(input_topic_relative_),
              count_publishers(occupied_topic_),
              count_publishers(danger_topic_));
        });

    RCLCPP_INFO(get_logger(),
                "SelfRecognitionVoxelBridgeNode initialized. input=%s occupied=%s danger=%s inflation=%.4f output_voxel_size=%.4f",
                input_topic_.c_str(), occupied_topic_.c_str(),
                danger_topic_.c_str(), danger_inflation_, output_voxel_size_);
  }

private:
  static std::vector<long> uniqueSorted(std::vector<long> values) {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
    return values;
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
      double output_voxel_size) const {
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
      const Eigen::Vector3f center =
          (input_idx.cast<float>() + Eigen::Vector3f::Constant(0.5f)) *
          static_cast<float>(input_voxel_size);
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
    grid_.setVoxelSize(output_voxel_size_);
    grid_.setIndexingParams(msg->x_shift, msg->y_shift, msg->z_shift, msg->offset);
    processor_.getGrid().setVoxelSize(output_voxel_size_);
    processor_.getGrid().setIndexingParams(msg->x_shift, msg->y_shift, msg->z_shift, msg->offset);

    latest_occupied_ = convertVoxelIds(
        std::vector<long>(msg->data.begin(), msg->data.end()),
        input_voxel_size, output_voxel_size_);
    latest_danger_ = computeDangerShell(latest_occupied_);
    has_new_data_ = true;

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Mask received: input_voxel_size=%.4f input=%zu occupied=%zu danger=%zu",
        input_voxel_size, msg->data.size(), latest_occupied_.size(),
        latest_danger_.size());
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
  std::string input_topic_relative_;
  std::string occupied_topic_;
  std::string danger_topic_;
  double danger_inflation_ = 0.0;
  double output_voxel_size_ = ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE;

  GNG::Analysis::IndexVoxelGrid grid_;
  robot_sim::analysis::VoxelProcessor processor_;
  std::vector<long> latest_occupied_;
  std::vector<long> latest_danger_;
  bool has_new_data_ = false;

  std::mutex mutex_;
  rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr mask_sub_;
  rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr mask_sub_relative_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr occupied_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr danger_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
};

} // namespace robot_sim::bridge

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::SelfRecognitionVoxelBridgeNode)

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<robot_sim::bridge::SelfRecognitionVoxelBridgeNode>(
          rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
