#include <candidate/top_grasp_surface_estimator.hpp>

#include <ais_gng_msgs/msg/planar_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace grasping_system::nodes
{

namespace candidate = grasping_system::candidate;

class TopGraspSurfaceEstimatorNode : public rclcpp::Node
{
public:
  TopGraspSurfaceEstimatorNode()
  : Node("top_grasp_surface_estimator_node"), estimator_(readConfig())
  {
    map_topic_ = declare_parameter<std::string>("topological_map_topic", "/topological_map");
    clusters_topic_ = declare_parameter<std::string>(
      "planar_clusters_topic", "/topological_planar_clusters_incremental");
    const std::string candidate_topic = declare_parameter<std::string>(
      "candidate_topic", "/top_grasp_pose_cands");
    const std::string score_topic = declare_parameter<std::string>(
      "score_topic", "/top_grasp_pose_cand_scores");
    const std::string summary_topic = declare_parameter<std::string>(
      "summary_topic", "/top_grasp_pose_cands/summary");

    const auto output_qos = rclcpp::QoS(1).reliable().transient_local();
    candidate_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      candidate_topic, output_qos);
    score_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      score_topic, output_qos);
    summary_publisher_ = create_publisher<std_msgs::msg::String>(summary_topic, output_qos);

    map_subscription_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      map_topic_, rclcpp::QoS(1),
      [this](ais_gng_msgs::msg::TopologicalMap::SharedPtr map) {
        map_ = std::move(map);
        processMatchedFrame();
      });
    clusters_subscription_ = create_subscription<ais_gng_msgs::msg::PlanarClusterArray>(
      clusters_topic_, output_qos,
      [this](ais_gng_msgs::msg::PlanarClusterArray::SharedPtr clusters) {
        clusters_ = std::move(clusters);
        processMatchedFrame();
      });

    RCLCPP_INFO(
      get_logger(),
      "Top-grasp surface estimator ready: map=%s planes=%s candidates=%s",
      map_topic_.c_str(), clusters_topic_.c_str(), candidate_topic.c_str());
  }

private:
  candidate::TopGraspSurfaceConfig readConfig()
  {
    candidate::TopGraspSurfaceConfig config;
    const auto up_axis = declare_parameter<std::vector<double>>(
      "up_axis", {0.0, 0.0, 1.0});
    if (up_axis.size() != 3U) {
      throw std::invalid_argument("up_axis must contain exactly three values");
    }
    config.up_axis = Eigen::Vector3d(up_axis[0], up_axis[1], up_axis[2]);
    config.minimum_protrusion_distance = declare_parameter<double>(
      "minimum_protrusion_distance", 0.01);
    config.minimum_region_nodes = positiveSizeParameter("minimum_region_nodes", 4);
    config.grasp_size_x = declare_parameter<double>("grasp_size_x", 0.061);
    config.grasp_size_y = declare_parameter<double>("grasp_size_y", 0.074);
    config.footprint_margin = declare_parameter<double>("footprint_margin", 0.005);
    config.footprint_padding = declare_parameter<double>("footprint_padding", 0.005);
    config.tcp_standoff = declare_parameter<double>("tcp_standoff", 0.0);
    config.maximum_candidates = positiveSizeParameter("maximum_candidates", 20);
    return config;
  }

  std::size_t positiveSizeParameter(const std::string &name, int default_value)
  {
    const int value = declare_parameter<int>(name, default_value);
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return static_cast<std::size_t>(value);
  }

  void processMatchedFrame()
  {
    if (!map_ || !clusters_ || map_->frame_number != clusters_->frame_number ||
      map_->header.frame_id != clusters_->header.frame_id)
    {
      return;
    }
    if (last_processed_frame_ == map_->frame_number &&
      last_processed_stamp_ == stampNanoseconds(map_->header.stamp))
    {
      return;
    }
    last_processed_frame_ = map_->frame_number;
    last_processed_stamp_ = stampNanoseconds(map_->header.stamp);

    const auto started = std::chrono::steady_clock::now();
    const auto result = estimator_.estimate(*map_, *clusters_);
    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    publishCandidates(result);
    publishSummary(result, processing_ms);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Top grasp: regions=%zu adjacent=%zu accepted=%zu small=%zu invalid=%zu oversize=%zu low_protrusion=%zu calc=%.2fms",
      result.region_count, result.adjacent_region_pair_count, result.candidates.size(),
      result.rejected_small_region, result.rejected_invalid_region,
      result.rejected_oversize_region, result.rejected_low_protrusion_region,
      processing_ms);
  }

  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time &stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
  }

  void publishCandidates(const candidate::TopGraspSurfaceResult &result)
  {
    geometry_msgs::msg::PoseArray poses;
    poses.header = map_->header;
    poses.poses.reserve(result.candidates.size());
    std_msgs::msg::Float32MultiArray scores;
    scores.data.reserve(result.candidates.size());
    for (const auto &surface : result.candidates) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = surface.tcp_position.x();
      pose.position.y = surface.tcp_position.y();
      pose.position.z = surface.tcp_position.z();
      pose.orientation.x = surface.tcp_orientation.x();
      pose.orientation.y = surface.tcp_orientation.y();
      pose.orientation.z = surface.tcp_orientation.z();
      pose.orientation.w = surface.tcp_orientation.w();
      poses.poses.push_back(std::move(pose));
      scores.data.push_back(static_cast<float>(surface.footprint_fill_ratio));
    }
    candidate_publisher_->publish(std::move(poses));
    score_publisher_->publish(std::move(scores));
  }

  void publishSummary(
    const candidate::TopGraspSurfaceResult &result, double processing_ms)
  {
    std_msgs::msg::String message;
    std::ostringstream stream;
    stream << "{\"status\":\"ready\""
           << ",\"frame_number\":" << map_->frame_number
           << ",\"region_count\":" << result.region_count
           << ",\"adjacent_region_pair_count\":" << result.adjacent_region_pair_count
           << ",\"candidate_count\":" << result.candidates.size()
           << ",\"rejected_small_region\":" << result.rejected_small_region
           << ",\"rejected_invalid_region\":" << result.rejected_invalid_region
           << ",\"rejected_oversize_region\":"
           << result.rejected_oversize_region
           << ",\"rejected_low_protrusion_region\":"
           << result.rejected_low_protrusion_region
           << ",\"processing_ms\":" << processing_ms
           << ",\"candidates\":[";
    for (std::size_t index = 0U; index < result.candidates.size(); ++index) {
      if (index != 0U) {
        stream << ',';
      }
      const auto &surface = result.candidates[index];
      stream << "{\"index\":" << index
             << ",\"cluster_id\":" << surface.cluster_id
             << ",\"node_count\":" << surface.node_indices.size()
             << ",\"adjacent_region_count\":" << surface.adjacent_region_count
             << ",\"minimum_neighbor_plane_distance\":";
      if (surface.has_neighbor_plane_distance) {
        stream << surface.minimum_neighbor_plane_distance;
      } else {
        stream << "null";
      }
      stream
             << ",\"extent_x\":" << surface.extent_x
             << ",\"extent_y\":" << surface.extent_y
             << ",\"surface_height\":" << surface.surface_height
             << ",\"footprint_fill_ratio\":" << surface.footprint_fill_ratio
             << '}';
    }
    stream << "]}";
    message.data = stream.str();
    summary_publisher_->publish(std::move(message));
  }

  candidate::TopGraspSurfaceEstimator estimator_;
  std::string map_topic_;
  std::string clusters_topic_;
  std::uint32_t last_processed_frame_ = std::numeric_limits<std::uint32_t>::max();
  std::int64_t last_processed_stamp_ = std::numeric_limits<std::int64_t>::min();
  ais_gng_msgs::msg::TopologicalMap::SharedPtr map_;
  ais_gng_msgs::msg::PlanarClusterArray::SharedPtr clusters_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_subscription_;
  rclcpp::Subscription<ais_gng_msgs::msg::PlanarClusterArray>::SharedPtr clusters_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr candidate_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr score_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_publisher_;
};

}  // namespace grasping_system::nodes

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grasping_system::nodes::TopGraspSurfaceEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
