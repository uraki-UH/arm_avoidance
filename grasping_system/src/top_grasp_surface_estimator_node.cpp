#include <candidate/top_grasp_surface_estimator.hpp>
#include <candidate/grasp_candidate_publisher.hpp>

#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
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
      "candidate_topic", "/grasp_pose_cands");
    const std::string candidate_nodes_topic = declare_parameter<std::string>(
      "candidate_nodes_topic", candidate_topic + "/nodes");
    candidate_node_diameter_ = declare_parameter<double>("candidate_node_diameter", 0.012);
    if (!std::isfinite(candidate_node_diameter_) || candidate_node_diameter_ <= 0.0) {
      throw std::invalid_argument("candidate_node_diameter must be finite and positive");
    }
    const std::string score_topic = declare_parameter<std::string>(
      "score_topic", "/grasp_pose_cand_scores");
    const std::string summary_topic = declare_parameter<std::string>(
      "summary_topic", "/grasp_pose_cands/summary");
    candidate_frame_ = declare_parameter<std::string>("candidate_frame", "");
    tcp_frame_ = declare_parameter<std::string>("tcp_frame", "L_tcp");
    if (tcp_frame_.empty()) {
      throw std::invalid_argument("tcp_frame is invalid");
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    const auto output_qos = rclcpp::QoS(1).reliable().transient_local();
    candidate_publisher_ = std::make_unique<candidate::grasp_candidate_publisher>(*this, candidate_topic);
    candidate_nodes_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      candidate_nodes_topic, output_qos);
    score_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      score_topic, output_qos);
    summary_publisher_ = create_publisher<std_msgs::msg::String>(summary_topic, output_qos);

    map_subscription_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      map_topic_, rclcpp::QoS(1),
      [this](ais_gng_msgs::msg::TopologicalMap::SharedPtr map) {
        map_ = std::move(map);
        processMatchedFrame();
      });
    clusters_subscription_ = create_subscription<ais_gng_msgs::msg::PlaneClusterArray>(
      clusters_topic_, output_qos,
      [this](ais_gng_msgs::msg::PlaneClusterArray::SharedPtr clusters) {
        clusters_ = std::move(clusters);
        processMatchedFrame();
      });

    RCLCPP_INFO(
      get_logger(),
      "Top-grasp surface estimator ready: map=%s planes=%s candidates=%s frame=%s tcp=%s",
      map_topic_.c_str(), clusters_topic_.c_str(), candidate_topic.c_str(),
      candidate_frame_.empty() ? "input" : candidate_frame_.c_str(), tcp_frame_.c_str());
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
    config.max_surface_tilt_deg = declare_parameter<double>("max_surface_tilt_deg", 25.0);
    config.enable_nonplane_attachment = declare_parameter<bool>("enable_nonplane_attachment", true);
    config.nonplane_margin = declare_parameter<double>("nonplane_margin", 0.03);
    config.max_nonplane_graph_dist = declare_parameter<double>("max_nonplane_graph_dist", 0.15);
    config.max_nonplane_depth = declare_parameter<double>("max_nonplane_depth", 0.08);
    config.max_nonplane_height = declare_parameter<double>("max_nonplane_height", 0.01);
    config.enable_approach_check = declare_parameter<bool>("enable_approach_check", true);
    config.approach_height = declare_parameter<double>("approach_height", 0.10);
    config.approach_margin = declare_parameter<double>("approach_margin", 0.01);
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
      map_->header.frame_id != clusters_->header.frame_id ||
      stampNanoseconds(map_->header.stamp) != stampNanoseconds(clusters_->header.stamp))
    {
      return;
    }
    if (last_processed_frame_ == map_->frame_number &&
      last_processed_stamp_ == stampNanoseconds(map_->header.stamp))
    {
      return;
    }
    ais_gng_msgs::msg::TopologicalMap candidate_map;
    ais_gng_msgs::msg::PlaneClusterArray candidate_clusters;
    if (!transformCandidateInputs(candidate_map, candidate_clusters)) {
      publishTfUnavailable();
      return;
    }

    const auto started = std::chrono::steady_clock::now();
    const auto result = estimator_.estimate(candidate_map, candidate_clusters);
    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    last_processed_frame_ = map_->frame_number;
    last_processed_stamp_ = stampNanoseconds(map_->header.stamp);
    publishCandidates(result, candidate_map.header);
    publish_candidate_nodes(result, candidate_map);
    publishSummary(result, processing_ms);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Top grasp: regions=%zu adjacent=%zu accepted=%zu small=%zu invalid=%zu oversize=%zu low_protrusion=%zu tilt=%zu attached_oversize=%zu approach_obstacle=%zu calc=%.2fms",
      result.region_count, result.adjacent_region_pair_count, result.candidates.size(),
      result.rejected_small_region, result.rejected_invalid_region,
      result.rejected_oversize_region, result.rejected_low_protrusion_region,
      result.rejected_surface_tilt, result.rejected_attached_oversize,
      result.rejected_approach_obstacle,
      processing_ms);
  }

  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time &stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
  }

  static void transformPoint(
    geometry_msgs::msg::Point32 &source,
    const geometry_msgs::msg::TransformStamped &transform)
  {
    geometry_msgs::msg::Point input;
    input.x = source.x;
    input.y = source.y;
    input.z = source.z;
    geometry_msgs::msg::Point output;
    tf2::doTransform(input, output, transform);
    source.x = static_cast<float>(output.x);
    source.y = static_cast<float>(output.y);
    source.z = static_cast<float>(output.z);
  }

  static void transformVector(
    geometry_msgs::msg::Vector3 &source,
    const geometry_msgs::msg::TransformStamped &transform)
  {
    geometry_msgs::msg::Vector3 output;
    tf2::doTransform(source, output, transform);
    source = output;
  }

  bool transformCandidateInputs(
    ais_gng_msgs::msg::TopologicalMap &candidate_map,
    ais_gng_msgs::msg::PlaneClusterArray &candidate_clusters)
  {
    candidate_map = *map_;
    candidate_clusters = *clusters_;
    if (map_->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Top grasp skipped: 入力グラフのframe_idが空です");
      return false;
    }
    if (candidate_frame_.empty() || map_->header.frame_id == candidate_frame_) {
      return true;
    }
    if (stampNanoseconds(map_->header.stamp) == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Top grasp skipped: frame=%sから%sへの時刻なしTF変換は許可しません",
        map_->header.frame_id.c_str(), candidate_frame_.c_str());
      return false;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        candidate_frame_, map_->header.frame_id,
        rclcpp::Time(map_->header.stamp));
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Top grasp skipped: frame=%sから%sへのTF取得失敗: %s",
        map_->header.frame_id.c_str(), candidate_frame_.c_str(), error.what());
      return false;
    }

    for (auto &node : candidate_map.nodes) {
      transformPoint(node.pos, transform);
      geometry_msgs::msg::Vector3 normal;
      normal.x = node.normal.x;
      normal.y = node.normal.y;
      normal.z = node.normal.z;
      transformVector(normal, transform);
      node.normal.x = static_cast<float>(normal.x);
      node.normal.y = static_cast<float>(normal.y);
      node.normal.z = static_cast<float>(normal.z);
    }
    // 候補推定で未使用の可視化クラスタの座標系混在防止
    candidate_map.clusters.clear();
    for (auto &cluster : candidate_clusters.clusters) {
      transformPoint(cluster.centroid, transform);
      transformVector(cluster.normal, transform);
      transformVector(cluster.tangent_u, transform);
      transformVector(cluster.tangent_v, transform);
      for (auto &point : cluster.boundary) {
        transformPoint(point, transform);
      }
      for (auto &point : cluster.support_edges) {
        transformPoint(point, transform);
      }
    }
    candidate_map.header.frame_id = candidate_frame_;
    candidate_clusters.header.frame_id = candidate_frame_;
    return true;
  }

  void publishCandidates(
    const candidate::TopGraspSurfaceResult &result,
    const std_msgs::msg::Header &header)
  {
    gng_control_msgs::msg::GraspCandidateArray poses;
    poses.header = header;
    poses.tcp_frame = tcp_frame_;
    poses.candidates.reserve(result.candidates.size());
    std_msgs::msg::Float32MultiArray scores;
    scores.data.reserve(result.candidates.size());
    for (const auto &surface : result.candidates) {
      gng_control_msgs::msg::GraspCandidate entry;
      entry.id = poses.candidates.size();
      entry.shape_score = static_cast<float>(surface.footprint_fill_ratio);
      auto &pose = entry.pose;
      pose.position.x = surface.tcp_position.x();
      pose.position.y = surface.tcp_position.y();
      pose.position.z = surface.tcp_position.z();
      pose.orientation.x = surface.tcp_orientation.x();
      pose.orientation.y = surface.tcp_orientation.y();
      pose.orientation.z = surface.tcp_orientation.z();
      pose.orientation.w = surface.tcp_orientation.w();
      poses.candidates.push_back(std::move(entry));
      scores.data.push_back(static_cast<float>(surface.footprint_fill_ratio));
    }
    candidate_publisher_->publish(std::move(poses));
    score_publisher_->publish(std::move(scores));
  }

  void publish_candidate_nodes(
    const candidate::TopGraspSurfaceResult &result,
    const ais_gng_msgs::msg::TopologicalMap &candidate_map)
  {
    using marker_msg = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.reserve(1U + 2U * result.candidates.size());
    marker_msg clear;
    clear.header = candidate_map.header;
    clear.action = marker_msg::DELETEALL;
    markers.markers.push_back(std::move(clear));
    // 毎回の全置換により、候補減少・空配信・遅延購読時の旧ノード残留を防止
    for (std::size_t candidate_idx = 0; candidate_idx < result.candidates.size(); ++candidate_idx) {
      const auto &surface = result.candidates[candidate_idx];
      for (const bool is_nonplane : {false, true}) {
        const auto &node_indices = is_nonplane ? surface.attached_node_indices : surface.node_indices;
        marker_msg marker;
        marker.header = candidate_map.header;
        marker.ns = is_nonplane ? "grasp_nonplane" : "grasp_plane";
        marker.id = static_cast<std::int32_t>(candidate_idx);
        marker.type = marker_msg::SPHERE_LIST;
        marker.action = marker_msg::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = candidate_node_diameter_;
        marker.color.r = is_nonplane ? 1.0F : 0.1F;
        marker.color.g = is_nonplane ? 0.55F : 0.85F;
        marker.color.b = is_nonplane ? 0.1F : 1.0F;
        marker.color.a = 1.0F;
        marker.points.reserve(node_indices.size());
        for (const auto node_idx : node_indices) {
          if (node_idx >= candidate_map.nodes.size()) {
            continue;
          }
          const auto &pos = candidate_map.nodes[node_idx].pos;
          if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) {
            continue;
          }
          geometry_msgs::msg::Point point;
          point.x = pos.x;
          point.y = pos.y;
          point.z = pos.z;
          marker.points.push_back(point);
        }
        if (!marker.points.empty()) {
          markers.markers.push_back(std::move(marker));
        }
      }
    }
    candidate_nodes_publisher_->publish(std::move(markers));
  }

  void publishTfUnavailable()
  {
    gng_control_msgs::msg::GraspCandidateArray poses;
    poses.header = map_->header;
    poses.header.frame_id = outputFrame();
    poses.tcp_frame = tcp_frame_;
    candidate_publisher_->publish(std::move(poses));
    std_msgs::msg::Float32MultiArray scores;
    score_publisher_->publish(std::move(scores));
    auto marker_map = *map_;
    marker_map.header.frame_id = outputFrame();
    publish_candidate_nodes({}, marker_map);

    std_msgs::msg::String summary;
    std::ostringstream stream;
    stream << "{\"status\":\"tf_unavailable\""
           << ",\"frame_number\":" << map_->frame_number
           << ",\"input_frame\":\"" << map_->header.frame_id << '\"'
           << ",\"candidate_frame\":\"" << outputFrame() << '\"'
           << ",\"tcp_frame\":\"" << tcp_frame_ << '\"'
           << ",\"candidate_count\":0}";
    summary.data = stream.str();
    summary_publisher_->publish(std::move(summary));
  }

  void publishSummary(
    const candidate::TopGraspSurfaceResult &result, double processing_ms)
  {
    std_msgs::msg::String message;
    std::ostringstream stream;
    stream << "{\"status\":\"ready\""
           << ",\"frame_number\":" << map_->frame_number
           << ",\"candidate_frame\":\"" << outputFrame() << '\"'
           << ",\"tcp_frame\":\"" << tcp_frame_ << '\"'
           << ",\"region_count\":" << result.region_count
           << ",\"adjacent_region_pair_count\":" << result.adjacent_region_pair_count
           << ",\"candidate_count\":" << result.candidates.size()
           << ",\"rejected_small_region\":" << result.rejected_small_region
           << ",\"rejected_invalid_region\":" << result.rejected_invalid_region
           << ",\"rejected_oversize_region\":"
           << result.rejected_oversize_region
           << ",\"rejected_low_protrusion_region\":"
           << result.rejected_low_protrusion_region
           << ",\"rejected_surface_tilt\":" << result.rejected_surface_tilt
           << ",\"rejected_attached_oversize\":" << result.rejected_attached_oversize
           << ",\"rejected_approach_obstacle\":" << result.rejected_approach_obstacle
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
             << ",\"attached_node_num\":" << surface.attached_node_indices.size()
             << ",\"attached_component_num\":" << surface.attached_component_num
             << ",\"target_extent_x\":" << surface.target_extent_x
             << ",\"target_extent_y\":" << surface.target_extent_y
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

  std::string outputFrame() const
  {
    if (!candidate_frame_.empty()) {
      return candidate_frame_;
    }
    return map_ ? map_->header.frame_id : std::string{};
  }

  std::string map_topic_;
  std::string clusters_topic_;
  std::string candidate_frame_;
  std::string tcp_frame_;
  std::uint32_t last_processed_frame_ = std::numeric_limits<std::uint32_t>::max();
  std::int64_t last_processed_stamp_ = std::numeric_limits<std::int64_t>::min();
  ais_gng_msgs::msg::TopologicalMap::SharedPtr map_;
  ais_gng_msgs::msg::PlaneClusterArray::SharedPtr clusters_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_subscription_;
  rclcpp::Subscription<ais_gng_msgs::msg::PlaneClusterArray>::SharedPtr clusters_subscription_;
  std::unique_ptr<candidate::grasp_candidate_publisher> candidate_publisher_;
  double candidate_node_diameter_ = 0.012;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_nodes_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr score_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace grasping_system::nodes

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grasping_system::nodes::TopGraspSurfaceEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
