#include <candidate/grasp_voxel_matcher.hpp>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace grasping_system::nodes
{

namespace candidate = grasping_system::candidate;

class GraspVoxelMatcherNode : public rclcpp::Node
{
public:
  GraspVoxelMatcherNode()
  : Node("grasp_voxel_matcher_node")
  {
    object_voxels_topic_ = declare_parameter<std::string>(
      "object_voxels_topic", "/topological_grid_voxels");
    environment_voxels_topic_ = declare_parameter<std::string>(
      "environment_voxels_topic", "");
    required_graph_topic_ = declare_parameter<std::string>(
      "required_graph_topic", "grip_V_topological_map");
    undersize_graph_topic_ = declare_parameter<std::string>(
      "undersize_graph_topic", "grip_minV_topological_map");
    forbidden_graph_topic_ = declare_parameter<std::string>(
      "forbidden_graph_topic", "grip_baseV_topological_map");
    const std::string candidate_topic = declare_parameter<std::string>(
      "candidate_topic", "/grasp_voxel_candidates");
    const std::string summary_topic = declare_parameter<std::string>(
      "summary_topic", "/grasp_voxel_candidates/summary");

    match_config_.minimum_required_occupancy_ratio = declare_parameter<double>(
      "minimum_required_occupancy_ratio", 0.1);
    match_config_.minimum_required_hits = positiveSizeParameter(
      "minimum_required_hits", 3, true);
    match_config_.minimum_outside_undersize_hits = positiveSizeParameter(
      "minimum_outside_undersize_hits", 1, true);
    match_config_.maximum_forbidden_hits = positiveSizeParameter(
      "maximum_forbidden_hits", 0, true);
    match_config_.maximum_anchor_voxels = positiveSizeParameter(
      "maximum_anchor_voxels", 500, false);
    match_config_.maximum_candidates = positiveSizeParameter(
      "maximum_candidates", 50, false);

    const int update_period_ms = declare_parameter<int>("update_period_ms", 500);
    if (update_period_ms <= 0) {
      throw std::invalid_argument("update_period_ms must be positive");
    }
    orientations_ = loadOrientations();

    const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    candidate_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      candidate_topic, transient_qos);
    summary_pub_ = create_publisher<std_msgs::msg::String>(summary_topic, transient_qos);

    object_voxels_sub_ = create_subscription<voxel_msgs::msg::Voxel>(
      object_voxels_topic_, transient_qos,
      [this](voxel_msgs::msg::Voxel::SharedPtr msg) {
        object_voxels_ = std::move(msg);
        dirty_ = true;
      });
    if (!environment_voxels_topic_.empty() &&
      environment_voxels_topic_ != object_voxels_topic_)
    {
      environment_voxels_sub_ = create_subscription<voxel_msgs::msg::Voxel>(
        environment_voxels_topic_, transient_qos,
        [this](voxel_msgs::msg::Voxel::SharedPtr msg) {
          environment_voxels_ = std::move(msg);
          dirty_ = true;
        });
    }

    required_graph_sub_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      required_graph_topic_, transient_qos,
      [this](ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
        required_graph_ = std::move(msg);
        template_dirty_ = true;
        dirty_ = true;
      });
    if (!undersize_graph_topic_.empty()) {
      undersize_graph_sub_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
        undersize_graph_topic_, transient_qos,
        [this](ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
          undersize_graph_ = std::move(msg);
          template_dirty_ = true;
          dirty_ = true;
        });
    }
    if (!forbidden_graph_topic_.empty()) {
      forbidden_graph_sub_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
        forbidden_graph_topic_, transient_qos,
        [this](ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
          forbidden_graph_ = std::move(msg);
          template_dirty_ = true;
          dirty_ = true;
        });
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(update_period_ms),
      std::bind(&GraspVoxelMatcherNode::updateMatches, this));

    RCLCPP_INFO(
      get_logger(),
      "Grasp voxel matcher ready: objects=%s environment=%s required=%s undersize=%s forbidden=%s orientations=%zu period=%dms",
      object_voxels_topic_.c_str(),
      environment_voxels_topic_.empty() ? "<object voxels>" : environment_voxels_topic_.c_str(),
      required_graph_topic_.c_str(),
      undersize_graph_topic_.empty() ? "<disabled>" : undersize_graph_topic_.c_str(),
      forbidden_graph_topic_.empty() ? "<disabled>" : forbidden_graph_topic_.c_str(),
      orientations_.size(), update_period_ms);
  }

private:
  std::size_t positiveSizeParameter(
    const std::string &name, int default_value, bool allow_zero)
  {
    const int value = declare_parameter<int>(name, default_value);
    if (value < 0 || (!allow_zero && value == 0)) {
      throw std::invalid_argument(
              name + (allow_zero ? " must be non-negative" : " must be positive"));
    }
    return static_cast<std::size_t>(value);
  }

  std::vector<Eigen::Quaterniond> loadOrientations()
  {
    constexpr double kTwoPi = 6.28318530717958647692;
    const auto explicit_rpy = declare_parameter<std::vector<double>>(
      "orientation_rpy", std::vector<double>{});
    const int yaw_samples = declare_parameter<int>("yaw_samples", 12);
    const double roll = declare_parameter<double>("roll", 0.0);
    const double pitch = declare_parameter<double>("pitch", 0.0);
    const double yaw_offset = declare_parameter<double>("yaw_offset", 0.0);
    std::vector<Eigen::Quaterniond> orientations;
    if (!explicit_rpy.empty()) {
      if (explicit_rpy.size() % 3U != 0U) {
        throw std::invalid_argument("orientation_rpy must contain roll,pitch,yaw triples");
      }
      orientations.reserve(explicit_rpy.size() / 3U);
      for (std::size_t index = 0; index < explicit_rpy.size(); index += 3U) {
        orientations.push_back(fromRpy(
          explicit_rpy[index], explicit_rpy[index + 1U], explicit_rpy[index + 2U]));
      }
      return orientations;
    }

    if (yaw_samples <= 0 || !std::isfinite(roll) || !std::isfinite(pitch) ||
      !std::isfinite(yaw_offset))
    {
      throw std::invalid_argument("yaw_samples must be positive and orientation values finite");
    }
    orientations.reserve(static_cast<std::size_t>(yaw_samples));
    for (int index = 0; index < yaw_samples; ++index) {
      const double yaw = yaw_offset + kTwoPi * static_cast<double>(index) /
        static_cast<double>(yaw_samples);
      orientations.push_back(fromRpy(roll, pitch, yaw));
    }
    return orientations;
  }

  static Eigen::Quaterniond fromRpy(double roll, double pitch, double yaw)
  {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
           Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())) *
           Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  }

  static std::vector<Eigen::Vector3d> graphPoints(
    const ais_gng_msgs::msg::TopologicalMap &graph)
  {
    std::vector<Eigen::Vector3d> points;
    points.reserve(graph.nodes.size());
    for (const auto &node : graph.nodes) {
      points.emplace_back(node.pos.x, node.pos.y, node.pos.z);
    }
    return points;
  }

  static std::unique_ptr<candidate::OccupiedVoxelGrid> makeGrid(
    const voxel_msgs::msg::Voxel &msg)
  {
    if (msg.voxel_size <= 0.0F || !std::isfinite(msg.voxel_size)) {
      throw std::invalid_argument("voxel message has an invalid voxel_size");
    }
    if (!msg.labels.empty() && msg.labels.size() != msg.data.size()) {
      throw std::invalid_argument("voxel labels must be empty or aligned with data");
    }

    voxel_idx::VoxelIndexingSchema indexing;
    indexing.x_shift = msg.x_shift;
    indexing.y_shift = msg.y_shift;
    indexing.z_shift = msg.z_shift;
    indexing.offset = msg.offset;
    indexing.voxel_size = msg.voxel_size;
    if (!indexing.isValid()) {
      throw std::invalid_argument("voxel message has an invalid indexing schema");
    }

    candidate::VoxelGridGeometry geometry;
    geometry.voxel_size = msg.voxel_size;
    geometry.origin = Eigen::Vector3d(msg.origin_x, msg.origin_y, msg.origin_z);
    auto grid = std::make_unique<candidate::OccupiedVoxelGrid>(geometry);
    for (const auto raw_id : msg.data) {
      grid->add(indexing.unpack(static_cast<std::uint64_t>(raw_id)));
    }
    return grid;
  }

  std::string waitingReason() const
  {
    if (!object_voxels_) {
      return "object_voxels";
    }
    if (!environment_voxels_topic_.empty() &&
      environment_voxels_topic_ != object_voxels_topic_ && !environment_voxels_)
    {
      return "environment_voxels";
    }
    if (!required_graph_) {
      return "required_graph";
    }
    if (!undersize_graph_topic_.empty() && !undersize_graph_) {
      return "undersize_graph";
    }
    if (!forbidden_graph_topic_.empty() && !forbidden_graph_) {
      return "forbidden_graph";
    }
    return {};
  }

  void updateMatches()
  {
    if (!dirty_) {
      return;
    }
    const std::string waiting_for = waitingReason();
    if (!waiting_for.empty()) {
      publishStatus("waiting", waiting_for);
      return;
    }
    dirty_ = false;

    try {
      const auto started = std::chrono::steady_clock::now();
      auto target_grid = makeGrid(*object_voxels_);
      const auto environment_msg = environment_voxels_ ? environment_voxels_ : object_voxels_;
      auto collision_grid = makeGrid(*environment_msg);

      if (!compiled_template_ || template_dirty_ ||
        std::abs(compiled_template_->voxel_size - target_grid->geometry().voxel_size) > 1e-9)
      {
        candidate::GraspVoxelTemplate grasp_template;
        grasp_template.required_occupied = graphPoints(*required_graph_);
        if (undersize_graph_) {
          grasp_template.optional_not_sole_support = graphPoints(*undersize_graph_);
        }
        if (forbidden_graph_) {
          grasp_template.required_empty = graphPoints(*forbidden_graph_);
        }
        compiled_template_ = std::make_unique<candidate::CompiledGraspVoxelTemplate>(
          candidate::GraspVoxelMatcher::compile(
            grasp_template, orientations_, target_grid->geometry().voxel_size));
        template_dirty_ = false;
      }
      const auto result = candidate::GraspVoxelMatcher::match(
        *target_grid, *collision_grid, *compiled_template_, match_config_);
      const double processing_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();

      publishCandidates(result);
      publishSummary(result, target_grid->size(), collision_grid->size(), processing_ms);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Grasp voxel match: objects=%zu anchors=%zu poses=%zu accepted=%zu processing=%.2fms",
        target_grid->size(), result.anchors_considered, result.poses_evaluated,
        result.candidates.size(), processing_ms);
    } catch (const std::exception &error) {
      publishStatus("error", error.what());
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Grasp voxel matching failed: %s", error.what());
    }
  }

  void publishCandidates(const candidate::GraspVoxelMatchResult &result)
  {
    geometry_msgs::msg::PoseArray msg;
    msg.header = object_voxels_->header;
    msg.poses.reserve(result.candidates.size());
    for (const auto &candidate_result : result.candidates) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = candidate_result.tcp_position.x();
      pose.position.y = candidate_result.tcp_position.y();
      pose.position.z = candidate_result.tcp_position.z();
      pose.orientation.x = candidate_result.tcp_orientation.x();
      pose.orientation.y = candidate_result.tcp_orientation.y();
      pose.orientation.z = candidate_result.tcp_orientation.z();
      pose.orientation.w = candidate_result.tcp_orientation.w();
      msg.poses.push_back(std::move(pose));
    }
    candidate_pub_->publish(msg);
  }

  void publishSummary(
    const candidate::GraspVoxelMatchResult &result,
    std::size_t object_voxel_count,
    std::size_t environment_voxel_count,
    double processing_ms)
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "{\"status\":\"ready\""
           << ",\"object_voxel_count\":" << object_voxel_count
           << ",\"environment_voxel_count\":" << environment_voxel_count
           << ",\"orientation_count\":" << orientations_.size()
           << ",\"anchors_considered\":" << result.anchors_considered
           << ",\"poses_evaluated\":" << result.poses_evaluated
           << ",\"candidate_count\":" << result.candidates.size()
           << ",\"rejected_required_occupancy\":"
           << result.rejected_required_occupancy
           << ",\"rejected_undersize_only\":" << result.rejected_undersize_only
           << ",\"rejected_forbidden_occupancy\":"
           << result.rejected_forbidden_occupancy
           << ",\"processing_ms\":" << processing_ms
           << ",\"candidates\":[";
    for (std::size_t index = 0; index < result.candidates.size(); ++index) {
      if (index > 0) {
        stream << ",";
      }
      const auto &candidate_result = result.candidates[index];
      stream << "{\"index\":" << index
             << ",\"score\":" << candidate_result.score
             << ",\"required_hits\":" << candidate_result.required_hits
             << ",\"required_samples\":" << candidate_result.required_samples
             << ",\"required_occupancy_ratio\":"
             << candidate_result.required_occupancy_ratio
             << ",\"outside_undersize_hits\":"
             << candidate_result.outside_undersize_hits
             << ",\"forbidden_hits\":" << candidate_result.forbidden_hits
             << ",\"anchor\":[" << candidate_result.anchor.x << ","
             << candidate_result.anchor.y << "," << candidate_result.anchor.z << "]"
             << ",\"orientation_index\":" << candidate_result.orientation_index
             << "}";
    }
    stream << "]}";
    msg.data = stream.str();
    summary_pub_->publish(msg);
  }

  void publishStatus(const char *status, const std::string &detail)
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "{\"status\":\"" << status << "\",\"detail\":\"";
    for (const char value : detail) {
      if (value == '\\' || value == '"') {
        stream << '\\';
      }
      stream << value;
    }
    stream << "\"}";
    msg.data = stream.str();
    summary_pub_->publish(msg);
  }

  std::string object_voxels_topic_;
  std::string environment_voxels_topic_;
  std::string required_graph_topic_;
  std::string undersize_graph_topic_;
  std::string forbidden_graph_topic_;
  candidate::GraspVoxelMatchConfig match_config_;
  std::vector<Eigen::Quaterniond> orientations_;
  bool dirty_ = true;
  bool template_dirty_ = true;
  std::unique_ptr<candidate::CompiledGraspVoxelTemplate> compiled_template_;

  voxel_msgs::msg::Voxel::SharedPtr object_voxels_;
  voxel_msgs::msg::Voxel::SharedPtr environment_voxels_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr required_graph_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr undersize_graph_;
  ais_gng_msgs::msg::TopologicalMap::SharedPtr forbidden_graph_;

  rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr object_voxels_sub_;
  rclcpp::Subscription<voxel_msgs::msg::Voxel>::SharedPtr environment_voxels_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr required_graph_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr undersize_graph_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr forbidden_graph_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr candidate_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace grasping_system::nodes

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grasping_system::nodes::GraspVoxelMatcherNode>());
  rclcpp::shutdown();
  return 0;
}
