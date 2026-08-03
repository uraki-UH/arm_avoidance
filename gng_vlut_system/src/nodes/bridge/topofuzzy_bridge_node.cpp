#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include <Eigen/Geometry>

#include <ais_gng_msgs/msg/topological_cluster.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/msg/topological_node.hpp>
#include <ais_gng_feature_msgs/msg/topological_node_feature_array.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common/resource_utils.hpp"
#include "safety_engine/vlut/safety_vlut_mapper.hpp"
#include "gng/GrowingNeuralGas.hpp"
#include "core/safety_engine/runtime/safety_system_loader.hpp"
#include "metrics/graph_topology_analyzer.hpp"
#include "core/common/constants.hpp"
#include "core/common/topological_map_message_builder.hpp"
#include "robot_model/urdf_loader.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "metrics/manipulability.hpp"
#include "visualization/visualization_gng.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

std::string activeEdgeModeName(int edge_mode) {
  if (edge_mode == 0) {
    return "angle";
  }
  if (edge_mode == 1) {
    return "coord";
  }
  return "auto";
}


} // namespace

class TopoFuzzyBridgeNode : public rclcpp::Node {
public:
  TopoFuzzyBridgeNode() : Node("topofuzzy_bridge_node") {
    declare_parameter("gng_model_path", "");
    declare_parameter("vlut_path", "");
    declare_parameter("publish_hz", ::robot_sim::common::Constants::DEFAULT_UPDATE_HZ);
    declare_parameter("edge_mode", -1);
    declare_parameter("frame_id", "world");
    declare_parameter("source_frame_id", "world");
    declare_parameter("occupied_voxels_topic", "occupied_voxels");
    declare_parameter("danger_voxels_topic", "danger_voxels");
    declare_parameter("node_feature_topic", "topological_node_features");
    declare_parameter("gng.data_directory", "gng_results");
    declare_parameter("gng.experiment_id", "standard_train");
    declare_parameter("gng.gng_model_filename", "gng.bin");
    declare_parameter("gng.vlut_filename", "vlut.bin");
    declare_parameter("topic_name", "topological_map");
    declare_parameter("visualization_gng.enabled", false);
    declare_parameter("visualization_gng.path_prefix", "");
    declare_parameter("visualization_gng.topic_prefix",
                      "topological_map_visualization");
    declare_parameter("visualization_gng.trajectory_input_topic",
                      "planned_topological_map");
    declare_parameter("visualization_gng.trajectory_topic_prefix",
                      "planned_topological_map_visualization");
    declare_parameter("visualization_gng.candidate_trajectory_input_topic",
                      "candidate_topological_map");
    declare_parameter("visualization_gng.candidate_trajectory_topic_prefix",
                      "candidate_topological_map_visualization");

    declare_parameter("urdf_path", "");
    declare_parameter("robot.arm_leaf_link_names", "");

    const std::string gng_path =
        resolveResultPath(get_parameter("gng_model_path").as_string(), false);
    const std::string vlut_path =
        resolveResultPath(get_parameter("vlut_path").as_string(), true);

    context_ =
        robot_sim::analysis::SafetySystemLoader::load(gng_path, vlut_path, 7);
    if (!context_ || !context_->gng || !context_->mapper) {
      RCLCPP_ERROR(get_logger(), "Failed to load GNG/VLUT data from %s",
                   gng_path.c_str());
      throw std::runtime_error(
          "topofuzzy_bridge: failed to load safety context");
    }

    calculateManipulabilityEllipsoidsDynamically();

    const int coord_layer_count = context_->gng->getCoordLayerCount();
    RCLCPP_INFO(get_logger(),
                "Loaded GNG/VLUT: gng=%s vlut=%s coord_layer_count=%d",
                gng_path.c_str(), vlut_path.c_str(), coord_layer_count);
    if (coord_layer_count > 1) {
      for (int i = 0; i < coord_layer_count; ++i) {
        RCLCPP_INFO(get_logger(), "  layer topic: %s_layer_%d",
                    get_parameter("topic_name").as_string().c_str(), i);
      }
    }

    edge_mode_ = get_parameter("edge_mode").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    source_frame_id_ = get_parameter("source_frame_id").as_string();
    publish_hz_ = std::max(0.1, get_parameter("publish_hz").as_double());

    // Resolve frame IDs with namespace if they are relative
    std::string ns = get_namespace();
    if (ns != "/" && !ns.empty()) {
        if (ns[0] == '/') ns = ns.substr(1);
        auto prefix_frame = [&](std::string &f) {
            if (!f.empty() && f != "world" && f[0] != '/') {
                f = ns + "/" + f;
            }
        };
        prefix_frame(frame_id_);
        prefix_frame(source_frame_id_);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    occupied_voxels_topic_ = get_parameter("occupied_voxels_topic").as_string();
    occupied_voxels_topic_relative_ = occupied_voxels_topic_.rfind('/') == 0
                                          ? occupied_voxels_topic_.substr(1)
                                          : occupied_voxels_topic_;
    danger_voxels_topic_ = get_parameter("danger_voxels_topic").as_string();
    danger_voxels_topic_relative_ = danger_voxels_topic_.rfind('/') == 0
                                        ? danger_voxels_topic_.substr(1)
                                        : danger_voxels_topic_;

    occupied_sub_ = create_subscription<std_msgs::msg::Int64MultiArray>(
        occupied_voxels_topic_, 10,
        std::bind(&TopoFuzzyBridgeNode::occupiedVoxelCallback, this,
                  std::placeholders::_1));
    if (occupied_voxels_topic_relative_ != occupied_voxels_topic_) {
      occupied_sub_relative_ = create_subscription<std_msgs::msg::Int64MultiArray>(
          occupied_voxels_topic_relative_, 10,
          std::bind(&TopoFuzzyBridgeNode::occupiedVoxelCallback, this,
                    std::placeholders::_1));
    }
    danger_sub_ = create_subscription<std_msgs::msg::Int64MultiArray>(
        danger_voxels_topic_, 10,
        std::bind(&TopoFuzzyBridgeNode::dangerVoxelCallback, this,
                  std::placeholders::_1));
    if (danger_voxels_topic_relative_ != danger_voxels_topic_) {
      danger_sub_relative_ = create_subscription<std_msgs::msg::Int64MultiArray>(
          danger_voxels_topic_relative_, 10,
          std::bind(&TopoFuzzyBridgeNode::dangerVoxelCallback, this,
                    std::placeholders::_1));
    }

    const std::string topic_name = get_parameter("topic_name").as_string();
    topological_map_pub_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
        topic_name, rclcpp::QoS(1).reliable().transient_local());
    const std::string node_feature_topic = get_parameter("node_feature_topic").as_string();
    node_feature_pub_ = create_publisher<ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray>(
        node_feature_topic, rclcpp::QoS(1).reliable().transient_local());

    const int layer_count = context_->gng->getCoordLayerCount();
    if (layer_count > 1) {
      for (int i = 0; i < layer_count; ++i) {
        layer_pubs_.push_back(create_publisher<ais_gng_msgs::msg::TopologicalMap>(
            topic_name + "_layer_" + std::to_string(i), rclcpp::QoS(1).reliable().transient_local()));
      }
    }
    initializeVisualizationGng(gng_path, layer_count);
    initializeVisualizationTrajectoryBridge();

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz_)),
        std::bind(&TopoFuzzyBridgeNode::publishIfDirty, this));

    latest_occ_vids_.clear();
    latest_dan_vids_.clear();
    graph_dirty_ = true;

    RCLCPP_INFO(get_logger(),
                "topofuzzy_bridge_node initialized. edge_mode=%s "
                "occupied_topic=%s danger_topic=%s",
                activeEdgeModeName(edge_mode_).c_str(),
                occupied_voxels_topic_.c_str(), danger_voxels_topic_.c_str());

    publishGraph();
  }

private:
  std::string resolveResultPath(const std::string &path, bool is_vlut) const {
    if (!path.empty()) {
      if (std::filesystem::path(path).is_absolute()) {
        return path;
      }
      if (path.rfind("gng_results/", 0) == 0 ||
          path.find('/') != std::string::npos) {
        return robot_sim::common::resolvePath(path);
      }
    }

    const std::string data_dir = robot_sim::common::resolveDataPath(
        get_parameter("gng.data_directory").as_string());
    const std::string exp_id = get_parameter("gng.experiment_id").as_string();
    std::string filename = path;

    if (filename.empty()) {
      if (is_vlut) {
        filename = get_parameter("gng.vlut_filename").as_string();
      } else {
        filename = get_parameter("gng.gng_model_filename").as_string();
        if (filename.empty()) {
          filename = exp_id + ".bin";
        }
      }
    }
    return robot_sim::common::resolvePath(data_dir + "/" + exp_id + "/" +
                                          filename);
  }

  void
  occupiedVoxelCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
    std::vector<long> occupied_vids;
    occupied_vids.reserve(msg->data.size());
    for (const auto value : msg->data) {
      occupied_vids.push_back(static_cast<long>(value));
    }

    {
      std::lock_guard<std::mutex> lock(update_mutex_);
      latest_occ_vids_ = std::move(occupied_vids);
      updateSafetyLocked();
      graph_dirty_ = true;
    }
  }

  void
  dangerVoxelCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
    std::vector<long> danger_vids;
    danger_vids.reserve(msg->data.size());
    for (const auto value : msg->data) {
      danger_vids.push_back(static_cast<long>(value));
    }

    {
      std::lock_guard<std::mutex> lock(update_mutex_);
      latest_dan_vids_ = std::move(danger_vids);
      updateSafetyLocked();
      graph_dirty_ = true;
    }
  }

  void updateSafetyLocked() {
    if (!context_) {
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();
    context_->update(latest_occ_vids_, latest_dan_vids_);
    const auto t1 = std::chrono::steady_clock::now();
    const double vlut_update_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "VLUT update: %.3f ms | occ=%zu dan=%zu",
        vlut_update_ms, latest_occ_vids_.size(),
        latest_dan_vids_.size());

    auto &gng = *context_->gng;
    const auto &col_counts = context_->mapper->getCollisionCounts();
    const auto &dgr_counts = context_->mapper->getDangerCounts();
    size_t safe_nodes = 0;
    size_t collision_nodes = 0;
    size_t danger_nodes = 0;

    for (size_t i = 0; i < gng.getNodes().size(); ++i) {
      auto &node = gng.getNodes()[i];
      if (node.id == -1) {
        continue;
      }

      auto &status = node.status;
      status.collision_count = (i < col_counts.size()) ? col_counts[i] : 0;
      status.danger_count = (i < dgr_counts.size()) ? dgr_counts[i] : 0;
      status.is_colliding = (status.collision_count > 0);
      status.is_danger = (status.danger_count > 0 && !status.is_colliding);

      if (status.is_colliding) {
        ++collision_nodes;
      } else if (status.is_danger) {
        ++danger_nodes;
      } else {
        ++safe_nodes;
      } 
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Safety updated: occupied=%zu danger=%zu -> safe=%zu collision=%zu danger=%zu",
        latest_occ_vids_.size(), latest_dan_vids_.size(),
        safe_nodes, collision_nodes, danger_nodes);
  }

  int selectedEdgeMode() const {
    if (edge_mode_ == 0 || edge_mode_ == 1) {
      return edge_mode_;
    }

    if (!context_ || !context_->gng) {
      return 1;
    }

    const auto &gng = *context_->gng;
    for (size_t i = 0; i < gng.getNodes().size(); ++i) {
      const auto &node = gng.getNodes()[i];
      if (node.id == -1) {
        continue;
      }
      if (!gng.getNeighborsCoord(static_cast<int>(i)).empty()) {
        return 1;
      }
      if (!gng.getNeighborsAngle(static_cast<int>(i)).empty()) {
        return 0;
      }
    }
    return 1;
  }

  void annotateTopologyLocked() {
    if (!context_ || !context_->gng) {
      return;
    }

    auto &gng = *context_->gng;
    const auto active_indices = gng.getActiveIndices();
    const int max_nodes = static_cast<int>(gng.getMaxNodeNum());
    const int mode = selectedEdgeMode();

    topology_analyzer_.analyze(
        active_indices, max_nodes,
        [&](int idx) {
          const auto &node = gng.nodeAt(idx);
          return node.id != -1 && node.status.self_collision_free && node.status.active;
        },
        [&](int idx) -> const std::vector<int> & {
          return (mode == 0) ? gng.getNeighborsAngle(idx)
                             : gng.getNeighborsCoord(idx);
        },
        true);

    const int mainland_id = topology_analyzer_.getMainlandId();
    for (int idx : active_indices) {
      if (idx < 0 || idx >= max_nodes) {
        continue;
      }
      auto &node = gng.nodeAt(idx);
      if (node.id == -1) {
        continue;
      }

      const int group_id = topology_analyzer_.getGroupId(idx);
      node.status.topology_group_id = group_id;
      node.status.is_mainland = (mainland_id != -1 && group_id == mainland_id);
    }
  }

  ais_gng_msgs::msg::TopologicalMap buildGraphMessage() {
    return robot_sim::bridge::topofuzzy::buildGraphMessage(
        *this, context_ ? context_->gng : nullptr, tf_buffer_, frame_id_,
        source_frame_id_, selectedEdgeMode());
  }

  void publishIfDirty() {
    std::lock_guard<std::mutex> lock(update_mutex_);
    if (!graph_dirty_) {
      return;
    }
    publishGraphLocked();
    graph_dirty_ = false;
  }

  ais_gng_msgs::msg::TopologicalMap buildLayerGraphMessage(int layer) {
    return robot_sim::bridge::topofuzzy::buildLayerGraphMessage(
        *this, context_ ? context_->gng : nullptr, tf_buffer_, layer,
        frame_id_, source_frame_id_);
  }

  void initializeVisualizationGng(const std::string &gng_path,
                                  int layer_count) {
    if (!get_parameter("visualization_gng.enabled").as_bool()) {
      return;
    }

    std::string path_prefix =
        get_parameter("visualization_gng.path_prefix").as_string();
    if (path_prefix.empty()) {
      path_prefix =
          (std::filesystem::path(gng_path).parent_path() / "visualization_gng")
              .string();
    } else if (!std::filesystem::path(path_prefix).is_absolute()) {
      path_prefix = robot_sim::common::resolvePath(path_prefix);
    }
    const std::string topic_prefix =
        get_parameter("visualization_gng.topic_prefix").as_string();
    const std::string trajectory_topic_prefix =
        get_parameter("visualization_gng.trajectory_topic_prefix").as_string();
    const std::string candidate_trajectory_topic_prefix =
        get_parameter("visualization_gng.candidate_trajectory_topic_prefix")
            .as_string();

    for (int layer = 0; layer < layer_count; ++layer) {
      const auto path = robot_sim::visualization::visualizationGngLayerPath(
          path_prefix, static_cast<std::uint32_t>(layer));
      robot_sim::visualization::VisualizationGngModel model;
      std::string error;
      if (!model.load(path, &error)) {
        RCLCPP_WARN(get_logger(), "Visualization GNG layer %d skipped: %s",
                    layer, error.c_str());
        continue;
      }
      const auto source_points =
          robot_sim::visualization::collectVisualizationGngSourcePoints(
              *context_->gng, layer);
      const std::uint64_t source_signature =
          robot_sim::visualization::computeVisualizationGngSourceSignature(
              source_points);
      if (model.coord_layer != static_cast<std::uint32_t>(layer) ||
          model.source_signature != source_signature) {
        RCLCPP_WARN(
            get_logger(),
            "Visualization GNG layer %d skipped because it does not match %s",
            layer, gng_path.c_str());
        continue;
      }

      VisualizationLayer visual_layer;
      visual_layer.layer = layer;
      visual_layer.model = std::move(model);
      int max_source_node_id = -1;
      for (const auto &source_node : context_->gng->getNodes()) {
        max_source_node_id = std::max(max_source_node_id, source_node.id);
      }
      visual_layer.source_to_visual.assign(
          static_cast<std::size_t>(max_source_node_id + 1), -1);
      std::size_t mapped_source_count = 0;
      for (std::size_t visual_index = 0;
           visual_index < visual_layer.model.nodes.size(); ++visual_index) {
        for (const int source_id :
             visual_layer.model.nodes[visual_index].source_node_ids) {
          if (source_id < 0 ||
              static_cast<std::size_t>(source_id) >=
                  visual_layer.source_to_visual.size()) {
            continue;
          }
          auto &mapped =
              visual_layer.source_to_visual[static_cast<std::size_t>(source_id)];
          if (mapped < 0) {
            mapped = static_cast<std::int32_t>(visual_index);
            ++mapped_source_count;
          } else if (mapped != static_cast<std::int32_t>(visual_index)) {
            RCLCPP_WARN(get_logger(),
                        "Visualization GNG layer %d source node %d has "
                        "multiple visual memberships",
                        layer, source_id);
          }
        }
      }
      visual_layer.transition_path_index.reserve(
          visual_layer.model.transition_paths.size());
      for (std::size_t transition_index = 0;
           transition_index < visual_layer.model.transition_paths.size();
           ++transition_index) {
        const auto &transition =
            visual_layer.model.transition_paths[transition_index];
        visual_layer.transition_path_index.emplace(
            robot_sim::visualization::visualizationGngSourceEdgeKey(
                transition.source_node_id, transition.target_node_id),
            transition_index);
      }
      visual_layer.publisher =
          create_publisher<ais_gng_msgs::msg::TopologicalMap>(
              topic_prefix + "_layer_" + std::to_string(layer),
              rclcpp::QoS(1).reliable().transient_local());
      if (!trajectory_topic_prefix.empty()) {
        visual_layer.trajectory_publisher =
            create_publisher<ais_gng_msgs::msg::TopologicalMap>(
                trajectory_topic_prefix + "_layer_" + std::to_string(layer),
                rclcpp::QoS(1).reliable().transient_local());
      }
      if (!candidate_trajectory_topic_prefix.empty()) {
        visual_layer.candidate_trajectory_publisher =
            create_publisher<ais_gng_msgs::msg::TopologicalMap>(
                candidate_trajectory_topic_prefix + "_layer_" +
                    std::to_string(layer),
                rclcpp::QoS(1).reliable().transient_local());
      }
      RCLCPP_INFO(get_logger(),
                  "Loaded visualization GNG layer %d: nodes=%zu edges=%zu "
                  "mapped_sources=%zu transition_overrides=%zu "
                  "topic=%s_layer_%d",
                  layer, visual_layer.model.nodes.size(),
                  visual_layer.model.edges.size(), mapped_source_count,
                  visual_layer.model.transition_paths.size(),
                  topic_prefix.c_str(), layer);
      visualization_layers_.push_back(std::move(visual_layer));
    }
  }

  void initializeVisualizationTrajectoryBridge() {
    if (visualization_layers_.empty()) {
      return;
    }
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    const std::string trajectory_input =
        get_parameter("visualization_gng.trajectory_input_topic").as_string();
    if (!trajectory_input.empty()) {
      visualization_trajectory_sub_ =
          create_subscription<ais_gng_msgs::msg::TopologicalMap>(
              trajectory_input, qos,
              [this](const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
                publishVisualizationTrajectory(*msg, false);
              });
    }
    const std::string candidate_input =
        get_parameter("visualization_gng.candidate_trajectory_input_topic")
            .as_string();
    if (!candidate_input.empty()) {
      visualization_candidate_trajectory_sub_ =
          create_subscription<ais_gng_msgs::msg::TopologicalMap>(
              candidate_input, qos,
              [this](const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
                publishVisualizationTrajectory(*msg, true);
              });
    }
  }

  void publishVisualizationTrajectory(
      const ais_gng_msgs::msg::TopologicalMap &source_path,
      bool candidate) {
    std::lock_guard<std::mutex> lock(update_mutex_);
    if (candidate) {
      latest_visualization_candidate_trajectory_ = source_path;
    } else {
      latest_visualization_trajectory_ = source_path;
    }
    publishVisualizationTrajectoryLocked(source_path, candidate);
  }

  void publishVisualizationTrajectoryLocked(
      const ais_gng_msgs::msg::TopologicalMap &source_path,
      bool candidate) {
    for (auto &visual_layer : visualization_layers_) {
      const auto &publisher = candidate
                                  ? visual_layer.candidate_trajectory_publisher
                                  : visual_layer.trajectory_publisher;
      if (!publisher || !visual_layer.cached_graph_valid) {
        continue;
      }
      publisher->publish(
          robot_sim::bridge::topofuzzy::buildVisualizationPathMessage(
              source_path, visual_layer.cached_graph,
              visual_layer.source_to_visual, visual_layer.model,
              visual_layer.transition_path_index));
    }
  }

  void publishGraph() {
    std::lock_guard<std::mutex> lock(update_mutex_);
    publishGraphLocked();
    graph_dirty_ = false;
  }

  void publishGraphLocked() {
    if (!context_ || !context_->gng || !topological_map_pub_) {
      return;
    }
    topological_map_pub_->publish(buildGraphMessage());
    if (node_feature_pub_) {
      node_feature_pub_->publish(robot_sim::bridge::topofuzzy::buildNodeFeatureArray(
          *this, context_->gng, frame_id_));
    }
    for (size_t i = 0; i < layer_pubs_.size(); ++i) {
      if (layer_pubs_[i]) {
        layer_pubs_[i]->publish(buildLayerGraphMessage(static_cast<int>(i)));
      }
    }
    for (auto &visual_layer : visualization_layers_) {
      if (!visual_layer.publisher) {
        continue;
      }
      auto message =
          robot_sim::bridge::topofuzzy::buildVisualizationGngMessage(
              *this, context_->gng, visual_layer.model, tf_buffer_, frame_id_,
              source_frame_id_);
      visual_layer.cached_graph_valid =
          message.nodes.size() == visual_layer.model.nodes.size();
      visual_layer.cached_graph = message;
      visual_layer.publisher->publish(std::move(message));
    }
    if (latest_visualization_trajectory_) {
      publishVisualizationTrajectoryLocked(
          *latest_visualization_trajectory_, false);
    }
    if (latest_visualization_candidate_trajectory_) {
      publishVisualizationTrajectoryLocked(
          *latest_visualization_candidate_trajectory_, true);
    }
  }

  void calculateManipulabilityEllipsoidsDynamically() {
    std::string urdf_path_raw = get_parameter("urdf_path").as_string();
    std::string leaf_link = get_parameter("robot.arm_leaf_link_names").as_string();

    if (urdf_path_raw.empty() || leaf_link.empty()) {
      RCLCPP_WARN(get_logger(), "topofuzzy_bridge: urdf_path or arm_leaf_link_names parameter is empty. Dynamic manipulability calculation skipped.");
      return;
    }

    std::string urdf_file = robot_sim::common::resolvePath(urdf_path_raw);
    if (!std::filesystem::exists(urdf_file)) {
      RCLCPP_WARN(get_logger(), "topofuzzy_bridge: URDF file does not exist: %s. Skip dynamic manipulability calculation.", urdf_file.c_str());
      return;
    }

    try {
      auto model_obj = simulation::loadRobotFromUrdf(urdf_file);
      simulation::RobotModel model(model_obj);
      kinematics::KinematicChain arm = simulation::createKinematicChainFromModel(model, leaf_link);
      arm.setBase(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());

      RCLCPP_INFO(get_logger(), "topofuzzy_bridge: Dynamically calculating manipulability ellipsoids for GNG nodes (DOF=%d, leaf=%s)",
                  arm.getTotalDOF(), leaf_link.c_str());

      auto &gng = *context_->gng;
      size_t count = 0;
      for (size_t i = 0; i < gng.getNodes().size(); ++i) {
        auto &node = gng.getNodes()[i];
        if (node.id == -1) continue;

        // Calculate FK & Jacobian
        std::vector<double> joints(node.weight_angle.size());
        for (int j = 0; j < node.weight_angle.size(); ++j) {
          joints[j] = node.weight_angle(j);
        }

        Eigen::MatrixXd J = arm.calculateJacobianAt(arm.getNumJoints() + 1, joints);
        Eigen::MatrixXd Jv = J.topRows(3);
        Eigen::MatrixXd Jr = J.bottomRows(3);

        node.status.manip_info = Manipulability::calculateManipulabilityEllipsoid(Jv, Manipulability::KINEMATIC);
        node.status.rotational_manip_info = Manipulability::calculateManipulabilityEllipsoid(Jr, Manipulability::KINEMATIC);

        node.status.min_singular_value = (float)node.status.manip_info.singular_values.minCoeff();

        count++;
      }
      RCLCPP_INFO(get_logger(), "topofuzzy_bridge: Finished dynamic calculation for %zu GNG nodes.", count);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "topofuzzy_bridge: Error in dynamic manipulability calculation: %s", e.what());
    }
  }

private:
  struct VisualizationLayer {
    int layer = 0;
    robot_sim::visualization::VisualizationGngModel model;
    std::vector<std::int32_t> source_to_visual;
    std::unordered_map<std::uint64_t, std::size_t> transition_path_index;
    ais_gng_msgs::msg::TopologicalMap cached_graph;
    bool cached_graph_valid = false;
    rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr publisher;
    rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr
        trajectory_publisher;
    rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr
        candidate_trajectory_publisher;
  };

  std::shared_ptr<robot_sim::analysis::SafetySystemContext> context_;
  robot_sim::analysis::GraphTopologyAnalyzer topology_analyzer_;

  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr occupied_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr occupied_sub_relative_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr danger_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr danger_sub_relative_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr
      topological_map_pub_;
  rclcpp::Publisher<ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray>::SharedPtr
      node_feature_pub_;
  std::vector<rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr> layer_pubs_;
  std::vector<VisualizationLayer> visualization_layers_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr
      visualization_trajectory_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr
      visualization_candidate_trajectory_sub_;
  std::optional<ais_gng_msgs::msg::TopologicalMap>
      latest_visualization_trajectory_;
  std::optional<ais_gng_msgs::msg::TopologicalMap>
      latest_visualization_candidate_trajectory_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex update_mutex_;
  std::vector<long> latest_occ_vids_;
  std::vector<long> latest_dan_vids_;
  bool graph_dirty_ = true;

  int edge_mode_ = -1;
  std::string frame_id_ = "world";
  std::string source_frame_id_ = "world";
  double publish_hz_ = ::robot_sim::common::Constants::DEFAULT_UPDATE_HZ;
  std::string occupied_voxels_topic_ = "occupied_voxels";
  std::string occupied_voxels_topic_relative_;
  std::string danger_voxels_topic_ = "danger_voxels";
  std::string danger_voxels_topic_relative_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopoFuzzyBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
