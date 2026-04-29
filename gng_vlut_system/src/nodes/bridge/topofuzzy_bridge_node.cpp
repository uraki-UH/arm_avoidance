#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include <Eigen/Geometry>

#include <ais_gng_msgs/msg/topological_cluster.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/msg/topological_node.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "common/resource_utils.hpp"
#include "safety_engine/vlut/safety_vlut_mapper.hpp"
#include "safety_engine/gng/GrowingNeuralGas.hpp"
#include "safety_engine/builder/safety_system_loader.hpp"
#include "metrics/graph_topology_analyzer.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

constexpr float kEps = 1e-6f;

Eigen::Isometry3d makeIsometry(const std::vector<double> &pos,
                               const std::vector<double> &rot_deg) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
  T.linear() =
      (Eigen::AngleAxisd(rot_deg[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rot_deg[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rot_deg[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return T;
}

uint8_t viewerLabelFromStatus(const GNG::Status &status) {
  if (status.is_colliding) {
    return 2; // COLLISION -> red
  }
  if (status.is_danger) {
    return 3; // DANGER -> yellow
  }
  return 1; // SAFE -> green
}

geometry_msgs::msg::Point32 toPoint32(const Eigen::Vector3f &v) {
  geometry_msgs::msg::Point32 p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

geometry_msgs::msg::Point32 toPoint32(const Eigen::Vector3d &v) {
  geometry_msgs::msg::Point32 p;
  p.x = static_cast<float>(v.x());
  p.y = static_cast<float>(v.y());
  p.z = static_cast<float>(v.z());
  return p;
}

geometry_msgs::msg::Quaternion toQuaternion(const Eigen::Quaternionf &q) {
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

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
    declare_parameter("publish_hz", 20.0);
    declare_parameter("edge_mode", -1);
    declare_parameter("frame_id", "world");
    declare_parameter("occupied_voxels_topic", "occupied_voxels");
    declare_parameter("danger_voxels_topic", "danger_voxels");
    declare_parameter("data_directory", "gng_results");
    declare_parameter("experiment_id", "standard_train");
    declare_parameter("gng_model_filename", "gng.bin");
    declare_parameter("vlut_filename", "vlut.bin");
    declare_parameter("topic_name", "topological_map");

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

    edge_mode_ = get_parameter("edge_mode").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    publish_hz_ = std::max(0.1, get_parameter("publish_hz").as_double());

    occupied_voxels_topic_ = get_parameter("occupied_voxels_topic").as_string();
    danger_voxels_topic_ = get_parameter("danger_voxels_topic").as_string();

    occupied_sub_ = create_subscription<std_msgs::msg::Int64MultiArray>(
        occupied_voxels_topic_, 10,
        std::bind(&TopoFuzzyBridgeNode::occupiedVoxelCallback, this,
                  std::placeholders::_1));
    danger_sub_ = create_subscription<std_msgs::msg::Int64MultiArray>(
        danger_voxels_topic_, 10,
        std::bind(&TopoFuzzyBridgeNode::dangerVoxelCallback, this,
                  std::placeholders::_1));

    const std::string topic_name = get_parameter("topic_name").as_string();
    topological_map_pub_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
        topic_name, rclcpp::QoS(1).reliable().transient_local());

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
        get_parameter("data_directory").as_string());
    const std::string exp_id = get_parameter("experiment_id").as_string();
    std::string filename = path;

    if (filename.empty()) {
      if (is_vlut) {
        filename = get_parameter("vlut_filename").as_string();
      } else {
        filename = get_parameter("gng_model_filename").as_string();
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

    context_->update(latest_occ_vids_, latest_dan_vids_);

    auto &gng = *context_->gng;
    const auto &col_counts = context_->mapper->getCollisionCounts();
    const auto &dgr_counts = context_->mapper->getDangerCounts();

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
    }
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
          return node.id != -1 && node.status.valid && node.status.active;
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
    // annotateTopologyLocked(); //
    // BFSによる連結成分分析だが未使用なためコメントアウト

    ais_gng_msgs::msg::TopologicalMap msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    if (!context_ || !context_->gng) {
      return msg;
    }

    const auto &gng = *context_->gng;
    const int mode = selectedEdgeMode();

    std::unordered_map<int, uint16_t> id_to_index;
    msg.nodes.reserve(gng.getNodes().size());

    for (size_t i = 0; i < gng.getNodes().size(); ++i) {
      const auto &node = gng.getNodes()[i];
      if (node.id == -1) {
        continue;
      }
      if (msg.nodes.size() >= std::numeric_limits<uint16_t>::max()) {
        RCLCPP_WARN(
            get_logger(),
            "topofuzzy_bridge: too many nodes for uint16 indexing, truncating");
        break;
      }

      ais_gng_msgs::msg::TopologicalNode out;
      out.id = static_cast<uint16_t>(node.id);
      out.pos = toPoint32(node.weight_coord);

      const Eigen::Vector3f normal = (node.status.ee_direction.norm() > kEps)
                                         ? node.status.ee_direction.normalized()
                                         : Eigen::Vector3f::UnitZ();
      out.normal = toPoint32(normal);
      out.label = viewerLabelFromStatus(node.status);

      const uint16_t published_index = static_cast<uint16_t>(msg.nodes.size());
      id_to_index.emplace(node.id, published_index);
      msg.nodes.push_back(std::move(out));
    }

    std::unordered_set<uint64_t> seen_edges;
    for (size_t i = 0; i < gng.getNodes().size(); ++i) {
      const auto &node = gng.getNodes()[i];
      if (node.id == -1) {
        continue;
      }

      const auto &neighbors = (mode == 0)
                                  ? gng.getNeighborsAngle(static_cast<int>(i))
                                  : gng.getNeighborsCoord(static_cast<int>(i));
      const auto src_it = id_to_index.find(node.id);
      if (src_it == id_to_index.end()) {
        continue;
      }

      for (const int neighbor_id : neighbors) {
        if (neighbor_id < 0) {
          continue;
        }
        const auto tgt_it = id_to_index.find(neighbor_id);
        if (tgt_it == id_to_index.end()) {
          continue;
        }

        const int src_original = node.id;
        const int tgt_original = neighbor_id;
        const int lo = std::min(src_original, tgt_original);
        const int hi = std::max(src_original, tgt_original);
        const uint64_t key =
            (static_cast<uint64_t>(static_cast<uint32_t>(lo)) << 32) |
            static_cast<uint32_t>(hi);
        if (!seen_edges.insert(key).second) {
          continue;
        }

        msg.edges.push_back(src_it->second);
        msg.edges.push_back(tgt_it->second);
      }
    }

    // Clusters are optional for this bridge. We keep them empty for now so the
    // viewer can focus on the node state + topology graph.
    msg.clusters.clear();

    return msg;
  }

  void publishIfDirty() {
    std::lock_guard<std::mutex> lock(update_mutex_);
    if (!graph_dirty_) {
      return;
    }
    publishGraphLocked();
    graph_dirty_ = false;
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
  }

private:
  std::shared_ptr<robot_sim::analysis::SafetySystemContext> context_;
  robot_sim::analysis::GraphTopologyAnalyzer topology_analyzer_;

  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr occupied_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr danger_sub_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr
      topological_map_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex update_mutex_;
  std::vector<long> latest_occ_vids_;
  std::vector<long> latest_dan_vids_;
  bool graph_dirty_ = true;

  int edge_mode_ = -1;
  std::string frame_id_ = "world";
  double publish_hz_ = 5.0;
  std::string occupied_voxels_topic_ = "occupied_voxels";
  std::string danger_voxels_topic_ = "danger_voxels";
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopoFuzzyBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
