#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include <ais_gng_msgs/msg/topological_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "common/constants.hpp"
#include "common/resource_utils.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include "safety_engine/gng/GrowingNeuralGas.hpp"

namespace {

static std::string joinStrings(const std::vector<std::string> &items,
                               const std::string &delimiter) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < items.size(); ++i) {
    if (i != 0) {
      oss << delimiter;
    }
    oss << items[i];
  }
  return oss.str();
}

static uint8_t labelFromStatus(const GNG::Status &status) {
  if (status.is_colliding) {
    return 2;
  }
  if (status.is_danger) {
    return 3;
  }
  return 1;
}

} // namespace

namespace robot_sim::planning {

class ArmAvoidNode : public rclcpp::Node {
public:
  using GNGType = ::GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

  explicit ArmAvoidNode(const rclcpp::NodeOptions &options)
      : Node("arm_avoid_node", options) {
    declare_parameter<std::string>("robot_urdf_path",
                                   "package://topoarm_description/urdf/topo_dual_arm.urdf.xacro");
    declare_parameter<std::string>("gng_model_path", "");
    declare_parameter<std::string>("root_link", "base_link");
    declare_parameter<std::string>("leaf_link", "right_end_effector_link");
    declare_parameter<std::string>("joint_topic", "joint_states");
    declare_parameter<std::string>("graph_topic", "arm_avoid/topological_map");
    declare_parameter<std::string>("safe_target_topic",
                                   "arm_avoid/safe_target_joint_states");
    declare_parameter<double>("publish_hz", 10.0);

    loadModelAndGraph();

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("joint_topic").as_string(), rclcpp::QoS(10).reliable(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_state_ = *msg;
          have_state_ = true;
        });

    graph_pub_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
        get_parameter("graph_topic").as_string(),
        rclcpp::QoS(1).reliable().transient_local());

    safe_target_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        get_parameter("safe_target_topic").as_string(),
        rclcpp::QoS(10).reliable());

    const double publish_hz = get_parameter("publish_hz").as_double();
    timer_ = create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int>(1000.0 / std::max(1.0, publish_hz))),
        [this]() { this->updateLocked(); });

    RCLCPP_INFO(get_logger(),
                "ArmAvoidNode ready. joint_topic=%s graph_topic=%s safe_target_topic=%s nearest=BFS safe-neighborhood mode joints=%s",
                get_parameter("joint_topic").as_string().c_str(),
                get_parameter("graph_topic").as_string().c_str(),
                get_parameter("safe_target_topic").as_string().c_str(),
                joinStrings(chain_joint_names_, ", ").c_str());
  }

private:
  void loadModelAndGraph() {
    const std::string resolved_urdf =
        robot_sim::common::resolvePath(get_parameter("robot_urdf_path").as_string());
    if (resolved_urdf.empty()) {
      throw std::runtime_error("Failed to resolve robot URDF path.");
    }

    auto model = std::make_shared<::simulation::RobotModel>(
        ::simulation::loadRobotFromUrdf(resolved_urdf));
    auto chain = std::make_shared<::kinematics::KinematicChain>(
        ::simulation::createKinematicChainFromModel(
            *model, get_parameter("leaf_link").as_string(),
            Eigen::Vector3d::Zero(), get_parameter("root_link").as_string()));

    if (!chain) {
      throw std::runtime_error("Failed to build kinematic chain.");
    }

    const int total_dof = chain->getTotalDOF();
    gng_ = std::make_unique<GNGType>(total_dof, 3, chain.get());

    const std::string gng_path =
        robot_sim::common::resolvePath(get_parameter("gng_model_path").as_string());
    if (gng_path.empty()) {
      throw std::runtime_error("Failed to resolve GNG model path.");
    }

    if (!gng_->load(gng_path)) {
      throw std::runtime_error("Failed to load GNG model: " + gng_path);
    }

    model_ = std::move(model);
    chain_ = std::move(chain);

    chain_joint_names_.clear();
    chain_joint_names_.reserve(static_cast<std::size_t>(chain_->getNumJoints()));
    for (int i = 0; i < chain_->getNumJoints(); ++i) {
      chain_joint_names_.push_back(chain_->getJointName(i));
    }

    if (chain_joint_names_.empty()) {
      throw std::runtime_error("ArmAvoidNode chain has no joints.");
    }
  }

  Eigen::VectorXf currentJointVectorLocked() const {
    Eigen::VectorXf q(static_cast<int>(chain_joint_names_.size()));
    q.setZero();

    std::unordered_map<std::string, double> value_by_name;
    value_by_name.reserve(latest_state_.name.size());

    for (std::size_t i = 0; i < latest_state_.name.size(); ++i) {
      if (i < latest_state_.position.size()) {
        value_by_name[latest_state_.name[i]] = latest_state_.position[i];
      }
    }

    for (std::size_t i = 0; i < chain_joint_names_.size(); ++i) {
      const auto it = value_by_name.find(chain_joint_names_[i]);
      q[static_cast<int>(i)] =
          (it != value_by_name.end()) ? static_cast<float>(it->second) : 0.0f;
    }

    return q;
  }

  int findNearestNode(const Eigen::VectorXf &posture) const {
    if (!gng_) {
      return -1;
    }

    float min_dist = std::numeric_limits<float>::infinity();
    int nearest_id = -1;

    gng_->forEachActiveValid([&](int i, const auto &node) {
      const int dim = std::min(static_cast<int>(node.weight_angle.size()),
                               static_cast<int>(posture.size()));
      if (dim <= 0) {
        return;
      }

      const float d = (node.weight_angle.head(dim) - posture.head(dim)).norm();
      if (d < min_dist) {
        min_dist = d;
        nearest_id = i;
      }
    });

    return nearest_id;
  }

  bool isSafeNode(int node_id) const {
    if (!gng_ || node_id < 0 || static_cast<std::size_t>(node_id) >= gng_->getMaxNodeNum()) {
      return false;
    }

    const auto &node = gng_->nodeAt(node_id);
    return node.id != -1 && node.status.active && node.status.valid &&
           !node.status.is_colliding && !node.status.is_danger;
  }

  bool hasAllSafeNeighbors(int node_id) const {
    if (!gng_ || node_id < 0 || static_cast<std::size_t>(node_id) >= gng_->getMaxNodeNum()) {
      return false;
    }

    const auto &neighbors = gng_->getNeighborsAngle(node_id);
    for (int neighbor_id : neighbors) {
      if (neighbor_id < 0 || static_cast<std::size_t>(neighbor_id) >= gng_->getMaxNodeNum()) {
        return false;
      }

      const auto &neighbor = gng_->nodeAt(neighbor_id);
      if (neighbor.id == -1 || !neighbor.status.active || !neighbor.status.valid ||
          neighbor.status.is_colliding || neighbor.status.is_danger) {
        return false;
      }
    }

    return true;
  }

  std::optional<int> findSafeNodeBfs(int start_id,
                                     std::vector<int> &explored_order) const {
    if (!gng_ || start_id < 0 || static_cast<std::size_t>(start_id) >= gng_->getMaxNodeNum()) {
      return std::nullopt;
    }

    std::queue<int> bfs_queue;
    std::unordered_set<int> visited;
    visited.reserve(gng_->getMaxNodeNum());

    bfs_queue.push(start_id);
    visited.insert(start_id);

    while (!bfs_queue.empty()) {
      const int current_id = bfs_queue.front();
      bfs_queue.pop();
      explored_order.push_back(current_id);

      if (isSafeNode(current_id) && hasAllSafeNeighbors(current_id)) {
        return current_id;
      }

      const auto &neighbors = gng_->getNeighborsAngle(current_id);
      for (int neighbor_id : neighbors) {
        if (neighbor_id < 0 || static_cast<std::size_t>(neighbor_id) >= gng_->getMaxNodeNum()) {
          continue;
        }
        if (visited.insert(neighbor_id).second) {
          bfs_queue.push(neighbor_id);
        }
      }
    }

    return std::nullopt;
  }

  void publishSafeTargetLocked(int node_id) {
    if (!gng_ || node_id < 0 || static_cast<std::size_t>(node_id) >= gng_->getMaxNodeNum()) {
      return;
    }

    const auto &node = gng_->nodeAt(node_id);
    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    out.name = chain_joint_names_;
    out.position.resize(chain_joint_names_.size(), 0.0);
    out.velocity.resize(chain_joint_names_.size(), 0.0);
    out.effort.resize(chain_joint_names_.size(), 0.0);

    const int copy_dim = std::min(static_cast<int>(chain_joint_names_.size()),
                                  static_cast<int>(node.weight_angle.size()));
    for (int i = 0; i < copy_dim; ++i) {
      out.position[static_cast<std::size_t>(i)] = node.weight_angle[i];
    }

    safe_target_pub_->publish(out);
  }

  void publishGraphLocked(int selected_id) {
    if (!gng_) {
      return;
    }

    ais_gng_msgs::msg::TopologicalMap msg;
    msg.header.stamp = now();
    msg.header.frame_id = get_parameter("root_link").as_string();
    msg.frame_number = 0;

    std::unordered_map<int, uint16_t> id_to_index;
    msg.nodes.reserve(gng_->getMaxNodeNum());

    for (std::size_t i = 0; i < gng_->getMaxNodeNum(); ++i) {
      const auto &node = gng_->nodeAt(static_cast<int>(i));
      if (node.id == -1) {
        continue;
      }

      ais_gng_msgs::msg::TopologicalNode out;
      out.id = static_cast<uint16_t>(node.id);
      out.pos.x = node.weight_coord.x();
      out.pos.y = node.weight_coord.y();
      out.pos.z = node.weight_coord.z();
      out.normal.x = node.status.ee_direction.x();
      out.normal.y = node.status.ee_direction.y();
      out.normal.z = node.status.ee_direction.z();
      out.rho = 0.0f;
      out.label = labelFromStatus(node.status);
      if (node.id == selected_id) {
        out.label = 1;
      }

      id_to_index[node.id] = static_cast<uint16_t>(msg.nodes.size());
      msg.nodes.push_back(std::move(out));
    }

    std::unordered_set<uint64_t> seen_edges;
    for (std::size_t i = 0; i < gng_->getMaxNodeNum(); ++i) {
      const auto &node = gng_->nodeAt(static_cast<int>(i));
      if (node.id == -1) {
        continue;
      }

      const auto &neighbors = gng_->getNeighborsAngle(static_cast<int>(i));
      for (int neighbor_id : neighbors) {
        const auto it = id_to_index.find(neighbor_id);
        if (it == id_to_index.end()) {
          continue;
        }

        const int lo = std::min(node.id, neighbor_id);
        const int hi = std::max(node.id, neighbor_id);
        const uint64_t key = (static_cast<uint64_t>(lo) << 32) |
                             static_cast<uint32_t>(hi);
        if (!seen_edges.insert(key).second) {
          continue;
        }

        msg.edges.push_back(id_to_index[node.id]);
        msg.edges.push_back(it->second);
      }
    }

    graph_pub_->publish(msg);
  }

  void updateLocked() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!gng_ || !have_state_) {
      return;
    }

    const Eigen::VectorXf current_q = currentJointVectorLocked();
    const int nearest_id = findNearestNode(current_q);
    if (nearest_id < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "No nearest GNG node found for current posture.");
      return;
    }

    std::vector<int> explored_order;
    const std::optional<int> safe_id = findSafeNodeBfs(nearest_id, explored_order);

    publishGraphLocked(safe_id.value_or(-1));

    if (!safe_id) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "No safe node found from nearest=%d after BFS exploration.", nearest_id);
      return;
    }

    publishSafeTargetLocked(*safe_id);

    const auto &selected = gng_->nodeAt(*safe_id);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Selected safe node=%d from nearest=%d explored=%zu safe_neighborhood=%s",
        *safe_id, nearest_id, explored_order.size(),
        hasAllSafeNeighbors(*safe_id) ? "true" : "false");

    (void)selected;
  }

  std::shared_ptr<::simulation::RobotModel> model_;
  std::shared_ptr<::kinematics::KinematicChain> chain_;
  std::unique_ptr<GNGType> gng_;
  std::vector<std::string> chain_joint_names_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr graph_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr safe_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex mutex_;
  sensor_msgs::msg::JointState latest_state_;
  bool have_state_ = false;
};

} // namespace robot_sim::planning

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::planning::ArmAvoidNode)

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_sim::planning::ArmAvoidNode>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}