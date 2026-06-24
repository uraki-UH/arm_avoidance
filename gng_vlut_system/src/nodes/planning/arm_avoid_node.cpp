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

#include "core/common/constants.hpp"
#include "common/resource_utils.hpp"
#include "planning/arm_avoid_helpers.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include "gng/GrowingNeuralGas.hpp"

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
    declare_parameter<std::string>("urdf_path", "");
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
        robot_sim::common::resolvePath(get_parameter("urdf_path").as_string());
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
    return arm_avoid::currentJointVector<GNGType>(chain_joint_names_, latest_state_);
  }

  int findNearestNode(const Eigen::VectorXf &posture) const {
    return arm_avoid::findNearestNode(gng_, posture);
  }

  bool isSafeNode(int node_id) const {
    return arm_avoid::isSafeNode(gng_, node_id);
  }

  bool hasAllSafeNeighbors(int node_id) const {
    return arm_avoid::hasAllSafeNeighbors(gng_, node_id);
  }

  std::optional<int> findSafeNodeBfs(int start_id,
                                     std::vector<int> &explored_order) const {
    return arm_avoid::findSafeNodeBfs(gng_, start_id, explored_order);
  }

  void publishSafeTargetLocked(int node_id) {
    if (!gng_ || node_id < 0 ||
        static_cast<std::size_t>(node_id) >= gng_->getMaxNodeNum()) {
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
    if (!graph_pub_) {
      return;
    }
    graph_pub_->publish(arm_avoid::buildGraphMessage(*this, gng_, selected_id));
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
