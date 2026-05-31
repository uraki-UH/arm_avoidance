#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <mutex>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "common/resource_utils.hpp"
#include "planning/gng_dijkstra_planner.hpp"
#include "planning/joint_linf_cost.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"
#include "safety_engine/gng/GrowingNeuralGas.hpp"

namespace {

static std::vector<std::string> collectTerminalLeafLinks(
    const ::simulation::RobotModel &model) {
  std::unordered_set<std::string> parent_links;
  for (const auto &[joint_name, joint_props] : model.getJoints()) {
    (void)joint_name;
    parent_links.insert(joint_props.parent_link);
  }

  std::vector<std::string> leaves;
  for (const auto &[link_name, link_props] : model.getLinks()) {
    (void)link_props;
    if (link_name == model.getRootLinkName()) {
      continue;
    }
    if (parent_links.count(link_name) == 0) {
      leaves.push_back(link_name);
    }
  }

  std::sort(leaves.begin(), leaves.end());
  leaves.erase(std::unique(leaves.begin(), leaves.end()), leaves.end());
  return leaves;
}

static std::string getStringWithFallback(
    rclcpp::Node &node, const std::string &nested_key,
    const std::string &legacy_key) {
  const std::string nested = node.get_parameter(nested_key).as_string();
  if (!nested.empty()) {
    return nested;
  }
  return node.get_parameter(legacy_key).as_string();
}

static std::vector<std::string> orderedJointNames(
    const ::kinematics::KinematicChain &chain) {
  std::vector<std::string> names;
  names.reserve(static_cast<std::size_t>(chain.getNumJoints()));
  for (int i = 0; i < chain.getNumJoints(); ++i) {
    names.push_back(chain.getJointName(i));
  }
  return names;
}

static uint8_t pathLabelFromStatus(const ::GNG::Status &status) {
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

class TopologicalMapAvoidanceNode : public rclcpp::Node {
public:
  using GNGType = ::GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
  using PlannerType =
      ::planning::GngDijkstraPlanner<Eigen::VectorXf, Eigen::Vector3f, GNGType>;
  using CostType = ::planning::JointLInfCost<Eigen::VectorXf, Eigen::Vector3f>;

  explicit TopologicalMapAvoidanceNode(const rclcpp::NodeOptions &options)
      : Node("topological_map_avoidance_node", options) {
    declare_parameter("robot_urdf_path",
                      "package://topoarm_description/urdf/topo_dual_arm.urdf.xacro");
    declare_parameter("gng_model_path", "");
    declare_parameter("gng.data_directory", "");
    declare_parameter("gng.experiment_id", "");
    declare_parameter("gng.gng_model_filename", "");
    declare_parameter("joint_topic", "/ToPoDualArm/joint_states");
    declare_parameter("topological_map_topic", "/ToPoDualArm/topological_map_static");
    declare_parameter("target_topic", "target_joint_states");
    declare_parameter("trajectory_topic", "/ToPoDualArm/planned_topological_map");
    declare_parameter("publish_hz", 20.0);
    declare_parameter("avoid_collisions", true);
    declare_parameter("strict_goal_collision_check", false);
    declare_parameter("trial_mode", false);
    declare_parameter("trial_goal_interval_sec", 4.0);
    declare_parameter("trial_safe_only", true);
    declare_parameter("trial_seed", 0);

    const std::string urdf_rel = get_parameter("robot_urdf_path").as_string();
    const std::string urdf_path = robot_sim::common::resolvePath(urdf_rel);
    if (urdf_path.empty()) {
      throw std::runtime_error("Failed to resolve robot URDF path.");
    }

    auto model = std::make_shared<::simulation::RobotModel>(
        ::simulation::loadRobotFromUrdf(urdf_path));

    const auto leaf_links = collectTerminalLeafLinks(*model);
    if (leaf_links.empty()) {
      throw std::runtime_error("No terminal leaf links found in robot model.");
    }

    std::vector<::simulation::ArmConfig> arm_configs;
    arm_configs.reserve(leaf_links.size());
    for (const auto &leaf : leaf_links) {
      ::simulation::ArmConfig cfg;
      cfg.root_link = model->getRootLinkName();
      cfg.leaf_link = leaf;
      cfg.prefix = "";
      arm_configs.push_back(cfg);
    }

    chain_ = std::shared_ptr<::kinematics::KinematicChain>(
        ::simulation::createMultiArmKinematicChain(*model, arm_configs,
                                                   Eigen::Vector3d::Zero())
            .release());
    if (!chain_) {
      throw std::runtime_error("Failed to build kinematic chain.");
    }

    joint_names_ = orderedJointNames(*chain_);
    joint_index_by_name_.reserve(joint_names_.size());
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_by_name_[joint_names_[i]] = i;
    }

    const int dof = chain_->getTotalDOF();
    gng_ = std::make_shared<GNGType>(dof, 3, chain_.get());

    std::string gng_model_path = get_parameter("gng_model_path").as_string();
    if (gng_model_path.empty()) {
      std::string data_dir = get_parameter("gng.data_directory").as_string();
      std::string exp_id = get_parameter("gng.experiment_id").as_string();
      std::string model_file = get_parameter("gng.gng_model_filename").as_string();
      if (!data_dir.empty() && !exp_id.empty() && !model_file.empty()) {
        gng_model_path = data_dir + "/" + exp_id + "/" + model_file;
      }
    }

    if (gng_model_path.empty()) {
      RCLCPP_WARN(get_logger(),
                  "gng_model_path is empty. The avoidance node will not publish until a model is provided.");
    } else if (!gng_->load(gng_model_path)) {
      throw std::runtime_error("Failed to load GNG model from: " + gng_model_path);
    }

    planner_.setCostEvaluator(
        std::make_shared<CostType>(1000.0f));
    planner_.setAvoidCollisions(get_parameter("avoid_collisions").as_bool());
    planner_.setStrictGoalCollisionCheck(
        get_parameter("strict_goal_collision_check").as_bool());

    trial_mode_ = get_parameter("trial_mode").as_bool();
    trial_goal_interval_sec_ = std::max(0.1, get_parameter("trial_goal_interval_sec").as_double());
    trial_safe_only_ = get_parameter("trial_safe_only").as_bool();
    const int trial_seed = get_parameter("trial_seed").as_int();
    if (trial_seed == 0) {
      rng_.seed(std::random_device{}());
    } else {
      rng_.seed(static_cast<std::mt19937::result_type>(trial_seed));
    }

    const std::string joint_topic = get_parameter("joint_topic").as_string();
    const std::string topological_map_topic =
        get_parameter("topological_map_topic").as_string();
    trajectory_topic_ = get_parameter("trajectory_topic").as_string();
    target_topic_ = get_parameter("target_topic").as_string();
    if (target_topic_.empty()) {
      target_topic_ = "target_joint_states";
    }

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, rclcpp::QoS(10).reliable(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_joint_state_ = *msg;
          have_joint_state_ = true;
        });

    map_sub_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
        topological_map_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_map_ = *msg;
          have_map_ = true;
          updateNodeStatusFromMapLocked(*msg);
        });

    trajectory_pub_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
        trajectory_topic_, rclcpp::QoS(1).reliable().transient_local());

    target_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        target_topic_, rclcpp::QoS(10).reliable());

    const double publish_hz = std::max(1.0, get_parameter("publish_hz").as_double());
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz)),
        [this]() { this->publishTargetLocked(); });

    RCLCPP_INFO(get_logger(),
                "TopologicalMapAvoidanceNode ready. joint_topic=%s map_topic=%s target_topic=%s trajectory_topic=%s dof=%d trial_mode=%d",
                joint_topic.c_str(), topological_map_topic.c_str(),
                target_topic_.c_str(), trajectory_topic_.c_str(), dof, trial_mode_ ? 1 : 0);
    RCLCPP_INFO(get_logger(),
                "Waiting for inputs: joint_topic=%s topological_map_topic=%s",
                joint_topic.c_str(), topological_map_topic.c_str());
  }

private:
  Eigen::VectorXf currentJointVectorLocked() const {
    Eigen::VectorXf q(static_cast<int>(joint_names_.size()));
    q.setZero();

    std::unordered_map<std::string, double> value_by_name;
    value_by_name.reserve(latest_joint_state_.name.size());
    for (std::size_t i = 0; i < latest_joint_state_.name.size(); ++i) {
      if (i < latest_joint_state_.position.size()) {
        value_by_name[latest_joint_state_.name[i]] =
            static_cast<float>(latest_joint_state_.position[i]);
      }
    }

    const bool order_matches =
        latest_joint_state_.name.size() == joint_names_.size();
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const auto it = value_by_name.find(joint_names_[i]);
      if (it != value_by_name.end()) {
        q[static_cast<int>(i)] = static_cast<float>(it->second);
      } else if (order_matches && i < latest_joint_state_.position.size()) {
        q[static_cast<int>(i)] =
            static_cast<float>(latest_joint_state_.position[i]);
      }
    }
    return q;
  }

  int findNearestActiveNodeLocked(const Eigen::VectorXf &q) const {
    if (!gng_) {
      return -1;
    }
    float min_dist = std::numeric_limits<float>::max();
    int nearest_id = -1;
    for (const auto &node : gng_->getNodes()) {
      if (node.id == -1 || !node.status.active) {
        continue;
      }
      const int dim = std::min(static_cast<int>(node.weight_angle.size()),
                               static_cast<int>(q.size()));
      if (dim <= 0) {
        continue;
      }
      const float d = (node.weight_angle.head(dim) - q.head(dim)).norm();
      if (d < min_dist) {
        min_dist = d;
        nearest_id = node.id;
      }
    }
    return nearest_id;
  }

  void updateNodeStatusFromMapLocked(const ais_gng_msgs::msg::TopologicalMap &msg) {
    if (!gng_) {
      return;
    }

    cached_safe_goal_ids_.clear();
    cached_safe_goal_ids_.reserve(msg.nodes.size());

    for (const auto &n : msg.nodes) {
      const int id = static_cast<int>(n.id);
      if (id < 0 || id >= static_cast<int>(gng_->getMaxNodeNum())) {
        continue;
      }
      auto &node = gng_->nodeAt(id);
      if (node.id == -1) {
        continue;
      }

      switch (n.label) {
      case 2: // collision
        node.status.is_colliding = true;
        node.status.is_danger = false;
        node.status.valid = false;
        node.status.collision_count = 1;
        node.status.danger_count = 0;
        break;
      case 3: // danger
        node.status.is_colliding = false;
        node.status.is_danger = true;
        node.status.valid = true;
        node.status.collision_count = 0;
        node.status.danger_count = 1;
        break;
      default:
        node.status.is_colliding = false;
        node.status.is_danger = false;
        node.status.valid = true;
        node.status.collision_count = 0;
        node.status.danger_count = 0;
        break;
      }
      node.status.active = true;

      if (!node.status.is_colliding && !node.status.is_danger) {
        cached_safe_goal_ids_.push_back(id);
      }
    }
  }

  void publishTargetLocked() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!gng_ || !have_joint_state_ || !have_map_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting: gng=%d joint=%d map=%d (joint_topic=%s map_topic=%s trial_mode=%d)",
          gng_ ? 1 : 0, have_joint_state_ ? 1 : 0, have_map_ ? 1 : 0,
          get_parameter("joint_topic").as_string().c_str(),
          get_parameter("topological_map_topic").as_string().c_str(),
          trial_mode_ ? 1 : 0);
      return;
    }

    const Eigen::VectorXf current_q = currentJointVectorLocked();
    if (current_q.size() == 0) {
      return;
    }

    const int start_id = findNearestActiveNodeLocked(current_q);
    if (start_id < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "No active nearest GNG node found for current joint state.");
      return;
    }

    const auto &start_node = gng_->nodeAt(start_id);
    Eigen::VectorXf target_q = current_q;

    if (trial_mode_) {
      const bool need_new_goal =
          current_goal_id_ < 0 ||
          (now().seconds() - last_goal_update_sec_) >= trial_goal_interval_sec_ ||
          current_goal_id_ == start_id ||
          !isGoalStillSelectableLocked(current_goal_id_);

      if (need_new_goal) {
        current_goal_id_ = pickRandomGoalLocked(start_id);
        last_goal_update_sec_ = now().seconds();
        if (current_goal_id_ >= 0) {
          RCLCPP_INFO(
              get_logger(),
              "Trial goal selected: start=%d goal=%d safe_only=%d safe_count=%zu",
              start_id, current_goal_id_, trial_safe_only_ ? 1 : 0,
              cached_safe_goal_ids_.size());
        }
        else {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Trial mode: no selectable goal found (safe_only=%d safe_count=%zu).",
              trial_safe_only_ ? 1 : 0, cached_safe_goal_ids_.size());
        }
      }

      if (current_goal_id_ >= 0) {
        const auto node_path =
            planner_.planNodeIndices(start_id, current_goal_id_, *gng_);
        publishTrajectoryPathLocked(node_path);
        if (!node_path.empty()) {
          const int target_node_id =
              (node_path.size() >= 2) ? node_path[1] : node_path[0];
          if (target_node_id >= 0 &&
              target_node_id < static_cast<int>(gng_->getMaxNodeNum())) {
            target_q = gng_->nodeAt(target_node_id).weight_angle;
          }
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Trial path: start=%d goal=%d len=%zu",
              start_id, current_goal_id_, node_path.size());
        } else {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Trial mode: planner returned empty path start=%d goal=%d",
              start_id, current_goal_id_);
        }
      }
    } else if (start_node.status.is_colliding || start_node.status.is_danger) {
      if (!cached_safe_goal_ids_.empty()) {
        auto [reached_goal_id, node_path] =
            planner_.planToAnyNode(start_id, cached_safe_goal_ids_, *gng_);
        publishTrajectoryPathLocked(node_path);
        if (!node_path.empty()) {
          const int target_node_id =
              (node_path.size() >= 2) ? node_path[1] : node_path[0];
          if (target_node_id >= 0 &&
              target_node_id < static_cast<int>(gng_->getMaxNodeNum())) {
            target_q = gng_->nodeAt(target_node_id).weight_angle;
          }
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Avoidance path: start=%d goal=%d len=%zu status(c=%d d=%d)",
              start_id, reached_goal_id, node_path.size(),
              start_node.status.is_colliding ? 1 : 0,
              start_node.status.is_danger ? 1 : 0);
        } else {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Avoidance mode: planner returned empty path start=%d safe_goals=%zu",
              start_id, cached_safe_goal_ids_.size());
        }
      } else {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Avoidance mode: no safe goal candidates available.");
      }
    }

    if (!trial_mode_ && !(start_node.status.is_colliding || start_node.status.is_danger)) {
      publishTrajectoryPathLocked({});
    }

    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    out.name = joint_names_;
    out.position.resize(static_cast<std::size_t>(target_q.size()));
    for (int i = 0; i < target_q.size(); ++i) {
      out.position[static_cast<std::size_t>(i)] = static_cast<double>(target_q[i]);
    }
    target_pub_->publish(out);
    last_target_q_ = target_q;
  }

  std::shared_ptr<::kinematics::KinematicChain> chain_;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::size_t> joint_index_by_name_;
  std::shared_ptr<GNGType> gng_;
  PlannerType planner_;

  std::string target_topic_;
  std::string trajectory_topic_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_pub_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr trajectory_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  ais_gng_msgs::msg::TopologicalMap latest_map_;
  bool have_joint_state_ = false;
  bool have_map_ = false;
  Eigen::VectorXf last_target_q_;
  std::vector<int> cached_safe_goal_ids_;
  bool trial_mode_ = false;
  double trial_goal_interval_sec_ = 4.0;
  bool trial_safe_only_ = true;
  int current_goal_id_ = -1;
  double last_goal_update_sec_ = 0.0;
  std::mt19937 rng_;

  int pickRandomGoalLocked(int start_id) {
    std::vector<int> candidates;
    if (trial_safe_only_ && !cached_safe_goal_ids_.empty()) {
      candidates = cached_safe_goal_ids_;
    } else if (gng_) {
      candidates.reserve(gng_->getMaxNodeNum());
      gng_->forEachActiveValid([&](int id, const auto &node) {
        (void)node;
        candidates.push_back(id);
      });
    }

    candidates.erase(
        std::remove(candidates.begin(), candidates.end(), start_id),
        candidates.end());
    if (candidates.empty()) {
      return -1;
    }

    std::uniform_int_distribution<std::size_t> dist(
        0, candidates.size() - 1);
    return candidates[dist(rng_)];
  }

  bool isGoalStillSelectableLocked(int goal_id) const {
    if (goal_id < 0 || !gng_) {
      return false;
    }
    if (trial_safe_only_) {
      return std::find(cached_safe_goal_ids_.begin(), cached_safe_goal_ids_.end(),
                       goal_id) != cached_safe_goal_ids_.end();
    }
    return goal_id < static_cast<int>(gng_->getMaxNodeNum()) &&
           gng_->nodeAt(goal_id).id != -1 &&
           gng_->nodeAt(goal_id).status.active &&
           gng_->nodeAt(goal_id).status.valid;
  }

  void publishTrajectoryPathLocked(const std::vector<int> &node_path) {
    if (!trajectory_pub_ || !gng_) {
      return;
    }

    ais_gng_msgs::msg::TopologicalMap msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.frame_number = 0;

    if (node_path.empty()) {
      trajectory_pub_->publish(msg);
      return;
    }

    std::unordered_map<int, uint16_t> id_to_index;
    msg.nodes.reserve(node_path.size());
    for (int node_id : node_path) {
      if (node_id < 0 || node_id >= static_cast<int>(gng_->getMaxNodeNum())) {
        continue;
      }
      const auto &node = gng_->nodeAt(node_id);
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
      out.label = pathLabelFromStatus(node.status);
      out.age = 0;
      const uint16_t published_index = static_cast<uint16_t>(msg.nodes.size());
      id_to_index.emplace(node.id, published_index);
      msg.nodes.push_back(std::move(out));
    }

    for (std::size_t i = 0; i + 1 < node_path.size(); ++i) {
      const auto ia = id_to_index.find(node_path[i]);
      const auto ib = id_to_index.find(node_path[i + 1]);
      if (ia == id_to_index.end() || ib == id_to_index.end()) {
        continue;
      }
      msg.edges.push_back(ia->second);
      msg.edges.push_back(ib->second);
    }

    trajectory_pub_->publish(msg);
  }
};

} // namespace robot_sim::planning

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::planning::TopologicalMapAvoidanceNode)

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::planning::TopologicalMapAvoidanceNode>(
      rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
