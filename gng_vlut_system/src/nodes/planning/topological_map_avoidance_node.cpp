#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <filesystem>
#include <mutex>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <gng_control_msgs/msg/joint_control_claim.hpp>

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

static std::vector<std::string> splitCommaSeparated(const std::string &text) {
  std::vector<std::string> items;
  std::stringstream ss(text);
  std::string token;
  while (std::getline(ss, token, ',')) {
    auto begin = token.find_first_not_of(" \t");
    auto end = token.find_last_not_of(" \t");
    if (begin == std::string::npos) {
      continue;
    }
    items.push_back(token.substr(begin, end - begin + 1));
  }
  return items;
}

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

static std::vector<std::string> orderedControlledJointNames(
    const ::kinematics::KinematicChain &chain) {
  std::vector<std::string> names;
  for (int i = 0; i < chain.getNumJoints(); ++i) {
    if (chain.getJointDOF(i) <= 0) {
      continue;
    }
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

static std::vector<double> eigenToStdVector(const Eigen::VectorXf &q) {
  std::vector<double> out(static_cast<std::size_t>(q.size()));
  for (int i = 0; i < q.size(); ++i) {
    out[static_cast<std::size_t>(i)] = static_cast<double>(q[i]);
  }
  return out;
}

static double quaternionAngularErrorDeg(
    const Eigen::Quaterniond &a, const Eigen::Quaterniond &b) {
  Eigen::Quaterniond qa = a.normalized();
  Eigen::Quaterniond qb = b.normalized();
  double dot = std::abs(qa.dot(qb));
  dot = std::min(1.0, std::max(-1.0, dot));
  return 2.0 * std::acos(dot) * 180.0 / M_PI;
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
    declare_parameter("gng.profile_names", "");
    declare_parameter("joint_topic", "/ToPoDualArm/joint_states");
    declare_parameter("topological_map_topic", "/ToPoDualArm/topological_map_static");
    declare_parameter("target_topic", "target_joint_states");
    declare_parameter("control_claim_topic", "");
    declare_parameter("control_claim_priority", 10);
    declare_parameter("control_claim_mode", static_cast<int>(gng_control_msgs::msg::JointControlClaim::MODE_EXCLUSIVE));
    declare_parameter("control_claim_enabled", true);
    declare_parameter("trajectory_topic", "/ToPoDualArm/planned_topological_map");
    declare_parameter("publish_hz", 20.0);
    declare_parameter("avoid_collisions", true);
    declare_parameter("avoid_danger", true);
    declare_parameter("strict_goal_collision_check", false);
    declare_parameter("trial_mode", false);
    declare_parameter("trial_goal_interval_sec", 4.0);
    declare_parameter("trial_safe_only", true);
    declare_parameter("trial_seed", 0);
    declare_parameter("waypoint_tolerance", 0.05);

    const std::string urdf_rel = get_parameter("robot_urdf_path").as_string();
    const std::string urdf_path = robot_sim::common::resolvePath(urdf_rel);
    if (urdf_path.empty()) {
      throw std::runtime_error("Failed to resolve robot URDF path.");
    }

    auto model = std::make_shared<::simulation::RobotModel>(
        ::simulation::loadRobotFromUrdf(urdf_path));

    std::vector<::simulation::ArmConfig> arm_configs;
    const auto selected_profiles =
        splitCommaSeparated(get_parameter("gng.profile_names").as_string());
    for (const auto &profile : selected_profiles) {
      const std::string root_param =
          "gng.profiles." + profile + ".root";
      const std::string eef_param =
          "gng.profiles." + profile + ".eef";
      const std::string root_link =
          declare_parameter<std::string>(root_param, model->getRootLinkName());
      const std::string eef_link =
          declare_parameter<std::string>(eef_param, "");
      if (eef_link.empty()) {
        RCLCPP_WARN(get_logger(),
                    "Skipping empty GNG profile arm config: profile=%s root=%s",
                    profile.c_str(), root_link.c_str());
        continue;
      }

      ::simulation::ArmConfig cfg;
      cfg.root_link = root_link;
      cfg.leaf_link = eef_link;
      cfg.prefix = "";
      arm_configs.push_back(cfg);
    }

    if (arm_configs.empty()) {
      const auto leaf_links = collectTerminalLeafLinks(*model);
      if (leaf_links.empty()) {
        throw std::runtime_error("No terminal leaf links found in robot model.");
      }

      arm_configs.reserve(leaf_links.size());
      for (const auto &leaf : leaf_links) {
        ::simulation::ArmConfig cfg;
        cfg.root_link = model->getRootLinkName();
        cfg.leaf_link = leaf;
        cfg.prefix = "";
        arm_configs.push_back(cfg);
      }
    }

    if (arm_configs.size() == 1) {
      chain_ = std::make_shared<::kinematics::KinematicChain>(
          ::simulation::createKinematicChainFromModel(
              *model, arm_configs.front().leaf_link, Eigen::Vector3d::Zero(),
              arm_configs.front().root_link));
    } else {
      chain_ = std::shared_ptr<::kinematics::KinematicChain>(
          ::simulation::createMultiArmKinematicChain(*model, arm_configs,
                                                     Eigen::Vector3d::Zero())
              .release());
    }
    if (!chain_) {
      throw std::runtime_error("Failed to build kinematic chain.");
    }

    chain_joint_names_ = orderedJointNames(*chain_);
    controlled_joint_names_ = orderedControlledJointNames(*chain_);

    const int dof = chain_->getTotalDOF();
    RCLCPP_INFO(get_logger(), "Selected GNG profiles: %s",
                joinStrings(selected_profiles, ", ").c_str());
    RCLCPP_INFO(get_logger(), "Chain joint order (%zu): %s",
                chain_joint_names_.size(),
                joinStrings(chain_joint_names_, ", ").c_str());
    RCLCPP_INFO(get_logger(), "Controlled joint order (%zu): %s",
                controlled_joint_names_.size(),
                joinStrings(controlled_joint_names_, ", ").c_str());
    gng_ = std::make_shared<GNGType>(dof, 3, chain_.get());

    std::string gng_model_path = get_parameter("gng_model_path").as_string();
    if (gng_model_path.empty()) {
      std::string data_dir = get_parameter("gng.data_directory").as_string();
      std::string exp_id = get_parameter("gng.experiment_id").as_string();
      std::string model_file = get_parameter("gng.gng_model_filename").as_string();
      if (!data_dir.empty() && std::filesystem::path(data_dir).is_absolute() &&
          !std::filesystem::exists(data_dir)) {
        data_dir = std::filesystem::path(data_dir).filename().string();
      }
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

    int loaded_angle_dim = -1;
    for (const auto &node : gng_->getNodes()) {
      if (node.id != -1) {
        loaded_angle_dim = static_cast<int>(node.weight_angle.size());
        break;
      }
    }
    if (loaded_angle_dim > 0 &&
        loaded_angle_dim != static_cast<int>(chain_->getTotalDOF())) {
      RCLCPP_WARN(
          get_logger(),
          "GNG angle dimension (%d) does not match chain DOF (%d). Check gng.profile_names / arm config.",
          loaded_angle_dim, chain_->getTotalDOF());
    }

    cached_safe_goal_ids_.clear();
    cached_safe_goal_ids_.reserve(gng_->getMaxNodeNum());
    for (const auto &node : gng_->getNodes()) {
      if (node.id != -1 && node.status.active && node.status.valid &&
          !node.status.is_colliding && !node.status.is_danger) {
        cached_safe_goal_ids_.push_back(node.id);
      }
    }
    if (!cached_safe_goal_ids_.empty()) {
      RCLCPP_INFO(
          get_logger(),
          "Cached %zu safe GNG nodes from loaded model before map updates.",
          cached_safe_goal_ids_.size());
    }

    planner_.setCostEvaluator(
        std::make_shared<CostType>(1000.0f));
    planner_.setAvoidCollisions(get_parameter("avoid_collisions").as_bool());
    avoid_danger_ = get_parameter("avoid_danger").as_bool();
    planner_.setAvoidDanger(avoid_danger_);
    planner_.setStrictGoalCollisionCheck(
        get_parameter("strict_goal_collision_check").as_bool());

    trial_mode_ = get_parameter("trial_mode").as_bool();
    trial_goal_interval_sec_ = std::max(0.1, get_parameter("trial_goal_interval_sec").as_double());
    trial_safe_only_ = get_parameter("trial_safe_only").as_bool();
    waypoint_tolerance_ = std::max(1e-6, get_parameter("waypoint_tolerance").as_double());
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
      const std::string ns_raw = std::string(get_namespace());
      const std::string ns = ns_raw.empty() ? "" : (ns_raw.front() == '/' ? ns_raw.substr(1) : ns_raw);
      target_topic_ = "/" + ns + "/target_joint_states";
    }
    control_claim_topic_ = get_parameter("control_claim_topic").as_string();
    if (control_claim_topic_.empty()) {
      const std::string ns_raw = std::string(get_namespace());
      const std::string ns = ns_raw.empty() ? "" : (ns_raw.front() == '/' ? ns_raw.substr(1) : ns_raw);
      control_claim_topic_ = "/" + ns + "/control_claims";
    }
    control_claim_priority_ = get_parameter("control_claim_priority").as_int();
    control_claim_mode_ = get_parameter("control_claim_mode").as_int();
    control_claim_enabled_ = get_parameter("control_claim_enabled").as_bool();

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

    control_claim_pub_ = create_publisher<gng_control_msgs::msg::JointControlClaim>(
        control_claim_topic_, rclcpp::QoS(1).reliable().transient_local());

    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          result.reason = "ok";
          std::lock_guard<std::mutex> lock(mutex_);
          for (const auto & param : params) {
            const auto & name = param.get_name();
            if (name == "control_claim_priority") {
              control_claim_priority_ = param.as_int();
            } else if (name == "control_claim_mode") {
              control_claim_mode_ = param.as_int();
            } else if (name == "control_claim_enabled") {
              control_claim_enabled_ = param.as_bool();
            }
          }
          return result;
        });

    request_update_srv_ = create_service<std_srvs::srv::Trigger>(
        "request_trajectory_update",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(mutex_);
          trajectory_update_requested_ = true;
          response->success = true;
          response->message = "trajectory update requested";
        });

    const double publish_hz = std::max(1.0, get_parameter("publish_hz").as_double());
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz)),
        [this]() { this->publishTargetLocked(); });

    RCLCPP_INFO(get_logger(),
                "TopologicalMapAvoidanceNode ready. joint_topic=%s map_topic=%s target_topic=%s claim_topic=%s trajectory_topic=%s dof=%d trial_mode=%d",
                joint_topic.c_str(), topological_map_topic.c_str(),
                target_topic_.c_str(), control_claim_topic_.c_str(), trajectory_topic_.c_str(), dof, trial_mode_ ? 1 : 0);
    RCLCPP_INFO(get_logger(),
                "Waiting for inputs: joint_topic=%s topological_map_topic=%s",
                joint_topic.c_str(), topological_map_topic.c_str());
  }

private:
  Eigen::VectorXf currentJointVectorLocked() const {
    Eigen::VectorXf q(static_cast<int>(chain_->getTotalDOF()));
    q.setZero();

    std::unordered_map<std::string, double> value_by_name;
    value_by_name.reserve(latest_joint_state_.name.size());
    for (std::size_t i = 0; i < latest_joint_state_.name.size(); ++i) {
      if (i < latest_joint_state_.position.size()) {
        value_by_name[latest_joint_state_.name[i]] =
            static_cast<float>(latest_joint_state_.position[i]);
      }
    }

    std::size_t dof_cursor = 0;
    for (int joint_index = 0; joint_index < chain_->getNumJoints(); ++joint_index) {
      const int joint_dof = chain_->getJointDOF(joint_index);
      if (joint_dof <= 0) {
        continue;
      }
      const std::string joint_name = chain_->getJointName(joint_index);
      const auto it = value_by_name.find(joint_name);
      const float v = (it != value_by_name.end())
                          ? static_cast<float>(it->second)
                          : 0.0f;
      for (int d = 0; d < joint_dof && dof_cursor < static_cast<std::size_t>(q.size()); ++d) {
        q[static_cast<int>(dof_cursor++)] = v;
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
    if (!gng_ || (!have_joint_state_ && !trial_mode_)) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting: gng=%d joint=%d map=%d (joint_topic=%s map_topic=%s trial_mode=%d)",
          gng_ ? 1 : 0, have_joint_state_ ? 1 : 0, have_map_ ? 1 : 0,
          get_parameter("joint_topic").as_string().c_str(),
          get_parameter("topological_map_topic").as_string().c_str(),
          trial_mode_ ? 1 : 0);
      return;
    }

    if (!have_map_ && !trial_mode_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for topological map: map_topic=%s trial_mode=%d",
          get_parameter("topological_map_topic").as_string().c_str(),
          trial_mode_ ? 1 : 0);
      return;
    }

    Eigen::VectorXf current_q = currentJointVectorLocked();
    if (current_q.size() == 0) {
      current_q = Eigen::VectorXf::Zero(chain_->getTotalDOF());
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No joint_state received yet; using zero current_q fallback for trial execution.");
    }

    const int start_id = findNearestActiveNodeLocked(current_q);
    if (start_id < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "No active nearest GNG node found for current joint state.");
      return;
    }

    const auto &start_node = gng_->nodeAt(start_id);
    Eigen::VectorXf target_q = current_q;

    if (active_trajectory_valid_) {
      if (active_waypoint_index_ >= active_node_path_.size()) {
        clearActiveTrajectoryLocked();
      } else {
        const int waypoint_node_id = active_node_path_[active_waypoint_index_];
        if (isWaypointReachedLocked(current_q, waypoint_node_id)) {
          ++active_waypoint_index_;
          if (active_waypoint_index_ >= active_node_path_.size()) {
            RCLCPP_INFO(
                get_logger(),
                "Trajectory completed: goal=%d path_len=%zu", active_goal_id_,
                active_node_path_.size());
            clearActiveTrajectoryLocked();
            trajectory_update_requested_ = true;
            goal_replan_failures_ = 0;
            force_goal_switch_ = false;
          }
        }
      }
    }

    if (trajectory_update_requested_ && !active_trajectory_valid_) {
      if (trial_mode_) {
        active_goal_id_ = pickRandomGoalLocked(start_id);
        if (active_goal_id_ >= 0) {
          active_node_path_ = planner_.planNodeIndices(start_id, active_goal_id_, *gng_);
          active_waypoint_index_ = active_node_path_.size() >= 2 ? 1U : 0U;
          active_trajectory_valid_ = !active_node_path_.empty();
          trajectory_update_requested_ = !active_trajectory_valid_;
          if (active_trajectory_valid_) {
            RCLCPP_INFO(
                get_logger(),
                "Trial path latched: start=%d goal=%d len=%zu safe_only=%d safe_count=%zu",
                start_id, active_goal_id_, active_node_path_.size(),
                trial_safe_only_ ? 1 : 0, cached_safe_goal_ids_.size());
          } else {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Trial mode: planner returned empty path start=%d goal=%d",
                start_id, active_goal_id_);
          }
        } else {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Trial mode: no selectable goal found (safe_only=%d safe_count=%zu).",
              trial_safe_only_ ? 1 : 0, cached_safe_goal_ids_.size());
        }
      } else if (start_node.status.is_colliding ||
                 (start_node.status.is_danger && avoid_danger_)) {
        if (!cached_safe_goal_ids_.empty()) {
          auto [reached_goal_id, node_path] =
              planner_.planToAnyNode(start_id, cached_safe_goal_ids_, *gng_);
          active_goal_id_ = reached_goal_id;
          active_node_path_ = node_path;
          active_waypoint_index_ = active_node_path_.size() >= 2 ? 1U : 0U;
          active_trajectory_valid_ = !active_node_path_.empty();
          trajectory_update_requested_ = !active_trajectory_valid_;
          if (active_trajectory_valid_) {
            RCLCPP_INFO(
                get_logger(),
                "Avoidance path latched: start=%d goal=%d len=%zu status(c=%d d=%d)",
                start_id, reached_goal_id, active_node_path_.size(),
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
    }

    if (active_trajectory_valid_) {
      publishTrajectoryPathLocked(active_node_path_);
      if (active_waypoint_index_ < active_node_path_.size()) {
        const int target_node_id = active_node_path_[active_waypoint_index_];
        if (target_node_id >= 0 &&
            target_node_id < static_cast<int>(gng_->getMaxNodeNum())) {
          const auto isUnsafeNode = [this](int node_id) {
            if (!gng_ || node_id < 0 ||
                node_id >= static_cast<int>(gng_->getMaxNodeNum())) {
              return true;
            }

            const auto &node = gng_->nodeAt(node_id);
            return node.id == -1 || !node.status.active || !node.status.valid ||
                   node.status.is_colliding || node.status.is_danger;
          };

          const bool has_next = active_waypoint_index_ + 1 < active_node_path_.size();
          const bool has_next_next = active_waypoint_index_ + 2 < active_node_path_.size();
          const int next_node_id = has_next ? active_node_path_[active_waypoint_index_ + 1] : -1;
          const int next_next_node_id =
              has_next_next ? active_node_path_[active_waypoint_index_ + 2] : -1;

          const bool next_unsafe = has_next && isUnsafeNode(next_node_id);
          const bool next_next_unsafe = has_next_next && isUnsafeNode(next_next_node_id);

          if (next_next_unsafe) {
            target_q = current_q;
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Hold current posture: next-next waypoint is unsafe. start=%d goal=%d wp_idx=%zu next_id=%d next_next_id=%d",
                start_id, active_goal_id_, active_waypoint_index_, next_node_id,
                next_next_node_id);
          } else if (next_unsafe) {
            if (active_waypoint_index_ > 0) {
              const int retreat_node_id = active_node_path_[active_waypoint_index_ - 1];
              if (retreat_node_id >= 0 &&
                  retreat_node_id < static_cast<int>(gng_->getMaxNodeNum())) {
                target_q = gng_->nodeAt(retreat_node_id).weight_angle;
                active_waypoint_index_ -= 1;
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Retreat to previous waypoint: next waypoint is unsafe. start=%d goal=%d retreat_id=%d wp_idx=%zu next_id=%d",
                    start_id, active_goal_id_, retreat_node_id,
                    active_waypoint_index_, next_node_id);
              } else {
                target_q = current_q;
              }
            } else {
              target_q = current_q;
            }
          } else {
            target_q = gng_->nodeAt(target_node_id).weight_angle;
          }

          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Waypoint debug: start=%d goal=%d wp_idx=%zu wp_id=%d q_dim=%d target_dim=%d",
              start_id, active_goal_id_, active_waypoint_index_, target_node_id,
              static_cast<int>(current_q.size()), static_cast<int>(target_q.size()));

          const auto target_node_fk_q = eigenToStdVector(target_q);
          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
              target_positions;
          std::vector<Eigen::Quaterniond,
                      Eigen::aligned_allocator<Eigen::Quaterniond>>
              target_orientations;
          chain_->forwardKinematicsAt(target_node_fk_q, target_positions,
                                      target_orientations);

          const auto current_fk_q = eigenToStdVector(current_q);
          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
              current_positions;
          std::vector<Eigen::Quaterniond,
                      Eigen::aligned_allocator<Eigen::Quaterniond>>
              current_orientations;
          chain_->forwardKinematicsAt(current_fk_q, current_positions,
                                      current_orientations);

          if (!target_positions.empty() && !target_orientations.empty() &&
              !current_positions.empty() && !current_orientations.empty()) {
            const auto &wp = gng_->nodeAt(target_node_id);
            const Eigen::Vector3d target_waypoint_pos = wp.weight_coord.cast<double>();
            const Eigen::Quaterniond target_waypoint_ori =
                wp.status.ee_orientation.cast<double>();
            const Eigen::Vector3d current_eef_pos = current_positions.back();
            const Eigen::Quaterniond current_eef_ori = current_orientations.back();
            const Eigen::Vector3d target_fk_pos = target_positions.back();
            const Eigen::Quaterniond target_fk_ori = target_orientations.back();

            const double cur_pos_err = (current_eef_pos - target_waypoint_pos).norm();
            const double tgt_pos_err = (target_fk_pos - target_waypoint_pos).norm();
            const double cur_ori_err =
                quaternionAngularErrorDeg(current_eef_ori, target_waypoint_ori);
            const double tgt_ori_err =
                quaternionAngularErrorDeg(target_fk_ori, target_waypoint_ori);

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "EEF debug: current=[%.4f %.4f %.4f] target_wp=[%.4f %.4f %.4f] target_fk=[%.4f %.4f %.4f] pos_err(cur=%.4f tgtfk=%.4f) ori_err_deg(cur=%.3f tgtfk=%.3f)",
                current_eef_pos.x(), current_eef_pos.y(), current_eef_pos.z(),
                target_waypoint_pos.x(), target_waypoint_pos.y(),
                target_waypoint_pos.z(), target_fk_pos.x(), target_fk_pos.y(),
                target_fk_pos.z(), cur_pos_err, tgt_pos_err, cur_ori_err,
                tgt_ori_err);
          }
        }
      }
    } else if (!trial_mode_ && !(start_node.status.is_colliding || start_node.status.is_danger)) {
      publishTrajectoryPathLocked({});
    }

    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    out.name = controlled_joint_names_;
    out.position.resize(controlled_joint_names_.size(), 0.0);
    out.velocity.resize(controlled_joint_names_.size(), 0.0);
    out.effort.resize(controlled_joint_names_.size(), 0.0);
    const int target_dim = static_cast<int>(target_q.size());
    const int joint_dim = static_cast<int>(controlled_joint_names_.size());
    const int copy_dim = std::min(target_dim, joint_dim);
    if (target_dim != joint_dim) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "target_joint_states dimension mismatch: target_dim=%d joint_dim=%d. Clipping to %d.",
          target_dim, joint_dim, copy_dim);
    }
    for (int i = 0; i < copy_dim; ++i) {
      out.position[static_cast<std::size_t>(i)] =
          static_cast<double>(target_q[i]);
    }
    out.velocity.resize(controlled_joint_names_.size(), 0.0);
    out.effort.resize(controlled_joint_names_.size(), 0.0);
    target_pub_->publish(out);

    gng_control_msgs::msg::JointControlClaim claim;
    claim.command_topic = target_topic_;
    claim.joint_names = controlled_joint_names_;
    claim.priority = control_claim_priority_;
    claim.mode = static_cast<uint8_t>(control_claim_mode_);
    claim.enabled = control_claim_enabled_;
    control_claim_pub_->publish(claim);

    last_target_q_ = target_q;
  }

  std::shared_ptr<::kinematics::KinematicChain> chain_;
  std::vector<std::string> chain_joint_names_;
  std::vector<std::string> controlled_joint_names_;
  std::shared_ptr<GNGType> gng_;
  PlannerType planner_;

  std::string target_topic_;
  std::string control_claim_topic_;
  int control_claim_priority_ = 10;
  int control_claim_mode_ = gng_control_msgs::msg::JointControlClaim::MODE_EXCLUSIVE;
  bool control_claim_enabled_ = true;
  std::string trajectory_topic_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_pub_;
  rclcpp::Publisher<gng_control_msgs::msg::JointControlClaim>::SharedPtr control_claim_pub_;
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
  bool avoid_danger_ = true;
  bool trajectory_update_requested_ = true;
  bool active_trajectory_valid_ = false;
  std::vector<int> active_node_path_;
  std::size_t active_waypoint_index_ = 0;
  int active_goal_id_ = -1;
  double waypoint_tolerance_ = 0.05;
  bool replan_on_path_collision_ = true;
  int goal_switch_failure_threshold_ = 3;
  int goal_replan_failures_ = 0;
  bool force_goal_switch_ = false;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_update_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
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

  bool isWaypointReachedLocked(const Eigen::VectorXf & current_q, int waypoint_node_id) const
  {
    if (!gng_ || waypoint_node_id < 0 ||
        waypoint_node_id >= static_cast<int>(gng_->getMaxNodeNum())) {
      return false;
    }
    const auto & waypoint = gng_->nodeAt(waypoint_node_id);
    if (waypoint.id == -1) {
      return false;
    }
    const int dim = std::min(static_cast<int>(waypoint.weight_angle.size()),
                             static_cast<int>(current_q.size()));
    if (dim <= 0) {
      return false;
    }
    const float d = (waypoint.weight_angle.head(dim) - current_q.head(dim)).norm();
    return d <= static_cast<float>(waypoint_tolerance_);
  }

  void clearActiveTrajectoryLocked()
  {
    active_trajectory_valid_ = false;
    active_node_path_.clear();
    active_waypoint_index_ = 0;
    active_goal_id_ = -1;
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
