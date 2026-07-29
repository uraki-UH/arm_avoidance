#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <array>
#include <fstream>
#include <limits>
#include <filesystem>
#include <iterator>
#include <mutex>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

#include <gng_control_msgs/msg/joint_control_claim.hpp>
#include <gng_control_msgs/msg/grasp_candidate_metric.hpp>
#include <gng_control_msgs/msg/grasp_candidate_metric_array.hpp>
// Candidate goal ids from the target-pose selector.
#include <std_msgs/msg/int32_multi_array.hpp>

#include "common/resource_utils.hpp"
#include "common/trajectory.hpp"
#include "planner/RRT/ik_rrt_planner.hpp"
#include "planner/RRT/rrt_params.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include "planning/gng_dijkstra_planner.hpp"
#include "planning/robot_stream_payload.hpp"
#include "planning/topological_map_avoidance_helpers.hpp"
#include "planning/joint_linf_cost.hpp"
#include "core/common/evaluation_metric_serialization.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"
#include "core/common/manipulability_serialization.hpp"
#include "core/metrics/manipulability.hpp"
#include "gng/GrowingNeuralGas.hpp"

namespace {

using robot_sim::common::TrajectoryState;

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
    declare_parameter("urdf_path", "");
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
    declare_parameter("candidate_trajectory_topic", "/ToPoDualArm/candidate_topological_map");
    declare_parameter("candidate_metrics_topic", "/ToPoDualArm/grasp_candidate_metrics");
    declare_parameter("evaluation_metrics_topic", "/evaluation_metrics");
    declare_parameter("current_ee_pose_topic", "");
    declare_parameter("goal_candidate_ids_topic", "/selected_goal_candidate_ids");
    declare_parameter("publish_hz", 20.0);
    declare_parameter("avoid_collisions", true);
    declare_parameter("avoid_danger", true);
    declare_parameter("allow_danger_goal", true);
    declare_parameter("strict_goal_collision_check", false);
    declare_parameter("replan_on_path_collision", true);
    declare_parameter("allow_zero_initial_joint_state", true);
    declare_parameter("publish_target_joint_states", true);
    declare_parameter("allow_safe_goal_fallback", true);
    declare_parameter("trial_mode", false);
    declare_parameter("trial_goal_interval_sec", 4.0);
    declare_parameter("trial_safe_only", true);
    declare_parameter("trial_return_home", false);
    declare_parameter("trial_auto_advance_goal", false);
    declare_parameter("trial_goal_candidate_count", 10);
    declare_parameter("trial_seed", 0);
    declare_parameter("waypoint_tolerance", 0.05);
    declare_parameter("robot_base_frame", "");
    declare_parameter("publish_candidate_robot_preview", true);
    declare_parameter("metrics_max_joint_velocity", 0.6);
    // ゴール姿勢スコアリング
    // score = ホップ数 + 0.5*関節距離
    //       + goal_rot_manip_weight  * log(回転可操作性 条件数)  ← 手首ねじれ抑制
    //       - goal_joint_limit_weight * 関節限界余裕[0,1]         ← 関節限界回避
    // 0 に設定すると無効化（旧動作）
    declare_parameter("goal_rot_manip_weight", 1.0);   // 条件数 log スケール; 1.0=約3〜5ホップ相当のペナルティ
    declare_parameter("goal_joint_limit_weight", 0.5); // 余裕[0,1] への重み; 0.5=最大0.5ホップ相当のボーナス

    const std::string urdf_rel = get_parameter("urdf_path").as_string();
    const std::string urdf_path = robot_sim::common::resolvePath(urdf_rel);
    if (urdf_path.empty()) {
      throw std::runtime_error("Failed to resolve robot URDF path.");
    }

    auto model = std::make_shared<::simulation::RobotModel>(
        ::simulation::loadRobotFromUrdf(urdf_path));
    robot_root_link_name_ = model->getRootLinkName();

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

    auto resolveNamespacedFrame = [this](const std::string &link_name) {
      std::string ns_raw = std::string(get_namespace());
      if (!ns_raw.empty() && ns_raw.front() == '/') {
        ns_raw.erase(ns_raw.begin());
      }
      if (ns_raw.empty()) {
        return link_name;
      }
      if (link_name.empty()) {
        return ns_raw;
      }
      return ns_raw + "/" + link_name;
    };

    if (robot_base_frame_.empty()) {
      robot_base_frame_ = resolveNamespacedFrame(robot_root_link_name_);
    }
    if (robot_base_frame_.empty()) {
      robot_base_frame_ = "base_link";
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
    allow_danger_goal_ = get_parameter("allow_danger_goal").as_bool();
    planner_.setAvoidDanger(avoid_danger_);
    planner_.setStrictGoalCollisionCheck(
        get_parameter("strict_goal_collision_check").as_bool());
    replan_on_path_collision_ = get_parameter("replan_on_path_collision").as_bool();
    allow_zero_initial_joint_state_ =
        get_parameter("allow_zero_initial_joint_state").as_bool();
    allow_safe_goal_fallback_ = get_parameter("allow_safe_goal_fallback").as_bool();

    trial_mode_ = get_parameter("trial_mode").as_bool();
    trial_goal_interval_sec_ = std::max(0.1, get_parameter("trial_goal_interval_sec").as_double());
    trial_safe_only_ = get_parameter("trial_safe_only").as_bool();
    trial_return_home_ = get_parameter("trial_return_home").as_bool();
    trial_auto_advance_goal_ = get_parameter("trial_auto_advance_goal").as_bool();
    const int trial_goal_candidate_count =
        get_parameter("trial_goal_candidate_count").as_int();
    trial_goal_candidate_count_ =
        (trial_goal_candidate_count < 1) ? 1 : trial_goal_candidate_count;
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
    goal_candidate_ids_topic_ = get_parameter("goal_candidate_ids_topic").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    if (robot_base_frame_.empty()) {
      const std::string ns_raw = std::string(get_namespace());
      const std::string ns = ns_raw.empty() ? "" : (ns_raw.front() == '/' ? ns_raw.substr(1) : ns_raw);
      const std::string root_link = robot_root_link_name_.empty() ? std::string("base_link") : robot_root_link_name_;
      robot_base_frame_ = ns.empty() ? root_link : (root_link.empty() ? ns : ns + "/" + root_link);
    }
    publish_candidate_robot_preview_ = get_parameter("publish_candidate_robot_preview").as_bool();
    metrics_max_joint_velocity_ =
        std::max(1e-6, get_parameter("metrics_max_joint_velocity").as_double());
    goal_rot_manip_weight_ =
        static_cast<float>(std::max(0.0, get_parameter("goal_rot_manip_weight").as_double()));
    goal_joint_limit_weight_ =
        static_cast<float>(std::max(0.0, get_parameter("goal_joint_limit_weight").as_double()));
    trajectory_topic_ = get_parameter("trajectory_topic").as_string();
    candidate_trajectory_topic_ = get_parameter("candidate_trajectory_topic").as_string();
    candidate_metrics_topic_ = get_parameter("candidate_metrics_topic").as_string();
    evaluation_metrics_topic_ = get_parameter("evaluation_metrics_topic").as_string();
    current_ee_pose_topic_ = get_parameter("current_ee_pose_topic").as_string();
    if (current_ee_pose_topic_.empty()) {
      const std::string ns_raw = std::string(get_namespace());
      const std::string ns = ns_raw.empty() ? "" : (ns_raw.front() == '/' ? ns_raw.substr(1) : ns_raw);
      current_ee_pose_topic_ = "/" + ns + "/current_ee_pose";
    }
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
    const bool publish_target_joint_states =
        get_parameter("publish_target_joint_states").as_bool();

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

    if (!goal_candidate_ids_topic_.empty()) {
      goal_candidate_ids_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        goal_candidate_ids_topic_, rclcpp::QoS(1).reliable().transient_local(),
          [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_goal_candidate_ids_.clear();
            latest_goal_candidate_ids_.reserve(msg->data.size());
            for (const auto id : msg->data) {
              latest_goal_candidate_ids_.push_back(static_cast<int>(id));
            }
            RCLCPP_INFO(
                get_logger(),
                "Received goal candidate ids: count=%zu first=%d topic=%s",
                latest_goal_candidate_ids_.size(),
                latest_goal_candidate_ids_.empty() ? -1 : latest_goal_candidate_ids_.front(),
                goal_candidate_ids_topic_.c_str());
            if (publish_candidate_robot_preview_) {
              publishCandidateRobotPreviewLocked();
            }
          });
    }

    trajectory_pub_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
        trajectory_topic_, rclcpp::QoS(1).reliable().transient_local());
    candidate_trajectory_pub_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
        candidate_trajectory_topic_, rclcpp::QoS(1).reliable().transient_local());
    candidate_metrics_pub_ =
        create_publisher<gng_control_msgs::msg::GraspCandidateMetricArray>(
            candidate_metrics_topic_, rclcpp::QoS(1).reliable().transient_local());
    evaluation_metrics_pub_ =
        create_publisher<gng_control_msgs::msg::EvaluationMetrics>(
            evaluation_metrics_topic_, rclcpp::QoS(1).reliable().transient_local());
    current_ee_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        current_ee_pose_topic_, rclcpp::QoS(1).reliable().transient_local());
    candidate_robot_description_pub_ = create_publisher<std_msgs::msg::String>(
        "/viewer/internal/stream/robot/description",
        rclcpp::QoS(1).reliable().transient_local());
    candidate_robot_pose_pub_ = create_publisher<std_msgs::msg::String>(
        "/viewer/internal/stream/robot/pose",
        rclcpp::QoS(1).reliable().transient_local());

    if (!loadRobotDescription(candidate_robot_urdf_content_, urdf_path)) {
      RCLCPP_WARN(
          get_logger(),
          "Failed to load robot description text for candidate robot preview: %s",
          urdf_path.c_str());
    }

    if (publish_target_joint_states) {
      target_pub_ = create_publisher<sensor_msgs::msg::JointState>(
          target_topic_, rclcpp::QoS(10).reliable());
    }

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
          trajectory_.update_requested = true;
          response->success = true;
          response->message = "trajectory update requested";
        });

    trial_goal_advance_srv_ = create_service<std_srvs::srv::Trigger>(
        "request_trial_goal_advance",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(mutex_);
          advanceTrialGoalLocked();
          response->success = true;
          response->message = "trial goal advanced";
        });

    const double publish_hz = std::max(1.0, get_parameter("publish_hz").as_double());
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz)),
        [this]() { this->publishTargetLocked(); });

    RCLCPP_INFO(get_logger(),
                "TopologicalMapAvoidanceNode ready. joint_topic=%s map_topic=%s goal_ids_topic=%s target_topic=%s claim_topic=%s trajectory_topic=%s candidate_trajectory_topic=%s candidate_metrics_topic=%s dof=%d trial_mode=%d",
                joint_topic.c_str(), topological_map_topic.c_str(),
                goal_candidate_ids_topic_.c_str(), target_topic_.c_str(), control_claim_topic_.c_str(), trajectory_topic_.c_str(),
                candidate_trajectory_topic_.c_str(),
                candidate_metrics_topic_.c_str(),
                dof, trial_mode_ ? 1 : 0);
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
    if (!gng_ || (!have_joint_state_ && !trial_mode_ && !allow_zero_initial_joint_state_)) {
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

    Eigen::VectorXf current_q;
    if (!have_joint_state_) {
      current_q = Eigen::VectorXf::Zero(chain_->getTotalDOF());
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No joint_state received yet; using zero current_q fallback until %s receives the first message.",
          get_parameter("joint_topic").as_string().c_str());
    } else {
      current_q = currentJointVectorLocked();
    }

    const int start_id = findNearestActiveNodeLocked(current_q);
    if (start_id < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "No active nearest GNG node found for current joint state.");
      return;
    }

    const auto &start_node = gng_->nodeAt(start_id);
    Eigen::VectorXf target_q = current_q;

    if (trajectory_.valid) {
      const bool local_neighborhood_blocked =
          replan_on_path_collision_ && nodeHasUnsafeNeighborLocked(start_id);
      if (trajectory_.waypoint_index >= trajectory_.node_path.size()) {
        clearActiveTrajectoryLocked();
      } else {
        const bool trajectory_blocked =
            replan_on_path_collision_ &&
            trajectoryHasUnsafeNodeLocked(trajectory_.waypoint_index);
        if (trajectory_blocked || local_neighborhood_blocked) {
          RCLCPP_INFO(
              get_logger(),
              "Trajectory blocked: start=%d goal=%d wp_idx=%zu path_len=%zu local_block=%d. Requesting immediate replan to the same goal.",
              start_id, trajectory_.goal_id, trajectory_.waypoint_index,
              trajectory_.node_path.size(), local_neighborhood_blocked ? 1 : 0);
          if (trial_mode_) {
            requestReplanCurrentTrialGoalLocked();
          } else {
            requestReplanSameGoalLocked();
          }
          target_q = current_q;
        } else {
          const int waypoint_node_id =
              trajectory_.node_path[trajectory_.waypoint_index];
          if (isWaypointReachedLocked(current_q, waypoint_node_id)) {
            ++trajectory_.waypoint_index;
            if (trajectory_.waypoint_index >= trajectory_.node_path.size()) {
              RCLCPP_INFO(
                  get_logger(),
                  "Trajectory completed: goal=%d path_len=%zu",
                  trajectory_.goal_id, trajectory_.node_path.size());
              if (trial_mode_) {
                handleTrialGoalCompletionLocked(current_q);
              } else {
                clearActiveTrajectoryLocked();
                trajectory_.update_requested = true;
              }
            }
          }
        }
      }
    }

    if (trajectory_.bridge_valid) {
      if (trajectory_.bridge_index >= trajectory_.bridge_path.size()) {
        trajectory_.bridge_valid = false;
        trajectory_.bridge_path.clear();
        trajectory_.bridge_index = 0;
      } else {
        target_q = trajectory_.bridge_path[trajectory_.bridge_index];
        if (postureReachedLocked(current_q, target_q)) {
          ++trajectory_.bridge_index;
          if (trajectory_.bridge_index >= trajectory_.bridge_path.size()) {
            trajectory_.bridge_valid = false;
            trajectory_.bridge_path.clear();
            trajectory_.bridge_index = 0;
          }
        }
      }
    }

    if (trial_mode_ && trial_waiting_for_key_) {
      if (trial_hold_target_valid_) {
        target_q = trial_hold_target_q_;
      }
    } else if (trajectory_.update_requested && !trajectory_.valid) {
      if (trial_mode_) {
        if (!trial_goal_coord_valid_ && !selectTrialGoalCoordLocked(start_id)) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Trial mode: failed to select a goal coordinate.");
        }

        if (trial_goal_coord_valid_) {
          trajectory_.goal_candidates = collectNearestGoalCandidatesLocked(
              trial_goal_coord_, trial_goal_candidate_count_);
          if (trajectory_.goal_candidates.empty()) {
            const int fallback_goal_id = topological_map_avoidance::pickRandomGoal(
                gng_, cached_safe_goal_ids_, trial_safe_only_, start_id, rng_);
            if (fallback_goal_id >= 0) {
              trajectory_.goal_candidates.push_back(fallback_goal_id);
            }
          }

          if (!latchTrajectoryFromCandidatesLocked(
                  current_q, start_id, trajectory_.goal_candidates, true,
                  "Trial", "Trial mode")) {
            trajectory_.goal_id = -1;
            trajectory_.goal_candidates.clear();
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Trial mode: planner returned empty path start=%d. Holding the same goal and retrying.",
                start_id);
          }
        } else {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Trial mode: no selectable goal found (safe_only=%d safe_count=%zu).",
              trial_safe_only_ ? 1 : 0, cached_safe_goal_ids_.size());
        }
      } else {
        const auto goal_candidates = selectedGoalCandidatesLocked(start_id);
        if (!goal_candidates.empty()) {
          if (!latchTrajectoryFromCandidatesLocked(
                  current_q, start_id, goal_candidates, false,
                  "GoalPlanning", "GoalPlanning")) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "GoalPlanning: planner returned empty path start=%d goal_candidates=%zu",
                start_id, goal_candidates.size());
          }
        } else if (allow_safe_goal_fallback_ &&
                   (start_node.status.is_colliding ||
                    (avoid_danger_ && start_node.status.is_danger))) {
          if (!cached_safe_goal_ids_.empty()) {
            if (!latchTrajectoryFromCandidatesLocked(
                    current_q, start_id, cached_safe_goal_ids_, false,
                    "Avoidance", "Avoidance mode")) {
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
    }

    if (trajectory_.valid) {
      advanceLatchedTrajectoryLocked(current_q, start_id, target_q);
    } else if (!trial_mode_ &&
               !(start_node.status.is_colliding ||
                 (avoid_danger_ && start_node.status.is_danger))) {
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
    if (target_pub_) {
      target_pub_->publish(out);
    }
    publishCurrentEefPoseLocked(current_q);

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
  bool allow_zero_initial_joint_state_ = true;
  bool allow_safe_goal_fallback_ = true;
  std::string trajectory_topic_;
  std::string candidate_trajectory_topic_;
  std::string candidate_metrics_topic_;
  std::string evaluation_metrics_topic_;
  std::string current_ee_pose_topic_;
  std::string goal_candidate_ids_topic_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr goal_candidate_ids_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_pub_;
  rclcpp::Publisher<gng_control_msgs::msg::JointControlClaim>::SharedPtr control_claim_pub_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr candidate_trajectory_pub_;
  rclcpp::Publisher<gng_control_msgs::msg::GraspCandidateMetricArray>::SharedPtr candidate_metrics_pub_;
  rclcpp::Publisher<gng_control_msgs::msg::EvaluationMetrics>::SharedPtr evaluation_metrics_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_ee_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_robot_description_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_robot_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  ais_gng_msgs::msg::TopologicalMap latest_map_;
  bool have_joint_state_ = false;
  bool have_map_ = false;
  Eigen::VectorXf last_target_q_;
  Eigen::VectorXf last_candidate_current_q_;
  std::vector<std::vector<int>> last_candidate_paths_;
  bool have_last_candidate_publish_ = false;
  std::vector<int> cached_safe_goal_ids_;
  std::vector<int> latest_goal_candidate_ids_;
  std::unordered_set<std::string> last_candidate_robot_tags_;
  bool trial_mode_ = false;
  double trial_goal_interval_sec_ = 4.0;
  bool trial_safe_only_ = true;
  bool trial_return_home_ = false;
  bool trial_auto_advance_goal_ = false;
  int trial_goal_candidate_count_ = 10;
  bool avoid_danger_ = true;
  bool allow_danger_goal_ = true;
  double waypoint_tolerance_ = 0.05;
  bool replan_on_path_collision_ = true;
  bool publish_candidate_robot_preview_ = true;
  double metrics_max_joint_velocity_ = 0.6;
  float goal_rot_manip_weight_ = 1.0f;
  float goal_joint_limit_weight_ = 0.5f;
  std::string robot_base_frame_ = "base_link";
  std::string robot_root_link_name_;
  std::string candidate_robot_urdf_content_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_update_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trial_goal_advance_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  std::mt19937 rng_;

  TrajectoryState trajectory_;

  enum class TrialPhase { kRandomGoal, kReturnHome };
  TrialPhase trial_phase_ = TrialPhase::kRandomGoal;
  Eigen::Vector3f trial_goal_coord_ = Eigen::Vector3f::Zero();
  bool trial_goal_coord_valid_ = false;
  Eigen::Vector3f trial_home_coord_ = Eigen::Vector3f::Zero();
  bool trial_home_coord_valid_ = false;
  bool trial_waiting_for_key_ = false;
  bool trial_hold_target_valid_ = false;
  Eigen::VectorXf trial_hold_target_q_;

  std::vector<int> collectNearestGoalCandidatesLocked(const Eigen::Vector3f &reference_coord,
                                                      int candidate_count) const {
    return topological_map_avoidance::collectNearestGoalCandidates(
        gng_, cached_safe_goal_ids_, trial_safe_only_, allow_danger_goal_,
        reference_coord, candidate_count);
  }

  std::vector<int> selectedGoalCandidatesLocked(int start_id) const {
    if (latest_goal_candidate_ids_.empty()) {
      if (goal_candidate_ids_topic_.empty()) {
        return cached_safe_goal_ids_;
      }
      return {};
    }
    const std::vector<int> &source = latest_goal_candidate_ids_;
    std::vector<int> out;
    out.reserve(source.size());
    for (int id : source) {
      if (id == start_id) {
        continue;
      }
      if (!gng_ || id < 0 || id >= static_cast<int>(gng_->getMaxNodeNum())) {
        continue;
      }
      const auto &node = gng_->nodeAt(id);
      if (node.id == -1 || !node.status.active || !node.status.valid) {
        continue;
      }
      if (node.status.is_colliding) {
        continue;
      }
      if (avoid_danger_ && node.status.is_danger && !allow_danger_goal_) {
        continue;
      }
      out.push_back(id);
    }
    return out;
  }

  std::vector<int> collectNearestStartCandidatesLocked(const Eigen::VectorXf &reference_q,
                                                       int candidate_count) const {
    return topological_map_avoidance::collectNearestStartCandidates(
        gng_, reference_q, candidate_count);
  }

  bool postureReachedLocked(const Eigen::VectorXf &current_q,
                            const Eigen::VectorXf &target_q) const {
    const int dim = std::min(static_cast<int>(current_q.size()),
                             static_cast<int>(target_q.size()));
    if (dim <= 0) {
      return false;
    }
    const float d = (current_q.head(dim) - target_q.head(dim)).norm();
    return d <= static_cast<float>(waypoint_tolerance_);
  }

  bool buildTrialGoalBridgeLocked(const Eigen::VectorXf &start_q,
                                  const Eigen::Vector3f &goal_coord,
                                  std::vector<Eigen::VectorXf> &out_path) const {
    return topological_map_avoidance::buildTrialGoalBridge(
        chain_, gng_, avoid_danger_, start_q, goal_coord, out_path);
  }

  bool goalCoordinateReachedLocked(const Eigen::VectorXf &current_q) const {
    return topological_map_avoidance::goalCoordinateReached(
        chain_, trial_goal_coord_valid_, trial_goal_coord_, current_q,
        waypoint_tolerance_);
  }

  std::vector<Eigen::VectorXf> buildBridgePathLocked(const Eigen::VectorXf &from_q,
                                                     const Eigen::VectorXf &to_q,
                                                     int steps) const {
    std::vector<Eigen::VectorXf> bridge;
    const int dim = std::min(static_cast<int>(from_q.size()), static_cast<int>(to_q.size()));
    if (dim <= 0) {
      return bridge;
    }

    const int clamped_steps = std::max(1, steps);
    bridge.reserve(static_cast<std::size_t>(clamped_steps));
    for (int i = 1; i <= clamped_steps; ++i) {
      const float t = static_cast<float>(i) / static_cast<float>(clamped_steps);
      Eigen::VectorXf q = from_q.head(dim) * (1.0f - t) + to_q.head(dim) * t;
      bridge.push_back(q);
    }
    return bridge;
  }

  void handleTrialGoalCompletionLocked(const Eigen::VectorXf &current_q) {
    const bool goal_coord_reached = goalCoordinateReachedLocked(current_q);
    if (goal_coord_reached && trial_auto_advance_goal_) {
      trial_waiting_for_key_ = false;
      trial_hold_target_valid_ = false;
      clearActiveTrajectoryLocked();
      trajectory_.goal_id = -1;
      trajectory_.goal_candidates.clear();
      trial_goal_coord_valid_ = false;
      trajectory_.update_requested = true;
      RCLCPP_INFO(
          get_logger(),
          "Trial goal coordinate reached. Auto-advancing to next goal coordinate.");
    } else if (goal_coord_reached) {
      trial_waiting_for_key_ = true;
      trial_hold_target_q_ = gng_->nodeAt(trajectory_.node_path.back()).weight_angle;
      trial_hold_target_valid_ = true;
      clearActiveTrajectoryLocked();
      trajectory_.update_requested = false;
      trajectory_.goal_id = -1;
      trajectory_.goal_candidates.clear();
      RCLCPP_INFO(
          get_logger(),
          "Trial goal coordinate reached. Waiting for manual advance.");
    } else {
      clearActiveTrajectoryLocked();
      trajectory_.goal_id = -1;
      trajectory_.goal_candidates.clear();
      trial_waiting_for_key_ = false;
      trial_hold_target_valid_ = false;
      trajectory_.update_requested = true;
      RCLCPP_INFO(
          get_logger(),
          "Trial path completed before goal coordinate was reached. Replanning the same goal coordinate.");
    }
  }

  std::pair<int, std::vector<int>> planFromStartCandidatesLocked(
      const Eigen::VectorXf &current_q, const std::vector<int> &start_candidates,
      const std::vector<int> &goal_candidates, int &selected_start_id,
      std::unordered_map<int, std::vector<int>> &candidate_path_by_goal,
      std::vector<std::vector<int>> &candidate_paths) {
    return topological_map_avoidance::planFromStartCandidates(
        gng_, planner_, current_q, start_candidates, goal_candidates,
        selected_start_id, candidate_path_by_goal, candidate_paths,
        allow_danger_goal_,
        goal_rot_manip_weight_, goal_joint_limit_weight_);
  }

  bool latchTrajectoryFromCandidatesLocked(
      const Eigen::VectorXf &current_q, int start_id,
      const std::vector<int> &goal_candidates, bool build_goal_bridge,
      const char *latched_label, const char *empty_label) {
    if (!gng_ || goal_candidates.empty()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "%s: no goal candidates available.", empty_label);
      return false;
    }

    const auto start_candidates = collectNearestStartCandidatesLocked(current_q, 5);
    std::vector<int> start_candidates_or_fallback = start_candidates;
    if (start_candidates_or_fallback.empty()) {
      start_candidates_or_fallback.push_back(start_id);
    }

    std::unordered_map<int, std::vector<int>> candidate_path_by_goal;
    std::vector<std::vector<int>> candidate_paths;
    candidate_paths.reserve(goal_candidates.size());

    int selected_start_id = -1;
    auto [reached_goal_id, node_path] = planFromStartCandidatesLocked(
        current_q, start_candidates_or_fallback, goal_candidates,
        selected_start_id, candidate_path_by_goal, candidate_paths);

    trajectory_.goal_id = reached_goal_id;
    publishGraspCandidateMetricsLocked(
        current_q, selected_start_id >= 0 ? selected_start_id : start_id,
        goal_candidates, candidate_path_by_goal);
    trajectory_.bridge_valid = false;
    trajectory_.bridge_path.clear();
    trajectory_.bridge_index = 0;
    trajectory_.goal_bridge_valid = false;
    trajectory_.goal_bridge_path.clear();
    trajectory_.goal_bridge_index = 0;

    if (trajectory_.goal_id < 0) {
      trajectory_.node_path.clear();
      trajectory_.update_requested = false;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "%s: planner returned empty path start=%d goals=%zu",
          empty_label, start_id, goal_candidates.size());
      return false;
    }

    const auto it = candidate_path_by_goal.find(trajectory_.goal_id);
    if (it != candidate_path_by_goal.end()) {
      trajectory_.node_path = it->second;
    } else {
      trajectory_.node_path = node_path;
    }

    if (candidate_trajectory_pub_ && !candidate_paths.empty()) {
      publishCandidateTrajectoryPathsLocked(current_q, candidate_paths);
    }

    if (build_goal_bridge && trial_goal_coord_valid_ && !trajectory_.node_path.empty()) {
      const auto &goal_start_node = gng_->nodeAt(trajectory_.node_path.back());
      trajectory_.goal_bridge_valid = buildTrialGoalBridgeLocked(
          goal_start_node.weight_angle, trial_goal_coord_,
          trajectory_.goal_bridge_path);
      if (trajectory_.goal_bridge_valid) {
        RCLCPP_INFO(
            get_logger(),
            "%s: goal bridge latched goal_node=%d bridge_len=%zu goal_coord=[%.4f %.4f %.4f]",
            latched_label, trajectory_.node_path.back(), trajectory_.goal_bridge_path.size(),
            trial_goal_coord_.x(), trial_goal_coord_.y(), trial_goal_coord_.z());
      } else {
        RCLCPP_WARN(
            get_logger(),
            "%s: goal bridge build failed goal_node=%d goal_coord=[%.4f %.4f %.4f]",
            latched_label, trajectory_.node_path.back(), trial_goal_coord_.x(),
            trial_goal_coord_.y(), trial_goal_coord_.z());
      }
    }

    if (selected_start_id >= 0 &&
        selected_start_id < static_cast<int>(gng_->getMaxNodeNum())) {
      const auto &bridge_start_q = gng_->nodeAt(selected_start_id).weight_angle;
      trajectory_.bridge_path = buildBridgePathLocked(current_q, bridge_start_q, 4);
      trajectory_.bridge_valid = !trajectory_.bridge_path.empty();
    }

    trajectory_.waypoint_index = trajectory_.node_path.size() >= 2 ? 1U : 0U;
    trajectory_.valid = !trajectory_.node_path.empty();
    trajectory_.update_requested = !trajectory_.valid;
    if (!trajectory_.valid) {
      return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "%s: path latched start=%d goal=%d len=%zu candidate_count=%zu",
        latched_label, start_id, trajectory_.goal_id, trajectory_.node_path.size(),
        goal_candidates.size());
    return true;
  }

  void logWaypointEefDebugLocked(
      const Eigen::VectorXf &current_q, const Eigen::VectorXf &target_q,
      int target_node_id) {
    const auto target_node_fk_q = eigenToStdVector(target_q);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        target_positions;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
        target_orientations;
    chain_->forwardKinematicsAt(target_node_fk_q, target_positions,
                                target_orientations);

    const auto current_fk_q = eigenToStdVector(current_q);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        current_positions;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
        current_orientations;
    chain_->forwardKinematicsAt(current_fk_q, current_positions,
                                current_orientations);

    if (target_positions.empty() || target_orientations.empty() ||
        current_positions.empty() || current_orientations.empty()) {
      return;
    }

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

  bool advanceLatchedTrajectoryLocked(const Eigen::VectorXf &current_q,
                                      int start_id,
                                      Eigen::VectorXf &target_q) {
    if (!trajectory_.valid) {
      return false;
    }

    publishTrajectoryPathLocked(trajectory_.node_path);
    if (trajectory_.waypoint_index < trajectory_.node_path.size()) {
      const int target_node_id = trajectory_.node_path[trajectory_.waypoint_index];
      if (target_node_id >= 0 &&
          target_node_id < static_cast<int>(gng_->getMaxNodeNum())) {
        const auto safety = topological_map_avoidance::inspectWaypointSafetyLookahead(
            gng_, trajectory_.node_path, trajectory_.waypoint_index, avoid_danger_);

        if (safety.next_next_unsafe) {
          target_q = current_q;
          if (trial_mode_) {
            requestReplanCurrentTrialGoalLocked();
          } else {
            requestReplanSameGoalLocked();
          }
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Hold current posture and replan: next-next waypoint is unsafe. start=%d goal=%d wp_idx=%zu next_id=%d next_next_id=%d",
              start_id, trajectory_.goal_id, trajectory_.waypoint_index, safety.next_node_id,
              safety.next_next_node_id);
        } else if (safety.next_unsafe) {
          if (trajectory_.waypoint_index > 0) {
            const int retreat_node_id = trajectory_.node_path[trajectory_.waypoint_index - 1];
            if (retreat_node_id >= 0 &&
                retreat_node_id < static_cast<int>(gng_->getMaxNodeNum())) {
              target_q = gng_->nodeAt(retreat_node_id).weight_angle;
              trajectory_.waypoint_index -= 1;
              if (trial_mode_) {
                requestReplanCurrentTrialGoalLocked();
              } else {
                requestReplanSameGoalLocked();
              }
              RCLCPP_INFO_THROTTLE(
                  get_logger(), *get_clock(), 2000,
                  "Retreat to previous waypoint and replan: next waypoint is unsafe. start=%d goal=%d retreat_id=%d wp_idx=%zu next_id=%d",
                  start_id, trajectory_.goal_id, retreat_node_id,
                  trajectory_.waypoint_index, safety.next_node_id);
            } else {
              target_q = current_q;
              if (trial_mode_) {
                requestReplanCurrentTrialGoalLocked();
              } else {
                requestReplanSameGoalLocked();
              }
            }
          } else {
            target_q = current_q;
            if (trial_mode_) {
              requestReplanCurrentTrialGoalLocked();
            } else {
              requestReplanSameGoalLocked();
            }
          }
        } else {
          target_q = gng_->nodeAt(target_node_id).weight_angle;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Waypoint debug: start=%d goal=%d wp_idx=%zu wp_id=%d q_dim=%d target_dim=%d",
            start_id, trajectory_.goal_id, trajectory_.waypoint_index, target_node_id,
            static_cast<int>(current_q.size()), static_cast<int>(target_q.size()));

        logWaypointEefDebugLocked(current_q, target_q, target_node_id);
      }
      return true;
    }

    if (trial_mode_ && trajectory_.goal_bridge_valid &&
        trajectory_.goal_bridge_index < trajectory_.goal_bridge_path.size()) {
      const Eigen::VectorXf &bridge_target_q =
          trajectory_.goal_bridge_path[trajectory_.goal_bridge_index];
      target_q = bridge_target_q;

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Goal bridge debug: start=%d goal=%d bridge_idx=%zu bridge_len=%zu q_dim=%d target_dim=%d",
          start_id, trajectory_.goal_id, trajectory_.goal_bridge_index,
          trajectory_.goal_bridge_path.size(), static_cast<int>(current_q.size()),
          static_cast<int>(target_q.size()));

      if (postureReachedLocked(current_q, bridge_target_q)) {
        ++trajectory_.goal_bridge_index;
        if (trajectory_.goal_bridge_index >= trajectory_.goal_bridge_path.size()) {
          RCLCPP_INFO(
              get_logger(),
              "Trial goal bridge completed. Evaluating goal coordinate reach.");
          trajectory_.goal_bridge_valid = false;
          trajectory_.goal_bridge_path.clear();
          trajectory_.goal_bridge_index = 0;
          handleTrialGoalCompletionLocked(current_q);
        }
      }
      return true;
    }

    RCLCPP_INFO(
        get_logger(),
        "Trajectory completed: goal=%d path_len=%zu", trajectory_.goal_id,
        trajectory_.node_path.size());
    if (trial_mode_) {
      handleTrialGoalCompletionLocked(current_q);
    } else {
      clearActiveTrajectoryLocked();
      trajectory_.update_requested = true;
    }
    return true;
  }

  bool selectTrialGoalCoordLocked(int start_id)
  {
    const auto selected_coord = topological_map_avoidance::selectTrialGoalCoord(
        gng_, trial_return_home_, trial_phase_ == TrialPhase::kReturnHome,
        trial_safe_only_, start_id, cached_safe_goal_ids_, trial_home_coord_,
        trial_home_coord_valid_, rng_);
    if (!selected_coord) {
      return false;
    }
    trial_goal_coord_ = *selected_coord;
    trial_goal_coord_valid_ = true;
    return true;
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

  bool trajectoryHasUnsafeNodeLocked(std::size_t from_index) const {
    if (!gng_) {
      return false;
    }

    if (from_index >= trajectory_.node_path.size()) {
      return false;
    }

    for (std::size_t i = from_index; i < trajectory_.node_path.size(); ++i) {
      const int node_id = trajectory_.node_path[i];
      if (node_id < 0 || node_id >= static_cast<int>(gng_->getMaxNodeNum())) {
        return true;
      }
      const auto &node = gng_->nodeAt(node_id);
      if (node.id == -1 || !node.status.active || !node.status.valid ||
          node.status.is_colliding ||
          (avoid_danger_ && node.status.is_danger)) {
        return true;
      }
    }
    return false;
  }

  bool nodeHasUnsafeNeighborLocked(int node_id) const {
    if (!gng_ || node_id < 0 ||
        node_id >= static_cast<int>(gng_->getMaxNodeNum())) {
      return false;
    }
    const auto &node = gng_->nodeAt(node_id);
    if (node.id == -1 || !node.status.active || !node.status.valid) {
      return true;
    }
    for (int neighbor_id : gng_->getNeighborsAngle(node_id)) {
      if (neighbor_id < 0 || neighbor_id >= static_cast<int>(gng_->getMaxNodeNum())) {
        return true;
      }
      const auto &neighbor = gng_->nodeAt(neighbor_id);
      if (neighbor.id == -1 || !neighbor.status.active || !neighbor.status.valid ||
          neighbor.status.is_colliding ||
          (avoid_danger_ && neighbor.status.is_danger)) {
        return true;
      }
    }
    return false;
  }

  void clearActiveTrajectoryLocked(bool keep_goal_id = false)
  {
    trajectory_.clear(keep_goal_id);
  }

  void publishEmptyCandidateTrajectoryLocked()
  {
    if (candidate_trajectory_pub_) {
      publishCandidateTrajectoryPathsLocked({});
    }
  }

  void requestReplanLocked()
  {
    clearActiveTrajectoryLocked(false);
    trajectory_.goal_candidates.clear();
    trajectory_.goal_id = -1;
    trajectory_.update_requested = true;
    publishEmptyCandidateTrajectoryLocked();
  }

  void requestReplanCurrentTrialGoalLocked()
  {
    clearActiveTrajectoryLocked(true);
    trajectory_.update_requested = true;
    publishEmptyCandidateTrajectoryLocked();
  }

  void requestReplanSameGoalLocked()
  {
    clearActiveTrajectoryLocked(true);
    trajectory_.update_requested = true;
    publishEmptyCandidateTrajectoryLocked();
  }

  void advanceTrialGoalLocked()
  {
    trial_waiting_for_key_ = false;
    trial_hold_target_valid_ = false;
    clearActiveTrajectoryLocked(false);
    trajectory_.goal_candidates.clear();
    trajectory_.goal_id = -1;
    trial_goal_coord_valid_ = false;
    trajectory_.update_requested = true;

    if (trial_return_home_) {
      trial_phase_ = (trial_phase_ == TrialPhase::kRandomGoal)
                         ? TrialPhase::kReturnHome
                         : TrialPhase::kRandomGoal;
      RCLCPP_INFO(
          get_logger(),
          "Trial goal coordinate advance requested: next phase=%s",
          trial_phase_ == TrialPhase::kReturnHome ? "return_home" : "random_goal");
    } else {
      RCLCPP_INFO(
          get_logger(),
          "Trial goal coordinate advance requested: random goal will be reselected.");
    }

    publishEmptyCandidateTrajectoryLocked();
  }

  void publishGraspCandidateMetricsLocked(
      const Eigen::VectorXf &current_q, int start_id,
      const std::vector<int> &goal_candidates,
      const std::unordered_map<int, std::vector<int>> &candidate_path_by_goal) {
    if (!candidate_metrics_pub_ || !gng_) {
      return;
    }

    const std::string frame_id =
        have_map_ && !latest_map_.header.frame_id.empty()
            ? latest_map_.header.frame_id
            : robot_base_frame_;

    std::string ns_raw = std::string(get_namespace());
    if (!ns_raw.empty() && ns_raw.front() == '/') {
      ns_raw.erase(ns_raw.begin());
    }
    const auto out = topological_map_avoidance::buildGraspCandidateMetricArray(
        now(), frame_id, ns_raw, robot_base_frame_, trajectory_.goal_id, current_q,
        start_id, goal_candidates, candidate_path_by_goal, gng_, chain_,
        controlled_joint_names_, metrics_max_joint_velocity_);
    candidate_metrics_pub_->publish(out);

    if (evaluation_metrics_pub_ && !evaluation_metrics_topic_.empty()) {
      const std::string profile_name = get_parameter("gng.profile_names").as_string();
      constexpr const char *kSchemaId = "grasp_candidate_metrics";
      constexpr uint32_t kSchemaRevision = 4;
      const auto stamp = now();
      const auto metrics = robot_sim::common::buildCandidateEvaluationMetrics(
          stamp, profile_name, "candidate", candidate_metrics_topic_, out,
          kSchemaId, kSchemaRevision);
      evaluation_metrics_pub_->publish(metrics);
    }
  }

  void publishTrajectoryPathLocked(const std::vector<int> &node_path) {
    if (!trajectory_pub_ || !gng_) {
      return;
    }
    trajectory_pub_->publish(
        topological_map_avoidance::buildPathMessage(
            *this, gng_, {node_path},
            have_map_ ? latest_map_.header.frame_id : std::string{}));
  }

  void publishCandidateTrajectoryPathsLocked(
      const std::vector<std::vector<int>> &candidate_paths) {
    if (!candidate_trajectory_pub_ || !gng_) {
      return;
    }
    if (have_last_candidate_publish_ && last_candidate_current_q_.size() == 0 &&
        last_candidate_paths_ == candidate_paths) {
      return;
    }
    last_candidate_current_q_.resize(0);
    last_candidate_paths_ = candidate_paths;
    have_last_candidate_publish_ = true;
    candidate_trajectory_pub_->publish(
        topological_map_avoidance::buildPathMessage(
            *this, gng_, candidate_paths,
            have_map_ ? latest_map_.header.frame_id : std::string{}));
  }

  void publishCandidateTrajectoryPathsLocked(
      const Eigen::VectorXf &current_q,
      const std::vector<std::vector<int>> &candidate_paths) {
    if (!candidate_trajectory_pub_ || !gng_) {
      return;
    }
    if (have_last_candidate_publish_ &&
        last_candidate_current_q_.size() == current_q.size() &&
        last_candidate_paths_ == candidate_paths &&
        (current_q.size() == 0 ||
         std::equal(last_candidate_current_q_.data(),
                    last_candidate_current_q_.data() + last_candidate_current_q_.size(),
                    current_q.data()))) {
      return;
    }
    last_candidate_current_q_ = current_q;
    last_candidate_paths_ = candidate_paths;
    have_last_candidate_publish_ = true;
    candidate_trajectory_pub_->publish(
        topological_map_avoidance::buildPathMessageWithCurrentPose(
            *this, gng_, current_q, chain_, candidate_paths,
            have_map_ ? latest_map_.header.frame_id : std::string{}));
  }

  void publishCurrentEefPoseLocked(const Eigen::VectorXf &current_q) {
    if (!current_ee_pose_pub_ || !chain_) {
      return;
    }

    const auto current_fk_q = eigenToStdVector(current_q);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
    chain_->forwardKinematicsAt(current_fk_q, positions, orientations);
    if (positions.empty() || orientations.empty()) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = robot_base_frame_;
    pose_msg.pose.position.x = positions.back().x();
    pose_msg.pose.position.y = positions.back().y();
    pose_msg.pose.position.z = positions.back().z();
    pose_msg.pose.orientation.x = orientations.back().x();
    pose_msg.pose.orientation.y = orientations.back().y();
    pose_msg.pose.orientation.z = orientations.back().z();
    pose_msg.pose.orientation.w = orientations.back().w();
    current_ee_pose_pub_->publish(pose_msg);
  }

  void publishCandidateRobotPreviewLocked() {
    if (!publish_candidate_robot_preview_ || !gng_ ||
        !candidate_robot_description_pub_ || !candidate_robot_pose_pub_ ||
        candidate_robot_urdf_content_.empty()) {
      return;
    }

    std::unordered_set<std::string> next_tags;
    const auto preview_payload = buildCandidateRobotPreviewPayload(
        std::string(get_namespace()), robot_base_frame_,
        candidate_robot_urdf_content_, controlled_joint_names_,
        latest_goal_candidate_ids_, gng_, chain_, this->now().seconds());

    if (!preview_payload) {
      for (const auto &old_tag : last_candidate_robot_tags_) {
        std_msgs::msg::String delete_msg;
        delete_msg.data = nlohmann::json({
            {"type", "stream.robot.delete"},
            {"tag", old_tag},
        }).dump();
        candidate_robot_pose_pub_->publish(delete_msg);
      }
      last_candidate_robot_tags_.clear();
      return;
    }

    next_tags.insert(preview_payload->tag);

    std_msgs::msg::String desc_msg;
    desc_msg.data = preview_payload->description_json;
    candidate_robot_description_pub_->publish(desc_msg);

    std_msgs::msg::String pose_msg;
    pose_msg.data = preview_payload->pose_json;
    candidate_robot_pose_pub_->publish(pose_msg);

    for (const auto &old_tag : last_candidate_robot_tags_) {
      if (next_tags.find(old_tag) != next_tags.end()) {
        continue;
      }
      std_msgs::msg::String delete_msg;
      delete_msg.data = nlohmann::json({
          {"type", "stream.robot.delete"},
          {"tag", old_tag},
      }).dump();
      candidate_robot_pose_pub_->publish(delete_msg);
    }
    last_candidate_robot_tags_ = std::move(next_tags);
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
