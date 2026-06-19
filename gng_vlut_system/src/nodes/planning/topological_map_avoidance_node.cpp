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
#include "planner/RRT/ik_rrt_planner.hpp"
#include "planner/RRT/rrt_params.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include "planning/gng_dijkstra_planner.hpp"
#include "planning/topological_map_avoidance_helpers.hpp"
#include "planning/joint_linf_cost.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"
#include "core/common/manipulability_serialization.hpp"
#include "core/metrics/manipulability.hpp"
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

static std::string detectLocalMeshPackageName(const std::string &source_path) {
  std::filesystem::path current(source_path);
  if (current.empty()) {
    return "";
  }

  current = std::filesystem::absolute(current).parent_path();
  while (!current.empty()) {
    if (std::filesystem::exists(current / "meshes")) {
      return current.filename().string();
    }
    const auto parent = current.parent_path();
    if (parent == current) {
      break;
    }
    current = parent;
  }
  return "";
}

static std::string rewriteRelativeMeshUris(
    const std::string &urdf_text, const std::string &package_name) {
  if (urdf_text.empty() || package_name.empty()) {
    return urdf_text;
  }

  std::string rewritten = urdf_text;
  const std::string prefix = "package://" + package_name + "/";

  const std::string double_quote_key = "filename=\"meshes/";
  std::size_t pos = 0;
  while ((pos = rewritten.find(double_quote_key, pos)) != std::string::npos) {
    rewritten.replace(pos, double_quote_key.size(), "filename=\"" + prefix);
    pos += prefix.size();
  }

  const std::string single_quote_key = "filename='meshes/";
  pos = 0;
  while ((pos = rewritten.find(single_quote_key, pos)) != std::string::npos) {
    rewritten.replace(pos, single_quote_key.size(), "filename='" + prefix);
    pos += prefix.size();
  }

  return rewritten;
}

static bool loadRobotDescription(std::string &out_text,
                                 const std::string &source_path) {
  if (source_path.empty()) {
    return false;
  }

  if (source_path.rfind(".xacro") != std::string::npos) {
    std::array<char, 128> buffer{};
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(
        popen(("xacro " + source_path).c_str(), "r"), pclose);
    if (!pipe) {
      return false;
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
      result += buffer.data();
    }
    out_text = result;
  } else {
    std::ifstream ifs(source_path);
    if (!ifs) {
      return false;
    }
    out_text = std::string((std::istreambuf_iterator<char>(ifs)),
                           std::istreambuf_iterator<char>());
  }
  out_text = rewriteRelativeMeshUris(out_text, detectLocalMeshPackageName(source_path));
  return !out_text.empty();
}

static nlohmann::json buildRobotPayloadJson(
    const std::string &frame_id, const std::string &urdf_content,
    const std::vector<std::string> &joint_names,
    const std::vector<double> &joint_values,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
    const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &orientations,
    double timestamp, double opacity,
    const Eigen::Vector3d &manip_center,
    const Manipulability::ManipulabilityEllipsoid *manip = nullptr,
    bool is_goal = false) {
  nlohmann::json robot;
  robot["timestamp"] = timestamp;
  robot["frameId"] = frame_id;
  robot["urdf"] = urdf_content;
  robot["jointNames"] = joint_names;
  robot["jointValues"] = joint_values;
  robot["opacity"] = opacity;

  auto &pos_arr = robot["positions"] = nlohmann::json::array();
  for (const auto &v : positions) {
    pos_arr.push_back({v.x(), v.y(), v.z()});
  }

  auto &quat_arr = robot["orientations"] = nlohmann::json::array();
  for (const auto &q : orientations) {
    quat_arr.push_back({q.x(), q.y(), q.z(), q.w()});
  }

  if (!positions.empty()) {
    robot["basePosition"] = {positions.front().x(), positions.front().y(), positions.front().z()};
  }
  if (!orientations.empty()) {
    robot["baseOrientation"] = {orientations.front().x(), orientations.front().y(),
                                orientations.front().z(), orientations.front().w()};
  }

  if (manip && manip->valid) {
    Eigen::Quaterniond q(manip->principal_directions);
    q.normalize();
    robot["manipValid"] = true;
    robot["isGoal"] = is_goal;
    robot["manipValue"] = manip->manipulability;
    robot["manipConditionNumber"] = manip->condition_number;
    robot["manipCenter"] = {manip_center.x(), manip_center.y(), manip_center.z()};
    robot["manipScale"] = {manip->singular_values.x(), manip->singular_values.y(), manip->singular_values.z()};
    robot["manipOrientation"] = {q.x(), q.y(), q.z(), q.w()};
  } else {
    robot["manipValid"] = false;
    robot["isGoal"] = is_goal;
    robot["manipValue"] = 0.0;
    robot["manipConditionNumber"] = 0.0;
    robot["manipCenter"] = {0.0, 0.0, 0.0};
    robot["manipScale"] = {0.0, 0.0, 0.0};
    robot["manipOrientation"] = {0.0, 0.0, 0.0, 1.0};
  }

  return robot;
}

static std::string buildRobotStreamJson(
    const std::string &type, const std::string &tag,
    const std::string &frame_id, const std::string &urdf_content,
    const std::vector<std::string> &joint_names,
    const std::vector<double> &joint_values,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
    const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &orientations,
    double timestamp, double opacity) {
  nlohmann::json root;
  root["type"] = type;
  root["tag"] = tag;
  root["robot"] = buildRobotPayloadJson(
      frame_id, urdf_content, joint_names, joint_values, positions, orientations,
      timestamp, opacity,
      positions.empty() ? Eigen::Vector3d::Zero() : positions.back());
  return root.dump();
}

static std::string buildRobotStreamJsonWithInstances(
    const std::string &type, const std::string &tag, const nlohmann::json &robot_payload,
    const nlohmann::json &instances) {
  nlohmann::json root;
  root["type"] = type;
  root["tag"] = tag;
  root["robot"] = robot_payload;
  root["robot"]["instances"] = instances;
  return root.dump();
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
    declare_parameter("urdf_path",
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
    declare_parameter("candidate_trajectory_topic", "/ToPoDualArm/candidate_topological_map");
    declare_parameter("candidate_metrics_topic", "/ToPoDualArm/grasp_candidate_metrics");
    declare_parameter("goal_candidate_ids_topic", "/selected_goal_candidate_ids");
    declare_parameter("publish_hz", 20.0);
    declare_parameter("avoid_collisions", true);
    declare_parameter("avoid_danger", true);
    declare_parameter("strict_goal_collision_check", false);
    declare_parameter("replan_on_path_collision", true);
    declare_parameter("allow_zero_initial_joint_state", true);
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
    declare_parameter("candidate_robot_preview_opacity", 0.18);
    declare_parameter("metrics_max_joint_velocity", 0.6);

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
    planner_.setAvoidDanger(avoid_danger_);
    planner_.setStrictGoalCollisionCheck(
        get_parameter("strict_goal_collision_check").as_bool());
    replan_on_path_collision_ = get_parameter("replan_on_path_collision").as_bool();
    allow_zero_initial_joint_state_ =
        get_parameter("allow_zero_initial_joint_state").as_bool();

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
    candidate_robot_preview_opacity_ = std::clamp(
        get_parameter("candidate_robot_preview_opacity").as_double(), 0.0, 1.0);
    metrics_max_joint_velocity_ =
        std::max(1e-6, get_parameter("metrics_max_joint_velocity").as_double());
    trajectory_topic_ = get_parameter("trajectory_topic").as_string();
    candidate_trajectory_topic_ = get_parameter("candidate_trajectory_topic").as_string();
    candidate_metrics_topic_ = get_parameter("candidate_metrics_topic").as_string();
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

    if (active_trajectory_valid_) {
      const bool local_neighborhood_blocked =
          replan_on_path_collision_ && nodeHasUnsafeNeighborLocked(start_id);
      if (active_waypoint_index_ >= active_node_path_.size()) {
        clearActiveTrajectoryLocked();
      } else {
        const bool trajectory_blocked =
            replan_on_path_collision_ &&
            trajectoryHasUnsafeNodeLocked(active_waypoint_index_);
        if (trajectory_blocked || local_neighborhood_blocked) {
          RCLCPP_INFO(
              get_logger(),
              "Trajectory blocked: start=%d goal=%d wp_idx=%zu path_len=%zu local_block=%d. Requesting immediate replan to the same goal.",
              start_id, active_goal_id_, active_waypoint_index_,
              active_node_path_.size(), local_neighborhood_blocked ? 1 : 0);
          if (trial_mode_) {
            requestReplanCurrentTrialGoalLocked();
          } else {
            requestReplanSameGoalLocked();
          }
          target_q = current_q;
        } else {
        const int waypoint_node_id = active_node_path_[active_waypoint_index_];
        if (isWaypointReachedLocked(current_q, waypoint_node_id)) {
          ++active_waypoint_index_;
          if (active_waypoint_index_ >= active_node_path_.size()) {
            RCLCPP_INFO(
                get_logger(),
                "Trajectory completed: goal=%d path_len=%zu", active_goal_id_,
                active_node_path_.size());
            if (trial_mode_) {
              const bool goal_coord_reached = goalCoordinateReachedLocked(current_q);
              if (goal_coord_reached && trial_auto_advance_goal_) {
                trial_waiting_for_key_ = false;
                trial_hold_target_valid_ = false;
                clearActiveTrajectoryLocked();
                active_goal_id_ = -1;
                active_goal_candidates_.clear();
                trial_goal_coord_valid_ = false;
                trajectory_update_requested_ = true;
                RCLCPP_INFO(
                    get_logger(),
                    "Trial goal coordinate reached. Auto-advancing to next goal coordinate.");
              } else if (goal_coord_reached) {
                trial_waiting_for_key_ = true;
                trial_hold_target_q_ = gng_->nodeAt(active_node_path_.back()).weight_angle;
                trial_hold_target_valid_ = true;
                clearActiveTrajectoryLocked();
                trajectory_update_requested_ = false;
                active_goal_id_ = -1;
                active_goal_candidates_.clear();
                RCLCPP_INFO(
                    get_logger(),
                    "Trial goal coordinate reached. Waiting for manual advance.");
              } else {
                clearActiveTrajectoryLocked();
                active_goal_id_ = -1;
                active_goal_candidates_.clear();
                trial_waiting_for_key_ = false;
                trial_hold_target_valid_ = false;
                trajectory_update_requested_ = true;
                RCLCPP_INFO(
                    get_logger(),
                    "Trial path completed before goal coordinate was reached. Replanning the same goal coordinate.");
              }
            } else {
              clearActiveTrajectoryLocked();
              trajectory_update_requested_ = true;
            }
          }
        }
        }
      }
    }

    if (active_bridge_valid_) {
      if (active_bridge_index_ >= active_bridge_path_.size()) {
        active_bridge_valid_ = false;
        active_bridge_path_.clear();
        active_bridge_index_ = 0;
      } else {
        target_q = active_bridge_path_[active_bridge_index_];
        if (postureReachedLocked(current_q, target_q)) {
          ++active_bridge_index_;
          if (active_bridge_index_ >= active_bridge_path_.size()) {
            active_bridge_valid_ = false;
            active_bridge_path_.clear();
            active_bridge_index_ = 0;
          }
        }
      }
    }

    if (trial_mode_ && trial_waiting_for_key_) {
      if (trial_hold_target_valid_) {
        target_q = trial_hold_target_q_;
      }
    } else if (trajectory_update_requested_ && !active_trajectory_valid_) {
      if (trial_mode_) {
        if (!trial_goal_coord_valid_ && !selectTrialGoalCoordLocked(current_q, start_id)) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Trial mode: failed to select a goal coordinate.");
        }

        if (trial_goal_coord_valid_) {
          active_goal_candidates_ = collectNearestGoalCandidatesLocked(
              trial_goal_coord_, trial_goal_candidate_count_);
          if (active_goal_candidates_.empty()) {
            const int fallback_goal_id = pickRandomGoalLocked(start_id);
            if (fallback_goal_id >= 0) {
              active_goal_candidates_.push_back(fallback_goal_id);
            }
          }

          if (!latchTrajectoryFromCandidatesLocked(
                  current_q, start_id, active_goal_candidates_, true,
                  "Trial", "Trial mode")) {
            active_goal_id_ = -1;
            active_goal_candidates_.clear();
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
        } else if (start_node.status.is_colliding ||
                   (avoid_danger_ && start_node.status.is_danger)) {
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

    if (active_trajectory_valid_) {
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
  bool allow_zero_initial_joint_state_ = true;
  std::string trajectory_topic_;
  std::string candidate_trajectory_topic_;
  std::string candidate_metrics_topic_;
  std::string goal_candidate_ids_topic_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr goal_candidate_ids_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_pub_;
  rclcpp::Publisher<gng_control_msgs::msg::JointControlClaim>::SharedPtr control_claim_pub_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr candidate_trajectory_pub_;
  rclcpp::Publisher<gng_control_msgs::msg::GraspCandidateMetricArray>::SharedPtr candidate_metrics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_robot_description_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_robot_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  ais_gng_msgs::msg::TopologicalMap latest_map_;
  bool have_joint_state_ = false;
  bool have_map_ = false;
  Eigen::VectorXf last_target_q_;
  std::vector<int> cached_safe_goal_ids_;
  std::vector<int> latest_goal_candidate_ids_;
  std::unordered_set<std::string> last_candidate_robot_tags_;
  std::vector<int> active_goal_candidates_;
  bool trial_mode_ = false;
  double trial_goal_interval_sec_ = 4.0;
  bool trial_safe_only_ = true;
  bool trial_return_home_ = false;
  bool trial_auto_advance_goal_ = false;
  int trial_goal_candidate_count_ = 10;
  bool avoid_danger_ = true;
  bool trajectory_update_requested_ = true;
  bool active_trajectory_valid_ = false;
  std::vector<Eigen::VectorXf> active_bridge_path_;
  std::size_t active_bridge_index_ = 0;
  bool active_bridge_valid_ = false;
  std::vector<Eigen::VectorXf> active_goal_bridge_path_;
  std::size_t active_goal_bridge_index_ = 0;
  bool active_goal_bridge_valid_ = false;
  std::vector<int> active_node_path_;
  std::size_t active_waypoint_index_ = 0;
  int active_goal_id_ = -1;
  double waypoint_tolerance_ = 0.05;
  bool replan_on_path_collision_ = true;
  bool publish_candidate_robot_preview_ = true;
  double candidate_robot_preview_opacity_ = 0.18;
  double metrics_max_joint_velocity_ = 0.6;
  std::string robot_base_frame_ = "base_link";
  std::string robot_root_link_name_;
  std::string candidate_robot_urdf_content_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_update_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trial_goal_advance_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  std::mt19937 rng_;

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
        gng_, cached_safe_goal_ids_, trial_safe_only_, reference_coord,
        candidate_count);
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
      if (avoid_danger_ && node.status.is_danger) {
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
    if (!trial_goal_coord_valid_ || !chain_) {
      return false;
    }

    const auto current_fk_q = eigenToStdVector(current_q);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
    chain_->forwardKinematicsAt(current_fk_q, positions, orientations);
    if (positions.empty()) {
      return false;
    }

    const Eigen::Vector3d current_eef_pos = positions.back();
    const Eigen::Vector3d goal_pos = trial_goal_coord_.cast<double>();
    const double pos_err = (current_eef_pos - goal_pos).norm();
    return pos_err <= waypoint_tolerance_;
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
      active_goal_id_ = -1;
      active_goal_candidates_.clear();
      trial_goal_coord_valid_ = false;
      trajectory_update_requested_ = true;
      RCLCPP_INFO(
          get_logger(),
          "Trial goal coordinate reached. Auto-advancing to next goal coordinate.");
    } else if (goal_coord_reached) {
      trial_waiting_for_key_ = true;
      trial_hold_target_q_ = gng_->nodeAt(active_node_path_.back()).weight_angle;
      trial_hold_target_valid_ = true;
      clearActiveTrajectoryLocked();
      trajectory_update_requested_ = false;
      active_goal_id_ = -1;
      active_goal_candidates_.clear();
      RCLCPP_INFO(
          get_logger(),
          "Trial goal coordinate reached. Waiting for manual advance.");
    } else {
      clearActiveTrajectoryLocked();
      active_goal_id_ = -1;
      active_goal_candidates_.clear();
      trial_waiting_for_key_ = false;
      trial_hold_target_valid_ = false;
      trajectory_update_requested_ = true;
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
        selected_start_id, candidate_path_by_goal, candidate_paths);
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

    if (candidate_trajectory_pub_) {
      publishCandidateTrajectoryPathsLocked(candidate_paths);
    }

    active_goal_id_ = reached_goal_id;
    publishGraspCandidateMetricsLocked(
        current_q, selected_start_id >= 0 ? selected_start_id : start_id,
        goal_candidates, candidate_path_by_goal);
    active_bridge_valid_ = false;
    active_bridge_path_.clear();
    active_bridge_index_ = 0;
    active_goal_bridge_valid_ = false;
    active_goal_bridge_path_.clear();
    active_goal_bridge_index_ = 0;

    if (active_goal_id_ < 0) {
      active_node_path_.clear();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "%s: planner returned empty path start=%d goals=%zu",
          empty_label, start_id, goal_candidates.size());
      return false;
    }

    const auto it = candidate_path_by_goal.find(active_goal_id_);
    if (it != candidate_path_by_goal.end()) {
      active_node_path_ = it->second;
    } else {
      active_node_path_ = node_path;
    }

    if (build_goal_bridge && trial_goal_coord_valid_ && !active_node_path_.empty()) {
      const auto &goal_start_node = gng_->nodeAt(active_node_path_.back());
      active_goal_bridge_valid_ = buildTrialGoalBridgeLocked(
          goal_start_node.weight_angle, trial_goal_coord_,
          active_goal_bridge_path_);
      if (active_goal_bridge_valid_) {
        RCLCPP_INFO(
            get_logger(),
            "%s: goal bridge latched goal_node=%d bridge_len=%zu goal_coord=[%.4f %.4f %.4f]",
            latched_label, active_node_path_.back(), active_goal_bridge_path_.size(),
            trial_goal_coord_.x(), trial_goal_coord_.y(), trial_goal_coord_.z());
      } else {
        RCLCPP_WARN(
            get_logger(),
            "%s: goal bridge build failed goal_node=%d goal_coord=[%.4f %.4f %.4f]",
            latched_label, active_node_path_.back(), trial_goal_coord_.x(),
            trial_goal_coord_.y(), trial_goal_coord_.z());
      }
    }

    if (selected_start_id >= 0 &&
        selected_start_id < static_cast<int>(gng_->getMaxNodeNum())) {
      const auto &bridge_start_q = gng_->nodeAt(selected_start_id).weight_angle;
      active_bridge_path_ = buildBridgePathLocked(current_q, bridge_start_q, 4);
      active_bridge_valid_ = !active_bridge_path_.empty();
    }

    active_waypoint_index_ = active_node_path_.size() >= 2 ? 1U : 0U;
    active_trajectory_valid_ = !active_node_path_.empty();
    trajectory_update_requested_ = !active_trajectory_valid_;
    if (!active_trajectory_valid_) {
      return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "%s: path latched start=%d goal=%d len=%zu candidate_count=%zu",
        latched_label, start_id, active_goal_id_, active_node_path_.size(),
        goal_candidates.size());
    return true;
  }

  bool advanceLatchedTrajectoryLocked(const Eigen::VectorXf &current_q,
                                      int start_id,
                                      Eigen::VectorXf &target_q) {
    if (!active_trajectory_valid_) {
      return false;
    }

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
                 node.status.is_colliding ||
                 (avoid_danger_ && node.status.is_danger);
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
          if (trial_mode_) {
            requestReplanCurrentTrialGoalLocked();
          } else {
            requestReplanSameGoalLocked();
          }
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Hold current posture and replan: next-next waypoint is unsafe. start=%d goal=%d wp_idx=%zu next_id=%d next_next_id=%d",
              start_id, active_goal_id_, active_waypoint_index_, next_node_id,
              next_next_node_id);
        } else if (next_unsafe) {
          if (active_waypoint_index_ > 0) {
            const int retreat_node_id = active_node_path_[active_waypoint_index_ - 1];
            if (retreat_node_id >= 0 &&
                retreat_node_id < static_cast<int>(gng_->getMaxNodeNum())) {
              target_q = gng_->nodeAt(retreat_node_id).weight_angle;
              active_waypoint_index_ -= 1;
              if (trial_mode_) {
                requestReplanCurrentTrialGoalLocked();
              } else {
                requestReplanSameGoalLocked();
              }
              RCLCPP_INFO_THROTTLE(
                  get_logger(), *get_clock(), 2000,
                  "Retreat to previous waypoint and replan: next waypoint is unsafe. start=%d goal=%d retreat_id=%d wp_idx=%zu next_id=%d",
                  start_id, active_goal_id_, retreat_node_id,
                  active_waypoint_index_, next_node_id);
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
      return true;
    }

    if (trial_mode_ && active_goal_bridge_valid_ &&
        active_goal_bridge_index_ < active_goal_bridge_path_.size()) {
      const Eigen::VectorXf &bridge_target_q =
          active_goal_bridge_path_[active_goal_bridge_index_];
      target_q = bridge_target_q;

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Goal bridge debug: start=%d goal=%d bridge_idx=%zu bridge_len=%zu q_dim=%d target_dim=%d",
          start_id, active_goal_id_, active_goal_bridge_index_,
          active_goal_bridge_path_.size(), static_cast<int>(current_q.size()),
          static_cast<int>(target_q.size()));

      if (postureReachedLocked(current_q, bridge_target_q)) {
        ++active_goal_bridge_index_;
        if (active_goal_bridge_index_ >= active_goal_bridge_path_.size()) {
          RCLCPP_INFO(
              get_logger(),
              "Trial goal bridge completed. Evaluating goal coordinate reach.");
          active_goal_bridge_valid_ = false;
          active_goal_bridge_path_.clear();
          active_goal_bridge_index_ = 0;
          handleTrialGoalCompletionLocked(current_q);
        }
      }
      return true;
    }

    RCLCPP_INFO(
        get_logger(),
        "Trajectory completed: goal=%d path_len=%zu", active_goal_id_,
        active_node_path_.size());
    if (trial_mode_) {
      const bool goal_coord_reached = goalCoordinateReachedLocked(current_q);
      if (goal_coord_reached && trial_auto_advance_goal_) {
        trial_waiting_for_key_ = false;
        trial_hold_target_valid_ = false;
        clearActiveTrajectoryLocked();
        active_goal_id_ = -1;
        active_goal_candidates_.clear();
        trial_goal_coord_valid_ = false;
        trajectory_update_requested_ = true;
        RCLCPP_INFO(
            get_logger(),
            "Trial goal coordinate reached. Auto-advancing to next goal coordinate.");
      } else if (goal_coord_reached) {
        trial_waiting_for_key_ = true;
        trial_hold_target_q_ = gng_->nodeAt(active_node_path_.back()).weight_angle;
        trial_hold_target_valid_ = true;
        clearActiveTrajectoryLocked();
        trajectory_update_requested_ = false;
        active_goal_id_ = -1;
        active_goal_candidates_.clear();
        RCLCPP_INFO(
            get_logger(),
            "Trial goal coordinate reached. Waiting for manual advance.");
      } else {
        clearActiveTrajectoryLocked();
        active_goal_id_ = -1;
        active_goal_candidates_.clear();
        trial_waiting_for_key_ = false;
        trial_hold_target_valid_ = false;
        trajectory_update_requested_ = true;
        RCLCPP_INFO(
            get_logger(),
            "Trial path completed before goal coordinate was reached. Replanning the same goal coordinate.");
      }
    } else {
      clearActiveTrajectoryLocked();
      trajectory_update_requested_ = true;
    }
    return true;
  }

  bool selectTrialGoalCoordLocked(const Eigen::VectorXf &current_q, int start_id)
  {
    if (!gng_) {
      return false;
    }

    if (trial_return_home_ && trial_phase_ == TrialPhase::kReturnHome) {
      if (!trial_home_coord_valid_) {
        if (start_id < 0 || start_id >= static_cast<int>(gng_->getMaxNodeNum())) {
          return false;
        }
        const auto &start_node = gng_->nodeAt(start_id);
        if (start_node.id == -1) {
          return false;
        }
        trial_home_coord_ = start_node.weight_coord;
        trial_home_coord_valid_ = true;
      }
      trial_goal_coord_ = trial_home_coord_;
      trial_goal_coord_valid_ = true;
      return true;
    }

    (void)current_q;
    const int random_goal_id = pickRandomGoalLocked(start_id);
    if (random_goal_id < 0 || random_goal_id >= static_cast<int>(gng_->getMaxNodeNum())) {
      return false;
    }
    const auto &goal_node = gng_->nodeAt(random_goal_id);
    if (goal_node.id == -1) {
      return false;
    }
    trial_goal_coord_ = goal_node.weight_coord;
    trial_goal_coord_valid_ = true;
    return true;
  }

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

    if (from_index >= active_node_path_.size()) {
      return false;
    }

    for (std::size_t i = from_index; i < active_node_path_.size(); ++i) {
      const int node_id = active_node_path_[i];
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
    active_trajectory_valid_ = false;
    active_bridge_valid_ = false;
    active_bridge_path_.clear();
    active_bridge_index_ = 0;
    active_goal_bridge_valid_ = false;
    active_goal_bridge_path_.clear();
    active_goal_bridge_index_ = 0;
    active_node_path_.clear();
    active_waypoint_index_ = 0;
    if (!keep_goal_id) {
      active_goal_id_ = -1;
    }
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
    active_goal_candidates_.clear();
    active_goal_id_ = -1;
    trajectory_update_requested_ = true;
    publishEmptyCandidateTrajectoryLocked();
  }

  void requestReplanCurrentTrialGoalLocked()
  {
    clearActiveTrajectoryLocked(true);
    trajectory_update_requested_ = true;
    publishEmptyCandidateTrajectoryLocked();
  }

  void requestReplanSameGoalLocked()
  {
    clearActiveTrajectoryLocked(true);
    trajectory_update_requested_ = true;
    publishEmptyCandidateTrajectoryLocked();
  }

  void advanceTrialGoalLocked()
  {
    trial_waiting_for_key_ = false;
    trial_hold_target_valid_ = false;
    clearActiveTrajectoryLocked(false);
    active_goal_candidates_.clear();
    active_goal_id_ = -1;
    trial_goal_coord_valid_ = false;
    trajectory_update_requested_ = true;

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

  static float nanMetric() {
    return std::numeric_limits<float>::quiet_NaN();
  }

  sensor_msgs::msg::JointState buildJointStateFromQ(
      const Eigen::VectorXf &q) const {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = controlled_joint_names_;
    msg.position.resize(controlled_joint_names_.size(), 0.0);
    msg.velocity.resize(controlled_joint_names_.size(), 0.0);
    msg.effort.resize(controlled_joint_names_.size(), 0.0);
    const std::size_t n = std::min<std::size_t>(
        controlled_joint_names_.size(), static_cast<std::size_t>(q.size()));
    for (std::size_t i = 0; i < n; ++i) {
      msg.position[i] = static_cast<double>(q[static_cast<int>(i)]);
    }
    return msg;
  }

  geometry_msgs::msg::Pose buildPoseFromNode(
      const GNGType::NodeType &node) const {
    geometry_msgs::msg::Pose pose;
    pose.position.x = static_cast<double>(node.weight_coord.x());
    pose.position.y = static_cast<double>(node.weight_coord.y());
    pose.position.z = static_cast<double>(node.weight_coord.z());
    pose.orientation.x = static_cast<double>(node.status.ee_orientation.x());
    pose.orientation.y = static_cast<double>(node.status.ee_orientation.y());
    pose.orientation.z = static_cast<double>(node.status.ee_orientation.z());
    pose.orientation.w = static_cast<double>(node.status.ee_orientation.w());
    return pose;
  }

  Manipulability::ManipulabilityEllipsoid calculateManipulabilityForQ(
      const Eigen::VectorXf &q, bool rotational) const {
    Manipulability::ManipulabilityEllipsoid out;
    if (!chain_ || q.size() == 0) {
      return out;
    }

    std::vector<double> joint_values;
    joint_values.reserve(static_cast<std::size_t>(q.size()));
    for (int i = 0; i < q.size(); ++i) {
      joint_values.push_back(static_cast<double>(q[i]));
    }

    try {
      const Eigen::MatrixXd J =
          chain_->calculateJacobianAt(chain_->getNumJoints() + 1, joint_values);
      if (J.rows() < 3) {
        return out;
      }
      const Eigen::MatrixXd Jpart =
          rotational && J.rows() >= 6 ? J.bottomRows(3) : J.topRows(3);
      out = Manipulability::calculateManipulabilityEllipsoid(Jpart);
    } 
    catch (const std::exception &e) {
      return out;
    }
    return out;
  }

  std::vector<float> buildPathManipulability(
      const std::vector<int> &path, bool rotational) const {
    std::vector<float> values;
    values.reserve(path.size());
    if (!gng_) {
      return values;
    }
    for (const int node_id : path) {
      if (node_id < 0 ||
          node_id >= static_cast<int>(gng_->getMaxNodeNum())) {
        values.push_back(nanMetric());
        continue;
      }
      const auto &node = gng_->nodeAt(node_id);
      if (node.id == -1 || node.weight_angle.size() == 0) {
        values.push_back(nanMetric());
        continue;
      }
      const auto manip =
          calculateManipulabilityForQ(node.weight_angle, rotational);
      values.push_back(manip.valid ? static_cast<float>(manip.manipulability)
                                   : nanMetric());
    }
    return values;
  }

  std::pair<float, float> estimatePathEnergyAndDuration(
      const Eigen::VectorXf &current_q, const std::vector<int> &path) const {
    if (!gng_ || path.empty()) {
      return {0.0f, 0.0f};
    }

    float energy = 0.0f;
    float duration = 0.0f;
    Eigen::VectorXf previous = current_q;

    for (const int node_id : path) {
      if (node_id < 0 ||
          node_id >= static_cast<int>(gng_->getMaxNodeNum())) {
        continue;
      }
      const auto &node = gng_->nodeAt(node_id);
      if (node.id == -1 || node.weight_angle.size() != previous.size()) {
        continue;
      }

      const Eigen::VectorXf delta = node.weight_angle - previous;
      energy += static_cast<float>(delta.squaredNorm());
      duration += static_cast<float>(
          delta.cwiseAbs().maxCoeff() / metrics_max_joint_velocity_);
      previous = node.weight_angle;
    }

    return {energy, duration};
  }

  void publishGraspCandidateMetricsLocked(
      const Eigen::VectorXf &current_q, int start_id,
      const std::vector<int> &goal_candidates,
      const std::unordered_map<int, std::vector<int>> &candidate_path_by_goal) {
    if (!candidate_metrics_pub_ || !gng_) {
      return;
    }

    gng_control_msgs::msg::GraspCandidateMetricArray out;
    out.header.stamp = now();
    out.header.frame_id =
        have_map_ && !latest_map_.header.frame_id.empty()
            ? latest_map_.header.frame_id
            : robot_base_frame_;

    std::string ns_raw = std::string(get_namespace());
    if (!ns_raw.empty() && ns_raw.front() == '/') {
      ns_raw.erase(ns_raw.begin());
    }
    out.robot_name = ns_raw;
    out.base_frame = robot_base_frame_;
    out.selected_goal_node_id = active_goal_id_;
    out.candidates.reserve(goal_candidates.size());

    for (std::size_t i = 0; i < goal_candidates.size(); ++i) {
      const int goal_id = goal_candidates[i];
      gng_control_msgs::msg::GraspCandidateMetric metric;
      metric.candidate_id = static_cast<int32_t>(i);
      metric.goal_node_id = goal_id;
      metric.start_node_id = start_id;
      metric.selected = goal_id == active_goal_id_;
      metric.feasible = candidate_path_by_goal.find(goal_id) != candidate_path_by_goal.end();

      metric.position_manipulability = nanMetric();
      metric.rotation_manipulability = nanMetric();
      metric.manipulability_condition_number = nanMetric();
      metric.min_singular_value = nanMetric();
      metric.joint_limit_margin_min = nanMetric();
      metric.joint_limit_margin_mean = nanMetric();
      metric.self_collision_margin = nanMetric();
      metric.environment_collision_margin = nanMetric();
      metric.gripper_width = nanMetric();
      metric.grasp_region_score = nanMetric();
      metric.estimated_energy = nanMetric();
      metric.estimated_duration = nanMetric();

      if (goal_id >= 0 &&
          goal_id < static_cast<int>(gng_->getMaxNodeNum())) {
        const auto &node = gng_->nodeAt(goal_id);
        if (node.id != -1) {
          metric.end_effector_pose = buildPoseFromNode(node);
          metric.final_joint_state = buildJointStateFromQ(node.weight_angle);
          metric.joint_limit_margin_min = node.status.joint_limit_score;
          metric.joint_limit_margin_mean = node.status.joint_limit_score;

          const auto position_manip =
              node.status.manip_info.valid
                  ? node.status.manip_info
                  : calculateManipulabilityForQ(node.weight_angle, false);
          const auto rotation_manip =
              calculateManipulabilityForQ(node.weight_angle, true);

          if (position_manip.valid) {
            metric.position_manipulability =
                static_cast<float>(position_manip.manipulability);
            metric.manipulability_condition_number =
                static_cast<float>(position_manip.condition_number);
            metric.min_singular_value =
                static_cast<float>(position_manip.min_singular_value);
            metric.manipulability_singular_values = {
                static_cast<float>(position_manip.singular_values.x()),
                static_cast<float>(position_manip.singular_values.y()),
                static_cast<float>(position_manip.singular_values.z())};
          }
          if (rotation_manip.valid) {
            metric.rotation_manipulability =
                static_cast<float>(rotation_manip.manipulability);
          }

          metric.metric_names = {
              "joint_limit_score",
              "collision_count",
              "danger_count",
              "is_colliding",
              "is_danger",
              "combined_score",
              "dynamic_manipulability"};
          metric.metric_values = {
              node.status.joint_limit_score,
              static_cast<float>(node.status.collision_count),
              static_cast<float>(node.status.danger_count),
              node.status.is_colliding ? 1.0f : 0.0f,
              node.status.is_danger ? 1.0f : 0.0f,
              node.status.combined_score,
              node.status.dynamic_manipulability};
        }
      }

      const auto path_it = candidate_path_by_goal.find(goal_id);
      if (path_it != candidate_path_by_goal.end()) {
        metric.path_node_ids.reserve(path_it->second.size());
        for (const int node_id : path_it->second) {
          metric.path_node_ids.push_back(static_cast<int32_t>(node_id));
        }
        metric.path_position_manipulability =
            buildPathManipulability(path_it->second, false);
        metric.path_rotation_manipulability =
            buildPathManipulability(path_it->second, true);
        const auto [energy, duration] =
            estimatePathEnergyAndDuration(current_q, path_it->second);
        metric.estimated_energy = energy;
        metric.estimated_duration = duration;
      }

      out.candidates.push_back(std::move(metric));
    }

    candidate_metrics_pub_->publish(out);
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
    candidate_trajectory_pub_->publish(
        topological_map_avoidance::buildPathMessage(
            *this, gng_, candidate_paths,
            have_map_ ? latest_map_.header.frame_id : std::string{}));
  }

  void publishCandidateRobotPreviewLocked() {
    if (!publish_candidate_robot_preview_ || !gng_ ||
        !candidate_robot_description_pub_ || !candidate_robot_pose_pub_ ||
        candidate_robot_urdf_content_.empty()) {
      return;
    }

    std::string ns_raw = std::string(get_namespace());
    if (!ns_raw.empty() && ns_raw.front() == '/') {
      ns_raw.erase(ns_raw.begin());
    }
    const std::string preview_tag = "candidate_goal_preview";
    const std::string preview_display_name =
        ns_raw.empty() ? preview_tag : (ns_raw + "/" + preview_tag);
    std::unordered_set<std::string> next_tags;

    nlohmann::json instance_payloads = nlohmann::json::array();
    const std::size_t preview_count =
        std::min<std::size_t>(8U, latest_goal_candidate_ids_.size());
    for (std::size_t i = 0; i < preview_count; ++i) {
      const int node_id = latest_goal_candidate_ids_[i];
      if (node_id < 0 || node_id >= static_cast<int>(gng_->getMaxNodeNum())) {
        continue;
      }
      const auto &node = gng_->nodeAt(node_id);
      if (node.id == -1 || !node.status.active || !node.status.valid) {
        continue;
      }

      std::vector<double> joint_values;
      joint_values.reserve(static_cast<std::size_t>(node.weight_angle.size()));
      for (int j = 0; j < node.weight_angle.size(); ++j) {
        joint_values.push_back(static_cast<double>(node.weight_angle[j]));
      }

      std::vector<double> fk_values = joint_values;
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
      std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
      if (chain_) {
        chain_->forwardKinematicsAt(fk_values, positions, orientations);
      }
      Manipulability::ManipulabilityEllipsoid manip;
      if (chain_) {
        const Eigen::MatrixXd J =
            chain_->calculateJacobianAt(chain_->getNumJoints() + 1, joint_values);
        const Eigen::MatrixXd Jv = J.topRows(3);
        manip = Manipulability::calculateManipulabilityEllipsoid(Jv);
      }
      const Eigen::Vector3d manip_center =
          positions.empty() ? Eigen::Vector3d::Zero() : positions.back();

      const double timestamp = this->now().seconds();
      instance_payloads.push_back(buildRobotPayloadJson(
          robot_base_frame_, candidate_robot_urdf_content_, controlled_joint_names_,
          joint_values, positions, orientations, timestamp,
          candidate_robot_preview_opacity_, manip_center, &manip));
    }

    if (instance_payloads.empty()) {
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

    next_tags.insert(preview_tag);

    auto primary_robot = instance_payloads.front();
    primary_robot["displayName"] = preview_display_name;
    const std::string desc_json = buildRobotStreamJsonWithInstances(
        "stream.robot.description", preview_tag, primary_robot, instance_payloads);
    const std::string pose_json = buildRobotStreamJsonWithInstances(
        "stream.robot.pose", preview_tag, primary_robot, instance_payloads);

    std_msgs::msg::String desc_msg;
    desc_msg.data = desc_json;
    candidate_robot_description_pub_->publish(desc_msg);

    std_msgs::msg::String pose_msg;
    pose_msg.data = pose_json;
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
