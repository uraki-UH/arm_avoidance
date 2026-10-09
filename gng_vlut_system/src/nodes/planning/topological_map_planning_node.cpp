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
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

#include <gng_control_msgs/msg/grasp_candidate_metric.hpp>
#include <gng_control_msgs/msg/grasp_candidate_metric_array.hpp>
// Candidate goal ids from the target-pose selector.
#include <std_msgs/msg/int32_multi_array.hpp>

#include "common/resource_utils.hpp"
#include "planner/RRT/ik_rrt_planner.hpp"
#include "planner/RRT/rrt_params.hpp"
#include "planner/RRT/state_validity_checker.hpp"
#include "planning/graph_planner_factory.hpp"
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

class TopologicalMapPathPlannerNode : public rclcpp::Node {
public:
  using GNGType = ::GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
  using CostType = ::planning::JointLInfCost<Eigen::VectorXf, Eigen::Vector3f>;

  explicit TopologicalMapPathPlannerNode(const rclcpp::NodeOptions &options)
      : Node("topological_map_path_planner_node", options) {
    declare_parameter("urdf_path", "");
    declare_parameter("gng_model_path", "");
    declare_parameter("gng.data_directory", "");
    declare_parameter("gng.experiment_id", "");
    declare_parameter("gng.gng_model_filename", "");
    declare_parameter("gng.profile_names", "");
    declare_parameter("joint_topic", "/ToPoDualArm/joint_states");
    declare_parameter("topological_map_topic", "/ToPoDualArm/Tmap_static");
    declare_parameter("trajectory_topic", "/ToPoDualArm/plan_Tmap");
    declare_parameter("candidate_trajectory_topic", "/ToPoDualArm/cand_Tmap");
    declare_parameter("candidate_metrics_topic", "/ToPoDualArm/grasp_candidate_metrics");
    declare_parameter("evaluation_metrics_topic", "/evaluation_metrics");
    declare_parameter("current_ee_pose_topic", "");
    declare_parameter("goal_candidate_ids_topic", "/selected_goal_candidate_ids");
    declare_parameter("publish_hz", 20.0);
    rcl_interfaces::msg::ParameterDescriptor component_descriptor;
    component_descriptor.read_only = true;
    declare_parameter("graph_planner", "gng_dijkstra", component_descriptor);
    declare_parameter("avoid_collisions", true);
    // 隣接危険ノード数の経路コスト加算。衝突・危険ノードへの進入判定とは独立
    declare_parameter("enable_safety_penalty", false);
    declare_parameter("avoid_danger", true);
    declare_parameter("allow_danger_goal", true);
    declare_parameter("strict_goal_collision_check", false);
    declare_parameter("allow_zero_initial_joint_state", true);
    declare_parameter("robot_base_frame", "");
    declare_parameter("frame_id", "");
    declare_parameter("publish_candidate_robot_preview", true);
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

    chain_joint_names_ = orderedJointNames(*chain_);
    controlled_joint_names_ = orderedControlledJointNames(*chain_);

    const int dof = chain_->getTotalDOF();
    RCLCPP_DEBUG(get_logger(), "Selected GNG profiles: %s",
                joinStrings(selected_profiles, ", ").c_str());
    RCLCPP_DEBUG(get_logger(), "Chain joint order (%zu): %s",
                chain_joint_names_.size(),
                joinStrings(chain_joint_names_, ", ").c_str());
    RCLCPP_DEBUG(get_logger(), "Controlled joint order (%zu): %s",
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
                  "gng_model_path is empty. Planning requires a GNG model.");
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
      if (node.id != -1 && node.status.active && node.status.self_collision_free &&
          !node.status.is_colliding) {
        cached_safe_goal_ids_.push_back(node.id);
      }
    }
    if (!cached_safe_goal_ids_.empty()) {
      RCLCPP_DEBUG(
          get_logger(),
          "Cached %zu safe GNG nodes from loaded model before map updates.",
          cached_safe_goal_ids_.size());
    }

    avoid_danger_ = get_parameter("avoid_danger").as_bool();
    allow_danger_goal_ = get_parameter("allow_danger_goal").as_bool();
    graph_planner_options planner_options;
    planner_options.enable_collision_check = get_parameter("avoid_collisions").as_bool();
    planner_options.enable_danger_check = avoid_danger_;
    planner_options.enable_safety_penalty = get_parameter("enable_safety_penalty").as_bool();
    planner_options.enable_strict_goal_check = get_parameter("strict_goal_collision_check").as_bool();
    planner_options.enable_static_graph = true;
    planner_ = make_graph_planner<Eigen::VectorXf, Eigen::Vector3f, GNGType>(
        get_parameter("graph_planner").as_string(), *gng_, planner_options,
        std::make_shared<CostType>(1000.0f));
    allow_zero_initial_joint_state_ =
        get_parameter("allow_zero_initial_joint_state").as_bool();

    const std::string joint_topic = get_parameter("joint_topic").as_string();
    const std::string topological_map_topic =
        get_parameter("topological_map_topic").as_string();
    goal_candidate_ids_topic_ = get_parameter("goal_candidate_ids_topic").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    if (robot_base_frame_.empty()) {
      // 通常ロボットと共通の表示基準。未指定時のみURDFルートへフォールバック
      robot_base_frame_ = get_parameter("frame_id").as_string();
      if (robot_base_frame_.empty()) {
        robot_base_frame_ = robot_root_link_name_.empty() ? "base_link" : robot_root_link_name_;
      }
      const std::string ns_raw = std::string(get_namespace());
      const std::string ns = ns_raw.empty() ? "" : (ns_raw.front() == '/' ? ns_raw.substr(1) : ns_raw);
      if (!ns.empty() && robot_base_frame_ != "world" &&
          robot_base_frame_.find('/') == std::string::npos) {
        robot_base_frame_ = ns + "/" + robot_base_frame_;
      }
    }
    publish_candidate_robot_preview_ = get_parameter("publish_candidate_robot_preview").as_bool();
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
          has_pending_plan_ = has_pending_plan_ || !have_map_ ||
              latest_map_.header.frame_id != msg->header.frame_id;
          latest_map_ = *msg;
          have_map_ = true;
          updateNodeStatusFromMapLocked(*msg);
        });

    if (!goal_candidate_ids_topic_.empty()) {
      goal_candidate_ids_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        goal_candidate_ids_topic_, rclcpp::QoS(1).reliable().transient_local(),
          [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            const std::vector<int> goal_ids(msg->data.begin(), msg->data.end());
            if (goal_ids == latest_goal_candidate_ids_) {
              return;
            }
            latest_goal_candidate_ids_.clear();
            latest_goal_candidate_ids_.reserve(msg->data.size());
            for (const auto id : msg->data) {
              latest_goal_candidate_ids_.push_back(static_cast<int>(id));
            }
            // 到達領域の変化による旧経路・旧評価の失効と、新しい候補への再計画
            has_pending_plan_ = true;
            RCLCPP_DEBUG(
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

    std::string full_robot_urdf_content;
    if (!loadRobotDescription(full_robot_urdf_content, urdf_path)) {
      RCLCPP_WARN(
          get_logger(),
          "Failed to load robot description text for candidate robot preview: %s",
          urdf_path.c_str());
    } else {
      candidate_robot_urdf_content_ = buildChainScopedUrdf(
          full_robot_urdf_content, *chain_);
      if (candidate_robot_urdf_content_.empty()) {
        // XML解析に失敗した場合でも、候補プレビュー自体は完全URDFで継続
        candidate_robot_urdf_content_ = std::move(full_robot_urdf_content);
        RCLCPP_WARN(
            get_logger(),
            "Failed to build chain-scoped URDF; using full URDF for candidate preview");
      }
    }

    request_update_srv_ = create_service<std_srvs::srv::Trigger>(
        "request_trajectory_update",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(mutex_);
          has_pending_plan_ = true;
          response->success = true;
          response->message = "trajectory update requested";
        });

    const double publish_hz = std::max(1.0, get_parameter("publish_hz").as_double());
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz)),
        [this]() { publish_candidate_paths(); });
  }

private:
  bool has_pending_plan_ = true;
  Eigen::VectorXf last_plan_q_;

  // 入力変化または明示要求に限定した候補経路・評価の更新。追従状態の更新なし
  void publish_candidate_paths() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!gng_ || !have_map_ || (!have_joint_state_ && !allow_zero_initial_joint_state_)) {
      return;
    }
    const Eigen::VectorXf current_q = have_joint_state_
        ? currentJointVectorLocked() : Eigen::VectorXf::Zero(chain_->getTotalDOF());
    if (!has_pending_plan_ && last_plan_q_.size() == current_q.size() &&
        std::equal(current_q.data(), current_q.data() + current_q.size(), last_plan_q_.data())) {
      return;
    }
    has_pending_plan_ = false;
    last_plan_q_ = current_q;
    // 候補選定から経路探索・選択までの実時間。配信用データ生成・配信の除外
    const auto plan_start = std::chrono::steady_clock::now();
    const auto goal_candidates = latest_goal_candidate_ids_.empty()
        ? std::vector<int>{} : selectedGoalCandidatesLocked(-1);
    const auto start_candidates = collectNearestStartCandidatesLocked(current_q, 5);
    int selected_start_id = -1;
    std::unordered_map<int, std::vector<int>> candidate_path_by_goal;
    std::vector<std::vector<int>> candidate_paths;
    auto [goal_id, node_path] = planFromStartCandidatesLocked(
        current_q, start_candidates, goal_candidates, selected_start_id,
        candidate_path_by_goal, candidate_paths);
    const double plan_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - plan_start).count();
    RCLCPP_INFO(get_logger(), "dof=%d Plan: %.2f ms Count: goal=%zu reach=%zu",
                chain_->getTotalDOF(), plan_ms,
                goal_candidates.size(), candidate_path_by_goal.size());
    selected_goal_id_ = goal_id;
    // 経路ID不変でも、更新された安全ラベル・座標系の再配信
    have_last_candidate_publish_ = false;
    if (candidate_paths.empty()) {
      publishCandidateTrajectoryPathsLocked(candidate_paths);
    } else {
      publishCandidateTrajectoryPathsLocked(current_q, candidate_paths);
    }
    publishTrajectoryPathLocked(current_q, node_path);
    publishGraspCandidateMetricsLocked(selected_start_id, goal_candidates, candidate_path_by_goal);
    publishCurrentEefPoseLocked(current_q);
  }

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

      const uint8_t next_label = n.label == 2 ? 2 : (n.label == 3 ? 3 : 1);
      has_pending_plan_ = has_pending_plan_ || !node.status.active ||
          pathLabelFromStatus(node.status) != next_label;

      // FIXME: self_collision_free は静的な自己干渉の判定結果であり、環境障害物の
      // ラベルで書き換えるべきではない。障害物が消えたときに true へ戻してしまうため、
      // 元々自己干渉していたノードが安全と誤判定される。挙動が変わる修正のため
      // 別途対応する。詳細は TASK_CANDIDATES.md を参照。
      switch (n.label) {
      case 2: // collision
        node.status.is_colliding = true;
        node.status.is_danger = false;
        node.status.self_collision_free = false;
        node.status.collision_count = 1;
        node.status.danger_count = 0;
        break;
      case 3: // danger
        node.status.is_colliding = false;
        node.status.is_danger = true;
        node.status.self_collision_free = true;
        node.status.collision_count = 0;
        node.status.danger_count = 1;
        break;
      default:
        node.status.is_colliding = false;
        node.status.is_danger = false;
        node.status.self_collision_free = true;
        node.status.collision_count = 0;
        node.status.danger_count = 0;
        break;
      }
      node.status.active = true;

      if (!node.status.is_colliding) {
        cached_safe_goal_ids_.push_back(id);
      }
    }
  }

  std::shared_ptr<::kinematics::KinematicChain> chain_;
  std::vector<std::string> chain_joint_names_;
  std::vector<std::string> controlled_joint_names_;
  std::shared_ptr<GNGType> gng_;
  std::unique_ptr<graph_planner<GNGType>> planner_;

  bool allow_zero_initial_joint_state_ = true;
  std::string trajectory_topic_;
  std::string candidate_trajectory_topic_;
  std::string candidate_metrics_topic_;
  std::string evaluation_metrics_topic_;
  std::string current_ee_pose_topic_;
  std::string goal_candidate_ids_topic_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr goal_candidate_ids_sub_;
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
  Eigen::VectorXf last_candidate_current_q_;
  std::vector<std::vector<int>> last_candidate_paths_;
  bool have_last_candidate_publish_ = false;
  std::vector<int> cached_safe_goal_ids_;
  std::vector<int> latest_goal_candidate_ids_;
  std::unordered_set<std::string> last_candidate_robot_tags_;
  bool candidate_preview_empty_sent_ = false;
  bool avoid_danger_ = true;
  bool allow_danger_goal_ = true;
  bool publish_candidate_robot_preview_ = true;
  float goal_rot_manip_weight_ = 1.0f;
  float goal_joint_limit_weight_ = 0.5f;
  std::string robot_base_frame_;
  std::string robot_root_link_name_;
  std::string candidate_robot_urdf_content_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_update_srv_;

  int selected_goal_id_ = -1;

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
      if (node.id == -1 || !node.status.active || !node.status.self_collision_free) {
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

  std::pair<int, std::vector<int>> planFromStartCandidatesLocked(
      const Eigen::VectorXf &current_q, const std::vector<int> &start_candidates,
      const std::vector<int> &goal_candidates, int &selected_start_id,
      std::unordered_map<int, std::vector<int>> &candidate_path_by_goal,
      std::vector<std::vector<int>> &candidate_paths) {
    return topological_map_avoidance::planFromStartCandidates(
        gng_, *planner_, current_q, start_candidates, goal_candidates,
        selected_start_id, candidate_path_by_goal, candidate_paths,
        allow_danger_goal_,
        goal_rot_manip_weight_, goal_joint_limit_weight_);
  }

  void publishGraspCandidateMetricsLocked(
      int start_id,
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
        now(), frame_id, ns_raw, robot_base_frame_, selected_goal_id_,
        start_id, goal_candidates, candidate_path_by_goal, gng_, chain_,
        controlled_joint_names_);
    candidate_metrics_pub_->publish(out);

    if (evaluation_metrics_pub_ && !evaluation_metrics_topic_.empty()) {
      const std::string profile_name = get_parameter("gng.profile_names").as_string();
      constexpr const char *kSchemaId = "grasp_candidate_metrics";
      constexpr uint32_t kSchemaRevision = 5;
      const auto stamp = now();
      const auto metrics = robot_sim::common::buildCandidateEvaluationMetrics(
          stamp, profile_name, "candidate", candidate_metrics_topic_, out,
          kSchemaId, kSchemaRevision);
      evaluation_metrics_pub_->publish(metrics);
    }
  }

  void publishTrajectoryPathLocked(const Eigen::VectorXf &current_q,
                                   const std::vector<int> &node_path) {
    if (!trajectory_pub_ || !gng_) {
      return;
    }
    trajectory_pub_->publish(
        topological_map_avoidance::buildPathMessageWithCurrentPose(
            *this, gng_, current_q, chain_, {node_path},
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
      clearCandidateRobotPreviewLocked();
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
    candidate_preview_empty_sent_ = false;
  }

  void clearCandidateRobotPreviewLocked() {
    if (!candidate_robot_pose_pub_ || candidate_preview_empty_sent_) {
      return;
    }

    std::unordered_set<std::string> tags = last_candidate_robot_tags_;
    tags.insert("candidate_goal_preview");
    for (const auto &tag : tags) {
      std_msgs::msg::String delete_msg;
      delete_msg.data = nlohmann::json({
          {"type", "stream.robot.delete"},
          {"tag", tag},
      }).dump();
      candidate_robot_pose_pub_->publish(delete_msg);
    }
    last_candidate_robot_tags_.clear();
    candidate_preview_empty_sent_ = true;
  }
};

} // namespace robot_sim::planning

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::planning::TopologicalMapPathPlannerNode)
