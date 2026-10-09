#include <rclcpp/rclcpp.hpp>
#include <gng_control_msgs/srv/check_arm_pair.hpp>
#include <nlohmann/json.hpp>
#include "collision/geometric_self_collision_checker.hpp"
#include "collision/independent_arm_model.hpp"
#include "collision/joint_segment_collision.hpp"
#include "common/resource_utils.hpp"
#include "gng/GrowingNeuralGas.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include <filesystem>
#include <fstream>
#include <map>
#include <set>

namespace {
using gng_type = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
using pair_service = gng_control_msgs::srv::CheckArmPair;

struct arm_model {
  nlohmann::json metadata;
  std::unique_ptr<gng_type> graph;
  std::vector<std::string> joint_names;
};

arm_model load_arm_model(const std::string &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot read arm metadata: " + path);
  arm_model result;
  input >> result.metadata;
  if (result.metadata.at("version") != 1 || result.metadata.at("mode") != "independent_arm" ||
      !result.metadata.at("enable_pair_collision_check").get<bool>()) {
    throw std::runtime_error("Unsupported arm metadata: " + path);
  }
  result.joint_names = result.metadata.at("joint_names").get<std::vector<std::string>>();
  if (result.joint_names.empty() ||
      std::set<std::string>(result.joint_names.begin(), result.joint_names.end()).size() != result.joint_names.size()) {
    throw std::runtime_error("Invalid arm joint names");
  }
  result.graph = std::make_unique<gng_type>(result.joint_names.size(), 3, nullptr);
  const auto gng_path = std::filesystem::path(path).parent_path() / result.metadata.at("gng_file").get<std::string>();
  if (!result.graph->load(gng_path.string()) || result.graph->getActiveIndices().empty()) {
    throw std::runtime_error("Cannot load independent arm GNG: " + gng_path.string());
  }
  for (int idx : result.graph->getActiveIndices()) {
    const auto &node = result.graph->nodeAt(idx);
    if (node.weight_angle.size() != static_cast<int>(result.joint_names.size()) || !node.weight_angle.allFinite()) {
      throw std::runtime_error("Arm GNG joint dimensions or values are invalid");
    }
  }
  if (result.metadata.at("num_nodes").get<std::size_t>() != result.graph->getActiveIndices().size()) {
    throw std::runtime_error("Arm metadata node count does not match GNG");
  }
  return result;
}

class independent_arm_pair_node : public rclcpp::Node {
public:
  independent_arm_pair_node() : Node("independent_arm_pair_node") {
    left_ = load_arm_model(declare_parameter<std::string>("left_model_path", ""));
    right_ = load_arm_model(declare_parameter<std::string>("right_model_path", ""));
    const auto urdf_path = robot_sim::common::resolvePath(declare_parameter<std::string>("urdf_path", ""));
    const auto resources = declare_parameter<std::string>("resource_root_dir", "");
    const auto meshes = declare_parameter<std::string>("mesh_root_dir", "");
    const double voxel_size = declare_parameter<double>("collision.voxel_size", 0.001);
    if (!std::isfinite(voxel_size) || voxel_size <= 0.0) throw std::invalid_argument("Invalid collision voxel size");
    max_joint_step_ = declare_parameter<double>("max_joint_step", 0.025);
    if (!std::isfinite(max_joint_step_) || max_joint_step_ <= 0.0 || max_joint_step_ > 0.025) {
      throw std::invalid_argument("Invalid max_joint_step");
    }
    // 別機体・異なる内部判定解像度のモデル混在の拒否
    for (const auto *arm : {&left_, &right_}) {
      const auto learned_urdf = robot_sim::common::resolvePath(arm->metadata.at("urdf_path").get<std::string>());
      if (std::filesystem::canonical(learned_urdf) != std::filesystem::canonical(urdf_path)) {
        throw std::invalid_argument("Arm model URDF does not match current robot");
      }
      if (arm->metadata.at("collision_voxel_size").get<double>() != voxel_size) {
        throw std::invalid_argument("Arm model collision voxel size does not match");
      }
    }
    if (left_.metadata.at("profile") != "left_arm" || right_.metadata.at("profile") != "right_arm") {
      throw std::invalid_argument("Independent arm profiles do not match");
    }
    model_ = simulation::loadRobotFromUrdf(urdf_path, resources, meshes);
    const std::string left_root = left_.metadata.at("root_link");
    const std::string right_root = right_.metadata.at("root_link");
    simulation::make_independent_arm_collision_model(model_, left_root, {left_root, right_root});
    chain_ = simulation::createMultiArmKinematicChain(model_, {
        {left_root, left_.metadata.at("eef_link"), ""},
        {right_root, right_.metadata.at("eef_link"), ""}});
    for (int idx = 0; idx < chain_->getNumJoints(); ++idx) {
      if (chain_->getJointDOF(idx) > 0) joint_names_.push_back(chain_->getJointName(idx));
    }
    auto expected = left_.joint_names;
    expected.insert(expected.end(), right_.joint_names.begin(), right_.joint_names.end());
    if (expected != joint_names_ || std::set<std::string>(expected.begin(), expected.end()).size() != expected.size()) {
      throw std::runtime_error("Independent arm joint order does not match URDF");
    }
    for (const auto &entry : model_.getJoints()) {
      const auto &name = entry.first;
      if (entry.second.type == kinematics::JointType::Fixed ||
          std::find(joint_names_.begin(), joint_names_.end(), name) != joint_names_.end()) continue;
      const double first = left_.metadata.at("fixed_joints").at(name).get<double>();
      const double second = right_.metadata.at("fixed_joints").at(name).get<double>();
      if (first != 0.0 || second != 0.0) throw std::runtime_error("Unsupported fixed joint condition: " + name);
      fixed_joints_.insert(name);
    }
    checker_ = std::make_unique<simulation::GeometricSelfCollisionChecker>(model_, *chain_, true, voxel_size);
#ifdef USE_FCL
    checker_->setStrictMode(true);
#endif
    const bool enable_exclusions = declare_parameter<bool>("collision.apply_self_collision_exclusion_pairs", true);
    const auto exclusions = declare_parameter<std::vector<std::string>>(
        "collision.self_collision_exclusion_pairs", std::vector<std::string>{});
    for (const auto &pair : exclusions) {
      if (!enable_exclusions) break;
      const auto separator = pair.find('|');
      if (separator == std::string::npos || !model_.getLink(pair.substr(0, separator)) ||
          !model_.getLink(pair.substr(separator + 1))) throw std::invalid_argument("Invalid collision pair");
      checker_->addCollisionExclusion(pair.substr(0, separator), pair.substr(separator + 1));
    }
    service_ = create_service<pair_service>("check_arm_pair", [this](
        const std::shared_ptr<pair_service::Request> request,
        std::shared_ptr<pair_service::Response> response) { check_pair(*request, *response); });
    RCLCPP_INFO(get_logger(), "Independent arm pair checker ready: left=%zu right=%zu",
                left_.graph->getActiveIndices().size(), right_.graph->getActiveIndices().size());
  }

private:
  const Eigen::VectorXf &get_angles(arm_model &arm, int node_id) {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= arm.graph->getNodes().size()) {
      throw std::invalid_argument("Unknown GNG node ID");
    }
    const auto &node = arm.graph->nodeAt(node_id);
    if (node.id != node_id || !node.status.active || !node.status.self_collision_free) {
      throw std::invalid_argument("Unavailable GNG node");
    }
    return node.weight_angle;
  }

  Eigen::VectorXd parse_start(const sensor_msgs::msg::JointState &state) {
    if (state.name.size() != state.position.size()) throw std::invalid_argument("JointState sizes do not match");
    std::map<std::string, double> values;
    for (std::size_t idx = 0; idx < state.name.size(); ++idx) {
      const auto &name = state.name[idx];
      const double value = state.position[idx];
      if (!std::isfinite(value) || !values.emplace(name, value).second) throw std::invalid_argument("Invalid start joints");
      if (fixed_joints_.count(name)) {
        if (value != 0.0) throw std::invalid_argument("Fixed joint condition changed: " + name);
      } else if (std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end()) {
        throw std::invalid_argument("Unknown start joint: " + name);
      }
    }
    Eigen::VectorXd angles(joint_names_.size());
    for (std::size_t idx = 0; idx < joint_names_.size(); ++idx) {
      if (!values.count(joint_names_[idx])) throw std::invalid_argument("Incomplete start joints");
      angles[idx] = values.at(joint_names_[idx]);
    }
    return angles;
  }

  void validate_angles(const Eigen::VectorXd &angles) const {
    const auto &values = angles;
    // 関節限界外の暗黙クランプを避ける入力検証
    for (std::size_t idx = 0; idx < values.size(); ++idx) {
      const auto *joint = model_.getJoint(joint_names_[idx]);
      if (!std::isfinite(values[idx]) || (joint->limits.lower < joint->limits.upper &&
          (values[idx] < joint->limits.lower || values[idx] > joint->limits.upper))) {
        throw std::invalid_argument("Joint value outside limits: " + joint_names_[idx]);
      }
    }
  }

  bool is_colliding(const Eigen::VectorXd &angles) {
    std::vector<double> values(angles.data(), angles.data() + angles.size());
    chain_->updateKinematics(values);
    checker_->updateBodyPoses(chain_->getLinkPositions(), chain_->getLinkOrientations());
    return checker_->checkCollision();
  }

  void check_pair(const pair_service::Request &request, pair_service::Response &response) {
    try {
      const auto &left = get_angles(left_, request.left_node_id);
      const auto &right = get_angles(right_, request.right_node_id);
      Eigen::VectorXd target(left.size() + right.size());
      target << left.cast<double>(), right.cast<double>();
      const bool has_start = !request.start_state.name.empty() || !request.start_state.position.empty();
      validate_angles(target);
      Eigen::VectorXd start;
      if (has_start) {
        start = parse_start(request.start_state);
        validate_angles(start);
      }
      const bool has_collision = has_start
          ? simulation::has_joint_segment_collision(start, target, max_joint_step_,
              [this](const Eigen::VectorXd &angles) { return is_colliding(angles); })
          : is_colliding(target);
      response.is_valid = true;
      response.is_collision_free = !has_collision;
      response.has_path_check = has_start;
      response.joint_state.header.stamp = now();
      response.joint_state.name = joint_names_;
      response.joint_state.position.assign(target.data(), target.data() + target.size());
      if (has_collision) {
        for (const auto &pair : checker_->collectSelfCollisionPairs()) {
          response.collision_pairs.push_back(pair.first + "|" + pair.second);
        }
      }
      response.message = has_collision ? "self_collision" : (has_start ? "sampled_path_clear" : "endpoint_clear");
    } catch (const std::exception &error) {
      response.is_valid = false;
      response.is_collision_free = false;
      response.message = error.what();
    }
  }

  arm_model left_, right_;
  simulation::RobotModel model_;
  std::unique_ptr<kinematics::KinematicChain> chain_;
  std::unique_ptr<simulation::GeometricSelfCollisionChecker> checker_;
  std::vector<std::string> joint_names_;
  std::set<std::string> fixed_joints_;
  double max_joint_step_ = 0.025;
  rclcpp::Service<pair_service>::SharedPtr service_;
};
} // 無名名前空間の終端

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<independent_arm_pair_node>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("independent_arm_pair_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
