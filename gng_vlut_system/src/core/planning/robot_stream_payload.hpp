#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include "core/kinematics/kinematic_chain.hpp"
#include "core/metrics/manipulability.hpp"

namespace robot_sim::planning {

inline std::string detectLocalMeshPackageName(const std::string &source_path) {
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

inline std::string rewriteRelativeMeshUris(
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

inline bool loadRobotDescription(std::string &out_text,
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

inline nlohmann::json buildRobotPayloadJson(
    const std::string &frame_id, const std::string &urdf_content,
    const std::vector<std::string> &joint_names,
    const std::vector<double> &joint_values,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
    const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &orientations,
    double timestamp,
    const Eigen::Vector3d &manip_center,
    const Manipulability::ManipulabilityEllipsoid *manip = nullptr,
    bool is_goal = false) {
  nlohmann::json robot;
  robot["timestamp"] = timestamp;
  robot["frameId"] = frame_id;
  robot["urdf"] = urdf_content;
  robot["jointNames"] = joint_names;
  robot["jointValues"] = joint_values;

  auto &pos_arr = robot["positions"] = nlohmann::json::array();
  for (const auto &v : positions) {
    pos_arr.push_back({v.x(), v.y(), v.z()});
  }

  auto &quat_arr = robot["orientations"] = nlohmann::json::array();
  for (const auto &q : orientations) {
    quat_arr.push_back({q.x(), q.y(), q.z(), q.w()});
  }

  if (!positions.empty()) {
    robot["basePosition"] = {
        positions.front().x(), positions.front().y(), positions.front().z()};
  }
  if (!orientations.empty()) {
    robot["baseOrientation"] = {
        orientations.front().x(), orientations.front().y(),
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
    robot["manipScale"] = {
        manip->singular_values.x(), manip->singular_values.y(),
        manip->singular_values.z()};
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

inline std::string buildRobotStreamJsonWithInstances(
    const std::string &type, const std::string &tag,
    const nlohmann::json &robot_payload,
    const nlohmann::json &instances) {
  nlohmann::json root;
  root["type"] = type;
  root["tag"] = tag;
  root["robot"] = robot_payload;
  root["robot"]["instances"] = instances;
  return root.dump();
}

inline double quaternionAngularErrorDeg(
    const Eigen::Quaterniond &a, const Eigen::Quaterniond &b) {
  Eigen::Quaterniond qa = a.normalized();
  Eigen::Quaterniond qb = b.normalized();
  double dot = std::abs(qa.dot(qb));
  dot = std::min(1.0, std::max(-1.0, dot));
  return 2.0 * std::acos(dot) * 180.0 / M_PI;
}

struct CandidateRobotPreviewPayload {
  std::string tag;
  std::string description_json;
  std::string pose_json;
};

template <typename GNGType>
inline std::optional<CandidateRobotPreviewPayload> buildCandidateRobotPreviewPayload(
    const std::string &node_namespace,
    const std::string &robot_base_frame,
    const std::string &urdf_content,
    const std::vector<std::string> &controlled_joint_names,
    const std::vector<int> &goal_candidate_ids,
    const std::shared_ptr<GNGType> &gng,
    const std::shared_ptr<::kinematics::KinematicChain> &chain,
    double timestamp) {
  if (!gng || !chain || urdf_content.empty()) {
    return std::nullopt;
  }

  std::string ns_raw = node_namespace;
  if (!ns_raw.empty() && ns_raw.front() == '/') {
    ns_raw.erase(ns_raw.begin());
  }
  const std::string preview_tag = "candidate_goal_preview";
  const std::string preview_display_name =
      ns_raw.empty() ? preview_tag : (ns_raw + "/" + preview_tag);

  nlohmann::json instance_payloads = nlohmann::json::array();
  const std::size_t preview_count =
      std::min<std::size_t>(8U, goal_candidate_ids.size());
  for (std::size_t i = 0; i < preview_count; ++i) {
    const int node_id = goal_candidate_ids[i];
    if (node_id < 0 || node_id >= static_cast<int>(gng->getMaxNodeNum())) {
      continue;
    }
    const auto &node = gng->nodeAt(node_id);
    if (node.id == -1 || !node.status.active || !node.status.self_collision_free) {
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
    chain->forwardKinematicsAt(fk_values, positions, orientations);

    Manipulability::ManipulabilityEllipsoid manip;
    const Eigen::MatrixXd jacobian =
        chain->calculateJacobianAt(chain->getNumJoints() + 1, joint_values);
    const Eigen::MatrixXd linear_jacobian = jacobian.topRows(3);
    manip = Manipulability::calculateManipulabilityEllipsoid(linear_jacobian);

    const Eigen::Vector3d manip_center =
        positions.empty() ? Eigen::Vector3d::Zero() : positions.back();

    instance_payloads.push_back(buildRobotPayloadJson(
        robot_base_frame, urdf_content, controlled_joint_names,
        joint_values, positions, orientations, timestamp, manip_center, &manip));
  }

  if (instance_payloads.empty()) {
    return std::nullopt;
  }

  auto primary_robot = instance_payloads.front();
  primary_robot["displayName"] = preview_display_name;

  CandidateRobotPreviewPayload payload;
  payload.tag = preview_tag;
  payload.description_json = buildRobotStreamJsonWithInstances(
      "stream.robot.description", preview_tag, primary_robot, instance_payloads);
  payload.pose_json = buildRobotStreamJsonWithInstances(
      "stream.robot.pose", preview_tag, primary_robot, instance_payloads);
  return payload;
}

}  // namespace robot_sim::planning
