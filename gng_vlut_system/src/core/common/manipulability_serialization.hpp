#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nlohmann/json.hpp>

#include "core/metrics/manipulability.hpp"
#include <ais_gng_feature_msgs/msg/topological_node_feature.hpp>

namespace robot_sim::common {

template <typename Derived>
inline geometry_msgs::msg::Point32 toPoint32(
    const Eigen::MatrixBase<Derived> &v) {
  geometry_msgs::msg::Point32 p;
  p.x = static_cast<float>(v.x());
  p.y = static_cast<float>(v.y());
  p.z = static_cast<float>(v.z());
  return p;
}

template <typename Derived>
inline geometry_msgs::msg::Vector3 toVector3(
    const Eigen::MatrixBase<Derived> &v) {
  geometry_msgs::msg::Vector3 out;
  out.x = static_cast<double>(v.x());
  out.y = static_cast<double>(v.y());
  out.z = static_cast<double>(v.z());
  return out;
}

inline geometry_msgs::msg::Pose toPose(const Eigen::Vector3f &position,
                                       const Eigen::Quaternionf &orientation) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = static_cast<double>(position.x());
  pose.position.y = static_cast<double>(position.y());
  pose.position.z = static_cast<double>(position.z());
  pose.orientation.x = static_cast<double>(orientation.x());
  pose.orientation.y = static_cast<double>(orientation.y());
  pose.orientation.z = static_cast<double>(orientation.z());
  pose.orientation.w = static_cast<double>(orientation.w());
  return pose;
}

inline std::vector<float> toFloat32Vector(const Eigen::VectorXf &values) {
  std::vector<float> out;
  out.reserve(static_cast<std::size_t>(values.size()));
  for (int i = 0; i < values.size(); ++i) {
    out.push_back(values[i]);
  }
  return out;
}

template <typename JointPositionsVector>
inline std::vector<geometry_msgs::msg::Point32> toPoint32Vector(
    const JointPositionsVector &values) {
  std::vector<geometry_msgs::msg::Point32> out;
  out.reserve(values.size());
  for (const auto &v : values) {
    out.push_back(toPoint32(v));
  }
  return out;
}

inline geometry_msgs::msg::Quaternion toQuaternion(
    const Eigen::Matrix3d &rotation) {
  Eigen::Quaterniond q(rotation);
  if (!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) ||
      !std::isfinite(q.z())) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

inline void fillManipulabilityFields(
    ais_gng_feature_msgs::msg::TopologicalNodeFeature &node_msg,
    const Manipulability::ManipulabilityEllipsoid &ellipsoid,
    bool is_goal = false) {
  node_msg.is_goal = is_goal;
  node_msg.manip_valid = ellipsoid.valid;
  node_msg.manip_value = static_cast<float>(ellipsoid.manipulability);
  node_msg.manip_condition_number =
      static_cast<float>(ellipsoid.condition_number);
  if (!ellipsoid.valid) {
    node_msg.manip_center = geometry_msgs::msg::Point32();
    node_msg.manip_scale = geometry_msgs::msg::Vector3();
    node_msg.manip_orientation = geometry_msgs::msg::Quaternion();
    node_msg.manip_orientation.w = 1.0;
    return;
  }

  node_msg.manip_center = toPoint32(ellipsoid.center);
  node_msg.manip_scale = toVector3(ellipsoid.singular_values);
  node_msg.manip_orientation = toQuaternion(ellipsoid.principal_directions);
}

template <typename AngleVector, typename JointPositionsVector>
inline void fillNodeKinematicsFields(
    ais_gng_feature_msgs::msg::TopologicalNodeFeature &node_msg,
    const AngleVector &weight_angle,
    const JointPositionsVector &joint_positions,
    const Eigen::Vector3f &eef_position,
    const Eigen::Quaternionf &eef_orientation) {
  node_msg.weight_angle.clear();
  node_msg.weight_angle.reserve(static_cast<std::size_t>(weight_angle.size()));
  for (int i = 0; i < weight_angle.size(); ++i) {
    node_msg.weight_angle.push_back(static_cast<float>(weight_angle[i]));
  }
  node_msg.ee_pose = toPose(eef_position, eef_orientation);
  node_msg.joint_positions = toPoint32Vector(joint_positions);
}

inline nlohmann::json manipulabilityToJson(
    const Manipulability::ManipulabilityEllipsoid &ellipsoid,
    const Eigen::Vector3d &center, bool is_goal = false) {
  nlohmann::json out;
  out["manipValid"] = ellipsoid.valid;
  out["isGoal"] = is_goal;
  out["manipValue"] = ellipsoid.manipulability;
  out["manipConditionNumber"] = ellipsoid.condition_number;
  out["manipCenter"] = {center.x(), center.y(), center.z()};
  if (ellipsoid.valid) {
    out["manipScale"] = {ellipsoid.singular_values.x(),
                          ellipsoid.singular_values.y(),
                          ellipsoid.singular_values.z()};
    const auto q = toQuaternion(ellipsoid.principal_directions);
    out["manipOrientation"] = {q.x, q.y, q.z, q.w};
  } else {
    out["manipScale"] = {0.0, 0.0, 0.0};
    out["manipOrientation"] = {0.0, 0.0, 0.0, 1.0};
  }
  return out;
}

inline void appendLinkManipulabilityJson(
    nlohmann::json &links_json, const std::string &link_name,
    const Eigen::Isometry3d &base_tf,
    const Eigen::Isometry3d &link_tf,
    const Manipulability::ManipulabilityEllipsoid &ellipsoid) {
  const Eigen::Vector3d world_center = link_tf.translation();
  const Eigen::Quaterniond world_orientation = ellipsoid.valid
      ? Eigen::Quaterniond(ellipsoid.principal_directions)
      : Eigen::Quaterniond::Identity();

  const Eigen::Quaterniond base_q(base_tf.rotation());
  const Eigen::Quaterniond inv_base_q = base_q.conjugate();
  const Eigen::Vector3d local_center = inv_base_q * (world_center - base_tf.translation());
  Eigen::Quaterniond local_orientation = inv_base_q * world_orientation;
  local_orientation.normalize();

  nlohmann::json entry;
  entry["linkName"] = link_name;
  entry["manipValid"] = ellipsoid.valid;
  entry["manipValue"] = ellipsoid.manipulability;
  entry["manipConditionNumber"] = ellipsoid.condition_number;
  entry["manipCenter"] = {local_center.x(), local_center.y(), local_center.z()};
  if (ellipsoid.valid) {
    entry["manipScale"] = {ellipsoid.singular_values.x(),
                           ellipsoid.singular_values.y(),
                           ellipsoid.singular_values.z()};
    entry["manipOrientation"] = {local_orientation.x(), local_orientation.y(),
                                 local_orientation.z(), local_orientation.w()};
  } else {
    entry["manipScale"] = {0.0, 0.0, 0.0};
    entry["manipOrientation"] = {0.0, 0.0, 0.0, 1.0};
  }
  links_json.push_back(std::move(entry));
}

} // namespace robot_sim::common
