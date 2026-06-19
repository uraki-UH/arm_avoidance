#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nlohmann/json.hpp>

#include "core/metrics/manipulability.hpp"
#include <ais_gng_msgs/msg/topological_node.hpp>

namespace robot_sim::common {

inline geometry_msgs::msg::Point32 toPoint32(const Eigen::Vector3d &v) {
  geometry_msgs::msg::Point32 p;
  p.x = static_cast<float>(v.x());
  p.y = static_cast<float>(v.y());
  p.z = static_cast<float>(v.z());
  return p;
}

inline geometry_msgs::msg::Vector3 toVector3(const Eigen::Vector3d &v) {
  geometry_msgs::msg::Vector3 out;
  out.x = v.x();
  out.y = v.y();
  out.z = v.z();
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
    ais_gng_msgs::msg::TopologicalNode &node_msg,
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
