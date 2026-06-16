#pragma once

#include <cstddef>
#include <array>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace robot_sim::common::grasp
{

struct PoseAxisMarkerOptions
{
  std::string marker_namespace{"grasp_pose_axes"};
  double primary_axis_length{0.12};
  double helper_axis_length_ratio{0.5};
  double shaft_diameter{0.01};
  double head_diameter{0.02};
  double alpha{1.0};
};

inline visualization_msgs::msg::MarkerArray buildPoseAxisMarkerArray(
  const geometry_msgs::msg::PoseArray &pose_array,
  const PoseAxisMarkerOptions &options = {})
{
  visualization_msgs::msg::MarkerArray out;
  out.markers.reserve(pose_array.poses.size() * 3);

  const std::array<std::array<double, 4>, 3> axis_colors{{
    {{1.0, 0.2, 0.2, 1.0}},
    {{0.2, 0.8, 0.2, 1.0}},
    {{0.2, 0.4, 1.0, 1.0}},
  }};
  const std::array<Eigen::Vector3d, 3> local_axes{{
    Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitZ(),
  }};
  const std::array<const char *, 3> axis_names{{"x", "y", "z"}};

  for (std::size_t pose_index = 0; pose_index < pose_array.poses.size(); ++pose_index) {
    const auto &pose = pose_array.poses[pose_index];
    const Eigen::Quaterniond q(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
    const Eigen::Vector3d origin(
      pose.position.x,
      pose.position.y,
      pose.position.z);

    for (std::size_t axis_index = 0; axis_index < local_axes.size(); ++axis_index) {
      const Eigen::Vector3d direction = (q * local_axes[axis_index]).normalized();
      const double length = options.primary_axis_length *
        (axis_index == 0 ? 1.0 : options.helper_axis_length_ratio);
      Eigen::Vector3d tip;
      if (axis_index == 0) {
        tip = origin + direction * length;
      } else {
        tip = local_axes[axis_index] * length;
      }

      visualization_msgs::msg::Marker marker;
      marker.header = pose_array.header;
      marker.ns = options.marker_namespace + "/" + axis_names[axis_index];
      marker.id = static_cast<int>(pose_index * 3 + axis_index);
      marker.type = axis_index == 0
        ? visualization_msgs::msg::Marker::ARROW
        : visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.points.resize(2);
      marker.points[0].x = axis_index == 0 ? origin.x() : 0.0;
      marker.points[0].y = axis_index == 0 ? origin.y() : 0.0;
      marker.points[0].z = axis_index == 0 ? origin.z() : 0.0;
      marker.points[1].x = tip.x();
      marker.points[1].y = tip.y();
      marker.points[1].z = tip.z();
      if (axis_index == 0) {
        marker.scale.x = length;
        marker.scale.y = options.shaft_diameter * 1.5;
        marker.scale.z = options.head_diameter * 1.5;
      } else {
        marker.scale.x = options.shaft_diameter * 1.2;
        marker.scale.y = options.shaft_diameter * 1.2;
        marker.scale.z = options.shaft_diameter * 1.2;
      }
      marker.color.r = static_cast<float>(axis_colors[axis_index][0]);
      marker.color.g = static_cast<float>(axis_colors[axis_index][1]);
      marker.color.b = static_cast<float>(axis_colors[axis_index][2]);
      marker.color.a = static_cast<float>(options.alpha);
      marker.frame_locked = false;
      if (axis_index != 0) {
        marker.pose.position.x = origin.x();
        marker.pose.position.y = origin.y();
        marker.pose.position.z = origin.z();
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
      }
      out.markers.push_back(std::move(marker));
    }
  }

  return out;
}

}  // namespace robot_sim::common::grasp
