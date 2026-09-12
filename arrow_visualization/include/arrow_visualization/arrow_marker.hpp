#pragma once

#include <array>
#include <algorithm>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <string>

namespace arrow_visualization {
struct arrow_style {
  std::string anchor = "tail";
  double length = 0.08;
  double shaft_diameter = 0.008;
  double head_length = 0.02;
  double head_diameter = 0.016;
  std_msgs::msg::ColorRGBA color;
  arrow_style() { color.g = 0.8F; color.b = 1.0F; color.a = 1.0F; }
};

// 方向だけの入力に補助軸を付加しない、標準Markerの始点終点方式
// 不正入力は同一IDのDELETEで旧表示を失効
inline visualization_msgs::msg::Marker make_arrow(
    const std_msgs::msg::Header &header, const std::string &marker_ns, int id,
    const geometry_msgs::msg::Point &position, const geometry_msgs::msg::Vector3 &direction,
    const arrow_style &style) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = marker_ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  const double norm = std::hypot(direction.x, direction.y, direction.z);
  if (!std::isfinite(norm) || norm <= 1e-12 || !std::isfinite(position.x) ||
      !std::isfinite(position.y) || !std::isfinite(position.z)) return marker;
  for (const double value : {style.length, style.shaft_diameter, style.head_length, style.head_diameter}) {
    if (!std::isfinite(value) || value <= 0.0) return marker;
  }
  if (style.head_length > style.length ||
      (style.anchor != "tail" && style.anchor != "tip" && style.anchor != "center")) return marker;
  for (const float value : {style.color.r, style.color.g, style.color.b, style.color.a}) {
    if (!std::isfinite(value) || value < 0.0F || value > 1.0F) return marker;
  }
  const double offset = style.anchor == "tip" ? -style.length : style.anchor == "center" ? -style.length * 0.5 : 0.0;
  geometry_msgs::msg::Point tail = position;
  tail.x += direction.x / norm * offset;
  tail.y += direction.y / norm * offset;
  tail.z += direction.z / norm * offset;
  auto tip = tail;
  tip.x += direction.x / norm * style.length;
  tip.y += direction.y / norm * style.length;
  tip.z += direction.z / norm * style.length;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.points = {tail, tip};
  marker.scale.x = style.shaft_diameter;
  marker.scale.y = style.head_diameter;
  marker.scale.z = style.head_length;
  marker.color = style.color;
  return marker;
}
struct pose_arrow_style {
  arrow_style primary;
  std::string marker_namespace{"pose_axes"};
  std::size_t primary_axis_idx{0};
  double primary_axis_sign{1.0};
  double helper_axis_length_ratio{0.5};
  bool enable_transverse_axes{true};
};

// 完全な姿勢からの主軸・補助2軸生成。補助軸原点は指定位置
inline visualization_msgs::msg::MarkerArray make_pose_arrows(
  const geometry_msgs::msg::PoseArray &pose_array,
  const pose_arrow_style &options = {})
{
  visualization_msgs::msg::MarkerArray out;
  const std::array<const char *, 3> axis_names{{"x", "y", "z"}};
  for (std::size_t pose_idx = 0; pose_idx < pose_array.poses.size(); ++pose_idx) {
    const auto &pose = pose_array.poses[pose_idx];
    const auto &q = pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    const bool has_orientation = std::isfinite(norm) && norm > 1e-12;
    const double x = has_orientation ? q.x / norm : 0.0;
    const double y = has_orientation ? q.y / norm : 0.0;
    const double z = has_orientation ? q.z / norm : 0.0;
    const double w = has_orientation ? q.w / norm : 0.0;
    const std::array<std::array<double, 3>, 3> axes{{
      {{1 - 2 * (y*y + z*z), 2 * (x*y + z*w), 2 * (x*z - y*w)}},
      {{2 * (x*y - z*w), 1 - 2 * (x*x + z*z), 2 * (y*z + x*w)}},
      {{2 * (x*z + y*w), 2 * (y*z - x*w), 1 - 2 * (x*x + y*y)}}}};
    for (std::size_t axis_idx = 0; axis_idx < 3; ++axis_idx) {
      const bool is_primary = axis_idx == options.primary_axis_idx;
      if (!is_primary && !options.enable_transverse_axes) continue;
      auto style = options.primary;
      if (!is_primary) {
        style.anchor = "tail";
        style.length *= options.helper_axis_length_ratio;
        style.head_length *= options.helper_axis_length_ratio;
        style.color.r = axis_idx == 0 ? 1.0F : 0.2F;
        style.color.g = axis_idx == 1 ? 1.0F : 0.2F;
        style.color.b = axis_idx == 2 ? 1.0F : 0.2F;
      }
      geometry_msgs::msg::Vector3 direction;
      if (has_orientation) {
        const double sign = is_primary ? options.primary_axis_sign : 1.0;
        direction.x = axes[axis_idx][0] * sign;
        direction.y = axes[axis_idx][1] * sign;
        direction.z = axes[axis_idx][2] * sign;
      }
      out.markers.push_back(make_arrow(pose_array.header,
        options.marker_namespace + "/" + axis_names[axis_idx], static_cast<int>(pose_idx * 3 + axis_idx),
        pose.position, direction, style));
    }
  }

  return out;
}
}  // arrow_visualization 名前空間
