#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace grasping_system::core
{

struct GripperSpec
{
  static constexpr double kDefaultMaxApproachAngleRad = 0.7853981633974483;

  std::string gripper_id;
  std::string eef_link;

  double min_width{0.0};
  double max_width{0.0};
  double finger_length{0.0};
  double finger_thickness{0.0};
  double approach_offset{0.0};
  double max_approach_angle_rad{kDefaultMaxApproachAngleRad};
  geometry_msgs::msg::Pose tool_in_eef{};

  bool valid() const noexcept
  {
    return !gripper_id.empty() &&
           max_width > 0.0 &&
           max_width >= min_width;
  }

  double clampWidth(double width) const noexcept
  {
    return std::clamp(width, min_width, max_width);
  }

  bool canGripWidth(double width, double tolerance = 0.0) const noexcept
  {
    return width >= (min_width - tolerance) && width <= (max_width + tolerance);
  }

  bool canGripSpan(double span, double tolerance = 0.0) const noexcept
  {
    return canGripWidth(span, tolerance);
  }

  double openingMargin(double width) const noexcept
  {
    if (max_width <= 0.0) {
      return 0.0;
    }
    return (max_width - width) / max_width;
  }

  double normalizedSpan(double span) const noexcept
  {
    if (max_width <= min_width) {
      return 0.0;
    }
    const double clamped = clampWidth(span);
    return (clamped - min_width) / (max_width - min_width);
  }
};

}  // namespace grasping_system::core
