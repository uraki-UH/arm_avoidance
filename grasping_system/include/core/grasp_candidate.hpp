#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <string>

namespace grasping_system::core
{

struct GraspCandidate
{
  std::string object_id;
  std::string eef_link;
  geometry_msgs::msg::Pose grasp_in_eef{};
  geometry_msgs::msg::Pose pregrasp_in_eef{};
  geometry_msgs::msg::Pose approach_in_eef{};
  double score{0.0};
  double clearance{0.0};
  bool collision_free{false};
};

}  // namespace grasping_system::core
