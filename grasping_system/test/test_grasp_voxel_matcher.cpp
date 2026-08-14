#include <candidate/grasp_voxel_matcher.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace
{

using grasping_system::candidate::GraspVoxelMatchConfig;
using grasping_system::candidate::GraspVoxelMatcher;
using grasping_system::candidate::GraspVoxelTemplate;
using grasping_system::candidate::OccupiedVoxelGrid;
using grasping_system::candidate::VoxelGridGeometry;
using grasping_system::candidate::VoxelIndex;

void expect(bool condition, const char *message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

GraspVoxelTemplate makeTemplate()
{
  GraspVoxelTemplate grasp_template;
  grasp_template.required_occupied = {
    {-1.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
  };
  grasp_template.optional_not_sole_support = {{0.0, 0.0, 0.0}};
  grasp_template.required_empty = {{0.0, 1.0, 0.0}};
  return grasp_template;
}

GraspVoxelMatchConfig strictConfig()
{
  GraspVoxelMatchConfig config;
  config.minimum_required_occupancy_ratio = 1.0;
  config.minimum_required_hits = 3;
  config.minimum_outside_undersize_hits = 1;
  config.maximum_forbidden_hits = 0;
  config.maximum_anchor_voxels = 100;
  config.maximum_candidates = 10;
  return config;
}

}  // namespace

int main()
{
  constexpr double kPi = 3.14159265358979323846;
  const VoxelGridGeometry geometry{1.0, Eigen::Vector3d::Zero()};
  const auto compiled = GraspVoxelMatcher::compile(
    makeTemplate(), {Eigen::Quaterniond::Identity()}, geometry.voxel_size);

  OccupiedVoxelGrid target(geometry);
  target.add(VoxelIndex{9, 0, 0});
  target.add(VoxelIndex{10, 0, 0});
  target.add(VoxelIndex{11, 0, 0});
  OccupiedVoxelGrid collision_free(geometry);

  const auto accepted = GraspVoxelMatcher::match(
    target, collision_free, compiled, strictConfig());
  expect(accepted.candidates.size() == 1U, "expected one exact grasp voxel match");
  expect(accepted.candidates.front().anchor == VoxelIndex{10, 0, 0}, "anchor mismatch");
  expect(accepted.candidates.front().required_hits == 3U, "required hit count mismatch");
  expect(
    accepted.candidates.front().outside_undersize_hits == 2U,
    "outside-undersize support was not counted");

  OccupiedVoxelGrid undersize_only(geometry);
  undersize_only.add(VoxelIndex{10, 0, 0});
  auto permissive = strictConfig();
  permissive.minimum_required_occupancy_ratio = 0.0;
  permissive.minimum_required_hits = 1;
  const auto undersize_rejected = GraspVoxelMatcher::match(
    undersize_only, collision_free, compiled, permissive);
  expect(undersize_rejected.candidates.empty(), "undersize-only match was accepted");
  expect(
    undersize_rejected.rejected_undersize_only == 1U,
    "undersize-only rejection was not reported");

  OccupiedVoxelGrid collision_blocked(geometry);
  collision_blocked.add(VoxelIndex{10, 1, 0});
  const auto collision_rejected = GraspVoxelMatcher::match(
    target, collision_blocked, compiled, strictConfig());
  expect(collision_rejected.candidates.empty(), "forbidden occupancy was accepted");
  expect(
    collision_rejected.rejected_forbidden_occupancy == 1U,
    "forbidden occupancy rejection was not reported");

  OccupiedVoxelGrid rotated_target(geometry);
  rotated_target.add(VoxelIndex{10, -1, 0});
  rotated_target.add(VoxelIndex{10, 0, 0});
  rotated_target.add(VoxelIndex{10, 1, 0});
  const Eigen::Quaterniond yaw_90(Eigen::AngleAxisd(kPi * 0.5, Eigen::Vector3d::UnitZ()));
  const auto rotated_template = GraspVoxelMatcher::compile(
    makeTemplate(), {Eigen::Quaterniond::Identity(), yaw_90}, geometry.voxel_size);
  const auto rotated_match = GraspVoxelMatcher::match(
    rotated_target, collision_free, rotated_template, strictConfig());
  expect(rotated_match.candidates.size() == 1U, "rotated grasp was not isolated");
  expect(rotated_match.candidates.front().orientation_index == 1U, "rotation index mismatch");

  return 0;
}
