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
using grasping_system::candidate::IncrementalGraspVoxelMatcher;
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

GraspVoxelTemplate makeSweepTemplate()
{
  GraspVoxelTemplate grasp_template;
  grasp_template.required_occupied = {{0.0, 0.0, 0.0}};
  // The closing finger may meet the object inside the initial open gap.
  grasp_template.collision_exempt = {{0.0, 1.0, 0.0}};
  grasp_template.required_empty = {
    {0.0, 1.0, 0.0},  // swept finger location inside the allowed gap
    {0.0, 2.0, 0.0},  // swept body location outside the allowed gap
  };
  return grasp_template;
}

GraspVoxelTemplate makeOpposingContactTemplate()
{
  GraspVoxelTemplate grasp_template;
  grasp_template.required_occupied = {{0.0, 0.0, 0.0}};
  grasp_template.opposing_contact_pairs.push_back({
    {{0.0, 1.0, 0.0}}, {{0.0, -1.0, 0.0}}});
  return grasp_template;
}

GraspVoxelTemplate makeLateralContinuationTemplate()
{
  GraspVoxelTemplate grasp_template;
  grasp_template.required_occupied = {{0.0, 0.0, 0.0}};
  grasp_template.lateral_continuation_pairs = {
    {{{1.0, 0.0, 0.0}}, {{-1.0, 0.0, 0.0}}},
    {{{0.0, 0.0, 1.0}}, {{0.0, 0.0, -1.0}}}};
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

GraspVoxelMatchConfig oneHitConfig()
{
  GraspVoxelMatchConfig config;
  config.minimum_required_occupancy_ratio = 1.0;
  config.minimum_required_hits = 1;
  config.minimum_outside_undersize_hits = 0;
  config.minimum_contact_hits_per_side = 0;
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

  IncrementalGraspVoxelMatcher incremental(geometry, compiled, strictConfig());
  incremental.reset(target, collision_free);
  const auto incremental_accepted = incremental.match();
  expect(
    incremental_accepted.candidates.size() == accepted.candidates.size(),
    "incremental matcher did not reproduce the initial full match");
  expect(
    incremental_accepted.candidates.front().anchor == VoxelIndex{10, 0, 0},
    "incremental matcher anchor mismatch");
  incremental.applyTargetDelta(VoxelIndex{11, 0, 0}, true, false);
  expect(
    incremental.match().candidates.empty(),
    "removed required voxel was not reflected in incremental state");
  incremental.applyTargetDelta(VoxelIndex{11, 0, 0}, false, true);
  expect(
    incremental.match().candidates.size() == 1U,
    "restored required voxel was not reflected in incremental state");
  incremental.applyCollisionDelta(VoxelIndex{10, 1, 0}, false, true);
  expect(
    incremental.match().candidates.empty(),
    "forbidden voxel was not reflected in incremental state");
  incremental.applyCollisionDelta(VoxelIndex{10, 1, 0}, true, false);
  expect(
    incremental.match().candidates.size() == 1U,
    "cleared forbidden voxel was not reflected in incremental state");

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

  const auto compiled_sweep = GraspVoxelMatcher::compile(
    makeSweepTemplate(), {Eigen::Quaterniond::Identity()}, geometry.voxel_size);
  OccupiedVoxelGrid sweep_target(geometry);
  sweep_target.add(VoxelIndex{10, 0, 0});
  OccupiedVoxelGrid allowed_sweep_contact(geometry);
  allowed_sweep_contact.add(VoxelIndex{10, 0, 0});
  allowed_sweep_contact.add(VoxelIndex{10, 1, 0});
  const auto sweep_contact_accepted = GraspVoxelMatcher::match(
    sweep_target, allowed_sweep_contact, compiled_sweep,
    oneHitConfig());
  expect(
    sweep_contact_accepted.candidates.size() == 1U,
    "swept contact inside the object volume was rejected");
  allowed_sweep_contact.add(VoxelIndex{10, 2, 0});
  const auto sweep_body_rejected = GraspVoxelMatcher::match(
    sweep_target, allowed_sweep_contact, compiled_sweep,
    oneHitConfig());
  expect(
    sweep_body_rejected.candidates.empty(),
    "swept body occupancy outside the object volume was accepted");

  const auto compiled_opposing_contacts = GraspVoxelMatcher::compile(
    makeOpposingContactTemplate(), {Eigen::Quaterniond::Identity()}, geometry.voxel_size);
  OccupiedVoxelGrid opposing_target(geometry);
  opposing_target.add(VoxelIndex{10, 0, 0});
  opposing_target.add(VoxelIndex{10, 1, 0});
  opposing_target.add(VoxelIndex{10, -1, 0});
  auto contact_config = oneHitConfig();
  contact_config.minimum_contact_hits_per_side = 1;
  const auto opposing_contact_accepted = GraspVoxelMatcher::match(
    opposing_target, collision_free, compiled_opposing_contacts, contact_config);
  expect(
    opposing_contact_accepted.candidates.size() == 1U,
    "opposing contacts did not produce a grasp match");
  OccupiedVoxelGrid one_sided_target(geometry);
  one_sided_target.add(VoxelIndex{10, 0, 0});
  one_sided_target.add(VoxelIndex{10, 1, 0});
  const auto one_sided_rejected = GraspVoxelMatcher::match(
    one_sided_target, collision_free, compiled_opposing_contacts, contact_config);
  expect(one_sided_rejected.candidates.empty(), "one-sided contact was accepted");
  expect(
    one_sided_rejected.rejected_missing_opposing_contact > 0U,
    "one-sided contact rejection was not reported");

  const auto compiled_lateral_continuation = GraspVoxelMatcher::compile(
    makeLateralContinuationTemplate(), {Eigen::Quaterniond::Identity()}, geometry.voxel_size);
  auto lateral_config = oneHitConfig();
  lateral_config.maximum_anchor_voxels = 1;
  lateral_config.maximum_lateral_continuation_axes = 1;
  OccupiedVoxelGrid rod_target(geometry);
  rod_target.add(VoxelIndex{10, 0, 0});
  rod_target.add(VoxelIndex{9, 0, 0});
  rod_target.add(VoxelIndex{11, 0, 0});
  const auto rod_accepted = GraspVoxelMatcher::match(
    rod_target, collision_free, compiled_lateral_continuation, lateral_config);
  expect(rod_accepted.candidates.size() == 1U, "one-axis object continuation was rejected");
  OccupiedVoxelGrid sheet_target(geometry);
  sheet_target.add(VoxelIndex{10, 0, 0});
  sheet_target.add(VoxelIndex{9, 0, 0});
  sheet_target.add(VoxelIndex{11, 0, 0});
  sheet_target.add(VoxelIndex{10, 0, -1});
  sheet_target.add(VoxelIndex{10, 0, 1});
  const auto sheet_rejected = GraspVoxelMatcher::match(
    sheet_target, collision_free, compiled_lateral_continuation, lateral_config);
  expect(sheet_rejected.candidates.empty(), "two-axis sheet continuation was accepted");
  expect(
    sheet_rejected.rejected_lateral_continuation == 1U,
    "sheet continuation rejection was not reported");

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
