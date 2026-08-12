#include <candidate/grasp_pose_occupancy_evaluator.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using grasping_system::candidate::GraspPoseOccupancyEvaluator;
using grasping_system::candidate::OccupancyRegionConstraint;
using grasping_system::candidate::OccupancyRule;
using grasping_system::candidate::OccupancyViolationReason;

OccupancyRegionConstraint region(
  const char *id, OccupancyRule rule, const Eigen::Vector3d &sample)
{
  return OccupancyRegionConstraint{id, rule, {sample}, 1};
}

GraspPoseOccupancyEvaluator::OccupancyQuery occupiedAt(
  std::vector<Eigen::Vector3d> occupied_points)
{
  return [points = std::move(occupied_points)](const Eigen::Vector3d &query) {
           for (const auto &point : points) {
             if ((point - query).squaredNorm() < 1e-12) {
               return true;
             }
           }
           return false;
         };
}

bool hasViolation(
  const grasping_system::candidate::GraspPoseOccupancyEvaluation &evaluation,
  OccupancyViolationReason reason)
{
  for (const auto &violation : evaluation.violations) {
    if (violation.reason == reason) {
      return true;
    }
  }
  return false;
}

void expect(bool condition, const char *message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main()
{
  constexpr double kPi = 3.14159265358979323846;
  expect(
    grasping_system::candidate::parseOccupancyRule("required_occupied") ==
    OccupancyRule::kMustContainOccupancy,
    "required_occupied parsing failed");
  expect(
    std::string(grasping_system::candidate::occupancyRuleName(
      OccupancyRule::kOptionalNotSoleSupport)) == "optional_not_sole_support",
    "optional_not_sole_support name failed");

  const std::vector<OccupancyRegionConstraint> all_rules{
    region("grasp_core", OccupancyRule::kMustContainOccupancy, {1.0, 0.0, 0.0}),
    region("finger_body", OccupancyRule::kMustBeEmpty, {0.0, 1.0, 0.0}),
    region("allowed", OccupancyRule::kOptional, {0.0, 0.0, 1.0}),
    region("undersize", OccupancyRule::kOptionalNotSoleSupport, {-1.0, 0.0, 0.0}),
  };
  const GraspPoseOccupancyEvaluator evaluator(all_rules);

  geometry_msgs::msg::Pose pose;
  pose.position.x = 10.0;
  pose.orientation.z = std::sin(kPi / 4.0);
  pose.orientation.w = std::cos(kPi / 4.0);

  const auto accepted = evaluator.evaluate(
    pose, occupiedAt({{10.0, 1.0, 0.0}, {10.0, -1.0, 0.0}}));
  expect(accepted.accepted, "valid candidate was rejected");
  expect(accepted.has_supporting_occupancy, "supporting occupancy was not detected");
  expect(accepted.regions[0].occupied_sample_count == 1U, "required count mismatch");
  expect(accepted.regions[3].occupied_sample_count == 1U, "not-sole count mismatch");

  const auto required_empty = evaluator.evaluate(pose, occupiedAt({}));
  expect(!required_empty.accepted, "empty required region was accepted");
  expect(
    hasViolation(required_empty, OccupancyViolationReason::kRequiredRegionEmpty),
    "required-region violation missing");

  const auto forbidden_occupied = evaluator.evaluate(
    pose, occupiedAt({{9.0, 0.0, 0.0}, {10.0, 1.0, 0.0}}));
  expect(!forbidden_occupied.accepted, "occupied forbidden region was accepted");
  expect(
    hasViolation(forbidden_occupied, OccupancyViolationReason::kForbiddenRegionOccupied),
    "forbidden-region violation missing");

  const GraspPoseOccupancyEvaluator not_sole_evaluator({
    region("undersize", OccupancyRule::kOptionalNotSoleSupport, {1.0, 0.0, 0.0}),
    region("allowed", OccupancyRule::kOptional, {0.0, 1.0, 0.0}),
  });
  geometry_msgs::msg::Pose identity;
  identity.orientation.w = 1.0;

  const auto sole_support = not_sole_evaluator.evaluate(
    identity, occupiedAt({{1.0, 0.0, 0.0}}));
  expect(!sole_support.accepted, "sole not-support occupancy was accepted");
  expect(
    hasViolation(sole_support, OccupancyViolationReason::kOptionalRegionIsSoleSupport),
    "sole-support violation missing");

  const auto supported = not_sole_evaluator.evaluate(
    identity, occupiedAt({{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}}));
  expect(supported.accepted, "not-sole occupancy with optional support was rejected");

  OccupancyRegionConstraint threshold_region;
  threshold_region.id = "noise_tolerant_core";
  threshold_region.rule = OccupancyRule::kMustContainOccupancy;
  threshold_region.sample_points_in_eef = {{0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}};
  threshold_region.occupancy_threshold = 2;
  const GraspPoseOccupancyEvaluator threshold_evaluator({threshold_region});
  expect(
    !threshold_evaluator.evaluate(identity, occupiedAt({{0.0, 0.0, 0.0}})).accepted,
    "occupancy below threshold was accepted");
  expect(
    threshold_evaluator.evaluate(
      identity, occupiedAt({{0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}})).accepted,
    "occupancy at threshold was rejected");

  grasping_system::graph::GraspGraphModel graph;
  grasping_system::graph::GraspGraphNode active_node;
  active_node.id = 1;
  active_node.pose_in_object.position.x = 0.1;
  graph.addNode(active_node);
  grasping_system::graph::GraspGraphNode inactive_node;
  inactive_node.id = 2;
  inactive_node.active = false;
  graph.addNode(inactive_node);
  const auto graph_region = grasping_system::candidate::makeOccupancyRegionFromGraph(
    "graph", OccupancyRule::kMustContainOccupancy, graph);
  expect(graph_region.sample_points_in_eef.size() == 1U, "inactive graph node was included");

  return 0;
}
