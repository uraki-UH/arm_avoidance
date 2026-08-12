#pragma once

#include <graph/grasp_graph_model.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace grasping_system::candidate
{

enum class OccupancyRule
{
  kMustContainOccupancy,
  kMustBeEmpty,
  kOptional,
  kOptionalNotSoleSupport
};

enum class OccupancyViolationReason
{
  kRequiredRegionEmpty,
  kForbiddenRegionOccupied,
  kOptionalRegionIsSoleSupport
};

inline const char *occupancyRuleName(OccupancyRule rule) noexcept
{
  switch (rule) {
    case OccupancyRule::kMustContainOccupancy:
      return "required_occupied";
    case OccupancyRule::kMustBeEmpty:
      return "required_empty";
    case OccupancyRule::kOptional:
      return "optional";
    case OccupancyRule::kOptionalNotSoleSupport:
      return "optional_not_sole_support";
  }
  return "optional";
}

inline OccupancyRule parseOccupancyRule(const std::string &name)
{
  if (name == "required_occupied") {
    return OccupancyRule::kMustContainOccupancy;
  }
  if (name == "required_empty") {
    return OccupancyRule::kMustBeEmpty;
  }
  if (name == "optional") {
    return OccupancyRule::kOptional;
  }
  if (name == "optional_not_sole_support") {
    return OccupancyRule::kOptionalNotSoleSupport;
  }
  throw std::invalid_argument("unsupported occupancy rule: " + name);
}

struct OccupancyRegionConstraint
{
  std::string id;
  OccupancyRule rule{OccupancyRule::kOptional};
  std::vector<Eigen::Vector3d> sample_points_in_eef;
  std::size_t occupancy_threshold{1};
};

struct OccupancyRegionResult
{
  std::string id;
  OccupancyRule rule{OccupancyRule::kOptional};
  std::size_t sample_count{0};
  std::size_t occupied_sample_count{0};
  bool occupied{false};
  bool satisfied{true};
};

struct OccupancyViolation
{
  OccupancyViolationReason reason{OccupancyViolationReason::kRequiredRegionEmpty};
  std::string region_id;
};

struct GraspPoseOccupancyEvaluation
{
  bool accepted{false};
  bool has_supporting_occupancy{false};
  std::vector<OccupancyRegionResult> regions;
  std::vector<OccupancyViolation> violations;
};

inline OccupancyRegionConstraint makeOccupancyRegionFromGraph(
  std::string id, OccupancyRule rule, const graph::GraspGraphModel &graph,
  std::size_t occupancy_threshold = 1)
{
  OccupancyRegionConstraint region;
  region.id = std::move(id);
  region.rule = rule;
  region.occupancy_threshold = occupancy_threshold;
  region.sample_points_in_eef.reserve(graph.nodes().size());
  for (const auto &node : graph.nodes()) {
    if (!node.active) {
      continue;
    }
    region.sample_points_in_eef.emplace_back(
      node.pose_in_object.position.x,
      node.pose_in_object.position.y,
      node.pose_in_object.position.z);
  }
  return region;
}

class GraspPoseOccupancyEvaluator
{
public:
  using OccupancyQuery = std::function<bool(const Eigen::Vector3d &)>;

  explicit GraspPoseOccupancyEvaluator(
    std::vector<OccupancyRegionConstraint> regions)
  : regions_(std::move(regions))
  {
    validateRegions();
  }

  const std::vector<OccupancyRegionConstraint> &regions() const noexcept
  {
    return regions_;
  }

  GraspPoseOccupancyEvaluation evaluate(
    const geometry_msgs::msg::Pose &eef_pose_in_reference,
    const OccupancyQuery &is_occupied) const
  {
    return evaluate(toIsometry(eef_pose_in_reference), is_occupied);
  }

  GraspPoseOccupancyEvaluation evaluate(
    const Eigen::Isometry3d &eef_pose_in_reference,
    const OccupancyQuery &is_occupied) const
  {
    if (!is_occupied) {
      throw std::invalid_argument("occupancy query must be callable");
    }
    if (!eef_pose_in_reference.matrix().allFinite()) {
      throw std::invalid_argument("candidate end-effector pose must be finite");
    }

    GraspPoseOccupancyEvaluation evaluation;
    evaluation.regions.reserve(regions_.size());

    for (const auto &region : regions_) {
      OccupancyRegionResult result;
      result.id = region.id;
      result.rule = region.rule;
      result.sample_count = region.sample_points_in_eef.size();

      for (const auto &sample_in_eef : region.sample_points_in_eef) {
        if (is_occupied(eef_pose_in_reference * sample_in_eef)) {
          ++result.occupied_sample_count;
        }
      }
      result.occupied = result.occupied_sample_count >= region.occupancy_threshold;

      switch (region.rule) {
        case OccupancyRule::kMustContainOccupancy:
          result.satisfied = result.occupied;
          if (!result.satisfied) {
            evaluation.violations.push_back(
              {OccupancyViolationReason::kRequiredRegionEmpty, region.id});
          }
          break;
        case OccupancyRule::kMustBeEmpty:
          result.satisfied = !result.occupied;
          if (!result.satisfied) {
            evaluation.violations.push_back(
              {OccupancyViolationReason::kForbiddenRegionOccupied, region.id});
          }
          break;
        case OccupancyRule::kOptional:
          evaluation.has_supporting_occupancy =
            evaluation.has_supporting_occupancy || result.occupied;
          break;
        case OccupancyRule::kOptionalNotSoleSupport:
          break;
      }

      if (region.rule == OccupancyRule::kMustContainOccupancy && result.occupied) {
        evaluation.has_supporting_occupancy = true;
      }
      evaluation.regions.push_back(std::move(result));
    }

    if (!evaluation.has_supporting_occupancy) {
      for (auto &result : evaluation.regions) {
        if (result.rule != OccupancyRule::kOptionalNotSoleSupport || !result.occupied) {
          continue;
        }
        result.satisfied = false;
        evaluation.violations.push_back(
          {OccupancyViolationReason::kOptionalRegionIsSoleSupport, result.id});
      }
    }

    evaluation.accepted = evaluation.violations.empty();
    return evaluation;
  }

private:
  static Eigen::Isometry3d toIsometry(const geometry_msgs::msg::Pose &pose)
  {
    const Eigen::Vector3d translation(
      pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond orientation(
      pose.orientation.w, pose.orientation.x,
      pose.orientation.y, pose.orientation.z);
    if (!translation.allFinite() || !orientation.coeffs().allFinite() ||
      orientation.squaredNorm() <= 1e-12)
    {
      throw std::invalid_argument(
              "candidate end-effector pose must be finite and its orientation non-zero");
    }
    orientation.normalize();

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = orientation.toRotationMatrix();
    transform.translation() = translation;
    return transform;
  }

  void validateRegions() const
  {
    if (regions_.empty()) {
      throw std::invalid_argument("at least one occupancy region is required");
    }
    for (std::size_t index = 0; index < regions_.size(); ++index) {
      const auto &region = regions_[index];
      if (region.id.empty()) {
        throw std::invalid_argument("occupancy region id must not be empty");
      }
      if (region.sample_points_in_eef.empty()) {
        throw std::invalid_argument(
                "occupancy region '" + region.id + "' must contain sample points");
      }
      if (region.occupancy_threshold == 0 ||
        region.occupancy_threshold > region.sample_points_in_eef.size())
      {
        throw std::invalid_argument(
                "occupancy region '" + region.id + "' has an invalid threshold");
      }
      for (const auto &sample : region.sample_points_in_eef) {
        if (!sample.allFinite()) {
          throw std::invalid_argument(
                  "occupancy region '" + region.id + "' contains a non-finite point");
        }
      }
      for (std::size_t other = index + 1; other < regions_.size(); ++other) {
        if (region.id == regions_[other].id) {
          throw std::invalid_argument("occupancy region ids must be unique: " + region.id);
        }
      }
    }
  }

  std::vector<OccupancyRegionConstraint> regions_;
};

}  // namespace grasping_system::candidate
