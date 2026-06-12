#pragma once

#include <core/grasp_candidate.hpp>
#include <core/grasp_types.hpp>

#include <string>
#include <vector>

namespace grasping_system::rigid
{

struct RigidGraspModel
{
  core::GraspObject object;
  std::string visual_mesh_resource;
  std::string collision_mesh_resource;
  std::vector<core::GraspCandidate> candidates;
};

}  // namespace grasping_system::rigid
