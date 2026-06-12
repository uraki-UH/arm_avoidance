#pragma once

#include <core/grasp_geometry.hpp>
#include <core/grasp_types.hpp>

#include <memory>
#include <string>

namespace grasping_system::core
{

struct GraspObjectModel
{
  GraspObject object;
  std::unique_ptr<GraspGeometryModel> geometry;

  bool valid() const noexcept
  {
    return object.valid() && static_cast<bool>(geometry);
  }
};

}  // namespace grasping_system::core
