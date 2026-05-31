#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <string>

namespace grasping_system::core
{

enum class GraspGeometryKind
{
  kGraph,
  kVoxel,
  kPrimitive
};

class GraspGeometryModel
{
public:
  virtual ~GraspGeometryModel() = default;

  virtual GraspGeometryKind kind() const noexcept = 0;
  virtual std::string typeName() const = 0;
  virtual std::unique_ptr<GraspGeometryModel> clone() const = 0;
};

}  // namespace grasping_system::core
