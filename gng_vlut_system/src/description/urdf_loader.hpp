#pragma once

#include "description/robot_model.hpp"
#include <string>

namespace simulation {

RobotModel loadRobotFromUrdf(const std::string& urdf_path,
                             const std::string& resource_root_dir = "",
                             const std::string& mesh_root_dir = "");

} // namespace simulation
