#pragma once

#include "description/robot_model.hpp"
#include <string>

namespace simulation {

RobotModel loadRobotFromUrdf(const std::string& urdf_path);

} // namespace simulation
