#pragma once

#include "kinematics/kinematic_chain.hpp"
#include "simulation/robot/robot_model.hpp"

#include <string>

namespace simulation {

/**
 * @brief Creates a kinematics::KinematicChain object from a
 * simulation::RobotModel.
 *
 * This function serves as an adapter, translating the general-purpose,
 * tree-like RobotModel structure (loaded from a URDF) into the linear chain
 * representation required by the KinematicChain class for kinematic
 * calculations.
 *
 * @param model The RobotModel object to convert.
 * @return A configured kinematics::KinematicChain object.
 * @throws std::runtime_error if the model is empty or describes a structure
 *         that cannot be represented as a single kinematic chain.
 */
kinematics::KinematicChain
createKinematicChainFromModel(const RobotModel &model,
                              const std::string &end_effector_name = "",
                              const Eigen::Vector3d &base_position = Eigen::Vector3d::Zero());

} // namespace simulation
