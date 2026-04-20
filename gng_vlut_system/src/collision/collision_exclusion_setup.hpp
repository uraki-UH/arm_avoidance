#pragma once

#include "collision/geometric_self_collision_checker.hpp"
#include <string>
#include <vector>

namespace simulation {

/**
 * @brief Helper to setup default collision exclusions (e.g., adjacent links).
 * This replaces the legacy version that was in simulation/world/.
 */
inline void setupDefaultCollisionExclusions(GeometricSelfCollisionChecker &checker) {
  // Most exclusions are already handled within GeometricSelfCollisionChecker 
  // via its constructor using the joint connectivity. 
  // This function can be used to add manual exclusions if needed.
  
  // Example for common naming patterns if needed:
  // checker.addCollisionExclusion("base_link", "link_1");
  // etc.
}

} // namespace simulation
