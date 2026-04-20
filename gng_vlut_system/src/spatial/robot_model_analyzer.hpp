#pragma once

#include "description/robot_model.hpp"
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace GNG {
namespace Analysis {

struct ConsistencyReport {
  std::string link_name;
  bool matches;
  std::string reason;
};

/**
 * @brief Utility to analyze and compare robot model components (Visual vs
 * Collision).
 */
class RobotModelAnalyzer {
public:
  /**
   * @brief Checks if visual and collision geometries match for all links in the
   * model.
   */
  static std::vector<ConsistencyReport>
  checkConsistency(const simulation::RobotModel &model);

  /**
   * @brief Computes Signed Distance from a point to a geometry.
   * @param pt Point in World Space.
   * @param geom Geometry properties.
   * @param transform World transform of the geometry.
   * @return Negative if inside, positive if outside.
   */
  static float computeSDF(const Eigen::Vector3f &pt,
                          const simulation::Geometry &geom,
                          const Eigen::Isometry3d &transform);

  /**
   * @brief Formats consistency reports into a human-readable string.
   */
  static std::string
  formatConsistencyReport(const std::vector<ConsistencyReport> &reports);
};

} // namespace Analysis
} // namespace GNG
