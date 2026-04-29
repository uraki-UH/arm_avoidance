#include "analysis/robot_model_analyzer.hpp"
#include <iomanip>
#include <sstream>

namespace GNG {
namespace Analysis {

std::vector<ConsistencyReport>
RobotModelAnalyzer::checkConsistency(const simulation::RobotModel &model) {
  std::vector<ConsistencyReport> reports;
  for (const auto &kv : model.getLinks()) {
    const auto &link = kv.second;
    if (link.visuals.empty() || link.collisions.empty()) {
      reports.push_back(
          {link.name, false, "Missing visual or collision data."});
      continue;
    }

    // Compare the first visual with the first collision
    const auto &vis = link.visuals[0];
    const auto &col = link.collisions[0];

    bool matches = true;
    std::stringstream ss;

    if (vis.geometry.type != col.geometry.type) {
      matches = false;
      ss << "Geometry type mismatch: Visual=" << (int)vis.geometry.type
         << " vs Collision=" << (int)col.geometry.type << ". ";
    } else {
      if ((vis.geometry.size - col.geometry.size).norm() > 1e-5) {
        matches = false;
        ss << "Geometry size mismatch: Visual=["
           << vis.geometry.size.transpose() << "] vs Collision=["
           << col.geometry.size.transpose() << "]. ";
      }
    }

    // Check relative origin (origin in URDF is relative to parent link frame)
    // Note: RobotModel stores Isometry3d origin for both.
    Eigen::Vector3d v_pos = vis.origin.translation();
    Eigen::Vector3d c_pos = col.origin.translation();
    if ((v_pos - c_pos).norm() > 1e-5) {
      matches = false;
      ss << "Origin translation mismatch: Visual=[" << v_pos.transpose()
         << "] vs Collision=[" << c_pos.transpose() << "]. ";
    }

    Eigen::Quaterniond v_rot(vis.origin.rotation());
    Eigen::Quaterniond c_rot(col.origin.rotation());
    if (v_rot.angularDistance(c_rot) > 1e-5) {
      matches = false;
      ss << "Origin rotation mismatch. ";
    }

    reports.push_back({link.name, matches, matches ? "OK" : ss.str()});
  }
  return reports;
}

float RobotModelAnalyzer::computeSDF(const Eigen::Vector3f &pt,
                                     const simulation::Geometry &geom,
                                     const Eigen::Isometry3d &transform) {
  Eigen::Vector3d pt_d = pt.cast<double>();
  // Transform point to local geometry frame
  Eigen::Vector3d p = transform.inverse() * pt_d;

  if (geom.type == simulation::GeometryType::BOX) {
    Eigen::Vector3d b = geom.size * 0.5;
    Eigen::Vector3d d = p.cwiseAbs() - b;
    return (float)((d.cwiseMax(0.0)).norm() +
                   std::min(std::max(d.x(), std::max(d.y(), d.z())), 0.0));
  } else if (geom.type == simulation::GeometryType::CYLINDER) {
    // URDF Cylinder: size.x = radius, size.y = length, aligned along local Z
    double r = geom.size.x();
    double h = geom.size.y() * 0.5;
    double r_dist = Eigen::Vector2d(p.x(), p.y()).norm();
    double dr = r_dist - r;
    double dz = std::abs(p.z()) - h;

    return (float)(std::sqrt(std::max(dr, 0.0) * std::max(dr, 0.0) +
                             std::max(dz, 0.0) * std::max(dz, 0.0)) +
                   std::min(std::max(dr, dz), 0.0));
  } else if (geom.type == simulation::GeometryType::SPHERE) {
    return (float)(p.norm() - geom.size.x());
  }

  return (float)p.norm(); // Fallback
}

std::string RobotModelAnalyzer::formatConsistencyReport(
    const std::vector<ConsistencyReport> &reports) {
  std::stringstream ss;
  ss << "--- Visual-Collision Consistency Analysis ---\n";
  int fails = 0;
  for (const auto &r : reports) {
    ss << "  [" << (r.matches ? "OK" : "!!") << "] " << std::left
       << std::setw(15) << r.link_name << ": " << r.reason << "\n";
    if (!r.matches)
      fails++;
  }
  if (fails == 0)
    ss << "Result: All clear.\n";
  else
    ss << "Result: " << fails << " inconsistency issues found!\n";
  return ss.str();
}

} // namespace Analysis
} // namespace GNG
