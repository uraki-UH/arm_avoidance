#pragma once

#include <tinyxml2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot_model/robot_model.hpp"
#include "robot_model/voxel_capsule_packer.hpp"

namespace simulation {

namespace detail {

inline std::string formatDouble(double v) {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(8);
  oss << v;
  return oss.str();
}

inline std::array<double, 3> quaternionToRpy(const Eigen::Quaterniond &q_in) {
  Eigen::Quaterniond q = q_in.normalized();
  const Eigen::Matrix3d R = q.toRotationMatrix();
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double pitch = std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  return {roll, pitch, yaw};
}

inline bool linkIsPrimitiveOnly(const simulation::LinkProperties &link) {
  if (link.collisions.empty()) {
    return false;
  }
  for (const auto &c : link.collisions) {
    if (c.geometry.type == simulation::GeometryType::MESH) {
      return false;
    }
  }
  return true;
}

inline tinyxml2::XMLElement *findLinkElement(tinyxml2::XMLDocument &doc, const std::string &link_name) {
  auto *robot = doc.FirstChildElement("robot");
  if (!robot) {
    return nullptr;
  }
  for (auto *link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
    const char *name = link->Attribute("name");
    if (name && link_name == name) {
      return link;
    }
  }
  return nullptr;
}

inline void removeCollisionChildren(tinyxml2::XMLElement *link) {
  if (!link) {
    return;
  }
  std::vector<tinyxml2::XMLElement *> to_remove;
  for (auto *child = link->FirstChildElement("collision"); child; child = child->NextSiblingElement("collision")) {
    to_remove.push_back(child);
  }
  for (auto *child : to_remove) {
    link->DeleteChild(child);
  }
}

inline tinyxml2::XMLElement *addCollisionElement(tinyxml2::XMLDocument &doc,
                                                 tinyxml2::XMLElement *link,
                                                 const simulation::SimplifiedPrimitive &primitive) {
  if (!link) {
    return nullptr;
  }

  auto *collision = doc.NewElement("collision");
  auto *origin = doc.NewElement("origin");
  origin->SetAttribute("xyz", (formatDouble(primitive.xyz.x()) + " " +
                               formatDouble(primitive.xyz.y()) + " " +
                               formatDouble(primitive.xyz.z())).c_str());
  const auto rpy = quaternionToRpy(primitive.quat);
  origin->SetAttribute("rpy", (formatDouble(rpy[0]) + " " +
                               formatDouble(rpy[1]) + " " +
                               formatDouble(rpy[2])).c_str());
  collision->InsertEndChild(origin);

  auto *geometry = doc.NewElement("geometry");
  if (primitive.type == simulation::SimplifiedPrimitive::Type::Sphere) {
    auto *sphere = doc.NewElement("sphere");
    sphere->SetAttribute("radius", formatDouble(primitive.radius).c_str());
    geometry->InsertEndChild(sphere);
  } else {
    auto *cylinder = doc.NewElement("cylinder");
    cylinder->SetAttribute("radius", formatDouble(primitive.radius).c_str());
    cylinder->SetAttribute("length", formatDouble(std::max(0.0, primitive.length)).c_str());
    geometry->InsertEndChild(cylinder);
  }
  collision->InsertEndChild(geometry);
  link->InsertEndChild(collision);
  return collision;
}

inline std::string serializeDocument(tinyxml2::XMLDocument &doc) {
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return printer.CStr();
}

} // namespace detail

inline std::string buildSimplifiedUrdfXml(
    const std::string &original_xml,
    const simulation::RobotModel &model,
    const std::vector<simulation::VoxelSphereLinkResult> &voxel_sphere_links,
    double voxel_size,
    std::size_t capsule_min_chain_spheres,
    double capsule_axis_ratio_threshold,
    double capsule_radius_cv_threshold) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(original_xml.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to parse original URDF XML for simplification");
  }

  for (const auto &entry : voxel_sphere_links) {
    const auto *link = model.getLink(entry.link_name);
    if (!link) {
      continue;
    }
    if (detail::linkIsPrimitiveOnly(*link)) {
      continue;
    }

    auto primitives = simulation::convertSpheresToPrimitives(
        entry.spheres, voxel_size, capsule_min_chain_spheres,
        capsule_axis_ratio_threshold, capsule_radius_cv_threshold,
        entry.voxel_centers.empty() ? nullptr : &entry.voxel_centers);

    auto *link_el = detail::findLinkElement(doc, entry.link_name);
    if (!link_el) {
      continue;
    }
    detail::removeCollisionChildren(link_el);
    for (const auto &primitive : primitives) {
      detail::addCollisionElement(doc, link_el, primitive);
    }
  }

  return detail::serializeDocument(doc);
}

} // namespace simulation
