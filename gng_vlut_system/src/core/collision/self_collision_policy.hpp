#pragma once

#include "robot_model/robot_model.hpp"
#include <map>
#include <iterator>
#include <set>
#include <string>
#include <utility>

namespace simulation {

// 同一剛体と、可動関節の基幹リンク対の構造的な接触除外。
// 固定アクセサリへの隣接除外の拡張なし。
inline std::set<std::pair<std::string, std::string>>
collect_self_collision_exclusions(const RobotModel &model) {
  std::map<std::string, std::string> groups;
  for (const auto &entry : model.getLinks()) groups[entry.first] = entry.first;
  const auto find_group = [&groups](std::string name) {
    while (groups.at(name) != name) name = groups.at(name);
    return name;
  };
  for (const auto &entry : model.getJoints()) {
    const auto &joint = entry.second;
    if (joint.type == kinematics::JointType::Fixed) {
      groups.at(find_group(joint.child_link)) = find_group(joint.parent_link);
    }
  }
  const auto ordered_pair = [](std::string first, std::string second) {
    if (second < first) std::swap(first, second);
    return std::make_pair(first, second);
  };
  std::set<std::pair<std::string, std::string>> result;
  for (auto first = groups.begin(); first != groups.end(); ++first) {
    for (auto second = std::next(first); second != groups.end(); ++second) {
      const auto first_group = find_group(first->first);
      const auto second_group = find_group(second->first);
      if (first_group == second_group) {
        result.insert(ordered_pair(first->first, second->first));
      }
    }
  }
  std::map<std::string, const JointProperties *> parent_joints;
  std::map<std::string, std::vector<const JointProperties *>> child_joints;
  for (const auto &entry : model.getJoints()) {
    parent_joints[entry.second.child_link] = &entry.second;
    child_joints[entry.second.parent_link].push_back(&entry.second);
  }
  const auto has_shape = [&model](const std::string &name) {
    const auto *link = model.getLink(name);
    return link && !link->collisions.empty();
  };
  for (const auto &entry : model.getJoints()) {
    const auto &joint = entry.second;
    if (joint.type == kinematics::JointType::Fixed) continue;
    std::string parent = joint.parent_link;
    while (!has_shape(parent)) {
      const auto found = parent_joints.find(parent);
      if (found == parent_joints.end() || found->second->type != kinematics::JointType::Fixed) break;
      parent = found->second->parent_link;
    }
    if (!has_shape(parent)) continue;
    std::vector<std::string> children{joint.child_link};
    for (std::size_t idx = 0; idx < children.size(); ++idx) {
      const std::string child = children[idx];
      if (has_shape(child)) {
        result.insert(ordered_pair(parent, child));
        continue;
      }
      for (const auto *next : child_joints[child]) {
        if (next->type == kinematics::JointType::Fixed) children.push_back(next->child_link);
      }
    }
  }
  return result;
}

} // namespace simulation
