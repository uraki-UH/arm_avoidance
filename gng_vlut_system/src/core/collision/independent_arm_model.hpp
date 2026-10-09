#pragma once

#include "robot_model/robot_model.hpp"
#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace simulation {

// 腕の基部より先の可動リンク集合。基部に固定された外装は胴体側として保持。
inline std::set<std::string> collect_moving_arm_links(
    const RobotModel &model, const std::string &root) {
  if (root.empty() || root == model.getRootLinkName() || !model.getLink(root)) {
    throw std::invalid_argument("Independent arm requires a valid non-body root: " + root);
  }
  std::vector<std::pair<std::string, bool>> pending{{root, false}};
  std::set<std::string> visited;
  std::set<std::string> result;
  for (std::size_t idx = 0; idx < pending.size(); ++idx) {
    const auto [name, has_motion] = pending[idx];
    if (!visited.insert(name).second) throw std::invalid_argument("Cyclic arm topology");
    if (has_motion) result.insert(name);
    for (const auto &entry : model.getJoints()) {
      const auto &joint = entry.second;
      if (joint.parent_link == name) {
        pending.emplace_back(joint.child_link,
            has_motion || joint.type != kinematics::JointType::Fixed);
      }
    }
  }
  if (result.empty()) throw std::invalid_argument("Arm has no movable links: " + root);
  return result;
}

// 片腕学習用の形状選択。反対腕の可動部分のみを組合せ時の検査へ移管。
inline RobotModel make_independent_arm_collision_model(
    const RobotModel &model, const std::string &active_root,
    const std::vector<std::string> &arm_roots) {
  const auto active_links = collect_moving_arm_links(model, active_root);
  if (std::set<std::string>(arm_roots.begin(), arm_roots.end()).size() != arm_roots.size() ||
      std::find(arm_roots.begin(), arm_roots.end(), active_root) == arm_roots.end()) {
    throw std::invalid_argument("Invalid independent arm roots");
  }
  RobotModel selected_model = model;
  for (const auto &root : arm_roots) {
    if (root == active_root) continue;
    const auto other_links = collect_moving_arm_links(model, root);
    for (const auto &name : other_links) {
      if (active_links.count(name)) throw std::invalid_argument("Overlapping independent arms");
      auto link = *model.getLink(name);
      link.collisions.clear();
      selected_model.addLink(link);
    }
  }
  return selected_model;
}

} // 名前空間 simulation の終端
