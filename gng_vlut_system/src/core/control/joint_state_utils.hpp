#pragma once

#include <Eigen/Core>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace joint_state_utils {

// 位置を持つ関節の名前別索引。重複名は末尾の有効値を採用
inline std::unordered_map<std::string, double> build_position_map(
    const sensor_msgs::msg::JointState &message) {
  std::unordered_map<std::string, double> positions;
  positions.reserve(message.name.size());
  for (std::size_t idx = 0; idx < message.name.size(); ++idx) {
    if (idx < message.position.size()) {
      positions[message.name[idx]] = message.position[idx];
    }
  }
  return positions;
}

struct joint_position_pair {
  Eigen::VectorXf current;
  Eigen::VectorXf target;
};

// 指定順の現在角・目標角。現在角欠損はゼロ、目標角欠損は現在角を保持
inline joint_position_pair positions_in_order(
    const sensor_msgs::msg::JointState &state,
    const sensor_msgs::msg::JointState &target,
    const std::vector<std::string> &joint_names) {
  const auto current_map = build_position_map(state);
  const auto target_map = build_position_map(target);
  joint_position_pair positions;
  positions.current.resize(static_cast<Eigen::Index>(joint_names.size()));
  positions.target.resize(static_cast<Eigen::Index>(joint_names.size()));
  for (std::size_t idx = 0; idx < joint_names.size(); ++idx) {
    const auto current_it = current_map.find(joint_names[idx]);
    const auto target_it = target_map.find(joint_names[idx]);
    const double current = current_it != current_map.end() ? current_it->second : 0.0;
    const double desired = target_it != target_map.end() ? target_it->second : current;
    positions.current[static_cast<Eigen::Index>(idx)] = static_cast<float>(current);
    positions.target[static_cast<Eigen::Index>(idx)] = static_cast<float>(desired);
  }
  return positions;
}

} // 名前空間joint_state_utils
